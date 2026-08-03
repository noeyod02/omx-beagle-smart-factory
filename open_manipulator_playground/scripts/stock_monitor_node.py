#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Watch stock bins through a fixed camera and report which ones are empty.

Two detection backends are available:

``reference``
    Compares each bin's region against a reference photograph of the same bins
    when empty, and calls a bin empty when little has changed.  Needs no model
    and no training data, so the rest of the pipeline can be brought up first.
    Capture the reference once the bins are empty and the lighting is final::

        ros2 topic pub --once /stock/capture_reference std_msgs/Empty {}

``yolo``
    Runs an Ultralytics model that has been trained to tell an empty bin from a
    filled one, and assigns each detection to the bin whose region contains it.

Either way a bin has to read the same for several frames in a row before it is
reported as stable, and readings are ignored entirely while the arm is moving
through the camera's view.
"""

import json
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
import yaml

STATE_EMPTY = 'empty'
STATE_FILLED = 'filled'
STATE_UNKNOWN = 'unknown'


class BinTracker:
    """Debounces one bin's readings so a single bad frame cannot trigger a job."""

    def __init__(self, bin_id, roi, stable_frames):
        self.bin_id = bin_id
        self.roi = roi
        self.stable_frames = stable_frames
        self.state = STATE_UNKNOWN
        self.confidence = 0.0
        self.agree_count = 0

    def update(self, state, confidence):
        """Fold a new reading in and report whether the bin is settled."""
        if state == self.state:
            self.agree_count = min(self.agree_count + 1, self.stable_frames)
        else:
            self.state = state
            self.agree_count = 1
        self.confidence = confidence
        return self.is_stable

    @property
    def is_stable(self):
        return self.state != STATE_UNKNOWN and self.agree_count >= self.stable_frames

    def to_dict(self):
        return {
            'id': self.bin_id,
            'state': self.state,
            'confidence': round(float(self.confidence), 3),
            'stable': self.is_stable,
        }


class StockMonitor(Node):
    """Publishes the state of every stock bin seen by a fixed camera."""

    def __init__(self):
        super().__init__('stock_monitor')

        self.declare_parameter('layout_file', '')
        self.declare_parameter('image_topic', '/camera1/image_raw')
        self.declare_parameter('status_topic', '/stock/status')
        self.declare_parameter('task_state_topic', '/stock/task_state')
        self.declare_parameter('debug_image_topic', '/stock/debug_image')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('backend', 'reference')
        self.declare_parameter('rate_hz', 4.0)
        self.declare_parameter('stable_frames', 5)

        # reference backend
        self.declare_parameter('reference_path', '/tmp/stock_reference.png')
        self.declare_parameter('pixel_threshold', 30)
        self.declare_parameter('occupancy_threshold', 0.08)

        # yolo backend
        self.declare_parameter('model_path', '')
        self.declare_parameter('empty_class', 'empty')
        self.declare_parameter('filled_class', 'filled')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('imgsz', 640)

        layout_file = self.get_parameter('layout_file').value
        if not layout_file or not os.path.exists(layout_file):
            raise RuntimeError(f'layout_file not found: {layout_file!r}')

        self.backend = self.get_parameter('backend').value
        if self.backend not in ('reference', 'yolo'):
            raise RuntimeError(
                f"backend must be 'reference' or 'yolo', got {self.backend!r}"
            )

        stable_frames = int(self.get_parameter('stable_frames').value)
        self.trackers = []
        with open(layout_file) as handle:
            layout = yaml.safe_load(handle)
        for entry in layout['bins']:
            roi = entry.get('roi')
            if not roi or len(roi) != 4:
                raise RuntimeError(
                    f"bin {entry['id']!r} has no usable roi; run stock_calibrate.py"
                )
            self.trackers.append(BinTracker(entry['id'], [int(v) for v in roi],
                                            stable_frames))

        self.bridge = CvBridge()
        self.last_frame = None
        self.reference = None
        self.model = None
        self.class_names = {}
        self.vision_clear = True
        self.last_process_time = 0.0
        self.min_interval = 1.0 / max(1e-3, float(self.get_parameter('rate_hz').value))

        if self.backend == 'reference':
            self._load_reference()
        else:
            self._load_model()

        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10
        )
        self.debug_pub = None
        if bool(self.get_parameter('publish_debug_image').value):
            self.debug_pub = self.create_publisher(
                Image, self.get_parameter('debug_image_topic').value, 1
            )

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self.get_parameter('task_state_topic').value,
            self._on_task_state,
            10,
        )
        self.create_subscription(
            Empty, '/stock/capture_reference', self._on_capture_reference, 10
        )

        self.get_logger().info(
            f'Stock monitor started: backend={self.backend}, '
            f'{len(self.trackers)} bins'
        )

    # ------------------------------------------------------------- backends

    def _load_reference(self):
        """Load the empty-bin reference photo if one has been captured."""
        path = self.get_parameter('reference_path').value
        if os.path.exists(path):
            self.reference = cv2.imread(path)
            self.get_logger().info(f'Loaded empty-bin reference from {path}')
        else:
            self.get_logger().warn(
                f'No reference image at {path}. Empty the bins and run: '
                f'ros2 topic pub --once /stock/capture_reference std_msgs/Empty {{}}'
            )

    def _load_model(self):
        """Load the Ultralytics model used by the yolo backend."""
        model_path = self.get_parameter('model_path').value
        if not model_path or not os.path.exists(model_path):
            raise RuntimeError(
                f'backend is yolo but model_path is not usable: {model_path!r}'
            )
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'backend is yolo but ultralytics is not installed. Install it in '
                'the container, or switch to backend:=reference.'
            ) from exc

        self.model = YOLO(model_path)
        self.class_names = dict(self.model.names)
        self.get_logger().info(
            f'Loaded {model_path} with classes {sorted(self.class_names.values())}'
        )

    # ------------------------------------------------------------- callbacks

    def _on_task_state(self, msg):
        """Track whether the arm is currently blocking the camera's view."""
        try:
            self.vision_clear = bool(json.loads(msg.data).get('vision_clear', True))
        except json.JSONDecodeError:
            self.vision_clear = True

    def _on_capture_reference(self, _msg):
        """Store the current frame as the empty-bin reference."""
        if self.last_frame is None:
            self.get_logger().warn('No camera frame received yet')
            return
        path = self.get_parameter('reference_path').value
        cv2.imwrite(path, self.last_frame)
        self.reference = self.last_frame.copy()
        self.get_logger().info(f'Captured empty-bin reference to {path}')

    def _on_image(self, msg):
        """Evaluate every bin in the frame, at the configured rate."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - cv_bridge raises broadly
            self.get_logger().warn(f'Could not convert image: {exc}')
            return
        self.last_frame = frame

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_process_time < self.min_interval:
            return
        self.last_process_time = now

        # The arm sweeps through the bins during a job; readings taken then say
        # more about where the arm is than about how full the bins are.
        if not self.vision_clear:
            return

        if self.backend == 'reference':
            readings = self._read_by_reference(frame)
        else:
            readings = self._read_by_yolo(frame)

        for tracker in self.trackers:
            state, confidence = readings.get(tracker.bin_id, (STATE_UNKNOWN, 0.0))
            tracker.update(state, confidence)

        self._publish_status()
        if self.debug_pub is not None:
            self._publish_debug_image(frame)

    # -------------------------------------------------------------- reading

    def _crop(self, frame, roi):
        """Clip ``roi`` to the frame and return that region."""
        height, width = frame.shape[:2]
        x1 = max(0, min(int(roi[0]), width - 1))
        y1 = max(0, min(int(roi[1]), height - 1))
        x2 = max(x1 + 1, min(int(roi[2]), width))
        y2 = max(y1 + 1, min(int(roi[3]), height))
        return frame[y1:y2, x1:x2]

    def _read_by_reference(self, frame):
        """Call a bin empty when its region still matches the empty reference."""
        if self.reference is None:
            return {}
        if self.reference.shape != frame.shape:
            self.get_logger().warn(
                'Reference image size does not match the camera; recapture it'
            )
            return {}

        threshold = int(self.get_parameter('pixel_threshold').value)
        occupancy_limit = float(self.get_parameter('occupancy_threshold').value)

        readings = {}
        for tracker in self.trackers:
            live = self._crop(frame, tracker.roi)
            base = self._crop(self.reference, tracker.roi)
            diff = cv2.absdiff(
                cv2.cvtColor(live, cv2.COLOR_BGR2GRAY),
                cv2.cvtColor(base, cv2.COLOR_BGR2GRAY),
            )
            changed = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)[1]
            # Speckle from sensor noise is not a part sitting in the bin.
            changed = cv2.morphologyEx(
                changed, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
            )
            occupancy = float(np.count_nonzero(changed)) / changed.size

            if occupancy < occupancy_limit:
                state = STATE_EMPTY
                # Far below the limit means a confident empty, and vice versa.
                confidence = 1.0 - occupancy / occupancy_limit
            else:
                state = STATE_FILLED
                confidence = min(1.0, occupancy / occupancy_limit - 1.0)
            readings[tracker.bin_id] = (state, confidence)
        return readings

    def _read_by_yolo(self, frame):
        """Assign model detections to bins and read each bin's state off them."""
        empty_class = self.get_parameter('empty_class').value
        filled_class = self.get_parameter('filled_class').value
        conf_threshold = float(self.get_parameter('confidence_threshold').value)
        imgsz = int(self.get_parameter('imgsz').value)

        results = self.model.predict(
            frame, imgsz=imgsz, conf=conf_threshold, verbose=False
        )
        best = {}
        for result in results:
            for box in result.boxes:
                label = self.class_names.get(int(box.cls[0]), '')
                if label not in (empty_class, filled_class):
                    continue
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                centre = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

                bin_id = self._bin_at(centre)
                if bin_id is None:
                    continue
                if bin_id not in best or confidence > best[bin_id][1]:
                    state = STATE_EMPTY if label == empty_class else STATE_FILLED
                    best[bin_id] = (state, confidence)
        return best

    def _bin_at(self, point):
        """Return the id of the bin whose region contains ``point``."""
        x, y = point
        for tracker in self.trackers:
            x1, y1, x2, y2 = tracker.roi
            if x1 <= x <= x2 and y1 <= y <= y2:
                return tracker.bin_id
        return None

    # ------------------------------------------------------------- reporting

    def _publish_status(self):
        """Publish the current view of every bin."""
        self.status_pub.publish(String(data=json.dumps({
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'backend': self.backend,
            'bins': [tracker.to_dict() for tracker in self.trackers],
        })))

    def _publish_debug_image(self, frame):
        """Publish the frame with each bin's region and verdict drawn on it."""
        canvas = frame.copy()
        for tracker in self.trackers:
            x1, y1, x2, y2 = tracker.roi
            if tracker.state == STATE_EMPTY:
                colour = (0, 0, 255) if tracker.is_stable else (0, 140, 255)
            elif tracker.state == STATE_FILLED:
                colour = (0, 200, 0)
            else:
                colour = (150, 150, 150)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(
                canvas,
                f'{tracker.bin_id} {tracker.state} {tracker.confidence:.2f}',
                (x1, max(14, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                colour,
                1,
                cv2.LINE_AA,
            )
        if not self.vision_clear:
            cv2.putText(
                canvas, 'arm in view - paused', (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA
            )
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = StockMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
