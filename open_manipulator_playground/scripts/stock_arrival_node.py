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

"""See the carrier arrive, and only then let the arm reach for it.

The bridge already reports `Arrived`, but that report is dead reckoning: it
says the wheels turned the right amount, not that the carrier is in the bay.
On 2026-08-06 one return leg drifted -14.7 degrees and the bridge reported
`Arrived` all the same; on 2026-08-07 a return leg drifted +6.7.  An arm that
trusts the report alone will sooner or later place a part on a tray that is
not where the tray was taught to be.

This node watches the bay through a fixed camera and a YOLO model trained on
the carrier itself.  A detection whose centre sits inside the bay ROI, held
for `need_frames` consecutive frames, is an arrival; `lost_frames` consecutive
frames without one is a departure.  The hysteresis matters: a single frame
where the model blinks must not read as the carrier leaving and returning,
because every false return is a false trigger.

On a real arrival - a transition from confirmed-away to confirmed-here, not
the carrier merely being found parked at startup - the node can start the
warehouse arm's load by publishing a transfer to its task manager.  Startup
is excluded on purpose: between missions the carrier lives in this bay, so
"the node came up and saw it" would start a job nobody asked for.

The trigger defers to everything that already sequences the cell: it is
skipped while the relay is mid-mission (the relay does its own loading), when
the arm is not idle, before the arm has been heard from at all, and during a
cooldown after a fired trigger.  The Bool on `arrived_topic` is published
regardless, so the relay can later use this node as an arrival *gate* rather
than a *starter*.
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


class StockArrival(Node):
    """Publish visually-confirmed carrier arrivals, and optionally act on one."""

    def __init__(self):
        super().__init__('stock_arrival')

        self.declare_parameter('model_path', '')
        # A V4L2 index ('0'), a device path, or an MJPEG stream URL. The URL
        # form lets this node share a camera that cctv_server already holds
        # (V4L2 devices are exclusive; the HTTP stream is not).
        self.declare_parameter('video_device', '0')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        # x1, y1, x2, y2 in pixels of the bay, measured with get_roi-style
        # dragging on a frame from the same camera at the same resolution.
        self.declare_parameter('roi', [179, 142, 486, 410])
        self.declare_parameter('confidence', 0.6)
        self.declare_parameter('target_class', 'beagle')
        self.declare_parameter('need_frames', 8)
        # A real departure leaves the bay empty for the whole drive - ten
        # seconds at the least - while a person leaning through the view
        # blocks it for two or three. Confirming a departure slowly costs
        # nothing and stops the lean from reading as leave-and-return,
        # which is the false trigger that matters (2026-08-07, caught live).
        self.declare_parameter('lost_frames', 45)
        self.declare_parameter('arrived_topic', '/stock/beagle_arrived')
        self.declare_parameter('debug_topic', '/stock/arrival_image/compressed')

        # The trigger. An empty transfer_topic turns it off, leaving a node
        # that only reports.
        self.declare_parameter('transfer_topic', '/station_a/stock/transfer')
        self.declare_parameter('task_state_topic', '/station_a/stock/task_state')
        self.declare_parameter('relay_state_topic', '/stock/relay_state')
        self.declare_parameter('transfer_from', 'warehouse')
        self.declare_parameter('transfer_to', 'carrier')
        self.declare_parameter('trigger_cooldown', 30.0)
        # Fire on the first sighting after startup too. Off by default: the
        # carrier parks in this bay between missions, so being seen at
        # startup is where it was left, not an arrival.
        self.declare_parameter('trigger_on_initial', False)

        if cv2 is None:
            raise RuntimeError('stock_arrival needs OpenCV (cv2)')

        model_path = self.get_parameter('model_path').value
        if not model_path:
            raise RuntimeError(
                'model_path is required: the YOLO weights that know the carrier'
            )
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'stock_arrival needs ultralytics. Install it where this node '
                'runs (the host): pip3 install --user --break-system-packages '
                'ultralytics "numpy<2"'
            ) from exc
        self.model = YOLO(model_path)
        names = getattr(self.model, 'names', {}) or {}
        target = self.get_parameter('target_class').value
        self.target_ids = {i for i, n in names.items() if n == target}
        if not self.target_ids:
            raise RuntimeError(
                f'model at {model_path} has no class named {target!r}; '
                f'it knows {sorted(names.values())}'
            )

        self.arrived_pub = self.create_publisher(
            Bool, self.get_parameter('arrived_topic').value, 10
        )
        debug_topic = self.get_parameter('debug_topic').value
        self.debug_pub = None
        if debug_topic:
            from sensor_msgs.msg import CompressedImage
            self._CompressedImage = CompressedImage
            self.debug_pub = self.create_publisher(CompressedImage, debug_topic, 1)

        self.transfer_topic = self.get_parameter('transfer_topic').value
        self.transfer_pub = None
        if self.transfer_topic:
            self.transfer_pub = self.create_publisher(String, self.transfer_topic, 10)
            self.create_subscription(
                String, self.get_parameter('task_state_topic').value,
                self._on_task_state, 10,
            )
            self.create_subscription(
                String, self.get_parameter('relay_state_topic').value,
                self._on_relay_state, 10,
            )

        self.arm_state = None       # last JSON 'state' from the task manager
        self.relay_state = None     # last JSON 'state' from the relay, if any
        self.arrived = False
        self.seen_away = self.get_parameter('trigger_on_initial').value
        self.last_trigger = 0.0
        self.job_seq = 9000  # far from the relay's own sequence numbers

        # Republish the current verdict at 1 Hz so late subscribers hear it.
        self.create_timer(1.0, self._heartbeat)

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._watch, daemon=True)
        self._worker.start()

        roi = list(self.get_parameter('roi').value)
        self.get_logger().info(
            f'Arrival watch started: class {target!r} in roi {roi} at conf '
            f'>= {self.get_parameter("confidence").value}, trigger '
            f'{"-> " + self.transfer_topic if self.transfer_pub else "off"}'
        )

    # ------------------------------------------------------------ listening

    def _on_task_state(self, msg):
        try:
            self.arm_state = json.loads(msg.data).get('state')
        except json.JSONDecodeError:
            pass

    def _on_relay_state(self, msg):
        try:
            self.relay_state = json.loads(msg.data).get('state')
        except json.JSONDecodeError:
            pass

    def _heartbeat(self):
        self.arrived_pub.publish(Bool(data=self.arrived))

    # ------------------------------------------------------------- watching

    def _watch(self):
        """Their detect_arrival.py loop, minus the file and the window."""
        device = self.get_parameter('video_device').value
        if isinstance(device, str) and device.isdigit():
            device = int(device)
        cap = cv2.VideoCapture(device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('frame_width').value)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('frame_height').value)
        if not cap.isOpened():
            self.get_logger().error(f'camera {device!r} would not open')
            return

        roi = list(self.get_parameter('roi').value)
        conf = float(self.get_parameter('confidence').value)
        need = int(self.get_parameter('need_frames').value)
        lost = int(self.get_parameter('lost_frames').value)

        hit = miss = 0
        frame_no = 0
        while not self._stop.is_set():
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warning('camera read failed; retrying')
                time.sleep(1.0)
                continue

            result = self.model(frame, imgsz=640, conf=conf, verbose=False)[0]
            detected = False
            for box in result.boxes:
                if int(box.cls) not in self.target_ids:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                if roi[0] <= cx <= roi[2] and roi[1] <= cy <= roi[3]:
                    detected = True
                    break

            # While the arm is working it reaches over the bay and hides the
            # carrier from the camera. Whatever the model sees during that
            # is the arm, not an arrival or a departure - hold the counters
            # until the arm is out of the way. Only 'busy' means the arm is
            # actually moving; blocked or cooldown is an arm parked at home,
            # in full view of nothing, and freezing on those left the watch
            # blind after a refused job (2026-08-07).
            if self.transfer_pub is not None and self.arm_state == 'busy':
                hit = miss = 0
            elif detected:
                hit, miss = hit + 1, 0
            else:
                hit, miss = 0, miss + 1

            if not self.arrived and not self.seen_away and miss >= lost:
                # Starting over an empty bay is as good as watching the
                # carrier leave: the first arrival after this is real.
                self.seen_away = True
                self.get_logger().info('bay confirmed empty at start')

            if not self.arrived and hit >= need:
                self.arrived = True
                self._on_arrive()
            elif self.arrived and miss >= lost:
                self.arrived = False
                self.seen_away = True
                self.get_logger().info('carrier departed the bay')
                self.arrived_pub.publish(Bool(data=False))

            frame_no += 1
            if self.debug_pub is not None and frame_no % 5 == 0:
                vis = result.plot()
                colour = (0, 0, 255) if self.arrived else (0, 255, 0)
                cv2.rectangle(vis, (roi[0], roi[1]), (roi[2], roi[3]), colour, 2)
                cv2.putText(
                    vis, f'arrived={self.arrived} hit={hit} miss={miss}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2,
                )
                ok, jpeg = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    msg = self._CompressedImage()
                    msg.format = 'jpeg'
                    msg.data = jpeg.tobytes()
                    self.debug_pub.publish(msg)

        cap.release()

    # ------------------------------------------------------------ triggering

    def _on_arrive(self):
        self.get_logger().info('carrier arrived in the bay')
        self.arrived_pub.publish(Bool(data=True))

        if self.transfer_pub is None:
            return
        if not self.seen_away:
            self.get_logger().info(
                'found parked at startup - not an arrival, no trigger'
            )
            return
        if self.relay_state not in (None, 'idle'):
            self.get_logger().info(
                f'relay is {self.relay_state!r}; it does its own loading - no trigger'
            )
            return
        if self.arm_state != 'idle':
            self.get_logger().warning(
                f'arm state is {self.arm_state!r}, not idle - no trigger'
            )
            return
        now = time.monotonic()
        cooldown = float(self.get_parameter('trigger_cooldown').value)
        if now - self.last_trigger < cooldown:
            self.get_logger().info('inside trigger cooldown - no trigger')
            return

        self.last_trigger = now
        self.job_seq += 1
        request = {
            'from': self.get_parameter('transfer_from').value,
            'to': self.get_parameter('transfer_to').value,
            'id': self.job_seq,
        }
        self.transfer_pub.publish(String(data=json.dumps(request)))
        self.get_logger().info(
            f'arrival confirmed by sight: asked {self.transfer_topic} for '
            f'{request["from"]} -> {request["to"]} (job {self.job_seq})'
        )

    def destroy_node(self):
        self._stop.set()
        self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StockArrival()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
