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

r"""Mark out where each stock bin sits in the camera image.

Shows the live camera feed and lets you drag a rectangle around each bin in the
order they are listed in the layout file.  The regions are written back into
that file, leaving its comments and every other value untouched.

    ros2 run open_manipulator_playground stock_calibrate.py \\
        --ros-args -p layout_file:=<path to stock_layout.yaml>

Drag to draw the current bin's rectangle, then:
    n / space  accept it and move to the next bin
    r          redraw the current bin
    s          save every rectangle to the layout file
    q / Esc    quit without saving
"""

import os
import re

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

WINDOW = 'stock bin calibration'


def write_rois(path, rois):
    """Replace each bin's ``roi`` line in the layout file, keeping the comments.

    ``rois`` maps a bin id to ``[x1, y1, x2, y2]``.  Returns the number of
    lines that were rewritten.
    """
    with open(path) as handle:
        lines = handle.readlines()

    current_bin = None
    replaced = 0
    for index, line in enumerate(lines):
        id_match = re.match(r'^(\s*)-\s*id:\s*(\S+)\s*$', line)
        if id_match:
            current_bin = id_match.group(2).strip('"\'')
            continue
        if current_bin is None or current_bin not in rois:
            continue
        roi_match = re.match(r'^(\s*)roi:\s*\[.*\]\s*$', line)
        if roi_match:
            x1, y1, x2, y2 = rois[current_bin]
            lines[index] = f'{roi_match.group(1)}roi: [{x1}, {y1}, {x2}, {y2}]\n'
            replaced += 1
            current_bin = None

    with open(path, 'w') as handle:
        handle.writelines(lines)
    return replaced


class StockCalibrate(Node):
    """Interactive picker for the bin regions."""

    def __init__(self):
        super().__init__('stock_calibrate')

        self.declare_parameter('layout_file', '')
        self.declare_parameter('image_topic', '/camera1/image_raw')

        self.layout_file = self.get_parameter('layout_file').value
        if not self.layout_file or not os.path.exists(self.layout_file):
            raise RuntimeError(f'layout_file not found: {self.layout_file!r}')

        with open(self.layout_file) as handle:
            layout = yaml.safe_load(handle)
        self.bin_ids = [entry['id'] for entry in layout['bins']]
        if not self.bin_ids:
            raise RuntimeError('layout has no bins to calibrate')

        self.bridge = CvBridge()
        self.frame = None
        self.rois = {}
        self.index = 0
        self.drag_start = None
        self.drag_current = None

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._on_image,
            qos_profile_sensor_data,
        )

        cv2.namedWindow(WINDOW)
        cv2.setMouseCallback(WINDOW, self._on_mouse)
        self.create_timer(0.03, self._draw)

        self.get_logger().info(f'Calibrating bins: {", ".join(self.bin_ids)}')
        self.get_logger().info('drag = mark, n = next, r = redo, s = save, q = quit')

    def _on_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - cv_bridge raises broadly
            self.get_logger().warn(f'Could not convert image: {exc}')

    def _on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.drag_current = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_current = (x, y)
            self._commit_drag()

    def _commit_drag(self):
        """Turn the finished drag into the current bin's region."""
        (x1, y1), (x2, y2) = self.drag_start, self.drag_current
        self.drag_start = None
        self.drag_current = None
        if abs(x2 - x1) < 5 or abs(y2 - y1) < 5:
            self.get_logger().warn('Rectangle too small, ignored')
            return
        roi = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
        if self.index < len(self.bin_ids):
            self.rois[self.bin_ids[self.index]] = roi
            self.get_logger().info(f'{self.bin_ids[self.index]} -> {roi}')

    def _draw(self):
        """Render the live view with the regions marked so far."""
        if self.frame is None:
            return
        canvas = self.frame.copy()

        for position, bin_id in enumerate(self.bin_ids):
            roi = self.rois.get(bin_id)
            if roi is None:
                continue
            colour = (0, 200, 0) if position != self.index else (0, 200, 255)
            cv2.rectangle(canvas, (roi[0], roi[1]), (roi[2], roi[3]), colour, 2)
            cv2.putText(
                canvas, bin_id, (roi[0], max(14, roi[1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA
            )

        if self.drag_start is not None and self.drag_current is not None:
            cv2.rectangle(canvas, self.drag_start, self.drag_current, (255, 200, 0), 1)

        if self.index < len(self.bin_ids):
            banner = f'draw {self.bin_ids[self.index]}  ({self.index + 1}/{len(self.bin_ids)})'
        else:
            banner = 'all bins marked - press s to save'
        cv2.putText(
            canvas, banner, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 2, cv2.LINE_AA
        )

        cv2.imshow(WINDOW, canvas)
        self._handle_key(cv2.waitKey(1) & 0xFF)

    def _handle_key(self, key):
        if key in (ord('n'), ord(' ')):
            if self.index < len(self.bin_ids):
                self.index += 1
        elif key == ord('r'):
            if self.index < len(self.bin_ids):
                self.rois.pop(self.bin_ids[self.index], None)
        elif key == ord('s'):
            self._save()
        elif key in (ord('q'), 27):
            self.get_logger().info('Quit without saving')
            raise KeyboardInterrupt

    def _save(self):
        """Write the marked regions back into the layout file."""
        missing = [b for b in self.bin_ids if b not in self.rois]
        if missing:
            self.get_logger().warn(f'Not saving: still missing {", ".join(missing)}')
            return
        replaced = write_rois(self.layout_file, self.rois)
        self.get_logger().info(
            f'Wrote {replaced} regions to {self.layout_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StockCalibrate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
