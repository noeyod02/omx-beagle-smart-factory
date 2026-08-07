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

r"""Teach the camera where the table is, in the arm's own coordinates.

Needed before anything can be picked by camera.  A detector reports a part at
some pixel; only this mapping turns that into a place the arm can be sent to.

The two halves are measured separately and joined here:

Robot side
    Jog the gripper down onto each marker and press ``p``::

        ros2 run open_manipulator_playground stock_jog.py \
            --arm-action /station_a/arm_controller/follow_joint_trajectory \
            --gripper-action /station_a/gripper_controller/gripper_cmd

    Save what it prints on quitting into a file - this reads the jogger's own
    ``{x: ..., y: ..., z: ...}`` lines directly, so the block can be pasted in
    untouched.

Camera side
    Move the arm out of the way, leave the markers where they are, and click
    each one here in the same order the arm visited them.

    ros2 run open_manipulator_playground stock_hand_eye.py \
        --points-file markers.txt --image-topic /camera2/image_raw

Use six or seven markers spread to the corners of the area parts can appear
in, not four in a tight cluster: four points fit any homography exactly and
tell you nothing about accuracy, and the fit is only trustworthy inside the
region the markers surround.  The tool leaves each point out of the fit in turn
and reports how far off the mapping then puts it, which is an honest estimate
of how close the gripper will land on a part it has never seen.

Keys:
    click     mark the next point
    u         undo the last point
    r         start the clicking over
    enter     fit the mapping once every point is marked
    q / Esc   quit without fitting
"""

import argparse
import re
import sys

import cv2
import numpy as np

from stock_camera import apply_homography, format_homography, solve_homography

WINDOW = 'hand-eye calibration - click each marker in order'

# Drawn over the camera picture; chosen to stay visible on both the pale table
# and the darker parts.
MARK_COLOUR = (0, 240, 255)
TEXT_COLOUR = (255, 255, 255)

POINT_PATTERN = re.compile(
    r'x:\s*(?P<x>[-+0-9.eE]+)\s*,\s*y:\s*(?P<y>[-+0-9.eE]+)'
    r'(?:\s*,\s*z:\s*(?P<z>[-+0-9.eE]+))?'
)


def read_points(path):
    """Pull robot points out of whatever the jogger printed.

    Every line holding an ``x:`` and a ``y:`` counts, so the recorded block can
    be pasted in with its numbering and surrounding prose left in place.
    """
    with open(path) as handle:
        text = handle.read()

    points, heights = [], []
    for match in POINT_PATTERN.finditer(text):
        points.append((float(match.group('x')), float(match.group('y'))))
        if match.group('z') is not None:
            heights.append(float(match.group('z')))

    if len(points) < 4:
        raise SystemExit(
            f'{path} holds {len(points)} points; at least 4 are needed, and 6 '
            f'or 7 spread over the area give an error estimate worth having.'
        )
    return points, heights


def pick_height(heights):
    """Report the plane the markers were measured on, and whether they agree.

    The mapping is only valid on one plane.  Markers touched at visibly
    different heights mean either the table is not level or the gripper was not
    brought down to it, and both put every later pick out by the difference.
    """
    if not heights:
        return None
    spread = (max(heights) - min(heights)) * 1000.0
    if spread > 5.0:
        print(
            f'  warning: the markers were touched over a {spread:.0f} mm range '
            f'of height ({min(heights):.3f} to {max(heights):.3f}).\n'
            f'  The mapping holds on one plane only. Re-take any marker the '
            f'gripper did not reach the table on.'
        )
    return sum(heights) / len(heights)


class Clicker:
    """Collects one image point per robot point, in order."""

    def __init__(self, frame, wanted):
        self.frame = frame
        self.wanted = wanted
        self.pixels = []

    def on_mouse(self, event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.pixels) < self.wanted:
            self.pixels.append((float(x), float(y)))

    def draw(self):
        canvas = self.frame.copy()
        for index, (u, v) in enumerate(self.pixels, start=1):
            centre = (int(u), int(v))
            cv2.drawMarker(canvas, centre, MARK_COLOUR, cv2.MARKER_CROSS, 18, 2)
            cv2.circle(canvas, centre, 12, MARK_COLOUR, 1)
            cv2.putText(canvas, str(index), (centre[0] + 14, centre[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, MARK_COLOUR, 2)

        done = len(self.pixels)
        if done < self.wanted:
            message = f'click marker {done + 1} of {self.wanted}'
        else:
            message = 'all marked - press Enter to fit, u to undo'
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(canvas, message, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOUR, 1)
        return canvas


def collect_pixels(frame, wanted):
    """Show the picture and return the clicked points, or None if abandoned."""
    clicker = Clicker(frame, wanted)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW, clicker.on_mouse)
    try:
        while True:
            cv2.imshow(WINDOW, clicker.draw())
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord('q')):
                return None
            if key == ord('u') and clicker.pixels:
                clicker.pixels.pop()
            elif key == ord('r'):
                clicker.pixels.clear()
            elif key in (13, 10) and len(clicker.pixels) == wanted:
                return clicker.pixels
    finally:
        cv2.destroyWindow(WINDOW)


def leave_one_out(pixels, points):
    """Refit without each point in turn, and see where the fit then puts it.

    This is the number that matters.  The residual of a point the mapping was
    fitted through flatters it; the miss on a point it has never seen is what a
    part in an unvisited spot will suffer.
    """
    misses = []
    for index in range(len(pixels)):
        rest_pixels = pixels[:index] + pixels[index + 1:]
        rest_points = points[:index] + points[index + 1:]
        if len(rest_pixels) < 4:
            return None
        matrix, _ = solve_homography(rest_pixels, rest_points)
        fx, fy = apply_homography(matrix, *pixels[index])
        x, y = points[index]
        misses.append(float(np.hypot(fx - x, fy - y)) * 1000.0)
    return misses


def frame_from_topic(topic, timeout):
    """Wait for one picture from a running camera."""
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node('stock_hand_eye')
    bridge = CvBridge()
    holder = {}

    node.create_subscription(
        Image, topic,
        lambda msg: holder.setdefault('frame', bridge.imgmsg_to_cv2(msg, 'bgr8')),
        qos_profile_sensor_data,
    )
    try:
        waited = 0.0
        while 'frame' not in holder and waited < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            waited += 0.2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if 'frame' not in holder:
        raise SystemExit(
            f'no picture arrived on {topic} within {timeout:.0f} s. Check that '
            f'the camera is running:  ros2 topic info {topic}'
        )
    return holder['frame']


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--points-file', required=True,
                        help='file holding what stock_jog.py printed on quitting')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--image-topic', help='take the picture from a running camera')
    source.add_argument('--image', help='use a saved picture instead')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='seconds to wait for a picture from the topic')
    known, _ = parser.parse_known_args(argv)
    return known


def main():
    argv = list(sys.argv[1:])
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parse_args(argv)

    points, heights = read_points(args.points_file)
    print(f'\n{len(points)} robot points read from {args.points_file}.')
    plane = pick_height(heights)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f'could not read {args.image}')
    else:
        frame = frame_from_topic(args.image_topic, args.timeout)

    print('Click each marker in the order the arm visited them.\n')
    pixels = collect_pixels(frame, len(points))
    if pixels is None:
        print('Abandoned; nothing was fitted.\n')
        return 1

    matrix, residuals = solve_homography(pixels, points)
    misses = leave_one_out(pixels, points)

    print('\nHow well each marker is placed by the mapping:\n')
    print('  marker   fitted    left out')
    for index, residual in enumerate(residuals):
        held = f'{misses[index]:7.1f} mm' if misses else '        -'
        print(f'  {index + 1:>6}  {residual:6.1f} mm  {held}')

    if misses:
        worst = max(misses)
        print(f'\nWorst miss on a marker the mapping had not seen: {worst:.1f} mm.')
        print('That is roughly how far off centre the gripper will close on a')
        print('part it finds somewhere new. Compare it against half the gap')
        print('between the jaws before trusting it.')

    print("\nPaste this into the station's layout file:\n")
    print(format_homography(matrix, errors_mm=residuals, pick_z=plane))
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
