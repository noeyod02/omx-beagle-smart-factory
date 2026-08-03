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

"""Drive the gripper to a point by hand, and read the coordinate back out.

Two jobs, both needed before the layout can be trusted:

Checking the coordinate frame
    Send the arm to a known point and measure where the gripper actually ends
    up.  If the two disagree, every coordinate measured afterwards inherits
    that error, so this is worth doing first.

Teaching positions
    Nudge the gripper over a bin or a part, then print the coordinate in the
    exact form config/stock_layout.yaml expects and paste it in.  More accurate
    than measuring with a ruler, and it cannot get the sign of y wrong.

    ros2 run open_manipulator_playground stock_jog.py

Keys:
    w / s    forward / back   (+x / -x)
    a / d    left / right     (+y / -y)
    r / f    up / down        (+z / -z)
    [ / ]    smaller / bigger step
    o / c    gripper open / close
    p        print the current point as a layout entry
    h        return to the start point
    q        quit
"""

import argparse
import math
import sys
import termios
import tty

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from omx_kinematics import inverse_kinematics, JOINT_NAMES
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

STEP_SIZES = [0.001, 0.002, 0.005, 0.010, 0.020, 0.050]
DEFAULT_STEP_INDEX = 2

# Slow: this is used with hands near the robot.
MOVE_DURATION = 1.0
FIRST_MOVE_DURATION = 4.0


class Jogger(Node):
    """Moves the gripper in small cartesian steps and reports where it is."""

    def __init__(self, args):
        super().__init__('stock_jog')

        self.arm = ActionClient(self, FollowJointTrajectory, args.arm_action)
        self.gripper = ActionClient(self, GripperCommand, args.gripper_action)

        self.pitch = math.radians(args.pitch_deg)
        self.roll = math.radians(args.roll_deg)
        self.limits = {
            'x': (args.x_min, args.x_max),
            'y': (args.y_min, args.y_max),
            'z': (args.z_min, args.z_max),
        }
        self.gripper_open = args.gripper_open
        self.gripper_closed = args.gripper_closed
        self.gripper_effort = args.gripper_effort

        self.start = (args.x, args.y, args.z)
        self.point = list(self.start)
        self.step_index = DEFAULT_STEP_INDEX
        self.recorded = []

    @property
    def step(self):
        return STEP_SIZES[self.step_index]

    def wait_for_controllers(self, timeout=20.0):
        """Refuse to start until the arm can actually be commanded."""
        if not self.arm.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                f'no action server at {self.arm._action_name}. '
                f'Is omx_f.launch.py running?'
            )
        if not self.gripper.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                'no gripper action server; gripper keys will do nothing'
            )
            self.gripper = None

    def solve(self, x, y, z):
        """Return joint angles for a point, or None with a printed reason."""
        for axis, value in (('x', x), ('y', y), ('z', z)):
            low, high = self.limits[axis]
            if not low <= value <= high:
                print(f'\r  refused: {axis}={value:.3f} outside [{low}, {high}]      ')
                return None
        joints = inverse_kinematics(x, y, z, pitch=self.pitch, roll=self.roll)
        if joints is None:
            print(f'\r  refused: ({x:.3f}, {y:.3f}, {z:.3f}) is out of reach      ')
        return joints

    def move_to(self, x, y, z, duration=MOVE_DURATION):
        """Send the arm to a point and wait for it to arrive."""
        joints = self.solve(x, y, z)
        if joints is None:
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in joints]
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration - int(duration)) * 1e9)
        )
        goal.trajectory.points = [point]

        if not self._send(self.arm, goal):
            return False
        self.point = [x, y, z]
        return True

    def set_gripper(self, position):
        """Open or close the gripper."""
        if self.gripper is None:
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.gripper_effort
        self._send(self.gripper, goal)

    def _send(self, client, goal):
        """Send a goal and block until it finishes."""
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('\r  goal rejected by the controller      ')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        return True

    def status_line(self):
        x, y, z = self.point
        return (
            f'\r  x {x:+.3f}  y {y:+.3f}  z {z:+.3f}   '
            f'step {self.step * 1000:.0f} mm   '
            f'recorded {len(self.recorded)}      '
        )

    def record(self):
        """Print the current point in the form the layout file uses."""
        x, y, z = self.point
        entry = f'{{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}'
        self.recorded.append(entry)
        print(f'\r  recorded #{len(self.recorded)}: {entry}' + ' ' * 12)

    def report(self):
        """Print everything recorded, ready to paste into the layout."""
        if not self.recorded:
            return
        print('\n\nRecorded points, in order:\n')
        for index, entry in enumerate(self.recorded, start=1):
            print(f'  {index}. {entry}')
        print(
            '\nPaste these into config/stock_layout.yaml as warehouse pick_points\n'
            "or as a bin's place value, then run:\n"
            '  python3 stock_reach_check.py ../config/stock_layout.yaml\n'
        )


def read_key():
    """Read one keypress without waiting for Enter.

    Falls back to a plain read when stdin is not a terminal, so a canned key
    sequence can be piped in to rehearse a sequence against mock hardware.
    """
    if not sys.stdin.isatty():
        return sys.stdin.read(1) or 'q'

    handle = sys.stdin.fileno()
    saved = termios.tcgetattr(handle)
    try:
        tty.setraw(handle)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(handle, termios.TCSADRAIN, saved)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--arm-action', default='/arm_controller/follow_joint_trajectory')
    parser.add_argument('--gripper-action', default='/gripper_controller/gripper_cmd')
    parser.add_argument('--x', type=float, default=0.124, help='start x, metres')
    parser.add_argument('--y', type=float, default=0.0, help='start y, metres')
    parser.add_argument('--z', type=float, default=0.150, help='start z, metres')
    parser.add_argument('--pitch-deg', type=float, default=-90.0,
                        help='gripper approach angle; -90 points straight down')
    parser.add_argument('--roll-deg', type=float, default=0.0)
    parser.add_argument('--x-min', type=float, default=0.05)
    parser.add_argument('--x-max', type=float, default=0.28)
    parser.add_argument('--y-min', type=float, default=-0.20)
    parser.add_argument('--y-max', type=float, default=0.20)
    parser.add_argument('--z-min', type=float, default=0.02)
    parser.add_argument('--z-max', type=float, default=0.22)
    parser.add_argument('--gripper-open', type=float, default=0.40,
                        help='joint value for open; gap_mm = 100*value + 10')
    parser.add_argument('--gripper-closed', type=float, default=0.15,
                        help='joint value for closed; set narrower than the part')
    parser.add_argument('--gripper-effort', type=float, default=10.0)
    # Everything after --ros-args belongs to rclpy, not to us.
    known, _ = parser.parse_known_args(argv)
    return known


def main():
    argv = list(sys.argv[1:])
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parse_args(argv)

    rclpy.init()
    node = Jogger(args)
    try:
        node.wait_for_controllers()
    except RuntimeError as exc:
        print(f'\n{exc}\n')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(__doc__)
    print(f'Moving to the start point {node.start} over {FIRST_MOVE_DURATION:.0f} s.')
    print('Keep clear of the arm.\n')
    if not node.move_to(*node.start, duration=FIRST_MOVE_DURATION):
        print('Could not reach the start point; pick another with --x --y --z')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    moves = {
        'w': (0, +1), 's': (0, -1),
        'a': (1, +1), 'd': (1, -1),
        'r': (2, +1), 'f': (2, -1),
    }

    try:
        while True:
            print(node.status_line(), end='', flush=True)
            key = read_key().lower()

            if key in ('q', '\x03'):
                break
            elif key in moves:
                axis, sign = moves[key]
                target = list(node.point)
                target[axis] += sign * node.step
                node.move_to(*target)
            elif key == '[':
                node.step_index = max(0, node.step_index - 1)
            elif key == ']':
                node.step_index = min(len(STEP_SIZES) - 1, node.step_index + 1)
            elif key == 'o':
                node.set_gripper(node.gripper_open)
            elif key == 'c':
                node.set_gripper(node.gripper_closed)
            elif key == 'p':
                node.record()
            elif key == 'h':
                node.move_to(*node.start, duration=FIRST_MOVE_DURATION)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
