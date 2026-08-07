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

"""Nudge the arm one joint at a time, and read the pose back out.

stock_jog.py moves in cartesian steps through the closed-form IK, which means
every pose it can reach has the approach pitch it was told at startup.  That
is the right constraint for teaching layout points - the task manager will
replay them at exactly one pitch - but the wrong tool when the question is
"what CAN this arm reach", or when a pose needs an elbow the IK's one branch
never chooses.  This tool has no opinion: each joint moves where it is sent.

The status line always shows where the gripper ended up (through the same
forward kinematics the layouts trust) and, more importantly, the approach
pitch the current joints produce.  A pose is only worth saving for the
layouts when that pitch is close to the layout's ``approach_pitch_deg``;
save it with ``p`` and the pitch is recorded alongside, so a mismatch is
visible in the file rather than discovered by the gripper coming in at the
wrong angle.

The arm starts from wherever it actually is - the first joint state heard is
the starting target, so launching this does not move the robot at all.

    ros2 run open_manipulator_playground stock_joint_jog.py \\
        --arm-action /station_b/arm_controller/follow_joint_trajectory \\
        --gripper-action /station_b/gripper_controller/gripper_cmd \\
        --joint-states /station_b/joint_states

Keys:
    1..5     select joint1..joint5
    w / s    selected joint + / -
    [ / ]    smaller / bigger step (degrees)
    o / c    gripper open / close
    p        save the current pose to the points file
    l        list saved points
    q        quit
"""

import argparse
import contextlib
import math
import sys
import termios
import tty

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from omx_kinematics import forward_kinematics, JOINT_NAMES
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
import stock_points
from trajectory_msgs.msg import JointTrajectoryPoint

STEP_SIZES_DEG = [0.5, 1.0, 2.0, 5.0, 10.0]
DEFAULT_STEP_INDEX = 2

# Slow: this is used with hands near the robot.
MOVE_DURATION = 0.8
MOVE_GRACE = 4.0
ANSWER_TIMEOUT = 5.0
GRIPPER_TIMEOUT = 5.0

# Keep commands inside what the cell can survive, not what the URDF allows -
# the URDF says +-2 pi on every joint, which through a tangle of cables is a
# promise nobody wants kept.
JOINT_LIMITS_DEG = {
    'joint1': (-150.0, 150.0),
    'joint2': (-100.0, 100.0),
    'joint3': (-100.0, 100.0),
    'joint4': (-110.0, 130.0),
    'joint5': (-150.0, 150.0),
}


def notice(text):
    erase = '\x1b[K' if sys.stdout.isatty() else ''
    print(f'\r  {text}{erase}')


class JointJogger(Node):
    """Moves one joint at a time and reports the pose that results."""

    def __init__(self, args):
        super().__init__('stock_joint_jog')
        self.arm = ActionClient(self, FollowJointTrajectory, args.arm_action)
        self.gripper = ActionClient(self, GripperCommand, args.gripper_action)
        self.args = args
        self.joints = None      # current target, radians, [j1..j5]
        self.selected = 0
        self.step_index = DEFAULT_STEP_INDEX

        self._seen = {}
        self.create_subscription(
            JointState, args.joint_states, self._on_joint_state, 10
        )

    # ------------------------------------------------------------- startup

    def _on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._seen[name] = position

    def wait_until_ready(self):
        if not self.arm.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f'no action server at {self.args.arm_action}. '
                f'Is the arm launch running?'
            )
        if not self.gripper.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                'no gripper action server; gripper keys will do nothing'
            )
            self.gripper = None

        # The first full joint state heard becomes the starting target, so
        # bringing the tool up moves nothing.
        deadline = self.get_clock().now().nanoseconds + int(5e9)
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self._seen for name in JOINT_NAMES):
                self.joints = [self._seen[name] for name in JOINT_NAMES]
                return
        raise RuntimeError(
            f'no joint state on {self.args.joint_states} - wrong namespace?'
        )

    # ------------------------------------------------------------- moving

    def _send(self, client, goal, timeout):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=ANSWER_TIMEOUT)
        handle = future.result()
        if handle is None or not handle.accepted:
            notice('the controller did not answer. Is it still running?')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        return result.done()

    def nudge(self, delta_deg):
        name = JOINT_NAMES[self.selected]
        target = list(self.joints)
        target[self.selected] += math.radians(delta_deg)

        low, high = JOINT_LIMITS_DEG[name]
        value_deg = math.degrees(target[self.selected])
        if not low <= value_deg <= high:
            notice(f'refused: {name} at {value_deg:.1f} deg outside [{low}, {high}]')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in target]
        point.time_from_start = Duration(
            sec=0, nanosec=int(MOVE_DURATION * 1e9)
        )
        goal.trajectory.points = [point]
        if self._send(self.arm, goal, timeout=MOVE_DURATION + MOVE_GRACE):
            self.joints = target

    def move_gripper(self, position):
        if self.gripper is None:
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.args.gripper_effort)
        self._send(self.gripper, goal, timeout=GRIPPER_TIMEOUT)

    # ------------------------------------------------------------- reading

    def pose(self):
        """(x, y, z, pitch_deg) of the current target joints."""
        x, y, z = forward_kinematics(self.joints)[:3]
        # Parallel pitch axes: the approach pitch is set by joints 2..4 alone,
        # mirroring the convention inside omx_kinematics.
        j2, j3, j4 = self.joints[1], self.joints[2], self.joints[3]
        pitch = -(j2 + j3 + j4)
        return x, y, z, math.degrees(pitch)

    def status_line(self):
        x, y, z, pitch = self.pose()
        step = STEP_SIZES_DEG[self.step_index]
        angles = ' '.join(
            f'{"*" if i == self.selected else " "}{name[-1]}:{math.degrees(v):7.1f}'
            for i, (name, v) in enumerate(zip(JOINT_NAMES, self.joints))
        )
        return (
            f'{angles} | ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'pitch {pitch:6.1f} deg | step {step} deg'
        )


@contextlib.contextmanager
def raw_keys():
    handle = sys.stdin.fileno()
    saved = termios.tcgetattr(handle)
    try:
        tty.setcbreak(handle)
        yield
    finally:
        termios.tcsetattr(handle, termios.TCSADRAIN, saved)


def prompt_name(jogger):
    """Ask for a point name in ordinary line mode."""
    handle = sys.stdin.fileno()
    saved = termios.tcgetattr(handle)
    line = termios.tcgetattr(handle)
    line[3] |= termios.ICANON | termios.ECHO
    termios.tcsetattr(handle, termios.TCSADRAIN, line)
    try:
        return input('  name: ').strip()
    finally:
        termios.tcsetattr(handle, termios.TCSADRAIN, saved)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--arm-action', default='/arm_controller/follow_joint_trajectory')
    parser.add_argument('--gripper-action', default='/gripper_controller/gripper_cmd')
    parser.add_argument('--joint-states', default='/joint_states',
                        help='joint state topic, namespaced like the actions')
    parser.add_argument('--gripper-open', type=float, default=0.40)
    parser.add_argument('--gripper-closed', type=float, default=0.15)
    parser.add_argument('--gripper-effort', type=float, default=10.0)
    parser.add_argument('--points-file', default=stock_points.DEFAULT_POINTS_FILE)
    args = parser.parse_args()

    rclpy.init()
    jogger = JointJogger(args)
    try:
        jogger.wait_until_ready()
    except RuntimeError as error:
        print(f'error: {error}')
        return 1

    print(__doc__.split('Keys:')[1])
    print('  Starting from the arm\'s current pose - nothing moves yet.')

    with raw_keys():
        while True:
            print(f'\r  {jogger.status_line()}\x1b[K', end='', flush=True)
            key = sys.stdin.read(1)
            if key == 'q':
                print()
                break
            elif key in '12345':
                jogger.selected = int(key) - 1
            elif key == 'w':
                jogger.nudge(+STEP_SIZES_DEG[jogger.step_index])
            elif key == 's':
                jogger.nudge(-STEP_SIZES_DEG[jogger.step_index])
            elif key == '[':
                jogger.step_index = max(0, jogger.step_index - 1)
            elif key == ']':
                jogger.step_index = min(len(STEP_SIZES_DEG) - 1, jogger.step_index + 1)
            elif key == 'o':
                jogger.move_gripper(args.gripper_open)
            elif key == 'c':
                jogger.move_gripper(args.gripper_closed)
            elif key == 'p':
                x, y, z, pitch = jogger.pose()
                name = prompt_name(jogger)
                if not name:
                    notice('no name, not saved')
                    continue
                try:
                    stock_points.save_point(args.points_file, stock_points.make_point(
                        name, x, y, z, pitch_deg=pitch, roll_deg=0.0,
                    ))
                except ValueError as error:
                    notice(f'not saved: {error}')
                    continue
                notice(f'saved {name}: ({x:.3f}, {y:.3f}, {z:.3f}) pitch {pitch:.1f}')
            elif key == 'l':
                try:
                    points = stock_points.load_points(args.points_file)
                except FileNotFoundError:
                    points = []
                if not points:
                    notice('no points saved yet')
                for point in points:
                    notice(
                        f'{point["name"]}: ({point["x"]:.3f}, {point["y"]:.3f}, '
                        f'{point["z"]:.3f}) pitch {point["pitch_deg"]:.1f}'
                    )

    jogger.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
