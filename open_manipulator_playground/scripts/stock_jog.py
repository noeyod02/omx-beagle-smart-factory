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
    m        switch cartesian <-> joint mode
    1..5     (joint mode) pick joint1..joint5, then w / s move it
    [ / ]    smaller / bigger step (mm, or degrees in joint mode)
    o / c    move the jaws to the open / closed width
    - / =    closed width narrower / wider
    _ / +    open width narrower / wider
    p        save the current point under a name
    l        list the points saved so far
    h        return to the start point
    q        quit

Both widths are shown as the gap between the finger faces in millimetres, and
the jaws move to whichever one you are changing, so each can be sized against
the real part.  If either is changed, quitting prints the gripper block for
config/stock_layout.yaml alongside the recorded points.

Saved points
    Each p asks for a name and writes the point straight into the points file
    (--points-file, ~/.ros/stock_points.yaml by default), so a session can hold
    as many points as you care to teach and none of them are lost if the tool
    is killed rather than quit.  A name that is already in the file overwrites
    that point, which is how a point gets corrected.

    l lists what the file holds, so a name is not reused by accident.  The
    format is scripts/stock_points.py, plain YAML, and one file per station:
    the coordinates are in that arm's own frame and mean nothing at the other.
"""

import argparse
import contextlib
import math
import sys
import termios
import tty

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from omx_kinematics import forward_kinematics, inverse_kinematics, JOINT_NAMES
from stock_joint_jog import JOINT_LIMITS_DEG
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import stock_points
from trajectory_msgs.msg import JointTrajectoryPoint

STEP_SIZES = [0.001, 0.002, 0.005, 0.010, 0.020, 0.050]
DEFAULT_STEP_INDEX = 2

# Joint-mode steps, in degrees. Coarser at the top than the cartesian steps
# because a joint degree near the base moves the gripper several millimetres.
JOINT_STEP_SIZES_DEG = [0.5, 1.0, 2.0, 5.0, 10.0]
DEFAULT_JOINT_STEP_INDEX = 2

# Slow: this is used with hands near the robot.
MOVE_DURATION = 1.0
FIRST_MOVE_DURATION = 4.0

# How long past a move's own duration to keep waiting before calling it stuck.
# Generous, because a controller under load can report late and giving up on a
# move that was going to arrive is worse than waiting another second.
MOVE_GRACE = 4.0
# A gripper close ends by stalling against the part rather than by arriving, so
# it is waited out rather than expected to report.
GRIPPER_TIMEOUT = 5.0
# How long to wait for the controller to say anything at all - accept a goal,
# acknowledge a cancellation. Unrelated to how long a move takes.
ANSWER_TIMEOUT = 5.0

# Gripper joint values, which the finger gap follows as gap_mm = 100*value + 10.
# The limits are the ends of the range the gripper was measured over, 10 mm to
# 90 mm; the step is 2 mm, fine enough to close on a part without crushing it.
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.8
GRIPPER_STEP = 0.02


def gap_mm(joint):
    """Finger gap for a gripper joint value, from the measurement in the layout."""
    return 100.0 * joint + 10.0


def notice(text):
    """Print a message over the status line, clearing what the line held.

    Messages are shorter than the status line they land on, so without the
    erase the tail of a stale position stays on screen next to them - and a
    recorded coordinate sitting beside leftover digits is worth avoiding.
    """
    erase = '\x1b[K' if sys.stdout.isatty() else ''
    print(f'\r  {text}{erase}')


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
        # Kept so the report can stay quiet about widths that were never touched.
        self.gripper_start = (self.gripper_open, self.gripper_closed)

        self.start = (args.x, args.y, args.z)
        self.point = list(self.start)
        self.step_index = DEFAULT_STEP_INDEX
        # Joint mode: nudge one joint at a time, no IK in the way. The last
        # commanded joints are remembered by every move, so switching modes
        # continues from wherever the arm is.
        self.mode = 'cartesian'
        self.joints = None
        self.joint_index = 0
        self.joint_step_index = DEFAULT_JOINT_STEP_INDEX
        self.points_file = args.points_file
        # Names saved this session, in order, for the report at the end.  The
        # file itself holds everything ever saved, this session or not.
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
                notice(f'refused: {axis}={value:.3f} outside [{low}, {high}]')
                return None
        joints = inverse_kinematics(x, y, z, pitch=self.pitch, roll=self.roll)
        if joints is None:
            notice(f'refused: ({x:.3f}, {y:.3f}, {z:.3f}) is out of reach')
        return joints

    def _send_joints(self, joints, duration):
        """Send one joint target and wait for it."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in joints]
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration - int(duration)) * 1e9)
        )
        goal.trajectory.points = [point]
        return self._send(self.arm, goal, timeout=duration + MOVE_GRACE)

    def move_to(self, x, y, z, duration=MOVE_DURATION):
        """Send the arm to a point and wait for it to arrive."""
        joints = self.solve(x, y, z)
        if joints is None:
            return False
        if not self._send_joints(joints, duration):
            return False
        self.point = [x, y, z]
        self.joints = list(joints)
        return True

    def nudge_joint(self, delta_deg, duration=MOVE_DURATION):
        """Move the selected joint alone, then read the pose back through FK.

        The pitch and roll follow the joints here rather than the other way
        round, so after a joint move the recorded angles are whatever the arm
        is actually doing - a point saved in joint mode carries the approach
        angle and wrist roll it was posed at, not the ones the session started
        with.
        """
        name = JOINT_NAMES[self.joint_index]
        target = list(self.joints)
        target[self.joint_index] += math.radians(delta_deg)

        low, high = JOINT_LIMITS_DEG[name]
        value_deg = math.degrees(target[self.joint_index])
        if not low <= value_deg <= high:
            notice(f'refused: {name} at {value_deg:.1f} deg outside [{low}, {high}]')
            return False

        if not self._send_joints(target, duration):
            return False
        self.joints = target
        self.point = list(forward_kinematics(target)[:3])
        self.pitch = -(target[1] + target[2] + target[3])
        self.roll = target[4]
        return True

    def adjust_gripper(self, which, delta):
        """Widen or narrow one of the two gripper widths, and show the result.

        The right widths are found the same way the coordinates are - by trying
        them against the real part - so they are adjustable while jogging
        rather than fixed for the run.  The jaws move to the width being
        changed, because the number is not what tells you whether the part is
        held; the fingers are.
        """
        value = getattr(self, f'gripper_{which}') + delta
        # Rounded so a walk up and back down lands on the value it started at,
        # rather than a hair off it that the report would then call a change.
        value = round(min(GRIPPER_MAX, max(GRIPPER_MIN, value)), 3)
        setattr(self, f'gripper_{which}', value)
        self.set_gripper(value)

    def set_gripper(self, position):
        """Open or close the gripper."""
        if self.gripper is None:
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.gripper_effort
        # The jaws stall against the part on purpose, so a close that never
        # reports arrival is the normal case rather than a fault; it only has
        # to be waited out, not diagnosed.
        self._send(self.gripper, goal, timeout=GRIPPER_TIMEOUT)

    def _send(self, client, goal, timeout):
        """Send a goal and wait for it, giving up rather than waiting forever.

        Every wait here is bounded.  A move that cannot finish - torque off,
        so the arm never reaches the trajectory it was given - otherwise parks
        the jogger inside this call, and since the key loop is one level up,
        the whole tool goes silent: keys do nothing, nothing is printed, and it
        reads as the keyboard having failed rather than the arm.
        """
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=ANSWER_TIMEOUT)
        if not send.done():
            notice('the controller did not answer. Is it still running?')
            return False

        handle = send.result()
        if handle is None or not handle.accepted:
            notice('goal rejected by the controller')
            return False

        result = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        except KeyboardInterrupt:
            # Ctrl-C during a move means stop the arm, not just stop the
            # program. The trajectory is already with the controller and runs
            # to its end whatever this process does, so it has to be called
            # off; quitting without doing so leaves the arm moving after the
            # operator believes they have stopped it.
            notice('cancelling the move')
            self._cancel(handle)
            raise

        if not result.done():
            notice(f'the move did not finish within {timeout:.0f} s - cancelling.')
            notice('The commonest cause is the arm having no torque: check with')
            notice('  ros2 topic echo <ns>/dynamixel_hardware_interface/dxl_state')
            self._cancel(handle)
            return False
        return True

    def _cancel(self, handle):
        """Call off a goal, without waiting forever for the acknowledgement."""
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel, timeout_sec=ANSWER_TIMEOUT)

    def status_line(self):
        x, y, z = self.point
        if self.mode == 'joint':
            angles = ' '.join(
                f'{"*" if i == self.joint_index else " "}{name[-1]}:'
                f'{math.degrees(v):6.1f}'
                for i, (name, v) in enumerate(zip(JOINT_NAMES, self.joints))
            )
            return (
                f'\r  [joint]{angles}  '
                f'({x:+.3f}, {y:+.3f}, {z:+.3f})  '
                f'pitch {math.degrees(self.pitch):5.1f}   '
                f'step {JOINT_STEP_SIZES_DEG[self.joint_step_index]} deg   '
                f'grip {gap_mm(self.gripper_open):.0f}/'
                f'{gap_mm(self.gripper_closed):.0f} mm      '
            )
        return (
            f'\r  x {x:+.3f}  y {y:+.3f}  z {z:+.3f}   '
            f'step {self.step * 1000:.0f} mm   '
            f'grip {gap_mm(self.gripper_open):.0f}/{gap_mm(self.gripper_closed):.0f} mm   '
            f'recorded {len(self.recorded)}      '
        )

    def record(self):
        """Save the current point under a name the operator types.

        Written to the file as soon as it is named, rather than kept until the
        session ends: teaching a set of points takes a while, the arm is being
        driven by hand throughout, and a session that ends by having the arm
        switched off should still leave every point taught before that moment.
        """
        x, y, z = self.point
        suggestion = f'point{len(stock_points.load_points(self.points_file)[0]) + 1}'
        print()
        try:
            name = read_line(f'  name for ({x:+.3f}, {y:+.3f}, {z:+.3f}) '
                             f'[{suggestion}]: ') or suggestion
        except KeyboardInterrupt:
            # Ctrl-C at the prompt means drop this point, not leave the tool -
            # the arm is parked over the spot and quitting would waste it.
            notice('not saved')
            return

        reason = stock_points.check_name(name)
        if reason is not None:
            notice(f'not saved: {reason}')
            return

        point = stock_points.make_point(
            name, x, y, z, math.degrees(self.pitch), math.degrees(self.roll)
        )
        try:
            outcome = stock_points.save_point(self.points_file, point)
        except OSError as error:
            notice(f'could not write {self.points_file}: {error}')
            return

        if name not in self.recorded:
            self.recorded.append(name)
        notice(f'{outcome} {name}: x {x:+.3f}  y {y:+.3f}  z {z:+.3f}')

    def list_points(self):
        """Show what the file holds, so a name is not reused by accident."""
        points, problems = stock_points.load_points(self.points_file)
        print(f'\n\n  {self.points_file}\n')
        if not points:
            print('  nothing saved yet\n')
        for point in points:
            here = '  <- here' if [point['x'], point['y'], point['z']] == [
                round(value, 4) for value in self.point
            ] else ''
            print(f"  {point['name']:<20} x {point['x']:+.3f}  y {point['y']:+.3f}  "
                  f"z {point['z']:+.3f}{here}")
        for problem in problems:
            print(f'  ! {problem}')
        print()

    def report(self):
        """Say where the points went, and print anything not saved for them."""
        widths_changed = (self.gripper_open, self.gripper_closed) != self.gripper_start
        if not self.recorded and not widths_changed:
            return

        if self.recorded:
            print(f'\n\nSaved to {self.points_file}, this session:\n')
            for index, name in enumerate(self.recorded, start=1):
                print(f'  {index}. {name}')
            print(
                '\nA point that belongs in the layout - a bin place, the carrier\n'
                'transfer point, a warehouse pick_point - still has to be copied\n'
                'into the station layout by hand, then checked with:\n'
                '  python3 stock_reach_check.py ../config/stock_layout_b.yaml\n'
            )

        if widths_changed:
            print('\nGripper widths, as the gripper block of the same file:\n')
            print('gripper:')
            print(f'  open_position: {self.gripper_open:.2f}    '
                  f'# {gap_mm(self.gripper_open):.0f} mm')
            print(f'  closed_position: {self.gripper_closed:.2f}  '
                  f'# {gap_mm(self.gripper_closed):.0f} mm')
            print(f'  max_effort: {self.gripper_effort:.1f}\n')


# The terminal settings from before single-key mode was entered, kept so a
# point can be named with echo and backspace working for the moment that takes.
_LINE_MODE = None


@contextlib.contextmanager
def keys_without_enter():
    """Take keys one at a time for the whole session, and keep Ctrl-C working.

    Held rather than taken per keypress.  Each move blocks for a second or
    more, and a terminal left in line mode for that second echoes whatever is
    typed into the middle of the status line and then holds it back waiting for
    Enter, so a key pressed while the arm is still moving is both visible and
    ignored - which reads as the jogger dropping the key.

    cbreak rather than raw, which differ in exactly one way that matters here:
    raw mode also stops Ctrl-C becoming a signal.  Holding raw across a move
    would therefore take away the operator's way of stopping a moving arm,
    since the loop is inside the action call and not reading keys - the one
    moment the stop is most likely to be wanted.  cbreak turns off line
    buffering and echo and leaves the signal alone.

    Does nothing when stdin is not a terminal, so a canned key sequence can
    still be piped in to rehearse a sequence against mock hardware.
    """
    global _LINE_MODE
    if not sys.stdin.isatty():
        yield
        return

    handle = sys.stdin.fileno()
    saved = termios.tcgetattr(handle)
    _LINE_MODE = saved
    try:
        tty.setcbreak(handle)
        yield
    finally:
        termios.tcsetattr(handle, termios.TCSADRAIN, saved)
        _LINE_MODE = None


def read_key():
    """Read one keypress, assuming keys_without_enter() is holding the terminal."""
    return sys.stdin.read(1) or 'q'


def read_line(prompt):
    """Ask for a line of text, then hand the terminal back to single keys.

    Naming a point is the one moment the operator types more than a character,
    and single-key mode makes that unusable: nothing typed is echoed and
    backspace arrives as a character of the name rather than as a correction.
    So line mode is put back for the length of the prompt and taken away again
    afterwards, leaving the rest of the session as it was.
    """
    if _LINE_MODE is None:
        # Either stdin is a pipe rehearsing a sequence, or the terminal was
        # never taken; input() already behaves in both cases.
        try:
            return input(prompt).strip()
        except EOFError:
            return ''

    handle = sys.stdin.fileno()
    single_key = termios.tcgetattr(handle)
    termios.tcsetattr(handle, termios.TCSADRAIN, _LINE_MODE)
    try:
        return input(prompt).strip()
    except EOFError:
        return ''
    finally:
        termios.tcsetattr(handle, termios.TCSADRAIN, single_key)


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
    parser.add_argument('--points-file', default=stock_points.DEFAULT_POINTS_FILE,
                        help='where p saves points. One file per station: the '
                             "coordinates are in that arm's own frame")
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
    existing, problems = stock_points.load_points(args.points_file)
    print(f'Points file: {args.points_file} ({len(existing)} already saved)')
    for problem in problems:
        print(f'  ! {problem}')
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
    # Same two physical keys for both widths, shifted for the open one: the
    # closed width is the one that decides whether the part is held, so it gets
    # the unshifted pair.
    widths = {
        '-': ('closed', -1), '=': ('closed', +1),
        '_': ('open', -1), '+': ('open', +1),
    }

    try:
        with keys_without_enter():
            while True:
                print(node.status_line(), end='', flush=True)
                key = read_key().lower()

                if key in ('q', '\x03'):
                    break
                elif key == 'm':
                    node.mode = 'joint' if node.mode == 'cartesian' else 'cartesian'
                    notice(f'{node.mode} mode')
                elif node.mode == 'joint' and key in '12345':
                    node.joint_index = int(key) - 1
                elif node.mode == 'joint' and key in ('w', 's'):
                    sign = +1 if key == 'w' else -1
                    node.nudge_joint(
                        sign * JOINT_STEP_SIZES_DEG[node.joint_step_index]
                    )
                elif node.mode == 'joint' and key in moves:
                    notice('joint mode: w/s move the joint picked with 1..5; m returns')
                elif key in moves:
                    axis, sign = moves[key]
                    target = list(node.point)
                    target[axis] += sign * node.step
                    node.move_to(*target)
                elif key == '[':
                    if node.mode == 'joint':
                        node.joint_step_index = max(0, node.joint_step_index - 1)
                    else:
                        node.step_index = max(0, node.step_index - 1)
                elif key == ']':
                    if node.mode == 'joint':
                        node.joint_step_index = min(
                            len(JOINT_STEP_SIZES_DEG) - 1, node.joint_step_index + 1
                        )
                    else:
                        node.step_index = min(len(STEP_SIZES) - 1, node.step_index + 1)
                elif key == 'o':
                    node.set_gripper(node.gripper_open)
                elif key == 'c':
                    node.set_gripper(node.gripper_closed)
                elif key in widths:
                    which, sign = widths[key]
                    node.adjust_gripper(which, sign * GRIPPER_STEP)
                elif key == 'p':
                    node.record()
                elif key == 'l':
                    node.list_points()
                elif key == 'h':
                    node.move_to(*node.start, duration=FIRST_MOVE_DURATION)
                else:
                    # Shown rather than ignored: a key that arrives as something
                    # else - an input method translating letters, a terminal
                    # sending an escape sequence - looks exactly like a key the
                    # jogger chose not to act on, and the two need different
                    # fixes.
                    notice(f'{key!r} is not a key here; see the list above')
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
