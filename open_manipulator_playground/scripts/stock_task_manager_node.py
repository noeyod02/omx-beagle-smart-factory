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

"""Refill empty stock bins by fetching parts from a warehouse.

Listens for a bin that has been reported empty, then runs a pick and place job:
warehouse -> the empty bin -> home.  Every waypoint is solved with the
closed-form OMX-F kinematics in omx_kinematics.py and executed through the
joint trajectory controller's action interface, so each motion is known to have
finished before the next one starts.

The node can be driven without any camera:

    ros2 topic pub --once /stock/refill_request std_msgs/String "{data: bin2}"
"""

import json
import math
import os

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from omx_kinematics import inverse_kinematics, JOINT_NAMES
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

GRIPPER_JOINT = 'gripper_joint_1'

STATE_WAITING = 'waiting_for_controller'
STATE_IDLE = 'idle'
STATE_BUSY = 'busy'
STATE_COOLDOWN = 'cooldown'
STATE_BLOCKED = 'blocked'


def _to_duration(seconds):
    """Convert float seconds to a builtin_interfaces Duration."""
    return Duration(
        sec=int(seconds),
        nanosec=int((seconds - int(seconds)) * 1e9),
    )


class Waypoint:
    """A cartesian target together with the joint solution that reaches it."""

    def __init__(self, name, x, y, z, joints, duration):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.joints = joints
        self.duration = duration
        # Set for grasp points: the waypoint directly above this one.
        self.hover = None

    def __repr__(self):
        return f'{self.name}({self.x:.3f}, {self.y:.3f}, {self.z:.3f})'


class StockTaskManager(Node):
    """State machine that refills empty bins from the warehouse."""

    def __init__(self):
        super().__init__('stock_task_manager')

        self.declare_parameter('layout_file', '')
        self.declare_parameter('arm_action', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_mode', 'action')
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('status_topic', '/stock/status')
        self.declare_parameter('request_topic', '/stock/refill_request')
        self.declare_parameter('state_topic', '/stock/task_state')
        self.declare_parameter('auto_refill', True)
        self.declare_parameter('cooldown_sec', 5.0)
        self.declare_parameter('controller_wait_sec', 30.0)

        layout_file = self.get_parameter('layout_file').value
        if not layout_file or not os.path.exists(layout_file):
            raise RuntimeError(f'layout_file not found: {layout_file!r}')
        self.layout = self._load_layout(layout_file)

        self.gripper_mode = self.get_parameter('gripper_mode').value
        if self.gripper_mode not in ('action', 'joint'):
            raise RuntimeError(
                f"gripper_mode must be 'action' or 'joint', got {self.gripper_mode!r}"
            )
        self.auto_refill = bool(self.get_parameter('auto_refill').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        self.state = STATE_WAITING
        self.steps = []
        self.step_index = 0
        self.current_job = None
        self.cooldown_until = 0.0
        self.next_pick_index = 0

        self.arm_client = ActionClient(
            self, FollowJointTrajectory, self.get_parameter('arm_action').value
        )
        self.gripper_client = None
        if self.gripper_mode == 'action':
            self.gripper_client = ActionClient(
                self, GripperCommand, self.get_parameter('gripper_action').value
            )

        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10
        )
        self.create_subscription(
            String, self.get_parameter('status_topic').value, self._on_status, 10
        )
        self.create_subscription(
            String, self.get_parameter('request_topic').value, self._on_request, 10
        )

        self._report_layout()
        self.create_timer(0.5, self._publish_state)
        self._startup_timer = self.create_timer(1.0, self._wait_for_controllers)
        self._startup_elapsed = 0.0

        self.get_logger().info('Stock task manager started')

    # ------------------------------------------------------------------ setup

    def _load_layout(self, path):
        """Read the layout file and pre-solve every waypoint it describes."""
        with open(path) as handle:
            layout = yaml.safe_load(handle)

        workspace = layout['workspace']
        self.hover_height = float(workspace['hover_height'])
        self.pitch = math.radians(float(workspace['approach_pitch_deg']))
        self.roll = math.radians(float(workspace['approach_roll_deg']))
        self.limits = workspace['limits']
        self.durations = workspace['durations']

        gripper = layout['gripper']
        self.gripper_open = float(gripper['open_position'])
        self.gripper_closed = float(gripper['closed_position'])
        self.gripper_effort = float(gripper['max_effort'])

        home = workspace['home']
        self.home = self._solve('home', home['x'], home['y'], home['z'], 'transit')
        transit = workspace['transit']
        self.transit = self._solve(
            'transit', transit['x'], transit['y'], transit['z'], 'transit'
        )

        self.pick_points = [
            self._solve_target(f'warehouse[{i}]', p)
            for i, p in enumerate(layout['warehouse']['pick_points'])
        ]
        if not self.pick_points:
            raise RuntimeError('layout has no warehouse pick points')

        self.bins = {}
        self.bin_order = []
        for entry in layout['bins']:
            bin_id = entry['id']
            self.bins[bin_id] = self._solve_target(f'{bin_id}.place', entry['place'])
            self.bin_order.append(bin_id)
        if not self.bins:
            raise RuntimeError('layout has no bins')

        return layout

    def _solve(self, name, x, y, z, duration_key):
        """Turn a cartesian point into a Waypoint, failing loudly if unreachable."""
        for axis, value in (('x', x), ('y', y), ('z', z)):
            low, high = self.limits[axis]
            if not low <= value <= high:
                raise RuntimeError(
                    f'{name}: {axis}={value:.3f} is outside the allowed '
                    f'range [{low}, {high}]'
                )

        joints = inverse_kinematics(x, y, z, pitch=self.pitch, roll=self.roll)
        if joints is None:
            raise RuntimeError(
                f"{name}: ({x:.3f}, {y:.3f}, {z:.3f}) is out of the arm's reach"
            )
        return Waypoint(name, x, y, z, joints, float(self.durations[duration_key]))

    def _solve_target(self, name, point):
        """Solve a grasp point together with the hover above it.

        Both are resolved now rather than when a job starts, so an unreachable
        layout is caught at startup instead of stranding the arm mid-motion.
        """
        target = self._solve(name, point['x'], point['y'], point['z'], 'descend')
        target.hover = self._solve(
            f'{name}.hover',
            point['x'],
            point['y'],
            point['z'] + self.hover_height,
            'approach',
        )
        return target

    def _report_layout(self):
        """Log the resolved layout so the operator can sanity check it."""
        self.get_logger().info(f'home        {self.home}')
        self.get_logger().info(f'warehouse   {len(self.pick_points)} pick points')
        for bin_id in self.bin_order:
            self.get_logger().info(f'  {bin_id:8s} place at {self.bins[bin_id]}')

    def _wait_for_controllers(self):
        """Hold in the waiting state until the controller actions show up."""
        limit = float(self.get_parameter('controller_wait_sec').value)
        ready = self.arm_client.server_is_ready()
        if self.gripper_client is not None:
            ready = ready and self.gripper_client.server_is_ready()

        if ready:
            self._startup_timer.cancel()
            self.state = STATE_IDLE
            self.get_logger().info('Controllers ready - accepting refill requests')
            return

        self._startup_elapsed += 1.0
        if self._startup_elapsed >= limit:
            self._startup_timer.cancel()
            self.state = STATE_BLOCKED
            self.get_logger().error(
                f'No controller action server after {limit:.0f}s. Is the arm bringup '
                f'running, and does arm_action match the controller name?'
            )
        elif self._startup_elapsed % 5.0 < 1e-6:
            self.get_logger().warn('Waiting for controller action servers...')

    # --------------------------------------------------------------- requests

    def _on_status(self, msg):
        """React to the stock monitor's view of the bins."""
        if not self.auto_refill:
            return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed stock status')
            return

        for entry in status.get('bins', []):
            if entry.get('state') == 'empty' and entry.get('stable'):
                self._start_job(entry['id'])
                return

    def _on_request(self, msg):
        """Handle a refill asked for by hand."""
        self._start_job(msg.data.strip())

    def _start_job(self, bin_id):
        """Begin a refill for ``bin_id`` if the arm is free to take it."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state == STATE_COOLDOWN and now >= self.cooldown_until:
            self.state = STATE_IDLE

        if self.state != STATE_IDLE:
            self.get_logger().debug(f'Ignoring refill for {bin_id}: state is {self.state}')
            return
        if bin_id not in self.bins:
            self.get_logger().warn(
                f'Unknown bin {bin_id!r}; known bins are {sorted(self.bins)}'
            )
            return
        if self.next_pick_index >= len(self.pick_points):
            self.state = STATE_BLOCKED
            self.get_logger().error(
                'Warehouse is empty - refill it and restart the node to continue'
            )
            return

        pick = self.pick_points[self.next_pick_index]
        place = self.bins[bin_id]
        self.next_pick_index += 1

        self.current_job = bin_id
        self.state = STATE_BUSY
        self.steps = self._build_steps(pick, place)
        self.step_index = 0
        self.get_logger().info(
            f'Refilling {bin_id}: {pick.name} -> {place.name} '
            f'({len(self.steps)} steps)'
        )
        self._run_next_step()

    def _build_steps(self, pick, place):
        """Lay out the full pick and place sequence as a list of steps."""
        return [
            ('gripper', self.gripper_open),
            ('arm', self.transit),
            ('arm', pick.hover),
            ('arm', pick),
            ('gripper', self.gripper_closed),
            ('arm', pick.hover),
            ('arm', self.transit),
            ('arm', place.hover),
            ('arm', place),
            ('gripper', self.gripper_open),
            ('arm', place.hover),
            ('arm', self.home),
        ]

    # -------------------------------------------------------------- execution

    def _run_next_step(self):
        """Send the next step, or finish the job when the list runs out."""
        if self.step_index >= len(self.steps):
            self._finish_job()
            return

        kind, payload = self.steps[self.step_index]
        self.get_logger().info(
            f'  step {self.step_index + 1}/{len(self.steps)}: {kind} {payload}'
        )
        if kind == 'arm':
            self._send_arm_goal(payload)
        else:
            self._send_gripper_goal(payload)

    def _send_arm_goal(self, waypoint):
        """Command the arm to a single waypoint."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in waypoint.joints]
        point.time_from_start = _to_duration(waypoint.duration)
        goal.trajectory.points = [point]
        self._send_goal(self.arm_client, goal)

    def _send_gripper_goal(self, position):
        """Command the gripper, through whichever interface the bringup exposes."""
        if self.gripper_mode == 'action':
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = self.gripper_effort
            self._send_goal(self.gripper_client, goal)
            return

        # The AI follower bringup folds the gripper into the arm controller, so
        # the same action moves it - partial joint goals must be enabled there.
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = _to_duration(float(self.durations['gripper']))
        goal.trajectory.points = [point]
        self._send_goal(self.arm_client, goal)

    def _send_goal(self, client, goal):
        """Send ``goal`` and wire its result back into the step sequence."""
        future = client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self._abort('controller rejected the goal')
            return
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        result = future.result()
        # Status 4 is SUCCEEDED in action_msgs/GoalStatus.
        if result.status != 4:
            self._abort(f'step failed with status {result.status}')
            return
        self.step_index += 1
        self._run_next_step()

    def _abort(self, reason):
        """Give up on the current job and park the node."""
        self.get_logger().error(f'Refill of {self.current_job} aborted: {reason}')
        self.steps = []
        self.step_index = 0
        self.current_job = None
        self.state = STATE_BLOCKED

    def _finish_job(self):
        """Wrap up a completed job and start the cooldown."""
        self.get_logger().info(f'Refill of {self.current_job} complete')
        self.steps = []
        self.step_index = 0
        self.current_job = None
        self.cooldown_until = (
            self.get_clock().now().nanoseconds / 1e9 + self.cooldown_sec
        )
        self.state = STATE_COOLDOWN

    def _publish_state(self):
        """Broadcast what the arm is doing, so the monitor knows when to look."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state == STATE_COOLDOWN and now >= self.cooldown_until:
            self.state = STATE_IDLE

        self.state_pub.publish(String(data=json.dumps({
            'state': self.state,
            'job': self.current_job,
            'parts_left': max(0, len(self.pick_points) - self.next_pick_index),
            # The camera only sees the bins clearly while the arm is parked.
            'vision_clear': self.state in (STATE_IDLE, STATE_BLOCKED),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = StockTaskManager()
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
