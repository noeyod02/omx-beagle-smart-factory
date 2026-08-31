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

"""Move parts between named places with one arm.

Every job is a pick and place between two places this arm's layout knows about,
run as a step sequence: each motion is known to have finished before the next
one starts.  Waypoints are solved with the closed-form OMX-F kinematics in
omx_kinematics.py and executed through the joint trajectory controller's action
interface.  The places are

``warehouse``
    The stock of spare parts, consumed one point at a time.  Only station A has
    one.
``carrier``
    The Beagle's tray.  Both stations reach into the same tray, from their own
    sides and therefore at their own coordinates.
``<bin id>``
    A stock bin, the final destination of a part.  Only station B has these.

One instance runs per arm, under that arm's namespace, reading that arm's
layout file.  Ask for a transfer with::

    ros2 topic pub --once /station_a/stock/transfer std_msgs/String \\
        "{data: '{\\"from\\": \\"warehouse\\", \\"to\\": \\"carrier\\"}'}"

For a single-arm cell the whole job is one transfer, so the older shortcut
still works - a bin name on ``refill_request`` means warehouse to that bin::

    ros2 topic pub --once /stock/refill_request std_msgs/String "{data: bin2}"

In a relay that shortcut is what the coordinator replaces, so launch it with
``request_topic:=''`` and ``auto_refill:=false``: the arms then move only when
stock_relay_node.py says the carrier is really there to reach into.
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
from std_msgs.msg import Empty, String
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
        # Optional higher pull-away above a place, used after the release
        # instead of the hover - see _solve_target.
        self.retreat = None

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
        self.declare_parameter('transfer_topic', '/stock/transfer')
        self.declare_parameter('restock_topic', '/stock/restock')
        # Whether the warehouse pick points start over once they run out.
        #
        # Off, the count is a one-way tally: every job spends a taught point
        # and the station refuses jobs once they are gone, however full the
        # box actually is. That is the right answer when nothing else can see
        # the box. When a camera watches it and the approval refuses on what
        # the camera reads, this tally is a second, blind gate - and it is the
        # one that stops a cell whose box was refilled between jobs.
        self.declare_parameter('warehouse_cycles', False)
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
        self.warehouse_cycles = bool(self.get_parameter('warehouse_cycles').value)
        # Identifies the running job and reports how the last one ended, so a
        # coordinator can tell its own job's outcome from somebody else's.
        self.job_id = 0
        self.last_job = None

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
        # Left empty in a relay: there the coordinator owns the sequence, and a
        # bare bin name has no meaning for an arm that reaches only one end of
        # the job.
        request_topic = self.get_parameter('request_topic').value
        if request_topic:
            self.create_subscription(
                String, request_topic, self._on_request, 10
            )
        self.create_subscription(
            String, self.get_parameter('transfer_topic').value, self._on_transfer, 10
        )
        # The warehouse count is decided once, when this node reads the layout,
        # and only ever falls. Restocking the box by hand used to mean
        # restarting the node - which in a dashboard-driven cell means an
        # approved job dying on 'the warehouse is empty' with a full box in
        # front of the camera. This puts the count back without a restart.
        self.create_subscription(
            Empty, self.get_parameter('restock_topic').value, self._on_restock, 10
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
        self.home = self._solve(
            'home', home['x'], home['y'], home['z'], 'transit', *self._angles(home)
        )
        transit = workspace['transit']
        self.transit = self._solve(
            'transit', transit['x'], transit['y'], transit['z'], 'transit',
            *self._angles(transit)
        )
        # Whether the loaded carry passes through the transit point. Stations
        # whose transit sits opposite the place direction (station C: transit
        # at y=0, bins at -y) waste a there-and-back swing with the part in
        # the jaws; with pick retreats and place hovers already level, the
        # carry can go straight. Empty-handed legs always use the transit.
        self.carry_via_transit = bool(workspace.get('carry_via_transit', True))

        self.pick_points = [
            self._solve_target(f'warehouse[{i}]', p)
            for i, p in enumerate((layout.get('warehouse') or {}).get('pick_points') or [])
        ]

        self.bins = {}
        self.bin_order = []
        for entry in layout.get('bins') or []:
            bin_id = entry['id']
            self.bins[bin_id] = self._solve_target(f'{bin_id}.place', entry['place'])
            self.bin_order.append(bin_id)

        # Where this arm meets the Beagle's tray. Absent in a single-arm cell,
        # where nothing is ever handed to a carrier.
        carrier = layout.get('carrier') or {}
        self.carrier = (
            self._solve_target('carrier', carrier['transfer'])
            if 'transfer' in carrier else None
        )

        # A station that reaches nowhere cannot be given a job, and finding
        # that out now beats finding it out when one is asked for.
        if not (self.pick_points or self.bins or self.carrier):
            raise RuntimeError(
                'layout defines no places to move between: expected at least '
                'one of warehouse.pick_points, bins or carrier.transfer'
            )

        return layout

    def _angles(self, point):
        """The approach angles a point carries, or None to use the defaults.

        A point taught in the jog's joint mode records the pitch and roll it
        was posed at; replaying it at the workspace default instead puts the
        wrist in a pose nobody ever saw at that spot.
        """
        pitch = (
            math.radians(float(point['pitch_deg']))
            if 'pitch_deg' in point else None
        )
        roll = (
            math.radians(float(point['roll_deg']))
            if 'roll_deg' in point else None
        )
        return pitch, roll

    def _solve(self, name, x, y, z, duration_key, pitch=None, roll=None):
        """Turn a cartesian point into a Waypoint, failing loudly if unreachable."""
        for axis, value in (('x', x), ('y', y), ('z', z)):
            low, high = self.limits[axis]
            if not low <= value <= high:
                raise RuntimeError(
                    f'{name}: {axis}={value:.3f} is outside the allowed '
                    f'range [{low}, {high}]'
                )

        joints = inverse_kinematics(
            x, y, z,
            pitch=self.pitch if pitch is None else pitch,
            roll=self.roll if roll is None else roll,
        )
        if joints is None:
            raise RuntimeError(
                f"{name}: ({x:.3f}, {y:.3f}, {z:.3f}) is out of the arm's reach"
            )
        return Waypoint(name, x, y, z, joints, float(self.durations[duration_key]))

    def _solve_target(self, name, point):
        """Solve a grasp point together with the hover above it.

        Both are resolved now rather than when a job starts, so an unreachable
        layout is caught at startup instead of stranding the arm mid-motion.
        The hover shares the point's angles: the descent is a straight drop
        along the approach the point was taught with.  A point may carry its
        own ``hover`` height - a cell holding parts needs the carry to clear
        what is already standing in it, while the carrier's hover has to stay
        low because reach up there is scarce.
        """
        pitch, roll = self._angles(point)
        hover = float(point.get('hover', self.hover_height))
        target = self._solve(
            name, point['x'], point['y'], point['z'], 'descend', pitch, roll
        )
        target.hover = self._solve(
            f'{name}.hover',
            point['x'],
            point['y'],
            point['z'] + hover,
            'approach',
            pitch,
            roll,
        )
        # An approach pose tilted past vertical caps its hover low (station
        # A's tray pose has 25 mm of headroom). A 'retreat' pulls away higher
        # at its own angles - typically back at -90 - once the part is down
        # and the pose no longer has to match the taught approach.
        retreat = point.get('retreat')
        if retreat is not None:
            r_pitch, r_roll = self._angles({**point, **retreat})
            target.retreat = self._solve(
                f'{name}.retreat',
                point['x'],
                point['y'],
                float(retreat['z']),
                'approach',
                r_pitch,
                r_roll,
            )
        return target

    def _report_layout(self):
        """Log the resolved layout so the operator can sanity check it."""
        self.get_logger().info(f'home        {self.home}')
        self.get_logger().info(f'warehouse   {len(self.pick_points)} pick points')
        if self.carrier is not None:
            self.get_logger().info(f'carrier     tray at {self.carrier}')
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
                self._start_transfer('warehouse', entry['id'])
                return

    def _on_request(self, msg):
        """Handle a refill asked for by hand: warehouse to the named bin."""
        self._start_transfer('warehouse', msg.data.strip())

    def _on_restock(self, msg):
        """Declare the warehouse full again: the box has been refilled by hand.

        Only the count is reset. The pick points themselves are unchanged, so
        this says 'there is a part at every taught spot again' - which is what
        refilling the box means, and is the same claim the node makes for
        itself at startup.
        """
        del msg  # std_msgs/Empty carries nothing
        if not self.pick_points:
            self.get_logger().warn('restock ignored: this station has no warehouse')
            return
        if self.state == STATE_BUSY:
            # Mid-job the index is pointing at the part being carried right
            # now; moving it would hand the next job the same point twice.
            self.get_logger().warn('restock ignored: the arm is mid-job')
            return
        was = max(0, len(self.pick_points) - self.next_pick_index)
        self.next_pick_index = 0
        self.get_logger().info(
            f'warehouse restocked: {was} -> {len(self.pick_points)} parts'
        )

    def _on_transfer(self, msg):
        """Handle a transfer between two named places, as a relay asks for."""
        try:
            request = json.loads(msg.data)
            source = str(request['from'])
            destination = str(request['to'])
        except (json.JSONDecodeError, KeyError, TypeError):
            self.get_logger().warn(
                f'Ignoring malformed transfer {msg.data!r}; expected '
                f'{{"from": "<place>", "to": "<place>"}}'
            )
            return
        self._start_transfer(source, destination, request.get('id'))

    def _resolve(self, name):
        """Find the waypoint a place name stands for, at this station."""
        if name == 'warehouse':
            if not self.pick_points:
                raise LookupError('this station has no warehouse')
            if self.next_pick_index >= len(self.pick_points):
                if not self.warehouse_cycles:
                    raise LookupError('the warehouse is empty')
                # The count only ever falls, and it is blind: it says the box
                # is empty because this node handed out every taught point
                # once, not because anyone looked. With a camera watching the
                # box - and refusing the job when it reads empty - that count
                # is the wrong thing to refuse on, and refusing on it stops a
                # cell whose box was refilled between jobs.
                self.next_pick_index = 0
                self.get_logger().info('warehouse points exhausted - starting over')
            return self.pick_points[self.next_pick_index]
        if name == 'carrier':
            if self.carrier is None:
                raise LookupError('this station has no carrier point')
            return self.carrier
        if name in self.bins:
            return self.bins[name]
        raise LookupError(
            f'unknown place {name!r}; this station knows '
            f'{sorted(self._place_names())}'
        )

    def _place_names(self):
        names = list(self.bin_order)
        if self.pick_points:
            names.append('warehouse')
        if self.carrier is not None:
            names.append('carrier')
        return names

    def _start_transfer(self, source, destination, job_id=None):
        """Move a part from one named place to another, if the arm is free."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state == STATE_COOLDOWN and now >= self.cooldown_until:
            self.state = STATE_IDLE

        label = f'{source} -> {destination}'
        if self.state != STATE_IDLE:
            self.get_logger().warn(
                f'Ignoring {label}: state is {self.state}'
            )
            self._record_result(job_id, label, 'rejected', f'state is {self.state}')
            return

        try:
            pick = self._resolve(source)
            place = self._resolve(destination)
        except LookupError as error:
            # An empty warehouse is a cell that needs restocking, not a typo:
            # it stops the station rather than just failing this one job.
            if str(error) == 'the warehouse is empty':
                self.state = STATE_BLOCKED
                self.get_logger().error(
                    'Warehouse is empty - refill it and restart the node to continue'
                )
            else:
                self.get_logger().warn(f'Cannot run {label}: {error}')
            self._record_result(job_id, label, 'failed', str(error))
            return

        if source == 'warehouse':
            self.next_pick_index += 1

        self.job_id = job_id if job_id is not None else self.job_id + 1
        self.current_job = label
        self.state = STATE_BUSY
        self.steps = self._build_steps(pick, place)
        self.step_index = 0
        self.get_logger().info(
            f'Job {self.job_id}: {pick.name} -> {place.name} '
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
            # A retreat on the pick side lifts the part clear before the
            # swing to transit. The hover alone can be pinned low by the
            # approach pose's IK ceiling (station C's carrier: 20 mm), and
            # swinging from that height dragged the part across the tray,
            # ploughing through whatever else stood on it (2026-08-27).
            ('arm', pick.retreat or pick.hover),
            *([('arm', self.transit)] if self.carry_via_transit else []),
            ('arm', place.hover),
            ('arm', place),
            ('gripper', self.gripper_open),
            ('arm', place.retreat or place.hover),
            # Through the transit on the way home too: a hover is only as
            # high as its pose's IK allows, and swinging straight home from
            # a low one clipped the part just placed (station A, 2026-08-20).
            ('arm', self.transit),
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
        self.get_logger().error(f'Job {self.job_id} ({self.current_job}) aborted: {reason}')
        self._record_result(self.job_id, self.current_job, 'failed', reason)
        self.steps = []
        self.step_index = 0
        self.current_job = None
        self.state = STATE_BLOCKED

    def _finish_job(self):
        """Wrap up a completed job and start the cooldown."""
        self.get_logger().info(f'Job {self.job_id} ({self.current_job}) complete')
        self._record_result(self.job_id, self.current_job, 'ok', None)
        self.steps = []
        self.step_index = 0
        self.current_job = None
        self.cooldown_until = (
            self.get_clock().now().nanoseconds / 1e9 + self.cooldown_sec
        )
        self.state = STATE_COOLDOWN

    def _record_result(self, job_id, label, result, error):
        """Publish how a job ended, immediately rather than at the next tick.

        A relay waits on this to decide whether the next leg may start, so it
        carries the id it was asked with: a coordinator can then tell its own
        job's outcome from one somebody else set off by hand.
        """
        self.last_job = {
            'id': job_id,
            'job': label,
            'result': result,
            'error': error,
        }
        self._publish_state()

    def _publish_state(self):
        """Broadcast what the arm is doing, so the monitor knows when to look."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state == STATE_COOLDOWN and now >= self.cooldown_until:
            self.state = STATE_IDLE

        self.state_pub.publish(String(data=json.dumps({
            'state': self.state,
            'job': self.current_job,
            'job_id': self.job_id,
            'last_job': self.last_job,
            'places': self._place_names(),
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
