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

"""Run the whole relay: warehouse arm, carrier, destination arm.

Each of the three machines already knows how to do its own part and nothing
else.  This node is what makes them one job:

    1. station A's arm lifts a part out of the warehouse onto the Beagle's tray
    2. the Beagle drives from bay A to bay B
    3. station B's arm lifts that part off the tray into the empty bin
    4. the Beagle drives home to bay A, ready for the next job

Ask for one with the name of the bin to fill::

    ros2 topic pub --once /stock/refill_request std_msgs/String "{data: bin2}"

Nothing here reaches into the tray on trust.  Before either arm is told to
move, the Beagle must report ``ready_for_arm``, which it only does while parked
at a known station: a carrier that stopped short of the bay leaves the station
name empty, and the leg simply does not start.  The same goes the other way -
the Beagle is not sent anywhere until the arm that was working on it says it
has finished and gone home.

Any step that fails or runs out of time stops the relay in the ``failed``
state.  It stays there, moving nothing, until somebody has looked at the cell
and cleared it::

    ros2 topic pub --once /stock/relay_reset std_msgs/Empty {}

Run the two task managers with ``auto_refill:=false`` and ``request_topic:=''``
so that this node is the only thing giving them work.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String

RELAY_IDLE = 'idle'
RELAY_LOADING = 'loading'
RELAY_OUTBOUND = 'outbound'
RELAY_UNLOADING = 'unloading'
RELAY_RETURNING = 'returning'
RELAY_FAILED = 'failed'

ARM_IDLE = 'idle'
ARM_COOLDOWN = 'cooldown'
# States an arm may be in and still take a new job.
ARM_FREE = (ARM_IDLE, ARM_COOLDOWN)


class Leg:
    """One step of the relay, and what has to be true for it to be over."""

    def __init__(self, name, relay_state, timeout):
        self.name = name
        self.relay_state = relay_state
        self.timeout = timeout


class StockRelay(Node):
    """Sequences the two arms and the carrier through one refill job."""

    def __init__(self):
        super().__init__('stock_relay')

        self.declare_parameter('request_topic', '/stock/refill_request')
        self.declare_parameter('state_topic', '/stock/relay_state')
        self.declare_parameter('reset_topic', '/stock/relay_reset')
        self.declare_parameter('station_a_ns', '/station_a')
        self.declare_parameter('station_b_ns', '/station_b')
        self.declare_parameter('beagle_goto_topic', '/beagle/goto')
        self.declare_parameter('beagle_state_topic', '/beagle/state')
        self.declare_parameter('bay_a', 'station_a')
        self.declare_parameter('bay_b', 'station_b')
        # Seconds each leg may take before the relay gives up on it. Generous:
        # these exist to stop a silent hang, not to pace the cell.
        self.declare_parameter('arm_timeout_sec', 120.0)
        self.declare_parameter('drive_timeout_sec', 180.0)
        # Whether the Beagle drives back to bay A once the job is done. Turn it
        # off when working on the arms, so the carrier stays where you left it.
        self.declare_parameter('return_home', True)

        self.bay_a = self.get_parameter('bay_a').value
        self.bay_b = self.get_parameter('bay_b').value
        self.arm_timeout = float(self.get_parameter('arm_timeout_sec').value)
        self.drive_timeout = float(self.get_parameter('drive_timeout_sec').value)
        self.return_home = bool(self.get_parameter('return_home').value)

        self.state = RELAY_IDLE
        self.detail = ''
        self.bin_id = None
        self.legs = []
        self.leg_index = 0
        self.leg_started_at = 0.0
        # Job ids handed to the arms. They come back on the arm's state topic,
        # which is how a leg is known to be over rather than merely quiet.
        self.job_seq = 0
        self.awaiting_job = None
        self.awaiting_station = None
        self.seen_busy = False

        self.arm_state = {}
        self.beagle = {}

        station_a = self.get_parameter('station_a_ns').value.rstrip('/')
        station_b = self.get_parameter('station_b_ns').value.rstrip('/')
        self.arms = {'a': station_a, 'b': station_b}

        self.transfer_pubs = {
            key: self.create_publisher(String, f'{ns}/stock/transfer', 10)
            for key, ns in self.arms.items()
        }
        for key, ns in self.arms.items():
            self.create_subscription(
                String,
                f'{ns}/stock/task_state',
                lambda msg, key=key: self._on_arm_state(key, msg),
                10,
            )

        self.goto_pub = self.create_publisher(
            String, self.get_parameter('beagle_goto_topic').value, 10
        )
        self.create_subscription(
            String,
            self.get_parameter('beagle_state_topic').value,
            self._on_beagle_state,
            10,
        )

        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10
        )
        self.create_subscription(
            String, self.get_parameter('request_topic').value, self._on_request, 10
        )
        self.create_subscription(
            Empty, self.get_parameter('reset_topic').value, self._on_reset, 10
        )

        self.create_timer(0.2, self._tick)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(
            f'Relay ready: {station_a} at {self.bay_a} -> carrier -> '
            f'{station_b} at {self.bay_b}'
        )

    # ------------------------------------------------------------------ input

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_request(self, msg):
        """Start a relay for the named bin, if nothing else is running."""
        bin_id = msg.data.strip()
        if not bin_id:
            return

        if self.state == RELAY_FAILED:
            self.get_logger().warn(
                f'Ignoring {bin_id}: the relay is stopped after '
                f'"{self.detail}". Clear it on the reset topic once the cell '
                f'has been looked at.'
            )
            return
        if self.state != RELAY_IDLE:
            self.get_logger().warn(
                f'Ignoring {bin_id}: already relaying {self.bin_id} ({self.state})'
            )
            return

        # Check the destination before anything moves. Station B only finds out
        # it cannot reach a place when it is handed the job, and by then the
        # part has been taken out of the warehouse, set on the tray and driven
        # the length of the cell - to be stranded there.
        places = (self.arm_state.get('b') or {}).get('places')
        if places is None:
            self.get_logger().warn(
                f'Ignoring {bin_id}: station B has not reported yet, so there '
                f'is no telling whether it can reach that bin'
            )
            return
        if bin_id not in places:
            self.get_logger().warn(
                f'Ignoring {bin_id}: station B places into {sorted(places)}'
            )
            return

        self.bin_id = bin_id
        self.legs = [
            Leg('load', RELAY_LOADING, self.arm_timeout),
            Leg('drive_out', RELAY_OUTBOUND, self.drive_timeout),
            Leg('unload', RELAY_UNLOADING, self.arm_timeout),
        ]
        if self.return_home:
            self.legs.append(Leg('drive_home', RELAY_RETURNING, self.drive_timeout))

        self.leg_index = 0
        self.get_logger().info(f'Relay started for {bin_id}')
        self._begin_leg()

    def _on_reset(self, _msg):
        """Clear a stopped relay once a person has sorted out the cell."""
        if self.state != RELAY_FAILED:
            self.get_logger().info('Reset ignored: the relay is not stopped')
            return
        self.get_logger().info(f'Relay cleared after "{self.detail}"')
        self.state = RELAY_IDLE
        self.detail = ''
        self.bin_id = None
        self.legs = []
        self.awaiting_job = None
        self.awaiting_station = None

    def _on_arm_state(self, key, msg):
        try:
            self.arm_state[key] = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Ignoring malformed state from arm {key}')

    def _on_beagle_state(self, msg):
        try:
            self.beagle = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed carrier state')

    # -------------------------------------------------------------- sequencing

    def _begin_leg(self):
        """Set the current leg going, or finish the job if there are none left."""
        if self.leg_index >= len(self.legs):
            self._finish()
            return

        leg = self.legs[self.leg_index]
        self.state = leg.relay_state
        self.detail = leg.name
        self.leg_started_at = self._now()
        self.awaiting_job = None
        self.awaiting_station = None
        self.seen_busy = False
        self._dispatch(leg)

    def _dispatch(self, leg):
        """Try to set ``leg`` going.  Does nothing if what it needs is not ready.

        _tick calls this again every cycle until it takes, so a leg waits for
        its preconditions instead of failing on them.
        """
        if leg.name == 'load':
            self._start_arm('a', 'warehouse', 'carrier')
        elif leg.name == 'unload':
            self._start_arm('b', 'carrier', self.bin_id)
        elif leg.name == 'drive_out':
            self._start_drive(self.bay_b)
        elif leg.name == 'drive_home':
            self._start_drive(self.bay_a)

    def _start_arm(self, key, source, destination):
        """Hand one arm its leg, once the tray is really in front of it.

        The command is not sent yet if the carrier is not parked: _tick keeps
        calling back until it is, or until the leg times out.  Sending anyway
        would have the arm close its gripper on empty air and report success.
        """
        bay = self.bay_a if key == 'a' else self.bay_b
        if not self._carrier_parked_at(bay):
            return
        arm = self.arm_state.get(key)
        if arm is None:
            return
        # Strictly idle, not merely free: an arm still counting down its
        # cooldown turns the job away, and a turned-away job would read as a
        # failed leg rather than as one that had not started yet.
        if arm.get('state') != ARM_IDLE:
            return

        self.job_seq += 1
        self.awaiting_job = (key, self.job_seq)
        self.transfer_pubs[key].publish(String(data=json.dumps({
            'from': source,
            'to': destination,
            'id': self.job_seq,
        })))
        self.get_logger().info(
            f'{self.detail}: arm {key} job {self.job_seq}, {source} -> {destination}'
        )

    def _start_drive(self, station):
        """Send the carrier off, once no arm is still reaching into it."""
        if not self._arms_clear():
            return
        self.awaiting_station = station
        self.goto_pub.publish(String(data=station))
        self.get_logger().info(f'{self.detail}: carrier to {station}')

    def _tick(self):
        """Watch the running leg: start it when it can start, end it when done."""
        if self.state in (RELAY_IDLE, RELAY_FAILED):
            return

        leg = self.legs[self.leg_index]
        waited = self._now() - self.leg_started_at

        if self.awaiting_job is None and self.awaiting_station is None:
            # The leg has not been dispatched yet - something it needs is not
            # ready.  Keep trying until the timeout says to stop waiting.
            if waited >= leg.timeout:
                self._fail(f'{leg.name} never became possible: {self._why_blocked(leg)}')
                return
            self._dispatch(leg)
            return

        if waited >= leg.timeout:
            self._fail(f'{leg.name} timed out after {leg.timeout:.0f}s')
            return

        if self.awaiting_job is not None:
            self._check_arm_leg()
        else:
            self._check_drive_leg()

    def _check_arm_leg(self):
        """Finish an arm leg when that arm reports the job it was given."""
        key, job_id = self.awaiting_job
        arm = self.arm_state.get(key) or {}
        if arm.get('state') == 'busy' and arm.get('job_id') == job_id:
            self.seen_busy = True

        last = arm.get('last_job') or {}
        if last.get('id') != job_id:
            return

        if last.get('result') != 'ok':
            self._fail(
                f'arm {key} could not run {last.get("job")}: '
                f'{last.get("error") or last.get("result")}'
            )
            return

        # A success is only believed once the arm was seen working on this very
        # job. Job ids restart with this node, so a report left over from an
        # earlier run can carry the same number as the one just sent; having
        # watched the arm go busy on it is what tells the two apart.
        if self.seen_busy:
            self._advance()

    def _check_drive_leg(self):
        """Finish a drive when the carrier is parked where it was sent."""
        state = self.beagle.get('state')
        if state in ('error', 'estop'):
            self._fail(f'carrier stopped: {self.beagle.get("detail") or state}')
            return
        if self._carrier_parked_at(self.awaiting_station):
            self._advance()

    def _advance(self):
        self.leg_index += 1
        self._begin_leg()

    def _finish(self):
        self.get_logger().info(f'Relay for {self.bin_id} complete')
        self.state = RELAY_IDLE
        self.detail = ''
        self.bin_id = None
        self.legs = []
        self.awaiting_job = None
        self.awaiting_station = None

    def _fail(self, reason):
        """Stop the relay where it stands and leave it for a person."""
        self.get_logger().error(
            f'Relay for {self.bin_id} failed: {reason}. Nothing further will '
            f'move until the reset topic is published.'
        )
        self.state = RELAY_FAILED
        self.detail = reason
        self.awaiting_job = None
        self.awaiting_station = None
        self._publish_state()

    # ------------------------------------------------------------- interlocks

    def _carrier_parked_at(self, station):
        """True only while the Beagle is stopped, at ``station``, and reachable."""
        return (
            self.beagle.get('ready_for_arm') is True
            and self.beagle.get('station') == station
        )

    def _arms_clear(self):
        """True when neither arm is in the middle of a job over the tray."""
        for key in self.arms:
            arm = self.arm_state.get(key)
            if arm is None:
                # An arm that has never reported cannot be shown to be clear.
                return False
            if arm.get('state') not in ARM_FREE:
                return False
        return True

    def _why_blocked(self, leg):
        """Explain, for the log, what a leg spent its timeout waiting for."""
        if leg.name in ('load', 'unload'):
            key = 'a' if leg.name == 'load' else 'b'
            bay = self.bay_a if key == 'a' else self.bay_b
            if not self._carrier_parked_at(bay):
                return (
                    f'the carrier never reported parked at {bay} '
                    f'(state {self.beagle.get("state")!r}, '
                    f'station {self.beagle.get("station")!r})'
                )
            arm = self.arm_state.get(key)
            if arm is None:
                return f'arm {key} never reported its state'
            return f'arm {key} stayed in state {arm.get("state")!r}'
        if not self._arms_clear():
            busy = [
                key for key in self.arms
                if (self.arm_state.get(key) or {}).get('state') not in ARM_FREE
            ]
            return f'arms {busy or list(self.arms)} never went idle'
        return 'unknown'

    # -------------------------------------------------------------- reporting

    def _publish_state(self):
        leg = self.legs[self.leg_index] if self.leg_index < len(self.legs) else None
        self.state_pub.publish(String(data=json.dumps({
            'state': self.state,
            'bin': self.bin_id,
            'leg': leg.name if leg else None,
            'leg_index': self.leg_index if leg else None,
            'legs': len(self.legs),
            'detail': self.detail,
            'carrier': {
                'state': self.beagle.get('state'),
                'station': self.beagle.get('station'),
                'ready_for_arm': self.beagle.get('ready_for_arm'),
            },
            'arms': {
                key: (self.arm_state.get(key) or {}).get('state')
                for key in self.arms
            },
        })))


def main(args=None):
    rclpy.init(args=args)
    node = StockRelay()
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
