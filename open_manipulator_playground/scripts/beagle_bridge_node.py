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

r"""Expose the Beagle carrier to ROS 2 so the arm stations can dispatch it.

The Beagle is not a ROS robot: it is driven from this PC over a Bluetooth
dongle through the ``roboid`` library, whose calls block.  All of that is kept
on one worker thread which owns the robot outright, so no two callers ever talk
to the dongle at once.  ROS callbacks only queue commands or raise the estop.

    ros2 run open_manipulator_playground beagle_bridge_node.py \\
        --ros-args -p dry_run:=true -p route_file:=<beagle_route.yaml>

    ros2 topic pub --once /beagle/goto std_msgs/String "{data: station_b}"
    ros2 topic pub --once /beagle/estop std_msgs/Bool "{data: true}"

With ``dry_run`` set the node drives the course package's 2D simulator instead
of real hardware, which is how the routes and the mission sequencing above it
get tested before anything moves.
"""

import json
import math
import os
import queue
import threading

from beagle_lib.lidar import cardinal_distances
from beagle_lib.robot import SafeBeagle
from beagle_navigation import BeagleNavigator
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
import yaml

# The most a single nudge may move the carrier. This exists to settle a bay by
# a few centimetres; a move long enough to cross the cell is a route, where it
# is written down and can be read back later.
NUDGE_MAX_M = 0.30

STATE_STARTING = 'starting'
STATE_IDLE = 'idle'
STATE_DRIVING = 'driving'
STATE_ESTOP = 'estop'
STATE_ERROR = 'error'

# Distances the Beagle lidar cannot resolve, in metres.
SCAN_RANGE_MIN = 0.05
SCAN_RANGE_MAX = 5.0


class BeagleBridge(Node):
    """Drives the Beagle along named routes on behalf of the mission."""

    def __init__(self):
        super().__init__('beagle_bridge')

        self.declare_parameter('route_file', '')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('start_station', '')
        self.declare_parameter('scene', 'corridor')
        self.declare_parameter('goto_topic', '/beagle/goto')
        self.declare_parameter('nudge_topic', '/beagle/nudge')
        self.declare_parameter('estop_topic', '/beagle/estop')
        self.declare_parameter('state_topic', '/beagle/state')
        self.declare_parameter('scan_topic', '/beagle/scan')
        self.declare_parameter('publish_scan', True)
        self.declare_parameter('scan_frame_id', 'beagle_lidar')
        self.declare_parameter('telemetry_period', 0.5)

        route_file = self.get_parameter('route_file').value
        if not route_file or not os.path.exists(route_file):
            raise RuntimeError(f'route_file not found: {route_file!r}')
        with open(route_file) as handle:
            self.config = yaml.safe_load(handle)

        self.routes = self.config.get('routes', {})
        self.stations = self.config.get('stations', {})
        if not self.stations:
            raise RuntimeError('route file lists no stations')
        # Where the carrier is standing when this node starts.
        #
        # Nothing on the robot can answer that - it drives on dead reckoning
        # with no map - so the bridge has to be told, and until it is it
        # assumes the home station. That assumption is wrong every time the
        # bridge is restarted with the carrier parked at the far bay, and it is
        # wrong in the worst way: the bridge reports ready_for_arm at a station
        # the carrier is not at, and an arm reaches into empty air for a tray
        # that is at the other end of the cell.
        #
        # So it is a parameter. Set start_station to wherever the carrier
        # actually is before trusting anything this node says about position.
        start_station = self.get_parameter('start_station').value
        if start_station and start_station not in self.stations:
            raise RuntimeError(
                f'start_station {start_station!r} is not one of '
                f'{sorted(self.stations)}'
            )
        self.station = (
            start_station
            or self.config.get('home_station')
            or next(iter(self.stations))
        )

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.state = STATE_STARTING
        self.detail = ''
        self.estopped = False
        self.commands = queue.Queue()
        self._shutdown = threading.Event()
        self._last_scan = []

        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10
        )
        self.scan_pub = None
        if bool(self.get_parameter('publish_scan').value):
            self.scan_pub = self.create_publisher(
                LaserScan, self.get_parameter('scan_topic').value, 1
            )

        self.create_subscription(
            String, self.get_parameter('goto_topic').value, self._on_goto, 10
        )
        self.create_subscription(
            Float32, self.get_parameter('nudge_topic').value, self._on_nudge, 10
        )
        self.create_subscription(
            Bool, self.get_parameter('estop_topic').value, self._on_estop, 10
        )

        # Connected here, on the main thread, and handed to the worker that
        # owns it from then on.
        #
        # It used to be built on the worker instead, so that a dongle which is
        # not plugged in failed there and was reported as a state rather than
        # crashing the node. That is still what happens - the failure is caught
        # below - but the connecting itself has to happen here: the roboid
        # library installs signal handlers as it connects, and only the main
        # thread may do that. Off the main thread it raises, which is why real
        # hardware never came up. The handlers are also what stops the wheels
        # on Ctrl-C, so they are wanted, not merely tolerated.
        self.robot = None
        self.navigator = None
        self.connect_error = None
        self.lidar_ready = False
        try:
            self.robot = SafeBeagle(
                dry_run=self.dry_run,
                scene=self.get_parameter('scene').value,
                max_speed=float(self.config.get('robot', {}).get('max_speed', 25.0)),
            )
        except Exception as exc:  # noqa: BLE001 - report any connection failure
            self.connect_error = exc

        self.worker = threading.Thread(target=self._run_worker, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'Beagle bridge started ({"simulated" if self.dry_run else "real hardware"}), '
            f'stations: {", ".join(sorted(self.stations))}'
        )

    # -------------------------------------------------------------- callbacks

    def _on_goto(self, msg):
        """Queue a drive to the named station."""
        target = msg.data.strip()
        if target not in self.stations:
            self.get_logger().warn(
                f'Unknown station {target!r}; known: {sorted(self.stations)}'
            )
            return
        if self.estopped:
            self.get_logger().warn('Estop is latched; clear it before dispatching')
            return
        self.commands.put(target)
        self.get_logger().info(f'Queued drive to {target}')

    def _on_nudge(self, msg):
        """Queue a short move along the current heading, in metres.

        For settling the carrier where a bay actually wants it, which is a
        thing the routes cannot express: they run bay to bay, and the question
        here is where 'bay' should be. Positive is forward, negative back.

        Driven rather than pushed by hand on purpose. A carrier shoved into
        place stands somewhere no route will ever put it again, and the tray
        coordinate taught against that pose is wrong from the next arrival on.
        This moves it the same way a route would, so the pose can be reached
        again - but only once the route is edited to match, which is on
        whoever nudges it. Until then this position is a one-off.
        """
        distance = float(msg.data)
        if self.estopped:
            self.get_logger().warn('Estop is latched; clear it before dispatching')
            return
        if not math.isfinite(distance) or distance == 0.0:
            self.get_logger().warn(f'Ignoring nudge of {distance}')
            return
        if abs(distance) > NUDGE_MAX_M:
            self.get_logger().warn(
                f'Refusing a nudge of {distance:+.3f} m: this is for settling a '
                f'bay by a few centimetres, and anything over {NUDGE_MAX_M} m '
                f'belongs in a route where it can be reviewed'
            )
            return
        self.commands.put(('nudge', distance))
        self.get_logger().info(f'Queued nudge of {distance:+.3f} m')

    def _on_estop(self, msg):
        """Latch or clear the emergency stop."""
        if msg.data:
            self.estopped = True
            if self.navigator is not None:
                self.navigator.abort()
            if self.robot is not None:
                self.robot.stop()
            # Drop anything queued: after an estop the operator decides what
            # happens next, the old plan is not automatically still valid.
            while not self.commands.empty():
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break
            self.get_logger().error('ESTOP latched - Beagle stopped')
        else:
            self.estopped = False
            self.get_logger().warn('Estop cleared')

    # ------------------------------------------------------------------ worker

    def _run_worker(self):
        """Own the robot: connect, then alternate between routes and telemetry."""
        if self.connect_error is not None:
            self.state = STATE_ERROR
            self.detail = f'could not connect: {self.connect_error}'
            self.get_logger().error(self.detail)
            self._telemetry_loop_without_robot()
            return

        # The lidar is spun up here rather than beside the connection: it takes
        # seconds to come up to speed, and there is no reason to hold the
        # node's startup for it.
        #
        # A lidar that does not come up is not fatal. It is what approach and
        # square steer by, but a route made only of forward, backward and turn
        # never reads it, and refusing to start would take away driving that
        # works. Such a route is refused when it is asked for instead, in
        # _drive_to, where the steps are known.
        self.lidar_ready = False
        try:
            self.robot.start_lidar()
            self.robot.wait_until_lidar_ready()
            self.lidar_ready = True
        except Exception as exc:  # noqa: BLE001 - a missing lidar is survivable
            self.get_logger().warn(
                f'lidar not ready: {exc} Driving still works; docking steps '
                f'(approach, square, centre) will be refused.'
            )

        settings = self.config.get('robot', {})
        self.navigator = BeagleNavigator(
            self.robot,
            encoder_scale=float(settings.get('encoder_scale', 1.0)),
            heading_gain=float(settings.get('heading_gain', 0.6)),
            cruise_speed=float(settings.get('cruise_speed', 18.0)),
            turn_speed=float(settings.get('turn_speed', 11.0)),
            logger=lambda message: self.get_logger().info(f'  {message}'),
        )
        self.navigator.calibrate()
        self.state = STATE_IDLE

        period = float(self.get_parameter('telemetry_period').value)
        while not self._shutdown.is_set():
            try:
                target = self.commands.get(timeout=period)
            except queue.Empty:
                self._publish_telemetry()
                continue
            if isinstance(target, tuple):
                self._nudge(target[1])
            else:
                self._drive_to(target)
            self._publish_telemetry()

        self.robot.stop()

    def _telemetry_loop_without_robot(self):
        """Keep reporting the error state so the mission does not wait forever."""
        while not self._shutdown.is_set():
            self._publish_telemetry()
            self._shutdown.wait(float(self.get_parameter('telemetry_period').value))

    def _nudge(self, distance):
        """Drive a short way along the current heading, staying at this station.

        The station is deliberately left alone. The carrier has not gone
        anywhere else - it is still in this bay, a few centimetres deeper - so
        an arm that was allowed to reach into it still is. What has changed is
        that the route no longer describes where it stopped, and that is said
        plainly in the log rather than left for the next arrival to reveal.
        """
        action = 'forward' if distance > 0.0 else 'backward'
        step = {'action': action, 'distance_m': abs(distance)}

        self.state = STATE_DRIVING
        self.detail = f'nudge {distance:+.3f} m'
        self.get_logger().info(f'Nudging {distance:+.3f} m at {self.station or "?"}')
        try:
            self.navigator.run_route([step])
        except Exception as exc:  # noqa: BLE001 - a nudge must not kill the thread
            self.robot.stop()
            self.state = STATE_ERROR
            self.detail = f'nudge failed: {exc}'
            self.get_logger().error(self.detail)
            return

        self.state = STATE_IDLE
        self.detail = ''
        self.get_logger().warn(
            f'Nudged {distance:+.3f} m. The route into {self.station or "this bay"} '
            f'no longer ends here - edit its distance_m by the same amount, or '
            f'the next arrival will park where it used to.'
        )

    def _drive_to(self, target):
        """Run the route from the current station to ``target``."""
        if target == self.station:
            self.get_logger().info(f'Already at {target}')
            return

        key = f'{self.station}->{target}'
        steps = self.routes.get(key)
        if steps is None:
            self.state = STATE_ERROR
            self.detail = f'no route {key}'
            self.get_logger().error(f'{self.detail}; known routes: {sorted(self.routes)}')
            return

        # Refused rather than attempted: approach and square decide when to
        # stop from what the lidar sees ahead, and with no lidar they have
        # nothing to stop on. Driving the route anyway would take the carrier
        # past the bay rather than into it.
        needs_lidar = sorted(
            {step.get('action') for step in steps}
            & {'approach', 'square', 'centre'}
        )
        if needs_lidar and not self.lidar_ready:
            self.state = STATE_ERROR
            self.detail = (
                f'{key} needs the lidar for {", ".join(needs_lidar)}, '
                f'and it is not ready'
            )
            self.get_logger().error(self.detail)
            return

        self.state = STATE_DRIVING
        self.detail = key
        # Which station the Beagle is at is unknown while it is between them;
        # saying otherwise would let an arm reach for a tray that is not there.
        origin, self.station = self.station, ''
        self.get_logger().info(f'Driving {key} ({len(steps)} steps)')

        try:
            completed = self.navigator.run_route(steps)
        except Exception as exc:  # noqa: BLE001 - a route must not kill the thread
            self.robot.stop()
            self.state = STATE_ERROR
            self.detail = f'{key} failed: {exc}'
            self.get_logger().error(self.detail)
            return

        if completed:
            self.station = target
            self.state = STATE_IDLE
            self.detail = ''
            self.get_logger().info(f'Arrived at {target}')
        else:
            # Stopped part way, either cancelled or because docking fell short.
            # Either way self.station stays empty, so ready_for_arm is false and
            # no arm will reach for a tray that is not where it should be.
            reason = self.navigator.failure or 'stopped'
            self.state = STATE_ESTOP if self.estopped else STATE_ERROR
            self.detail = f'{key}: {reason}'
            self.get_logger().error(
                f'Route {key} did not complete ({reason}); '
                f'position unknown, last known station was {origin}'
            )

    # -------------------------------------------------------------- telemetry

    def _publish_telemetry(self):
        """Publish the carrier's state and, when available, its latest scan."""
        distances = {}
        if self.robot is not None and self.state != STATE_DRIVING:
            try:
                self._last_scan = self.robot.lidar()
                distances = {
                    name: (round(value / 1000.0, 3) if math.isfinite(value) else None)
                    for name, value in cardinal_distances(self._last_scan).items()
                }
            except Exception as exc:  # noqa: BLE001 - telemetry must not throw
                self.get_logger().warn(
                    f'lidar read failed: {exc}', throttle_duration_sec=10.0
                )

        # True only when the Beagle is parked at a known station, which is the
        # one condition under which an arm may reach into its tray.
        parked = self.state == STATE_IDLE and bool(self.station)
        self.state_pub.publish(String(data=json.dumps({
            'state': STATE_ESTOP if self.estopped else self.state,
            'station': self.station,
            'detail': self.detail,
            'ready_for_arm': parked and not self.estopped,
            'simulated': self.dry_run,
            'distances_m': distances,
        })))

        if self.scan_pub is not None and self._last_scan:
            self.scan_pub.publish(self._build_scan(self._last_scan))

    def _build_scan(self, scan_mm):
        """Convert a Beagle scan into a LaserScan so it can be seen in RViz."""
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('scan_frame_id').value
        count = len(scan_mm)
        message.angle_min = 0.0
        message.angle_max = 2.0 * math.pi
        message.angle_increment = 2.0 * math.pi / count
        message.range_min = SCAN_RANGE_MIN
        message.range_max = SCAN_RANGE_MAX
        # LaserScan wants infinity for "nothing there", which is what the
        # course library already uses for a rejected sample.
        message.ranges = [
            (value / 1000.0 if math.isfinite(value) else math.inf) for value in scan_mm
        ]
        return message

    def destroy_node(self):
        self._shutdown.set()
        if self.navigator is not None:
            self.navigator.abort()
        if self.worker.is_alive():
            self.worker.join(timeout=3.0)
        if self.robot is not None:
            # close(), not stop(): stopping only halts the wheels, and leaves
            # the library's own communication threads running, which are not
            # daemons - the process would sit there after Ctrl-C.
            self.robot.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BeagleBridge()
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
