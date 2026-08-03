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
from std_msgs.msg import Bool, String
import yaml

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
        self.declare_parameter('scene', 'corridor')
        self.declare_parameter('goto_topic', '/beagle/goto')
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
        self.station = self.config.get('home_station') or next(iter(self.stations))

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
            Bool, self.get_parameter('estop_topic').value, self._on_estop, 10
        )

        # The robot is created on the worker thread, so a dongle that is not
        # plugged in fails there and is reported as a state rather than
        # crashing the node before it can tell anyone why.
        self.robot = None
        self.navigator = None
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
        try:
            self.robot = SafeBeagle(
                dry_run=self.dry_run,
                scene=self.get_parameter('scene').value,
                max_speed=float(self.config.get('robot', {}).get('max_speed', 25.0)),
            )
            self.robot.start_lidar()
            self.robot.wait_until_lidar_ready()
        except Exception as exc:  # noqa: BLE001 - report any connection failure
            self.state = STATE_ERROR
            self.detail = f'could not connect: {exc}'
            self.get_logger().error(self.detail)
            self._telemetry_loop_without_robot()
            return

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
            self._drive_to(target)
            self._publish_telemetry()

        self.robot.stop()

    def _telemetry_loop_without_robot(self):
        """Keep reporting the error state so the mission does not wait forever."""
        while not self._shutdown.is_set():
            self._publish_telemetry()
            self._shutdown.wait(float(self.get_parameter('telemetry_period').value))

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
            self.robot.stop()
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
