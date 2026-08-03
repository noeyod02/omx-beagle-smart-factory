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

"""Drive the Beagle along a scripted route between two manipulator stations.

The route is a short list of steps rather than a planned path, because the
Beagle shuttles between two fixed stations.  Dead reckoning gets it roughly
there and the docking step resets the accumulated error against a wall, which
is what keeps the arms able to find the tray.

Every step can be aborted between control cycles, so a stop request or an
estop takes effect promptly rather than at the end of the current move.

Run this file directly to exercise a route in the simulator:

    python3 beagle_navigation.py --route station_a station_b --scene corridor
"""

from __future__ import annotations

import argparse
import math
import time

from beagle_lib.geometry import clamp
from beagle_lib.lidar import cardinal_distances
from beagle_lib.motion import calibrate_gyro_bias, center_one_axis, turn_degrees
from beagle_lib.robot import SafeBeagle

# Encoder readings are treated as millimetres travelled.  The simulator already
# reports millimetres; on real hardware run the course's encoder calibration and
# feed the result in as encoder_scale.
DEFAULT_ENCODER_SCALE = 1.0

# How hard straight driving corrects heading drift, in wheel percent per degree.
DEFAULT_HEADING_GAIN = 0.6


class RouteAborted(Exception):
    """Raised when a route is cancelled part way through."""


class RouteFailed(Exception):
    """Raised when a step ran to completion but did not achieve its goal.

    Docking steps in particular must fail loudly: if the Beagle stops short of
    the bay, the tray is not where the arm expects it, and reporting arrival
    anyway would send the arm reaching into empty space.
    """


class BeagleNavigator:
    """Executes route steps on a Beagle, with heading held by the gyro."""

    def __init__(
        self,
        robot: SafeBeagle,
        *,
        encoder_scale: float = DEFAULT_ENCODER_SCALE,
        heading_gain: float = DEFAULT_HEADING_GAIN,
        cruise_speed: float = 18.0,
        turn_speed: float = 11.0,
        control_period: float = 0.05,
        logger=None,
    ) -> None:
        self.robot = robot
        self.encoder_scale = float(encoder_scale)
        self.heading_gain = float(heading_gain)
        self.cruise_speed = float(cruise_speed)
        self.turn_speed = float(turn_speed)
        self.control_period = float(control_period)
        self.logger = logger

        self._abort = False
        self.gyro_bias = 0.0
        self.last_step = ''
        self.failure = ''

    # ----------------------------------------------------------------- helpers

    def log(self, message: str) -> None:
        if self.logger is not None:
            self.logger(message)
        else:
            print(message)

    def abort(self) -> None:
        """Ask the current route to stop at the next control cycle."""
        self._abort = True

    def clear_abort(self) -> None:
        self._abort = False

    def _check_abort(self) -> None:
        if self._abort:
            self.robot.stop()
            raise RouteAborted(f'aborted during {self.last_step}')

    def _travelled_mm(self, start_left: float, start_right: float) -> float:
        """Distance covered since the given encoder readings, in millimetres."""
        left = float(self.robot.left_encoder()) - start_left
        right = float(self.robot.right_encoder()) - start_right
        return (left + right) / 2.0 * self.encoder_scale

    def calibrate(self) -> float:
        """Measure the gyro's resting bias so heading integration does not drift."""
        self.gyro_bias = calibrate_gyro_bias(self.robot)
        self.log(f'gyro bias {self.gyro_bias:+.3f} deg/s')
        return self.gyro_bias

    # -------------------------------------------------------------- primitives

    def drive_distance(self, distance_m: float, speed: float | None = None) -> float:
        """Drive straight for ``distance_m``, holding heading with the gyro.

        Returns the distance actually covered, in metres.  Negative distances
        reverse; heading is still held.
        """
        speed = self.cruise_speed if speed is None else float(speed)
        direction = 1.0 if distance_m >= 0.0 else -1.0
        target_mm = abs(float(distance_m)) * 1000.0
        if target_mm < 1.0:
            return 0.0

        start_left = float(self.robot.left_encoder())
        start_right = float(self.robot.right_encoder())
        heading = 0.0
        previous = time.monotonic()
        # Allow generous headroom over the ideal time before giving up.
        deadline = previous + 6.0 + abs(distance_m) * 20.0

        try:
            while time.monotonic() < deadline:
                self._check_abort()

                now = time.monotonic()
                dt = min(0.2, max(0.0, now - previous))
                previous = now
                heading += (float(self.robot.gyroscope_z()) - self.gyro_bias) * dt

                travelled = abs(self._travelled_mm(start_left, start_right))
                if travelled >= target_mm:
                    break

                # Steer back onto the original heading.  Turn rate is
                # (right - left) / wheel_base, so the correction keeps the same
                # sign whichever way the robot is travelling - mirroring it when
                # reversing turns the feedback positive and the error grows.
                correction = clamp(self.heading_gain * heading, -speed * 0.6, speed * 0.6)
                base = speed * direction
                self.robot.wheels(base + correction, base - correction)
                time.sleep(self.control_period)
        finally:
            self.robot.stop()

        covered = abs(self._travelled_mm(start_left, start_right)) / 1000.0
        self.log(
            f'drove {covered * direction:+.3f} m of {distance_m:+.3f} m '
            f'(heading drift {heading:+.1f} deg)'
        )
        return covered * direction

    def turn(self, degrees: float) -> float:
        """Turn in place by ``degrees``; positive is counter-clockwise."""
        self._check_abort()
        turned = turn_degrees(self.robot, float(degrees), speed=self.turn_speed)
        self.log(f'turned {turned:+.1f} deg of {degrees:+.1f} deg')
        return turned

    def approach(self, target_mm: float, speed: float | None = None,
                 timeout_s: float = 45.0, tolerance_mm: float = 10.0) -> float:
        """Creep forward until the wall ahead is ``target_mm`` away.

        This is what stops dead reckoning error from accumulating across trips:
        however far off the drive left the robot, it finishes a fixed distance
        from the docking wall.  Raises RouteFailed if it cannot get there, since
        a Beagle parked short of the bay is not one an arm can safely unload.
        """
        # Cap the approach speed, then close the gap proportionally: full speed
        # while there is room, easing off near the bay so it does not overshoot
        # into the docking jig.
        speed = self.cruise_speed if speed is None else float(speed)
        crawl_speed = max(4.0, speed * 0.25)
        deadline = time.monotonic() + timeout_s
        front = math.inf
        arrived = False

        try:
            while time.monotonic() < deadline:
                self._check_abort()

                front = float(cardinal_distances(self.robot.lidar())['front'])
                if not math.isfinite(front):
                    # Nothing in view yet; edge forward rather than stopping,
                    # since the wall is usually just beyond lidar's near range.
                    self.robot.wheels(crawl_speed, crawl_speed)
                    time.sleep(self.control_period)
                    continue

                error = front - target_mm
                if abs(error) <= tolerance_mm:
                    arrived = True
                    break
                move = clamp(0.05 * error, -speed, speed)
                if abs(move) < crawl_speed:
                    move = math.copysign(crawl_speed, move)
                self.robot.wheels(move, move)
                time.sleep(self.control_period)
        finally:
            self.robot.stop()

        self.log(f'approached to {front:.0f} mm (target {target_mm:.0f} mm)')
        if not arrived:
            measured = f'{front:.0f} mm' if math.isfinite(front) else 'nothing in range'
            raise RouteFailed(
                f'approach did not reach {target_mm:.0f} mm within {timeout_s:.0f} s '
                f'(front reads {measured})'
            )
        return front

    def square_to_wall(self, tolerance_mm: float = 15.0, timeout_s: float = 12.0,
                       speed: float | None = None) -> float:
        """Rotate until the wall ahead is square on.

        The beams 35 degrees either side of straight ahead read the same
        distance only when the robot faces the wall head on; whichever side
        reads longer is the side the robot has turned towards.  Squaring up is
        what fixes the tray's angle, which matters more to the arm than the
        Beagle's exact position does.

        Raises RouteFailed if it cannot settle within ``timeout_s``.
        """
        speed = self.turn_speed if speed is None else float(speed)
        deadline = time.monotonic() + timeout_s
        error = math.inf
        squared = False

        try:
            while time.monotonic() < deadline:
                self._check_abort()

                distances = cardinal_distances(self.robot.lidar())
                left = float(distances['front_left'])
                right = float(distances['front_right'])
                if not (math.isfinite(left) and math.isfinite(right)):
                    self.robot.stop()
                    raise RouteFailed('cannot see the docking wall on both sides')

                error = left - right
                if abs(error) <= tolerance_mm:
                    squared = True
                    break
                # Longer on the left means the robot has swung anticlockwise,
                # so drive the left wheel forward to bring it back clockwise.
                command = clamp(0.03 * error, -speed, speed)
                if abs(command) < 5.0:
                    command = math.copysign(5.0, command)
                self.robot.wheels(command, -command)
                time.sleep(self.control_period)
        finally:
            self.robot.stop()

        self.log(f'squared to wall, front_left - front_right = {error:+.0f} mm')
        if not squared:
            raise RouteFailed(
                f'could not square to the wall within {tolerance_mm:.0f} mm '
                f'in {timeout_s:.0f} s (last error {error:+.0f} mm)'
            )
        return error

    def centre(self, tolerance_mm: float = 20.0, timeout_s: float = 8.0) -> bool:
        """Equalise the distance in front and behind, squaring up to the bay.

        Raises RouteFailed if it cannot settle, for the same reason approach
        does: an unsquared Beagle puts the tray at an angle the arm has not
        been taught.
        """
        self._check_abort()
        ok = center_one_axis(
            self.robot, tolerance_mm=tolerance_mm, timeout_s=timeout_s, verbose=False
        )
        self.log(f'centring {"succeeded" if ok else "FAILED"}')
        if not ok:
            raise RouteFailed(
                f'could not centre to within {tolerance_mm:.0f} mm in {timeout_s:.0f} s'
            )
        return ok

    def wait(self, seconds: float) -> None:
        """Hold still, still watching for an abort."""
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            self._check_abort()
            time.sleep(min(self.control_period, max(0.0, deadline - time.monotonic())))

    # ------------------------------------------------------------------ routes

    def run_route(self, steps) -> bool:
        """Run a list of step dictionaries.

        Returns True only if every step achieved its goal.  A route that was
        cancelled, or whose docking steps fell short, returns False and leaves
        the caller to treat the Beagle's position as unknown.
        """
        self.clear_abort()
        self.failure = ''
        try:
            for index, step in enumerate(steps, start=1):
                action = step.get('action')
                self.last_step = f'step {index} ({action})'
                self.log(f'{self.last_step}: {step}')

                if action == 'forward':
                    self.drive_distance(float(step['distance_m']), step.get('speed'))
                elif action == 'backward':
                    self.drive_distance(-abs(float(step['distance_m'])), step.get('speed'))
                elif action == 'turn':
                    self.turn(float(step['degrees']))
                elif action == 'approach':
                    self.approach(float(step['target_mm']), step.get('speed'))
                elif action == 'square':
                    self.square_to_wall(float(step.get('tolerance_mm', 15.0)))
                elif action == 'centre':
                    self.centre(float(step.get('tolerance_mm', 20.0)))
                elif action == 'wait':
                    self.wait(float(step.get('seconds', 1.0)))
                else:
                    raise ValueError(f'unknown route action: {action!r}')
        except RouteAborted as exc:
            self.failure = f'aborted: {exc}'
            self.log(f'route aborted: {exc}')
            return False
        except RouteFailed as exc:
            self.failure = str(exc)
            self.log(f'route FAILED: {exc}')
            return False
        finally:
            self.robot.stop()
        return True


def _demo() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default='corridor', help='MockBeagle scene to drive in')
    parser.add_argument('--distance', type=float, default=0.6, help='metres to drive')
    parser.add_argument('--turn', type=float, default=90.0, help='degrees to turn')
    args = parser.parse_args()

    steps = [
        {'action': 'forward', 'distance_m': args.distance},
        {'action': 'turn', 'degrees': args.turn},
        {'action': 'approach', 'target_mm': 150.0},
        {'action': 'centre'},
    ]

    with SafeBeagle(dry_run=True, scene=args.scene) as robot:
        robot.start_lidar()
        robot.wait_until_lidar_ready()
        navigator = BeagleNavigator(robot)
        navigator.calibrate()
        ok = navigator.run_route(steps)
        print(f'\nroute {"completed" if ok else "aborted"}; pose {robot.robot.pose}')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(_demo())
