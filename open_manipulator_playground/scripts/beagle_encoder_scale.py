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

"""Find out how far one encoder count moves the real Beagle.

Until this is measured every distance in beagle_route.yaml is a guess.  The
simulator reports millimetres directly, so ``encoder_scale: 1.0`` is right
there and wrong on the real carrier, and a route that asks for 1.20 m will
travel some other distance entirely.

The measurement cannot bootstrap itself - working out how far the robot went
is the whole problem - so the tape measure is the reference.  The robot drives
for a set time, you measure what it actually covered, and the scale falls out:

    encoder_scale = millimetres travelled / encoder counts elapsed

Run it in the container, with the floor between the bays clear:

    ros2 run open_manipulator_playground beagle_encoder_scale.py --seconds 4

Do this along the route the Beagle will actually drive, on the same floor.
Carpet and hard floor differ, and the wheels slip differently on each.
"""

import argparse
import sys
import time

from beagle_lib.lidar import cardinal_distances
from beagle_lib.robot import SafeBeagle

# Slow enough that a wheel slipping shows up as a bad repeat rather than a
# crash into the bay wall, and matching the cruise speed routes actually use.
DEFAULT_SPEED = 18.0
DEFAULT_SECONDS = 4.0

# The robot is stopped if anything comes within this of the front, leaving room
# to halt before touching it.
GUARD_MM = 300.0


def measure(robot, speed, seconds, guard):
    """Drive straight for a while and report the encoder counts it took.

    Returns ``(left_counts, right_counts, stopped_early)``.
    """
    start_left = float(robot.left_encoder())
    start_right = float(robot.right_encoder())

    robot.wheels(speed, speed)
    deadline = time.monotonic() + seconds
    stopped_early = False
    try:
        while time.monotonic() < deadline:
            if guard:
                front = cardinal_distances(robot.lidar())['front']
                if front == front and front < GUARD_MM:  # NaN-safe
                    stopped_early = True
                    print(f'\n  stopped: something {front:.0f} mm ahead')
                    break
            time.sleep(0.02)
    finally:
        robot.stop()
        # The wheels do not halt the instant they are told to; let the coast
        # finish before reading, or it lands in the next measurement instead.
        time.sleep(0.5)

    left = float(robot.left_encoder()) - start_left
    right = float(robot.right_encoder()) - start_right
    return left, right, stopped_early


def ask_distance():
    """Ask what the tape says, in millimetres.

    A blank line asks again rather than discarding the run.  Discarding is what
    Enter does by reflex, and the run it throws away cannot be recovered
    without driving the robot again - so throwing one away has to be said
    deliberately.
    """
    while True:
        reply = input('  How far did it actually travel, in mm? ').strip()
        if not reply:
            print('  Measure it and type the number. To throw this run away, '
                  "type 'skip'.")
            continue
        if reply.lower() in ('skip', 's'):
            return None
        try:
            value = float(reply)
        except ValueError:
            print("  Enter a number of millimetres, or 'skip' to discard.")
            continue
        if value <= 0:
            print('  It has to be a positive distance.')
            continue
        return value


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--seconds', type=float, default=DEFAULT_SECONDS,
                        help='how long to drive for each run')
    parser.add_argument('--speed', type=float, default=DEFAULT_SPEED,
                        help='wheel percent; keep it at the route cruise speed')
    parser.add_argument('--runs', type=int, default=3,
                        help='repeats; the spread between them is the honest '
                             'error bar on the result')
    parser.add_argument('--no-lidar-guard', action='store_true',
                        help='drive without the obstacle check, if the lidar '
                             'will not start')
    parser.add_argument('--dry-run', action='store_true',
                        help='rehearse against the simulator. The scale it '
                             'produces is meaningless - the simulator already '
                             'reports millimetres - but the procedure can be '
                             'walked through without the carrier moving.')
    known, _ = parser.parse_known_args(argv)
    return known


def main():
    argv = list(sys.argv[1:])
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parse_args(argv)

    print(__doc__)
    print(f'Driving {args.runs} times, {args.seconds:.1f} s at {args.speed:.0f}%.')
    print('Clear the floor ahead. Ctrl-C stops the wheels.\n')

    with SafeBeagle(dry_run=args.dry_run, scene='corridor') as robot:
        guard = not args.no_lidar_guard
        if guard:
            try:
                robot.start_lidar()
                robot.wait_until_lidar_ready()
            except Exception as exc:
                print(f'  lidar did not start ({exc}).')
                print('  Re-run with --no-lidar-guard to drive without it.\n')
                return 1

        scales = []
        for run in range(1, args.runs + 1):
            input(f'Run {run}: put the robot at the start line, then press Enter.')
            left, right, stopped = measure(robot, args.speed, args.seconds, guard)
            counts = (left + right) / 2.0
            print(f'  encoders: left {left:+.0f}, right {right:+.0f}, mean {counts:+.0f}')

            if abs(left - right) > 0.05 * max(abs(counts), 1.0):
                print('  note: the wheels disagree by more than 5%. The robot '
                      'veered, so the tape distance is longer than the straight '
                      'line it should have driven.')
            if counts == 0:
                print('  no counts at all - the robot did not move, or the '
                      'encoders are not being read.\n')
                continue
            if stopped:
                print('  this run was cut short; measure what it did travel.')

            distance_mm = ask_distance()
            if distance_mm is None:
                print('  discarded.\n')
                continue
            scale = distance_mm / counts
            scales.append(scale)
            print(f'  encoder_scale from this run: {scale:.6f}\n')

    if not scales:
        print('Nothing measured.\n')
        return 1

    mean = sum(scales) / len(scales)
    print(f'\n{len(scales)} run(s): ' + ', '.join(f'{s:.6f}' for s in scales))
    if len(scales) > 1:
        spread = (max(scales) - min(scales)) / mean * 100.0
        print(f'Spread {spread:.1f}% of the mean.')
        if spread > 5.0:
            print('That is a lot. The wheels are slipping, or the start line '
                  'was not the same each time. A route measured against this '
                  'will arrive somewhere different each trip, which is exactly '
                  'what the docking step at the end exists to absorb - but it '
                  'can only absorb so much.')

    print('\nPut this in config/beagle_route.yaml under robot:\n')
    print(f'  encoder_scale: {mean:.6f}')
    print('\nThen measure the bay-to-bay distance and set distance_m in the')
    print('station_a->station_b route to match.\n')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\nStopped.\n')
        sys.exit(1)
