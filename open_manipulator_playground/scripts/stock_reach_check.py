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

"""Check that every point in a stock layout is somewhere the arm can go.

Pointing the gripper straight down costs a lot of reach, because the wrist has
to sit 0.12 m above whatever is being grasped.  That makes it easy to write a
layout that looks reasonable but cannot be executed, so run this after editing
any coordinate:

    python3 stock_reach_check.py ../config/stock_layout.yaml

Every grasp point is checked along with the hover above it, and the reach
envelope is printed so nearby positions can be chosen with confidence.  Exits
non-zero if anything is unreachable.
"""

import argparse
import math
import os
import sys

from omx_kinematics import BASE_OFFSET_X, forward_kinematics, inverse_kinematics
import yaml

# Anything closer than this to the edge of the envelope is reachable but leaves
# the arm near a singular, fully stretched pose.
MARGIN = 0.95


def max_radial(z, pitch, roll):
    """Find the furthest the gripper reaches from the base axis at height ``z``."""
    low, high = 0.0, 0.40
    for _ in range(60):
        middle = (low + high) / 2.0
        if inverse_kinematics(middle + BASE_OFFSET_X, 0.0, z, pitch, roll) is not None:
            low = middle
        else:
            high = middle
    return low


def collect_points(layout, default_pitch, default_roll):
    """Flatten a layout into named points, including the hover above each grasp.

    Each point is checked at the angles it carries - a point taught in the
    jog's joint mode records its own pitch_deg/roll_deg, and that is the pose
    the task manager will replay it at - falling back to the workspace
    defaults when it carries none.
    """
    workspace = layout['workspace']
    hover_height = float(workspace['hover_height'])

    points = [
        ('home', workspace['home'], False),
        ('transit', workspace['transit'], False),
    ]
    # A station has only the places it does: the warehouse arm has no bins and
    # the destination arm has no warehouse, so neither section is required.
    warehouse = (layout.get('warehouse') or {}).get('pick_points') or []
    for index, point in enumerate(warehouse):
        points.append((f'warehouse[{index}]', point, True))
    for entry in layout.get('bins') or []:
        points.append((f"{entry['id']}.place", entry['place'], True))
    # The one point both arms of a relay have to reach, and the one worth
    # checking hardest: it is measured off a docked carrier rather than off
    # something bolted down.
    carrier = (layout.get('carrier') or {}).get('transfer')
    if carrier is not None:
        points.append(('carrier', carrier, True))

    resolved = []
    for name, point, has_hover in points:
        pitch = (
            math.radians(float(point['pitch_deg']))
            if 'pitch_deg' in point else default_pitch
        )
        roll = (
            math.radians(float(point['roll_deg']))
            if 'roll_deg' in point else default_roll
        )
        resolved.append((name, point['x'], point['y'], point['z'], pitch, roll))
        if has_hover:
            hover = float(point.get('hover', hover_height))
            resolved.append((
                f'{name}.hover',
                point['x'], point['y'], point['z'] + hover,
                pitch, roll,
            ))
        retreat = point.get('retreat')
        if retreat is not None:
            merged = {**point, **retreat}
            r_pitch = (
                math.radians(float(merged['pitch_deg']))
                if 'pitch_deg' in merged else default_pitch
            )
            r_roll = (
                math.radians(float(merged['roll_deg']))
                if 'roll_deg' in merged else default_roll
            )
            resolved.append((
                f'{name}.retreat',
                point['x'], point['y'], float(retreat['z']),
                r_pitch, r_roll,
            ))
    return resolved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('layout', help='path to stock_layout.yaml')
    args = parser.parse_args()

    if not os.path.exists(args.layout):
        print(f'no such layout file: {args.layout}', file=sys.stderr)
        return 2

    with open(args.layout) as handle:
        layout = yaml.safe_load(handle)

    workspace = layout['workspace']
    pitch = math.radians(float(workspace['approach_pitch_deg']))
    roll = math.radians(float(workspace['approach_roll_deg']))
    limits = workspace['limits']

    print(f'approach pitch {workspace["approach_pitch_deg"]}deg, '
          f'roll {workspace["approach_roll_deg"]}deg\n')
    print('reach envelope (radius from the base axis):')
    for z in (0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.21):
        reach = max_radial(z, pitch, roll)
        print(f'   z={z:.2f} m   up to {reach:.3f} m   '
              f'(stay under {reach * MARGIN:.3f} m)')
    print()

    failures = []
    tight = []
    for name, x, y, z, point_pitch, point_roll in collect_points(layout, pitch, roll):
        problems = [
            f'{axis}={value:.3f} outside {limits[axis]}'
            for axis, value in (('x', x), ('y', y), ('z', z))
            if not limits[axis][0] <= value <= limits[axis][1]
        ]

        joints = inverse_kinematics(x, y, z, point_pitch, point_roll)
        radial = math.hypot(x - BASE_OFFSET_X, y)
        envelope = max_radial(z, point_pitch, point_roll)
        if joints is None:
            problems.append(
                f'out of reach: needs {radial:.3f} m at z={z:.3f}, '
                f'arm reaches {envelope:.3f} m'
            )

        if problems:
            failures.append((name, problems))
            print(f'FAIL  {name:22s} ({x:.3f}, {y:.3f}, {z:.3f})')
            for problem in problems:
                print(f'         {problem}')
            continue

        error = math.dist((x, y, z), forward_kinematics(joints))
        angles = ''
        if (point_pitch, point_roll) != (pitch, roll):
            angles = (f'  at {math.degrees(point_pitch):.0f}/'
                      f'{math.degrees(point_roll):.0f}deg')
        note = ''
        if radial > envelope * MARGIN:
            note = f'  <- only {(envelope - radial) * 1000:.0f} mm of margin left'
            tight.append(name)
        print(f'ok    {name:22s} ({x:.3f}, {y:.3f}, {z:.3f})  '
              f'radial {radial:.3f}  err {error * 1e6:.1f} um{angles}{note}')

    print()
    if failures:
        print(f'{len(failures)} point(s) unreachable - fix the layout before running')
        return 1
    if tight:
        print(f'all points reachable, but close to the limit: {", ".join(tight)}')
        return 0
    print('all points reachable with margin')
    return 0


if __name__ == '__main__':
    sys.exit(main())
