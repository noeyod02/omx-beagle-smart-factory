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

"""The file of named points, and the one place its format is decided.

scripts/stock_jog.py writes points into it as they are taught, one line each,
so a teaching session leaves something a layout can be filled in from and a
later session can add to.  The format, the name rules and the default path live
here so that whatever reads the file next agrees with what wrote it.

The file is plain YAML and can be edited by hand:

    points:
      - {name: tray_a, x: 0.054, y: -0.175, z: 0.155, pitch_deg: -90.0, roll_deg: 0.0}

Every point carries the approach angles it was taught at, not just x/y/z.  The
same coordinate reached with the gripper pointing down and reached with it
pointing forwards are different arm poses, and only one of them was the pose
that looked right when it was taught.

Coordinates are in the arm's own base frame, so a file taught at one station of
a relay is meaningless at the other.  Give each station its own file.
"""

import os
import re

import yaml

# Default home for the file.  ~/.ros exists wherever ROS has been run and is
# not inside a workspace, so the points survive a rebuild of the package.
DEFAULT_POINTS_FILE = os.path.join(os.path.expanduser('~'), '.ros', 'stock_points.yaml')

# Names travel through YAML keys, a JSON request and an HTML page, so they are
# kept to characters that need no escaping anywhere along the way.
NAME_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,39}$')

HEADER = """\
# Points taught with scripts/stock_jog.py, in the arm's own base frame, metres.
#
# Safe to edit by hand, but rewritten whenever a point is taught, so comments
# added below this line will not survive that.
"""


def check_name(name):
    """Return None if the name is usable, or the reason it is not."""
    if not name:
        return 'a point needs a name'
    if not NAME_PATTERN.match(name):
        return (
            f'{name!r} cannot be a name: use letters, digits, _ . and -, '
            f'starting with a letter or digit, up to 40 characters'
        )
    return None


def make_point(name, x, y, z, pitch_deg, roll_deg):
    """Build one entry, rounded to the tenth of a millimetre the arm can hold."""
    return {
        'name': name,
        'x': round(float(x), 4),
        'y': round(float(y), 4),
        'z': round(float(z), 4),
        'pitch_deg': round(float(pitch_deg), 1),
        'roll_deg': round(float(roll_deg), 1),
    }


def load_points(path):
    """Read the points file.  Returns (points, problems).

    Malformed entries are dropped and described in ``problems`` rather than
    raising, because one bad line typed into the file by hand should not take
    the whole list - or the page serving it - down with it.  A missing file is
    not a problem: it is what the first teaching session starts from.
    """
    problems = []
    if not os.path.exists(path):
        return [], problems

    try:
        with open(path) as handle:
            document = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError) as error:
        return [], [f'cannot read {path}: {error}']

    if not isinstance(document, dict):
        return [], [f'{path} should hold a mapping with a points: list']

    points = []
    seen = set()
    for index, entry in enumerate(document.get('points') or []):
        if not isinstance(entry, dict):
            problems.append(f'point {index + 1} is not a mapping')
            continue
        name = str(entry.get('name', '')).strip()
        reason = check_name(name)
        if reason is not None:
            problems.append(f'point {index + 1}: {reason}')
            continue
        if name in seen:
            problems.append(f'{name!r} appears more than once; keeping the first')
            continue
        try:
            point = make_point(
                name,
                entry['x'],
                entry['y'],
                entry['z'],
                # Taught before the angles were recorded, or added by hand:
                # straight down is what every point in this cell has used.
                entry.get('pitch_deg', -90.0),
                entry.get('roll_deg', 0.0),
            )
        except (KeyError, TypeError, ValueError) as error:
            problems.append(f'{name!r} has no usable coordinate ({error})')
            continue
        seen.add(name)
        points.append(point)

    return points, problems


def save_point(path, point):
    """Add or replace one point.  Returns 'added' or 'replaced'.

    The whole file is rewritten each time, through a temporary file, so a
    session killed mid-write leaves the previous list intact rather than half
    a file - the points taught before this one are the ones worth protecting.
    """
    points, _ = load_points(path)
    outcome = 'added'
    for index, existing in enumerate(points):
        if existing['name'] == point['name']:
            points[index] = point
            outcome = 'replaced'
            break
    else:
        points.append(point)

    write_points(path, points)
    return outcome


def write_points(path, points):
    """Write the whole list, leaving the old file untouched if this fails."""
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    # Written a line at a time rather than dumped, so each point reads as one
    # row - the same shape the layout files use, and the shape a person editing
    # this file by hand would write. Safe to build by hand because names are
    # checked against NAME_PATTERN and everything else is a number.
    lines = [HEADER, 'points:\n']
    for point in points:
        lines.append(
            f"  - {{name: {point['name']}, "
            f"x: {point['x']:.4f}, y: {point['y']:.4f}, z: {point['z']:.4f}, "
            f"pitch_deg: {point['pitch_deg']:.1f}, roll_deg: {point['roll_deg']:.1f}}}\n"
        )

    temporary = f'{path}.new'
    with open(temporary, 'w') as handle:
        handle.writelines(lines)
    os.replace(temporary, path)
