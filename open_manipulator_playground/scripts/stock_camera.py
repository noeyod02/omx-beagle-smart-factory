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

"""Turn a place in the camera image into a place the arm can be sent to.

The monitor never needed this: it only names the bin that looks empty, and the
arm goes to a coordinate somebody taught it.  Picking a part the camera found
is a different question - the answer has to be a coordinate, and no amount of
detection accuracy supplies one on its own.

What makes it tractable is that the parts lie on a flat table and the camera is
fixed above it.  Two planes viewed through a pinhole are related by a single
3x3 homography, so four points whose position is known both in pixels and in
robot coordinates pin down the mapping for the whole table.  Collect them with
scripts/stock_hand_eye.py.

The mapping is only valid on the plane it was measured on.  A part standing on
top of another is nearer the camera than the table, and reads as though it were
somewhere else entirely - the further from the image centre, the worse.  This
is the same reason parts are not counted by stacking them.
"""

import math

import numpy as np


class NotCalibrated(Exception):
    """Raised when a pixel is asked about before the mapping is known."""


def solve_homography(pixels, points):
    """Fit the pixel -> table mapping, and say how well it fits.

    ``pixels`` are (u, v) in the image, ``points`` the (x, y) in metres of the
    same physical spots in the arm's frame.  Returns ``(H, errors_mm)``, where
    ``errors_mm[i]`` is how far the fitted mapping puts pixel i from where the
    arm actually measured it.

    The errors are the useful part.  A homography can be fitted through any
    four points and will pass through them exactly, so a clean fit proves
    nothing until there is a fifth point it was not fitted to - which is why
    the tool asks for more than four and reports each one.
    """
    if len(pixels) != len(points):
        raise ValueError(f'{len(pixels)} pixels but {len(points)} robot points')
    if len(pixels) < 4:
        raise ValueError(f'need at least 4 correspondences, got {len(pixels)}')

    src = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    dst = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)

    import cv2
    # Plain least squares, not RANSAC: these points were placed deliberately
    # and there are few of them, so a point that disagrees is a mistake to be
    # shown to the operator rather than an outlier to be quietly discarded.
    matrix, _ = cv2.findHomography(src, dst, method=0)
    if matrix is None:
        raise ValueError(
            'the points do not determine a mapping. Three or more lying on a '
            'straight line will do this; spread them over the working area.'
        )

    errors = []
    for (u, v), (x, y) in zip(pixels, points):
        fx, fy = apply_homography(matrix, u, v)
        errors.append(math.hypot(fx - x, fy - y) * 1000.0)
    return matrix, errors


def apply_homography(matrix, u, v):
    """Map one pixel to a point on the table, in metres."""
    x, y, w = np.asarray(matrix, dtype=np.float64) @ np.array([u, v, 1.0])
    if abs(w) < 1e-12:
        raise NotCalibrated(
            f'pixel ({u}, {v}) maps to the horizon, which is not a place on '
            f'the table. The calibration is wrong or the pixel is far outside '
            f'the area it was measured over.'
        )
    return x / w, y / w


def load_homography(layout):
    """Read the mapping out of a parsed layout file.

    Raises rather than returning None: every caller needs the mapping to do its
    job, and a missing one is a setup step that was skipped, not a condition to
    carry on without.
    """
    camera = layout.get('camera') or {}
    rows = camera.get('homography')
    if not rows:
        raise NotCalibrated(
            'this layout has no camera.homography. Measure it with '
            'scripts/stock_hand_eye.py before picking by camera.'
        )
    matrix = np.asarray(rows, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise NotCalibrated(f'camera.homography must be 3x3, got {matrix.shape}')
    return matrix


def format_homography(matrix, errors_mm=None, pick_z=None):
    """Print the mapping as the layout block it belongs in.

    Written out rather than edited in, the same way the jogger hands back
    coordinates: the operator sees what is about to be trusted, and the file
    keeps its comments.
    """
    lines = ['camera:']
    if errors_mm is not None:
        worst = max(errors_mm)
        lines.append(f'  # Fitted over {len(errors_mm)} points, worst miss '
                     f'{worst:.1f} mm, mean {sum(errors_mm) / len(errors_mm):.1f} mm.')
        lines.append('  # Measured with scripts/stock_hand_eye.py; valid only on the')
        lines.append('  # table plane the points were taken on.')
    if pick_z is not None:
        lines.append('  # Height the gripper closes at on this plane.')
        lines.append(f'  pick_z: {pick_z:.3f}')
    lines.append('  homography:')
    for row in np.asarray(matrix, dtype=np.float64):
        cells = ', '.join(f'{value: .8e}' for value in row)
        lines.append(f'    - [{cells}]')
    return '\n'.join(lines)
