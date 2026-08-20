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

"""Closed-form kinematics for OMX-F.

The arm is a yaw joint (joint1) followed by three pitch joints (joint2, joint3,
joint4) and a roll joint (joint5).  Because every pitch axis is parallel, the
end effector position and its approach pitch are fully determined by joint1..4,
which makes a closed-form solution possible: joint1 comes straight from the
target bearing and joint2..4 reduce to a planar two-link problem once the wrist
point is subtracted from the target.

Link parameters mirror open_manipulator_description/urdf/omx_f/omx_f_arm.urdf.xacro.
Run this file directly to check the inverse solution against forward kinematics.
"""

import math

# Joint origins, taken from the URDF (metres).
JOINT1_XYZ = (-0.01125, 0.0, 0.034)
JOINT2_XYZ = (0.0, 0.0, 0.0635)
JOINT3_XYZ = (0.0415, 0.0, 0.11315)
JOINT4_XYZ = (0.162, 0.0, 0.0)
JOINT5_XYZ = (0.0287, 0.0, 0.0)
END_EFFECTOR_XYZ = (0.09193, -0.0016, 0.0)

# Offset of the joint1 rotation axis from the base origin.
BASE_OFFSET_X = JOINT1_XYZ[0]
# Height of the joint2 axis above the base origin.
SHOULDER_Z = JOINT1_XYZ[2] + JOINT2_XYZ[2]

# joint2 -> joint3 is a dog-leg, so it is described by a length and the angle it
# makes with the horizontal when joint2 is at zero.
UPPER_ARM_LEN = math.hypot(JOINT3_XYZ[0], JOINT3_XYZ[2])
UPPER_ARM_BIAS = math.atan2(JOINT3_XYZ[2], JOINT3_XYZ[0])
# joint3 -> joint4.
FOREARM_LEN = JOINT4_XYZ[0]
# joint4 -> end effector, collapsed because joint5 rolls about that same axis.
WRIST_LEN = JOINT5_XYZ[0] + END_EFFECTOR_XYZ[0]
# Lateral offset of the end effector; joint5 swings it around the approach axis.
WRIST_LATERAL = END_EFFECTOR_XYZ[1]

REACH_MIN = abs(UPPER_ARM_LEN - FOREARM_LEN)
REACH_MAX = UPPER_ARM_LEN + FOREARM_LEN

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def forward_kinematics(joints):
    """Return the end effector position for ``joints`` = [j1..j5] in radians."""
    j1, j2, j3, j4, j5 = joints

    # Planar chain in the (radial, vertical) plane of the arm.
    s2 = -j2
    s3 = -(j2 + j3)
    s4 = -(j2 + j3 + j4)

    radial = UPPER_ARM_LEN * math.cos(UPPER_ARM_BIAS + s2)
    vertical = UPPER_ARM_LEN * math.sin(UPPER_ARM_BIAS + s2)
    radial += FOREARM_LEN * math.cos(s3)
    vertical += FOREARM_LEN * math.sin(s3)
    radial += WRIST_LEN * math.cos(s4)
    vertical += WRIST_LEN * math.sin(s4)

    # joint5 rolls the lateral end effector offset about the approach axis.
    lateral = WRIST_LATERAL * math.cos(j5)
    radial_from_roll = -WRIST_LATERAL * math.sin(j5) * math.sin(s4)
    vertical_from_roll = WRIST_LATERAL * math.sin(j5) * math.cos(s4)

    radial += radial_from_roll
    vertical += vertical_from_roll

    x = BASE_OFFSET_X + radial * math.cos(j1) - lateral * math.sin(j1)
    y = radial * math.sin(j1) + lateral * math.cos(j1)
    z = SHOULDER_Z + vertical

    return (x, y, z)


def inverse_kinematics(x, y, z, pitch=-math.pi / 2.0, roll=0.0, elbow_up=True):
    """Solve for [j1..j5] that puts the end effector at ``(x, y, z)``.

    ``pitch`` is the approach angle of the end effector in the vertical plane:
    ``-pi/2`` points the gripper straight down, ``0`` points it horizontally
    away from the base.  ``roll`` maps directly onto joint5.

    Returns ``None`` when the target is out of reach.
    """
    # joint1 aims the arm at the target.  The end effector sits 1.6 mm off the
    # arm plane and joint5 swings that offset around, so joint1 is not simply
    # the target bearing: it has to place the target at exactly that lateral
    # distance from the plane.  Rotating the target into the arm frame gives
    # y*cos(j1) - dx*sin(j1) = lateral, which solves in closed form.
    dx = x - BASE_OFFSET_X
    lateral = WRIST_LATERAL * math.cos(roll)
    bearing_len = math.hypot(dx, y)
    if bearing_len < abs(lateral):
        return None
    phase = math.atan2(dx, y)
    offset = math.acos(max(-1.0, min(1.0, lateral / bearing_len)))
    # Two branches; keep the one that leaves the arm reaching forwards.
    j1 = offset - phase
    if dx * math.cos(j1) + y * math.sin(j1) < 0.0:
        j1 = -offset - phase
    # offset - phase can land a turn outside [-pi, pi) for targets behind the
    # base, and a controller told to go to +259 deg swings the long way round
    # rather than to the equivalent -101 deg.
    j1 = wrap_to_pi(j1)

    # Target expressed in the arm plane, with the joint5 roll contribution removed.
    radial = dx * math.cos(j1) + y * math.sin(j1)
    vertical = z - SHOULDER_Z
    roll_offset = WRIST_LATERAL * math.sin(roll)
    radial += roll_offset * math.sin(pitch)
    vertical -= roll_offset * math.cos(pitch)

    # Back off along the approach axis to reach the wrist point.
    wrist_r = radial - WRIST_LEN * math.cos(pitch)
    wrist_z = vertical - WRIST_LEN * math.sin(pitch)

    distance = math.hypot(wrist_r, wrist_z)
    if distance > REACH_MAX or distance < REACH_MIN:
        return None

    cos_elbow = (distance ** 2 - UPPER_ARM_LEN ** 2 - FOREARM_LEN ** 2) / (
        2.0 * UPPER_ARM_LEN * FOREARM_LEN
    )
    # Guard against the target sitting exactly on the reach boundary.
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow = math.acos(cos_elbow)
    if elbow_up:
        elbow = -elbow

    bearing = math.atan2(wrist_z, wrist_r)
    upper_dir = bearing - math.atan2(
        FOREARM_LEN * math.sin(elbow),
        UPPER_ARM_LEN + FOREARM_LEN * math.cos(elbow),
    )
    forearm_dir = upper_dir + elbow

    s2 = upper_dir - UPPER_ARM_BIAS
    s3 = forearm_dir
    s4 = pitch

    j2 = -s2
    j3 = s2 - s3
    j4 = s3 - s4

    return [j1, j2, j3, j4, roll]


def wrap_to_pi(angle):
    """Wrap ``angle`` into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _self_test():
    """Check the inverse solution by feeding it back through forward kinematics."""
    import itertools

    worst = 0.0
    checked = 0
    skipped = 0
    for x, y, z, pitch, roll in itertools.product(
        (0.10, 0.16, 0.22, 0.28),
        (-0.12, -0.04, 0.0, 0.07, 0.15),
        (0.02, 0.08, 0.15, 0.24),
        (-math.pi / 2.0, -math.pi / 3.0, -math.pi / 6.0, 0.0),
        (0.0, math.pi / 4.0, -math.pi / 2.0),
    ):
        joints = inverse_kinematics(x, y, z, pitch=pitch, roll=roll)
        if joints is None:
            skipped += 1
            continue
        fx, fy, fz = forward_kinematics(joints)
        error = math.dist((x, y, z), (fx, fy, fz))
        worst = max(worst, error)
        checked += 1

    print(f'reach: {REACH_MIN * 1000:.1f} mm .. {REACH_MAX * 1000:.1f} mm (wrist point)')
    print(f'upper arm {UPPER_ARM_LEN * 1000:.2f} mm, bias {math.degrees(UPPER_ARM_BIAS):.2f} deg')
    print(f'forearm {FOREARM_LEN * 1000:.2f} mm, wrist {WRIST_LEN * 1000:.2f} mm')
    print(f'solved {checked} poses, {skipped} out of reach')
    print(f'worst position error: {worst * 1e6:.3f} um')

    assert checked > 0, 'no pose was solvable - check the link parameters'
    assert worst < 1e-9, f'inverse kinematics disagrees with forward kinematics ({worst} m)'
    print('OK')


if __name__ == '__main__':
    _self_test()
