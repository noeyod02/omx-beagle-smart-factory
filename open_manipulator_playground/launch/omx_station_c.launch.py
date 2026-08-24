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

"""Bring up station C's arm alone, on whichever machine its board is plugged.

Station C is the second arm at station B's bay: it faces station B's arm
across the six-cell box and owns cells 4-6, while B keeps 1-3.  It runs on
its own computer (PC2), so it gets its own launch instead of a seat in
omx_stock_relay.launch.py - the relay on PC1 reaches it over the network by
namespace, exactly as it reaches the others.

No task manager yet: that needs stock_layout_c.yaml, which needs the carrier
and cell coordinates taught first.  Bring this up, then teach with::

    ros2 run open_manipulator_playground stock_joint_jog.py \\
        --arm-action /station_c/arm_controller/follow_joint_trajectory \\
        --gripper-action /station_c/gripper_controller/gripper_cmd \\
        --joint-states /station_c/joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        # The by-id path of the board on PC2. Stable across reboots where
        # /dev/ttyACM0 is not. Re-read 2026-08-24: the board plugged at PC2
        # is 1825F0CB now, not the E314414A read on 2026-08-07 - the arms
        # were shuffled when station A's was replaced. If the 2026-08-21
        # taught points miss, suspect the hardware swap first (A's needed
        # its pick lowered 10 mm after the same shuffle).
        DeclareLaunchArgument(
            'port_c',
            default_value=(
                '/dev/serial/by-id/'
                'usb-ROBOTIS_OpenRB-150_1825F0CB5157375037202020FF0D102A-if00'
            ),
            description="Serial port of station C's arm",
        ),
        # omx_f.launch.py reads these through event handlers that fire after
        # the namespaced group is gone - they must exist at the top level
        # (same gotcha as omx_stock_relay.launch.py).
        DeclareLaunchArgument(
            'init_position',
            default_value='false',
            description='Drive the arm to its stored initial pose on startup',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to bring up rviz',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Run as mock hardware that mirrors its commands',
        ),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omx_f.launch.py',
            ])
        ]),
        launch_arguments={
            'port_name': LaunchConfiguration('port_c'),
            'init_position': LaunchConfiguration('init_position'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
        }.items(),
    )

    station_c = GroupAction([PushRosNamespace('/station_c'), bringup])

    return LaunchDescription(declared_arguments + [station_c])
