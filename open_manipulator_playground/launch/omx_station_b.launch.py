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

"""Bring up station B's arm alone, with its task manager.

The relay launch opens both stations' ports, so it cannot come up when
station A's arm is absent - which it has been since 2026-08-10.  This is the
bringup for working on station B by itself: teaching cell points, replaying
them, or running single transfers by hand.  Everything sits in /station_b
exactly as the relay would put it, so a jog, a transfer or a later relay on
top of it addresses the same names::

    ros2 topic pub --once /station_b/stock/transfer std_msgs/String \\
        "{data: '{\\"from\\": \\"carrier\\", \\"to\\": \\"bin_a\\"}'}"

The task manager here still answers only transfers: a bare bin name on a
request topic belongs to the single-arm cell, not to a station that reaches
one end of a relay.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('open_manipulator_playground')
    declared_arguments = [
        DeclareLaunchArgument(
            'port_b',
            default_value=(
                '/dev/serial/by-id/'
                'usb-ROBOTIS_OpenRB-150_147E67EC5157375037202020FF11280D-if00'
            ),
            description="Serial port of station B's arm",
        ),
        DeclareLaunchArgument(
            'layout_b',
            default_value=PathJoinSubstitution(
                [package, 'config', 'stock_layout_b.yaml']
            ),
            description="Station B's layout: carrier and cell coordinates",
        ),
        DeclareLaunchArgument(
            'gripper_mode',
            default_value='action',
            description="'action' for the omx_f bringup, 'joint' for the AI follower",
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
            'port_name': LaunchConfiguration('port_b'),
            'init_position': LaunchConfiguration('init_position'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
        }.items(),
    )

    task_manager = Node(
        package='open_manipulator_playground',
        executable='stock_task_manager_node.py',
        name='stock_task_manager',
        output='screen',
        parameters=[{
            'layout_file': LaunchConfiguration('layout_b'),
            'gripper_mode': LaunchConfiguration('gripper_mode'),
            'auto_refill': False,
            'request_topic': '',
            # Relative names resolve inside /station_b; the node's absolute
            # defaults would address an arm outside any namespace.
            'arm_action': 'arm_controller/follow_joint_trajectory',
            'gripper_action': 'gripper_controller/gripper_cmd',
            'transfer_topic': 'stock/transfer',
            'state_topic': 'stock/task_state',
        }],
    )

    station_b = GroupAction(
        [PushRosNamespace('/station_b'), bringup, task_manager]
    )

    return LaunchDescription(declared_arguments + [station_b])
