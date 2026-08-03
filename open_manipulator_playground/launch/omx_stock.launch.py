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

"""Bring up the stock replenishment demo: arm, camera, monitor and task manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_layout = PathJoinSubstitution([
        FindPackageShare('open_manipulator_playground'),
        'config',
        'stock_layout.yaml',
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            'layout_file',
            default_value=default_layout,
            description='Bin, warehouse and workspace layout for the demo',
        ),
        DeclareLaunchArgument(
            'start_robot',
            default_value='true',
            description='Whether to bring up the arm as part of this launch',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Whether to bring up the USB camera as part of this launch',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='Serial port the arm is connected to',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device of the fixed camera watching the bins',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera1',
            description='Namespace the camera publishes under',
        ),
        DeclareLaunchArgument(
            'backend',
            default_value='reference',
            description="Bin detector to use: 'reference' or 'yolo'",
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Ultralytics weights, required when backend is yolo',
        ),
        DeclareLaunchArgument(
            'reference_path',
            default_value='/tmp/stock_reference.png',
            description='Where the empty-bin reference photo is kept',
        ),
        DeclareLaunchArgument(
            'auto_refill',
            default_value='true',
            description='Refill automatically, instead of only on explicit request',
        ),
        DeclareLaunchArgument(
            'gripper_mode',
            default_value='action',
            description="'action' for the omx_f bringup, 'joint' for the AI follower",
        ),
    ]

    layout_file = LaunchConfiguration('layout_file')
    camera_name = LaunchConfiguration('camera_name')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omx_f.launch.py',
            ])
        ]),
        launch_arguments={
            'port_name': LaunchConfiguration('port_name'),
            'init_position': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_robot')),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'camera_usb_cam.launch.py',
            ])
        ]),
        launch_arguments={
            'name': camera_name,
            'video_device': LaunchConfiguration('video_device'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    monitor = Node(
        package='open_manipulator_playground',
        executable='stock_monitor_node.py',
        name='stock_monitor',
        output='screen',
        parameters=[{
            'layout_file': layout_file,
            'image_topic': [camera_name, '/image_raw'],
            'backend': LaunchConfiguration('backend'),
            'model_path': LaunchConfiguration('model_path'),
            'reference_path': LaunchConfiguration('reference_path'),
        }],
    )

    task_manager = Node(
        package='open_manipulator_playground',
        executable='stock_task_manager_node.py',
        name='stock_task_manager',
        output='screen',
        parameters=[{
            'layout_file': layout_file,
            'auto_refill': LaunchConfiguration('auto_refill'),
            'gripper_mode': LaunchConfiguration('gripper_mode'),
        }],
    )

    return LaunchDescription(declared_arguments + [robot, camera, monitor, task_manager])
