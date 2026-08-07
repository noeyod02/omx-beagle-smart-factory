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

"""Bring up the whole relay: two arms, the carrier, the camera and the sequencer.

Station A loads a part onto the Beagle, the Beagle drives to station B, and
station B's arm places it in the empty bin.  Each arm runs in its own
namespace, so the two controller managers, their controllers and their task
managers do not collide::

    /station_a/arm_controller/...   /station_b/arm_controller/...

Both arms are OMX-F and both boards enumerate as an OpenRB-150, so the ports
must be given by their stable by-id paths - /dev/ttyACM0 and /dev/ttyACM2 swap
between boots.  List them with::

    ls -l /dev/serial/by-id/

Then launch, passing the two ports and the camera that watches station B::

    ros2 launch open_manipulator_playground omx_stock_relay.launch.py \\
        port_a:=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_<A>-if00 \\
        port_b:=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_<B>-if00

The Beagle bridge starts in dry_run, driving the course package's simulator
rather than the real carrier, so the sequencing can be watched through before
anything rolls.  Pass beagle_dry_run:=false once the routes are measured.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _arm(namespace, port, layout, gripper_mode, start_robot, use_mock_hardware):
    """One station: the arm's bringup and its task manager, in a namespace.

    The bringup is namespaced rather than prefixed. Prefixing renames the
    joints, which the task manager's closed-form kinematics address by name;
    namespacing leaves them alone and separates the two arms where it matters,
    on their topics and actions. The controller configuration is written under
    a ``/**`` wildcard, so it applies in either namespace unchanged.
    """
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omx_f.launch.py',
            ])
        ]),
        launch_arguments={
            'port_name': port,
            'init_position': LaunchConfiguration('init_position'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'use_mock_hardware': use_mock_hardware,
        }.items(),
        condition=IfCondition(start_robot),
    )

    task_manager = Node(
        package='open_manipulator_playground',
        executable='stock_task_manager_node.py',
        name='stock_task_manager',
        output='screen',
        parameters=[{
            'layout_file': layout,
            'gripper_mode': gripper_mode,
            # The relay is the only thing that gives these arms work: each one
            # reaches just one end of a job, so a bare bin name means nothing
            # here, and reacting to the camera directly would have an arm grasp
            # at a tray that may not have arrived.
            'auto_refill': False,
            'request_topic': '',
            # Relative names: this node runs inside the station's namespace, so
            # they resolve to that station's own controllers and topics. The
            # node's absolute defaults would reach across to the other arm.
            'arm_action': 'arm_controller/follow_joint_trajectory',
            'gripper_action': 'gripper_controller/gripper_cmd',
            'transfer_topic': 'stock/transfer',
            'state_topic': 'stock/task_state',
        }],
    )

    return GroupAction([PushRosNamespace(namespace), bringup, task_manager])


def generate_launch_description():
    package = FindPackageShare('open_manipulator_playground')
    default_layout_a = PathJoinSubstitution([package, 'config', 'stock_layout_a.yaml'])
    default_layout_b = PathJoinSubstitution([package, 'config', 'stock_layout_b.yaml'])
    default_route = PathJoinSubstitution([package, 'config', 'beagle_route.yaml'])

    declared_arguments = [
        DeclareLaunchArgument(
            'layout_a',
            default_value=default_layout_a,
            description="Station A's layout: warehouse and carrier coordinates",
        ),
        DeclareLaunchArgument(
            'layout_b',
            default_value=default_layout_b,
            description="Station B's layout: carrier and bin coordinates",
        ),
        DeclareLaunchArgument(
            'route_file',
            default_value=default_route,
            description='Routes the carrier drives between the two bays',
        ),
        # The by-id paths of the two boards in this cell, as read on
        # 2026-08-04. They are stable across reboots where /dev/ttyACM0 and
        # /dev/ttyACM2 are not, which is why they are spelled out in full.
        #
        # Which board drives which station is not something the boards can be
        # asked - both enumerate identically as an OpenRB-150 - so it was
        # established at the cell and written down here: 0B4ABF0E is the
        # warehouse arm, station A.
        #
        # Should the boards ever be swapped between stations, swapping these
        # two values is the whole fix. Nothing else in the relay refers to a
        # port: the stations are told apart by namespace.
        DeclareLaunchArgument(
            'port_a',
            default_value=(
                '/dev/serial/by-id/'
                'usb-ROBOTIS_OpenRB-150_0B4ABF0E5157375037202020FF102616-if00'
            ),
            description="Serial port of station A's arm, the warehouse side",
        ),
        DeclareLaunchArgument(
            'port_b',
            default_value=(
                '/dev/serial/by-id/'
                'usb-ROBOTIS_OpenRB-150_147E67EC5157375037202020FF11280D-if00'
            ),
            description="Serial port of station B's arm, the destination side",
        ),
        DeclareLaunchArgument(
            'start_robots',
            default_value='true',
            description='Whether to bring up the two arms as part of this launch',
        ),
        # omx_f.launch.py starts these two through event handlers, which fire
        # once the controller spawner exits - by which time the namespaced
        # group they were included from has gone out of scope. Declaring them
        # here leaves something for the handlers to read at that point;
        # without it the second station brings the whole launch down with
        # "launch configuration 'init_position' does not exist".
        DeclareLaunchArgument(
            'init_position',
            default_value='false',
            description='Drive the arms to their stored initial pose on startup',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether either arm brings up its own rviz',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description=(
                'Run both arms as mock hardware that mirrors its commands. '
                'Lets the whole relay be watched through with no arms and no '
                'carrier attached.'
            ),
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Whether to bring up the camera watching station B',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description="Video device of the camera watching station B's bins",
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
            'gripper_mode',
            default_value='action',
            description="'action' for the omx_f bringup, 'joint' for the AI follower",
        ),
        DeclareLaunchArgument(
            'beagle_dry_run',
            default_value='true',
            description='Drive the simulator instead of the real carrier',
        ),
        # The bridge is the one node in this launch that cannot run beside the
        # others. roboid, the library that drives the carrier over its dongle,
        # is installed in the container while the arms run on the host, so a
        # real carrier means starting the bridge over there by hand and turning
        # this off - otherwise the copy started here fails at connect and sits
        # in the graph publishing nothing, which reads exactly like a carrier
        # that will not answer.
        DeclareLaunchArgument(
            'start_beagle',
            default_value='true',
            description=(
                'Whether to start the carrier bridge here. Off when the bridge '
                'runs in the container, which is what a real carrier needs.'
            ),
        ),
        DeclareLaunchArgument(
            'start_monitor',
            default_value='true',
            description=(
                'Whether to bring up the bin monitor. Off when the camera is '
                'not aimed yet and refills are asked for by hand.'
            ),
        ),
        DeclareLaunchArgument(
            'start_arrival',
            default_value='false',
            description=(
                'Watch the station A bay with the carrier-trained YOLO and '
                'start the warehouse arm when the carrier is SEEN to arrive, '
                'not merely reported. Needs its own camera on the bay and '
                'ultralytics on the host.'
            ),
        ),
        DeclareLaunchArgument(
            'arrival_model_path',
            default_value=PathJoinSubstitution([
                package, 'config', 'models', 'beagle_item_640.pt',
            ]),
            description='YOLO weights that know the carrier (class "beagle")',
        ),
        DeclareLaunchArgument(
            'arrival_video_device',
            default_value='0',
            description='OpenCV index of the camera watching the station A bay',
        ),
        DeclareLaunchArgument(
            'auto_relay',
            default_value='false',
            description=(
                'Start a relay the moment a bin reads empty. Left off: the '
                'monitor only reports, and a refill is asked for by hand.'
            ),
        ),
    ]

    layout_b = LaunchConfiguration('layout_b')
    camera_name = LaunchConfiguration('camera_name')
    gripper_mode = LaunchConfiguration('gripper_mode')
    start_robots = LaunchConfiguration('start_robots')

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    station_a = _arm(
        '/station_a', LaunchConfiguration('port_a'),
        LaunchConfiguration('layout_a'), gripper_mode, start_robots,
        use_mock_hardware,
    )
    station_b = _arm(
        '/station_b', LaunchConfiguration('port_b'),
        layout_b, gripper_mode, start_robots, use_mock_hardware,
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

    # The bins live at station B, so that is the arm whose motion blinds the
    # camera and the layout whose regions the monitor reads.
    monitor = Node(
        package='open_manipulator_playground',
        executable='stock_monitor_node.py',
        name='stock_monitor',
        output='screen',
        parameters=[{
            'layout_file': layout_b,
            'image_topic': [camera_name, '/image_raw'],
            'task_state_topic': '/station_b/stock/task_state',
            'backend': LaunchConfiguration('backend'),
            'model_path': LaunchConfiguration('model_path'),
            'reference_path': LaunchConfiguration('reference_path'),
        }],
        condition=IfCondition(LaunchConfiguration('start_monitor')),
    )

    beagle = Node(
        package='open_manipulator_playground',
        executable='beagle_bridge_node.py',
        name='beagle_bridge',
        output='screen',
        parameters=[{
            'route_file': LaunchConfiguration('route_file'),
            'dry_run': LaunchConfiguration('beagle_dry_run'),
        }],
        condition=IfCondition(LaunchConfiguration('start_beagle')),
    )

    relay = Node(
        package='open_manipulator_playground',
        executable='stock_relay_node.py',
        name='stock_relay',
        output='screen',
        parameters=[{
            'station_a_ns': '/station_a',
            'station_b_ns': '/station_b',
        }],
    )

    # Sees the carrier arrive at station A's bay and starts the warehouse
    # arm. Off by default: it needs a camera aimed at the bay and the
    # carrier-trained weights, neither of which this launch can conjure.
    arrival = Node(
        package='open_manipulator_playground',
        executable='stock_arrival_node.py',
        name='stock_arrival',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('arrival_model_path'),
            'video_device': LaunchConfiguration('arrival_video_device'),
        }],
        condition=IfCondition(LaunchConfiguration('start_arrival')),
    )

    # Turns a stable empty reading straight into a relay. Off by default: this
    # is the point the approval gate belongs at, and until there is one a
    # person asking for the refill is the gate.
    auto_starter = Node(
        package='open_manipulator_playground',
        executable='stock_auto_request.py',
        name='stock_auto_request',
        output='screen',
        parameters=[{'relay_state_topic': '/stock/relay_state'}],
        condition=IfCondition(LaunchConfiguration('auto_relay')),
    )

    return LaunchDescription(
        declared_arguments
        + [station_a, station_b, camera, monitor, beagle, relay, arrival,
           auto_starter]
    )
