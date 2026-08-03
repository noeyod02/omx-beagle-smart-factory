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

"""Report where the gripper is, so bin and warehouse positions can be measured.

Run this with the arm's torque disabled, move the gripper by hand to a spot you
want the robot to reach, and copy the printed coordinates into the ``place`` or
``pick_points`` entries of config/stock_layout.yaml.

    ros2 run open_manipulator_playground stock_teach.py

Positions are printed continuously, and every reading is also written to
stock_teach_log.txt in the working directory so nothing is lost off the top of
the terminal.
"""

import math

from omx_kinematics import forward_kinematics, JOINT_NAMES
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class StockTeach(Node):
    """Prints the end effector position derived from the measured joint angles."""

    def __init__(self):
        super().__init__('stock_teach')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('log_path', 'stock_teach_log.txt')
        self.declare_parameter('move_threshold', 0.003)

        self.last_position = None
        self.last_print = 0.0
        self.interval = 1.0 / max(1e-3, float(self.get_parameter('rate_hz').value))
        self.log_path = self.get_parameter('log_path').value

        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._on_joint_state,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Move the gripper by hand (torque off) and read off the coordinates.'
        )

    def _on_joint_state(self, msg):
        """Run forward kinematics on each joint state update."""
        try:
            joints = [msg.position[msg.name.index(name)] for name in JOINT_NAMES]
        except (ValueError, IndexError):
            self.get_logger().warn(
                f'/joint_states is missing one of {JOINT_NAMES}',
                throttle_duration_sec=5.0,
            )
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_print < self.interval:
            return
        self.last_print = now

        x, y, z = forward_kinematics(joints)

        # Only report once the arm has actually been moved somewhere new.
        threshold = float(self.get_parameter('move_threshold').value)
        if self.last_position is not None:
            if math.dist((x, y, z), self.last_position) < threshold:
                return
        self.last_position = (x, y, z)

        line = (
            f'{{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}   '
            f'joints(deg) ' + ' '.join(f'{math.degrees(q):7.2f}' for q in joints)
        )
        self.get_logger().info(line)
        try:
            with open(self.log_path, 'a') as handle:
                handle.write(line + '\n')
        except OSError as exc:
            self.get_logger().warn(
                f'Could not write {self.log_path}: {exc}', throttle_duration_sec=10.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = StockTeach()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
