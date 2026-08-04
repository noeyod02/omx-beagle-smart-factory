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

"""Turn an empty bin into a relay request, with nobody in the loop.

The monitor only reports what it sees and the relay only moves when asked, so
on their own the cell watches a bin go empty and does nothing about it.  This
node closes that gap for unattended running: one stably empty bin becomes one
refill request, and no further request goes out until the relay is idle again.

It is deliberately the smallest possible piece, because it is the piece that
gets replaced.  The point of the design is that a person approves the job
before the arms move; this node is what stands in until that gate exists, and
it is off unless the launch was given ``auto_relay:=true``.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

RELAY_IDLE = 'idle'


class StockAutoRequest(Node):
    """Asks the relay to refill whichever bin has read empty."""

    def __init__(self):
        super().__init__('stock_auto_request')

        self.declare_parameter('status_topic', '/stock/status')
        self.declare_parameter('relay_state_topic', '/stock/relay_state')
        self.declare_parameter('request_topic', '/stock/refill_request')
        # Seconds to leave between requests, on top of waiting for the relay to
        # go idle: a bin the arm has just filled needs a moment to read filled.
        self.declare_parameter('cooldown_sec', 10.0)

        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.relay_state = None
        self.quiet_until = 0.0

        self.request_pub = self.create_publisher(
            String, self.get_parameter('request_topic').value, 10
        )
        self.create_subscription(
            String, self.get_parameter('status_topic').value, self._on_status, 10
        )
        self.create_subscription(
            String,
            self.get_parameter('relay_state_topic').value,
            self._on_relay_state,
            10,
        )

        self.get_logger().warn(
            'Running unattended: an empty bin will start the arms with nobody '
            'asked. Stop this node to put a person back in the loop.'
        )

    def _on_relay_state(self, msg):
        try:
            self.relay_state = json.loads(msg.data).get('state')
        except json.JSONDecodeError:
            pass

    def _on_status(self, msg):
        # Waiting for the relay to have reported at all matters: without it the
        # first empty reading would go out before the relay could say it was
        # busy, and be lost.
        if self.relay_state != RELAY_IDLE:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.quiet_until:
            return

        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        for entry in status.get('bins', []):
            if entry.get('state') == 'empty' and entry.get('stable'):
                bin_id = entry['id']
                self.quiet_until = now + self.cooldown_sec
                self.request_pub.publish(String(data=bin_id))
                self.get_logger().info(f'{bin_id} reads empty - asked for a refill')
                return


def main(args=None):
    rclpy.init(args=args)
    node = StockAutoRequest()
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
