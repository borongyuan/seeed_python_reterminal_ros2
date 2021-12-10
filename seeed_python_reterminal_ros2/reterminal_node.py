# Copyright 2021 Borong Yuan
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

import rclpy
import seeed_python_reterminal.core as rt
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, ColorRGBA


class ReTerminalNode(Node):
    def __init__(self):
        super().__init__('reterminal_node')

        self.sta_led_sub = self.create_subscription(
            ColorRGBA, 'sta_led', self.sta_led_cb, qos_profile_sensor_data)
        self.usr_led_sub = self.create_subscription(
            ColorRGBA, 'usr_led', self.usr_led_cb, qos_profile_sensor_data)
        self.buzzer_sub = self.create_subscription(
            Bool, 'buzzer', self.buzzer_cb, qos_profile_sensor_data)

    def sta_led_cb(self, msg):
        rt.sta_led_red = bool(msg.r)
        rt.sta_led_green = bool(msg.g)

    def usr_led_cb(self, msg):
        rt.usr_led = bool(msg.g)

    def buzzer_cb(self, msg):
        rt.buzzer = msg.data


def main(args=None):
    rclpy.init(args=args)

    rt_node = ReTerminalNode()

    rclpy.spin(rt_node)

    rt_node.destroy_node()
    rclpy.shutdown()
