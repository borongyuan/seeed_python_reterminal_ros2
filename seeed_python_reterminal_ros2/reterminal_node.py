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

import asyncio
import rclpy
import threading
import seeed_python_reterminal.core as rt
import seeed_python_reterminal.acceleration as rt_accel
import seeed_python_reterminal.button as rt_btn
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, ColorRGBA


class ReTerminalNode(Node):
    def __init__(self):
        super().__init__('reterminal_node')

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        self.joy_msg = Joy()
        self.joy_msg.buttons = [0, 0, 0, 0]

        self.accel_device = rt.get_acceleration_device()
        self.btn_device = rt.get_button_device()

        self.thread = threading.Thread(target=self.thread_func)
        self.thread.start()

        self.sta_led_sub = self.create_subscription(
            ColorRGBA, 'sta_led', self.sta_led_cb, qos_profile_sensor_data)
        self.usr_led_sub = self.create_subscription(
            ColorRGBA, 'usr_led', self.usr_led_cb, qos_profile_sensor_data)
        self.buzzer_sub = self.create_subscription(
            Bool, 'buzzer', self.buzzer_cb, qos_profile_sensor_data)

        self.acceleration_pub = self.create_publisher(
            Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.button_pub = self.create_publisher(
            Joy, 'buttons', qos_profile_sensor_data)

        self.acceleration_timer = self.create_timer(0.01, self.acceleration_cb)
        self.button_timer = self.create_timer(0.01, self.button_cb)

    def thread_func(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        asyncio.ensure_future(self.accel_coroutine(self.accel_device))
        asyncio.ensure_future(self.btn_coroutine(self.btn_device))

        self.loop.run_forever()

        all_tasks = asyncio.Task.all_tasks()
        if all_tasks:
            cleanup = asyncio.ensure_future(asyncio.wait(all_tasks))
            self.loop.run_until_complete(cleanup)

    def shutdown(self):
        self.loop.call_soon_threadsafe(
            lambda: [task.cancel() for task in asyncio.Task.all_tasks()])
        self.loop.stop()
        self.thread.join()

    def sta_led_cb(self, msg):
        rt.sta_led_red = bool(msg.r)
        rt.sta_led_green = bool(msg.g)

    def usr_led_cb(self, msg):
        rt.usr_led = bool(msg.g)

    def buzzer_cb(self, msg):
        rt.buzzer = msg.data

    def acceleration_cb(self):
        self.acceleration_pub.publish(self.imu_msg)

    def button_cb(self):
        self.button_pub.publish(self.joy_msg)

    async def accel_coroutine(self, device):
        async for event in device.async_read_loop():
            accelEvent = rt_accel.AccelerationEvent(event)
            if accelEvent.name == rt_accel.AccelerationName.X:
                self.imu_msg.linear_acceleration.x = accelEvent.value * 9.8 / 1024
            elif accelEvent.name == rt_accel.AccelerationName.Y:
                self.imu_msg.linear_acceleration.y = accelEvent.value * 9.8 / 1024
            elif accelEvent.name == rt_accel.AccelerationName.Z:
                self.imu_msg.linear_acceleration.z = accelEvent.value * 9.8 / 1024

    async def btn_coroutine(self, device):
        async for event in device.async_read_loop():
            buttonEvent = rt_btn.ButtonEvent(event)
            if buttonEvent.name == rt_btn.ButtonName.F1:
                self.joy_msg.buttons[1] = buttonEvent.value
            elif buttonEvent.name == rt_btn.ButtonName.F2:
                self.joy_msg.buttons[2] = buttonEvent.value
            elif buttonEvent.name == rt_btn.ButtonName.F3:
                self.joy_msg.buttons[3] = buttonEvent.value
            elif buttonEvent.name == rt_btn.ButtonName.O:
                self.joy_msg.buttons[0] = buttonEvent.value


def main(args=None):
    rclpy.init(args=args)

    try:
        rt_node = ReTerminalNode()
        rclpy.spin(rt_node)
    except KeyboardInterrupt:
        pass
    finally:
        rt_node.shutdown()
        rt_node.destroy_node()
        rclpy.shutdown()
