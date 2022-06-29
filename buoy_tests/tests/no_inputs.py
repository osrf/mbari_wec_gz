#!/usr/bin/python3

# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
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

from threading import Thread
import unittest

from buoy_msgs.interface import Interface
import rclpy


class NoInputsPyNode(Interface):

    def __init__(self):
        rclpy.init()
        super().__init__('no_inputs_py')
        self.rpm = 0.0
        self.wind_curr = 0.0
        self.stop = False
        self.run()

    def power_callback(self, data):
        self.rpm = data.rpm
        self.wind_curr = data.rpm

    def run(self):
        def _run():
            rate = self.create_rate(100.0)
            while rclpy.ok() and not self.stop:
                rclpy.spin_once(self)
                rate.sleep()
            rclpy.shutdown()
        self._run_thread = Thread(target=_run)
        self._run_thread.daemon = True
        self._run_thread.start()

    def stop(self):
        self.stop = True
        self._run_thread.join()


class TestNoInputsPyNode(unittest.TestCase):

    node = NoInputsPyNode()

    def test_final_state(self):
        rate = self.node.create_rate(10.0)
        clock = self.node.get_clock()
        t, _ = clock.now().seconds_nanoseconds()
        while rclpy.ok() and t < 5:
            self.node.get_logger().info(f't = {t}')
            rate.sleep()
            t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, 5)
        self.assertLess(self.node.rpm_, 100.0)
        self.assertLess(self.node.wcurrent_, 0.1)
        self.node.stop()
        self.assertFalse(rclpy.ok())
