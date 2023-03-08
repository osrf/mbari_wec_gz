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

import time
import unittest

from buoy_api import Interface

import launch
import launch.actions
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

import launch_testing
import launch_testing.actions

import rclpy
from rclpy.parameter import Parameter

from testing_utils import regenerate_models


def generate_test_description():

    # GTest fixture
    gazebo_test_fixture = Node(
        package='buoy_tests',
        executable='no_inputs',
        output='screen'  # ,
        # TODO(anyone) for some reason this makes the test fail
        # on_exit=launch.actions.Shutdown()
    )

    bridge = Node(package='ros_gz_bridge',
                  executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                  output='screen')

    nodes = [gazebo_test_fixture,
             bridge]
    sim_params = dict(inc_wave_spectrum='inc_wave_spectrum_type:None',
                      physics_rtf=11.0,
                      physics_step=0.001)

    return launch.LaunchDescription([
        OpaqueFunction(function=regenerate_models,
                       args=nodes,
                       kwargs=sim_params),
    ]), locals()


class NoInputsPyNode(Interface):

    def __init__(self):
        rclpy.init()
        super().__init__('test_no_inputs_py', wait_for_services=True)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.rpm = 10000.0
        self.wcurrent = 10000.0

    def power_callback(self, data):
        self.rpm = data.rpm
        self.wcurrent = data.wcurrent


class NoInputsGazeboPyTest(unittest.TestCase):

    node = None

    def setUp(self):
        self.node = NoInputsPyNode()

    def tearDown(self):
        rclpy.shutdown()
        self.assertFalse(rclpy.ok())

    def test_final_state(self, gazebo_test_fixture, proc_info):
        rclpy.spin_once(self.node)
        clock = self.node.get_clock()
        t, _ = clock.now().seconds_nanoseconds()
        while rclpy.ok() and t < 15:
            rclpy.spin_once(self.node)
            t, _ = clock.now().seconds_nanoseconds()
        time.sleep(0.5)
        self.assertEqual(t, 15)
        self.assertLess(self.node.rpm, 1000.0)
        self.assertLess(self.node.wcurrent, 0.1)
        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=200)


@launch_testing.post_shutdown_test()
class NoInputsGazeboPyTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, gazebo_test_fixture, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            gazebo_test_fixture
        )
