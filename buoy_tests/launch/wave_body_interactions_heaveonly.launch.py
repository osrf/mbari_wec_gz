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

import unittest

import launch
import launch.actions
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

import launch_testing
import launch_testing.actions

from testing_utils import regenerate_models


def generate_test_description():

    # Test fixture
    gazebo_test_fixture = Node(
        package='buoy_tests',
        executable='wave_body_interactions_heaveonly',
        output='screen',
        on_exit=launch.actions.Shutdown()
    )

    nodes = [gazebo_test_fixture]
    sim_params = dict(inc_wave_spectrum='inc_wave_spectrum_type:None',
                      physics_rtf=11.0,
                      physics_step=0.001,
                      initial_piston_position=2.03,
                      initial_buoy_height=2.0)

    return launch.LaunchDescription([
        OpaqueFunction(function=regenerate_models,
                       args=nodes,
                       kwargs=sim_params),
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ]), locals()


class WaveBodyInteractionTest(unittest.TestCase):

    def test_termination(self, gazebo_test_fixture, proc_info):
        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=200)


@launch_testing.post_shutdown_test()
class WaveBodyInteractionTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, gazebo_test_fixture, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            gazebo_test_fixture
        )
