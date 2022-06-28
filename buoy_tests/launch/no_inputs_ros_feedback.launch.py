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

import os
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import launch_testing
import launch_testing.actions


def generate_test_description():

    # Test fixture
    gazebo_test_fixture = Node(
        package='buoy_tests',
        executable='no_inputs_ros_feedback',
        output='screen'
    )

    pkg_buoy_gazebo = get_package_share_directory('buoy_gazebo')

    # Lauch Gazebo with MBARI WEC Buoy
    mbari_wec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buoy_gazebo, 'launch', 'mbari_wec.launch.py'),
        ),
    )

    return launch.LaunchDescription([
        gazebo_test_fixture,
        mbari_wec,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ]), locals()


class NoInputsROSTest(unittest.TestCase):

    def test_termination(self, gazebo_test_fixture, proc_info):
        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=200)


@launch_testing.post_shutdown_test()
class NoInputsROSTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, gazebo_test_fixture, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            gazebo_test_fixture
        )
