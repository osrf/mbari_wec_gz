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

import launch

from launch_ros.actions import Node as launchNode


def generate_launch_description():

    # Test fixture
    gazebo_test_fixture = launchNode(
        package='buoy_tests',
        executable='fixture_server',
        output='screen'
    )

    bridge = launchNode(package='ros_ign_bridge',
                        executable='parameter_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                        output='screen')

    return launch.LaunchDescription([
        gazebo_test_fixture,
        bridge
    ])
