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
from launch.actions import OpaqueFunction

from launch_ros.actions import Node as launchNode

from testing_utils import regenerate_models


def generate_launch_description():

    # Test fixture
    gazebo_test_fixture = launchNode(
        package='buoy_tests',
        executable='fixture_server',
        output='screen',
        on_exit=launch.actions.Shutdown()
    )

    bridge = launchNode(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                        output='screen')

    nodes = [gazebo_test_fixture,
             bridge]
    sim_params = dict(inc_wave_spectrum='inc_wave_spectrum_type:None',
                      physics_rtf=11.0,
                      physics_step=0.01,
                      initial_piston_position=2.03,
                      initial_buoy_height=2.0)

    return launch.LaunchDescription([
        OpaqueFunction(function=regenerate_models,
                       args=nodes,
                       kwargs=sim_params),
    ]), locals()
