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

"""Launch Gazebo world with a buoy."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_buoy_gazebo = get_package_share_directory('buoy_gazebo')
    example_sim_params_yaml = os.path.join(pkg_buoy_gazebo, 'example_sim_params.yaml')

    sim_params_yaml_launch_arg = DeclareLaunchArgument(
        'sim_params_yaml', default_value=[example_sim_params_yaml],
        description='Input batch sim params yaml'
    )

    batch_sim = Node(
        package='buoy_gazebo',
        executable='mbari_wec_batch',
        arguments=[
            LaunchConfiguration('sim_params_yaml'),
        ],
        output='screen',
        on_exit=Shutdown()
    )

    return LaunchDescription([
        sim_params_yaml_launch_arg,
        batch_sim,
    ])
