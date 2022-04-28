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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Bridge all the topics provided by the PolytropicPneumaticSpring plugin."""

    # For now, each field is published in its own topic as a double / float64
    ros_msg_type = 'std_msgs/msg/Float64'
    gz_msg_type = 'ignition.msgs.Double'

    # Launch arguments
    chamber_arg = DeclareLaunchArgument(
        'chamber',
        default_value=TextSubstitution(text='upper_adiabatic'),
        description='Name of chamber')

    # Bridge each topic
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/force_', LaunchConfiguration('chamber'), '@', ros_msg_type, '[', gz_msg_type],
            ['/heat_rate_', LaunchConfiguration('chamber'), '@', ros_msg_type, '[', gz_msg_type],
            ['/pressure_', LaunchConfiguration('chamber'), '@', ros_msg_type, '[', gz_msg_type],
            ['/temperature_', LaunchConfiguration('chamber'), '@', ros_msg_type, '[', gz_msg_type],
            ['/volume_', LaunchConfiguration('chamber'), '@', ros_msg_type, '[', gz_msg_type],
        ],
        output='screen'
        )

    return LaunchDescription([
        chamber_arg,
        bridge
    ])
