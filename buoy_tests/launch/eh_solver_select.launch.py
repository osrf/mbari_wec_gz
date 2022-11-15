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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    manual_arg = DeclareLaunchArgument(
        'manual', default_value='False',
        description='visually inspect data'
    )

    # Test
    eh_solver = Node(
        package='buoy_tests',
        executable='eh_solver',
        output='both',
        parameters=[
            {'manual': False}
        ],
        namespace='/eh_solver',
        condition=UnlessCondition(LaunchConfiguration('manual'))
    )

    eh_solver_manual = Node(
        package='buoy_tests',
        executable='eh_solver',
        output='both',
        parameters=[
            {'manual': True}
        ],
        namespace='/eh_solver',
        prefix=['x-terminal-emulator -e'],
        shell=True,
        condition=IfCondition(LaunchConfiguration('manual'))
    )

    return launch.LaunchDescription([
        manual_arg,
        eh_solver,
        eh_solver_manual,
    ])
