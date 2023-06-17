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
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from testing_utils import regenerate_models


def generate_launch_description():

    manual_arg = DeclareLaunchArgument(
        'manual', default_value='False',
        description='visually inspect data'
    )

    # Test
    eh_windtarget = Node(
        package='buoy_tests',
        executable='eh_windtarget',
        output='both',
        parameters=[
            {'manual': False}
        ],
        namespace='/eh_windtarget',
        condition=UnlessCondition(LaunchConfiguration('manual'))
    )

    eh_windtarget_manual = Node(
        package='buoy_tests',
        executable='eh_windtarget',
        output='both',
        parameters=[
            {'manual': True}
        ],
        namespace='/eh_windtarget',
        prefix=['x-terminal-emulator -e'],
        shell=True,
        condition=IfCondition(LaunchConfiguration('manual'))
    )

    nodes = [eh_windtarget,
             eh_windtarget_manual]
    sim_params = dict(inc_wave_spectrum='inc_wave_spectrum_type:None',
                      physics_rtf=11.0,
                      physics_step=0.001,
                      initial_piston_position=2.03,
                      initial_buoy_height=2.0)

    return launch.LaunchDescription([
        manual_arg,
        OpaqueFunction(function=regenerate_models,
                       args=nodes,
                       kwargs=sim_params),
    ])
