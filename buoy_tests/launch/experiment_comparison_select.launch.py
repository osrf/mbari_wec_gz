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

import launch
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from testing_utils import regenerate_models


def generate_test_fixture(context, *args, **kwargs):
    """
    Find input data files from launch argument in buoy_tests to pass to test.

    For use as an OpaqueFunction:
        launch.actions.OpaqueFunction(function=generate_test_fixture)

    Will grab overridden inputs from launch arguments via context object and return
    a gazebo test fixture node
    """
    test_inputdata_exp = LaunchConfiguration('test_inputdata_exp').perform(context)
    test_inputdata_tst = LaunchConfiguration('test_inputdata_tst').perform(context)
    print(f'{test_inputdata_exp=}')
    print(f'{test_inputdata_tst=}')

    # Input data
    test_inputdata_exp = os.path.join(
        get_package_share_directory('buoy_tests'),
        'test_inputdata',
        test_inputdata_exp
    )

    test_inputdata_tst = os.path.join(
        get_package_share_directory('buoy_tests'),
        'test_inputdata',
        test_inputdata_tst
    )

    # Test fixture
    manual_test = LaunchConfiguration('manual').perform(context)
    manual_test = (manual_test == 'True')
    print(f'{manual_test=}')
    if manual_test:
        gazebo_test_fixture = Node(
            package='buoy_tests',
            executable='experiment_comparison',
            output='both',
            parameters=[
                {'use_sim_time': True},
                {'inputdata_filename': test_inputdata_exp},
                {'manual_comparison': True}
            ],
            namespace='/experiment_comparison',
            prefix=['x-terminal-emulator -e'],
            shell=True,
            on_exit=Shutdown()
        )

    else:
        gazebo_test_fixture = Node(
            package='buoy_tests',
            executable='experiment_comparison',
            output='both',
            parameters=[
                {'use_sim_time': True},
                {'inputdata_filename': test_inputdata_tst},
                {'manual_comparison': False}
            ],
            namespace='/experiment_comparison',
            on_exit=Shutdown()
        )

    # Regenerate models
    nodes = list(args) + [gazebo_test_fixture]
    sim_params = dict(inc_wave_spectrum='inc_wave_spectrum_type:None',
                      physics_rtf=11.0,
                      physics_step=0.001,
                      initial_piston_position=2.03,
                      initial_buoy_height=2.0)

    return OpaqueFunction(function=regenerate_models,
                          args=nodes,
                          kwargs=sim_params),


def generate_launch_description():

    manual_comparison_arg = DeclareLaunchArgument(
        'manual', default_value='False',
        description='compare data manually'
    )

    test_inputdata_exp_arg = DeclareLaunchArgument(
        'test_inputdata_exp', default_value='FrictionIDMovesWithBiasCurr.exp',
        description='.exp bench test file'
    )

    test_inputdata_tst_arg = DeclareLaunchArgument(
        'test_inputdata_tst', default_value='FrictionIDMovesWithBiasCurr.tst',
        description='.tst sim test file (approved by user and used for CI)'
    )

    bridge = Node(package='ros_gz_bridge',
                  executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                  output='screen')

    nodes = [bridge]

    return launch.LaunchDescription([
        manual_comparison_arg,
        test_inputdata_exp_arg,
        test_inputdata_tst_arg,
        OpaqueFunction(function=generate_test_fixture,
                       args=nodes)
    ])
