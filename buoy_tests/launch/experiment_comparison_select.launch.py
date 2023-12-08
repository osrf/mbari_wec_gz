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
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from testing_utils import regenerate_models


def generate_launch_description():

    test_inputdata_arg = DeclareLaunchArgument(
        'test_inputdata', default_value='FrictionIDMovesWithBiasCurr',
        description='base name of test data (assumes .exp/.tst)'
    )

    manual_comparison_arg = DeclareLaunchArgument(
        'manual', default_value='False',
        description='compare data manually'
    )

    # 1) Ignore file path
    # test_inputdata = tinp.split('/')[-1] if '/' in (tinp:=test_inputdata) else tinp
    test_inputdata = PythonExpression(["tinp.split('/')[-1] if '/' in (tinp:='",
                                       LaunchConfiguration('test_inputdata'),
                                       "') else tinp"])
    # 2) Ignore .exp and .tst in file extension
    # test_inputdata = tinp[:-4] if '.exp' in (tinp:=test_inputdata) or '.tst' in tinp else tinp
    test_inputdata = PythonExpression(["tinp[:-4] if '.exp' in (tinp:='", test_inputdata,
                                       "') or '.tst' in tinp else tinp"])
    # 3) Use file in buoy_tests/share/buoy_tests/test_inputdata/
    # test_inputdata = os.path.join(get_package_share('buoy_tests'),
    #                               'test_inputdata',
    #                               test_inputdata)
    test_inputdata = PathJoinSubstitution([FindPackageShare('buoy_tests'),
                                           'test_inputdata',
                                           test_inputdata])
    # 4) Now add extensions
    # test_input_exp = test_inputdata + '.exp'
    test_inputdata_exp = PythonExpression(["'", test_inputdata, "' + '.exp'"])
    # test_input_tst = test_inputdata + '.tst'
    test_inputdata_tst = PythonExpression(["'", test_inputdata, "' + '.tst'"])

    # Test fixture
    # manual:=False
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
        condition=UnlessCondition(LaunchConfiguration('manual')),
        on_exit=Shutdown()
    )

    # manual:=True
    gazebo_test_fixture_manual = Node(
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
        condition=IfCondition(LaunchConfiguration('manual')),
        on_exit=Shutdown()
    )

    bridge = Node(package='ros_gz_bridge',
                  executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                  output='screen')

    # passthru nodes dependent on regenerate_models
    nodes = [gazebo_test_fixture,
             gazebo_test_fixture_manual,
             bridge]
    # override default params for test
    sim_params = dict(inc_wave_spectrum='inc_wave_spectrum_type:None',
                      physics_rtf=11.0,
                      physics_step=0.001,
                      initial_piston_position=2.03,
                      initial_buoy_height=2.0)

    return launch.LaunchDescription([
        test_inputdata_arg,
        manual_comparison_arg,
        OpaqueFunction(function=regenerate_models,
                       args=nodes,
                       kwargs=sim_params),
    ])
