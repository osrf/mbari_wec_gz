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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    """Launch buoy simulation and bridge it to ROS."""
    # Arguments
    sim_args = DeclareLaunchArgument(
          'ign_args',
          default_value='mbari_wec.sdf',
          description='Simulator arguments')

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value=TextSubstitution(text='MBARI_WEC'),
        description='Name of model')

    has_pto_arg = DeclareLaunchArgument(
        'has_pto',
        default_value='True',
        description='Whether to bridge the ElectroHydraulicPTO.')

    pto_joint_name_arg = DeclareLaunchArgument(
        'pto_joint_name',
        default_value=TextSubstitution(text='HydraulicRam'),
        description='Name of PTO prismatic joint')

    has_lower_spring_arg = DeclareLaunchArgument(
        'has_lower_spring',
        default_value='true',
        description='Whether to bridge the lower PolytropicPneumaticSpring.')

    lower_spring_arg = DeclareLaunchArgument(
        'lower_spring',
        default_value=TextSubstitution(text='lower_polytropic'),
        description='Name of lower chamber for polytropic pneumatic spring')

    has_upper_spring_arg = DeclareLaunchArgument(
        'has_upper_spring',
        default_value='true',
        description='Whether to bridge the upper PolytropicPneumaticSpring.')

    upper_spring_arg = DeclareLaunchArgument(
        'upper_spring',
        default_value=TextSubstitution(text='upper_polytropic'),
        description='Name of upper chamber for polytropic pneumatic spring')

    # Launch the simulator
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    # Bridges to ROS
    pto_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bridge_pto.launch.py']),
        launch_arguments={
            'model_name': LaunchConfiguration('model_name'),
            'joint_name': LaunchConfiguration('pto_joint_name')
        }.items(),
        condition=IfCondition(LaunchConfiguration('has_pto')),
    )

    lower_spring_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bridge_spring.launch.py']),
        launch_arguments={
            'chamber': LaunchConfiguration('lower_spring'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('has_lower_spring')),
    )

    upper_spring_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bridge_spring.launch.py']),
        launch_arguments={
            'chamber': LaunchConfiguration('upper_spring'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('has_upper_spring')),
    )

    return LaunchDescription([
        # Args
        sim_args,
        model_name_arg,
        has_pto_arg,
        pto_joint_name_arg,
        has_lower_spring_arg,
        lower_spring_arg,
        has_upper_spring_arg,
        upper_spring_arg,
        # Bridges
        pto_bridge,
        lower_spring_bridge,
        upper_spring_bridge,
        # Launch files from this package must come before ros_ign_gazebo, otherwise
        # ThisLaunchFileDir misbehaves
        sim,
    ])
