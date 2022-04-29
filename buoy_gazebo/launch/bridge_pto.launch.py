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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Bridge all the topics provided by the ElectroHydraulicPTO plugin."""
    # For now, each field is published in its own topic as a double / float64
    ros_msg_type = 'std_msgs/msg/Float64'
    gz_msg_type = 'ignition.msgs.Double'

    # Launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value=TextSubstitution(text='Hydraulics_Test'),
        description='Name of model')

    joint_name_arg = DeclareLaunchArgument(
        'joint_name',
        default_value=TextSubstitution(text='HydraulicRam'),
        description='Name of joint within model')

    # Bridge each topic
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/battcurr_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/deltaP_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/loadcurr_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/pistonvel_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/retractfactor_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/rpm_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/scalefactor_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/targwindcurr_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/windcurr_', LaunchConfiguration('joint_name'),
                '@', ros_msg_type, '[', gz_msg_type],
            ['/model/', LaunchConfiguration('model_name'), '/joint/',
                LaunchConfiguration('joint_name'),
                '/UserCommandedCurr@', ros_msg_type, ']', gz_msg_type],
            ['/model/', LaunchConfiguration('model_name'), '/joint/',
                LaunchConfiguration('joint_name'),
                '/BiasCurrent@', ros_msg_type, ']', gz_msg_type],
            ['/model/', LaunchConfiguration('model_name'), '/joint/',
                LaunchConfiguration('joint_name'),
                '/ScaleFactor@', ros_msg_type, ']', gz_msg_type],
            ['/model/', LaunchConfiguration('model_name'), '/joint/',
                LaunchConfiguration('joint_name'),
                '/RetractFactor@', ros_msg_type, ']', gz_msg_type],
        ],
        output='screen'
        )

    return LaunchDescription([
        model_name_arg,
        joint_name_arg,
        bridge
    ])
