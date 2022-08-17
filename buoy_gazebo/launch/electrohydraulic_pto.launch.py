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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={'ign_args': '-r electrohydraulicPTO.sdf'}.items(),
    )

    # For now, each field is published in its own topic as a double / float64
    ros_msg_type = 'std_msgs/msg/Float64'
    gz_msg_type = 'gz.msgs.Double'
    joint_name = 'HydraulicRam'
    model_name = 'Hydraulics_Test'

    gz_to_ros = ros_msg_type + '[' + gz_msg_type
    ros_to_gz = ros_msg_type + ']' + gz_msg_type

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/battcurr_' + joint_name + '@' + gz_to_ros],
            ['/deltaP_' + joint_name + '@' + gz_to_ros],
            ['/loadcurr_' + joint_name + '@' + gz_to_ros],
            ['/pistonvel_' + joint_name + '@' + gz_to_ros],
            ['/retractfactor_' + joint_name + '@' + gz_to_ros],
            ['/rpm_' + joint_name + '@' + gz_to_ros],
            ['/scalefactor_' + joint_name + '@' + gz_to_ros],
            ['/targwindcurr_' + joint_name + '@' + gz_to_ros],
            ['/windcurr_' + joint_name + '@' + gz_to_ros],
            ['/model/' + model_name + '/joint/' + joint_name + '/UserCommandedCurr@' + ros_to_gz],
            ['/model/' + model_name + '/joint/' + joint_name + '/BiasCurrent@' + ros_to_gz],
            ['/model/' + model_name + '/joint/' + joint_name + '/ScaleFactor@' + ros_to_gz],
            ['/model/' + model_name + '/joint/' + joint_name + '/RetractFactor@' + ros_to_gz],
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge
    ])
