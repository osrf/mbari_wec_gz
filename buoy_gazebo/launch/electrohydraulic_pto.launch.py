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

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/battcurr_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/deltaP_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/loadcurr_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/pistonvel_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/retractfactor_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/rpm_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/scalefactor_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/targwindcurr_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/windcurr_HydraulicRam@std_msgs/msg/Float64[ignition.msgs.Double',
            '/model/Hydraulics_Test/joint/HydraulicRam/UserCommandedCurr@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/Hydraulics_Test/joint/HydraulicRam/BiasCurrent@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/Hydraulics_Test/joint/HydraulicRam/ScaleFactor@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/Hydraulics_Test/joint/HydraulicRam/RetractFactor@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge
    ])
