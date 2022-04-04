# TODO(chapulina) copyright header

"""Launch Gazebo world with a buoy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_buoy_gazebo = get_package_share_directory('buoy_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg_buoy_gazebo, 'worlds', 'buoy_no_plugins.sdf'), ''],
          description='Ignition Gazebo arguments'),
        gazebo
    ])
