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

"""Launch Gazebo Sim with command line arguments."""

from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')])}

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='',
                              description='Arguments to be passed to Gazebo'),
        # Gazebo's major version
        DeclareLaunchArgument('gz_version', default_value='6',
                              description="Gazebo's major version"),
        ExecuteProcess(
            cmd=['ruby $(which gz) sim -rs',
                 LaunchConfiguration('gz_args'),
                 '--force-version',
                 LaunchConfiguration('gz_version'),
                 ],
            prefix=['xterm -e gdb -ex run --args'],
            # prefix=['xterm -e valgrind'],
            output='screen',
            additional_env=env,
            shell=True
        )
    ])
