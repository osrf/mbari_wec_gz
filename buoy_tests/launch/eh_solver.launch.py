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
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import launch_testing
import launch_testing.actions


def generate_test_description():

    pkg_buoy_tests = get_package_share_directory('buoy_tests')

    eh_solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buoy_tests, 'launch', 'eh_solver_select.launch.py'),
        ),
        launch_arguments={
            'manual': 'False'
        }.items(),
    )

    return launch.LaunchDescription([
        eh_solver,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ]), locals()


def find_proc(ld):
    entities = ld.\
        _IncludeLaunchDescription__launch_description_source.\
        _LaunchDescriptionSource__launch_description.\
        _LaunchDescription__entities
    for entity in entities:
        if type(entity) is Node:
            if type(entity._Action__condition) is UnlessCondition:
                return entity
        elif type(entity) is OpaqueFunction:
            for node in entity._OpaqueFunction__args:
                if type(node) is Node:
                    if type(node._Action__condition) is UnlessCondition:
                        return node


class EHSolverTest(unittest.TestCase):

    def test_termination(self, eh_solver, proc_info):
        proc_info.assertWaitForShutdown(
            process=find_proc(eh_solver),
            timeout=200
        )


@launch_testing.post_shutdown_test()
class BuoyExperimentComparisonAfterShutdown(unittest.TestCase):

    def test_exit_code(self, eh_solver, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            find_proc(eh_solver)
        )
