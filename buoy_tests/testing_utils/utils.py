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

import asyncio
from threading import Thread
import unittest

from buoy_msgs.interface import Interface

from buoy_tests.srv import RunServer

# TODO(anyone) Put back when fixed upstream
# from ignition.common import set_verbosity
# from ignition.gazebo import TestFixture

import launch
import launch.actions

from launch_ros.actions import Node as launchNode

import launch_testing
import launch_testing.actions

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as ros2Node
from rclpy.parameter import Parameter


def default_generate_test_description():

    # Test fixture
    gazebo_test_fixture = launchNode(
        package='buoy_tests',
        executable='fixture_server',
        output='screen'
    )

    bridge = launchNode(package='ros_ign_bridge',
                        executable='parameter_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                        output='screen')

    return launch.LaunchDescription([
        gazebo_test_fixture,
        bridge,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ]), locals()


class BuoySCInterface(Interface):

    def __init__(self):
        super().__init__('test_sc_inputs_py', wait_for_services=True)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.range_finder_ = 0
        self.status_ = 0

    def spring_callback(self, data):
        self.range_finder_ = data.range_finder
        self.status_ = data.status

    """  TODO(anyone) put back when TestFixture fixed upstream
    def start(self):
        self.thread = Thread(target=rclpy.spin, args=(self,))
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.thread.join()
    """


class TestHelper(ros2Node):

    def __init__(self):
        super().__init__('gz_fixture_client')
        self.fixture_client = self.create_client(RunServer, 'run_server')
        while rclpy.ok() and not self.fixture_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for fixture server...')
        self.iterations = 0
        self.run_status = False
        """ TODO(anyone) see .run() below
        self.fixture = TestFixture('mbari_wec.sdf')
        self.fixture.on_post_update(self.on_post_update)
        self.fixture.finalize()
        self.server = self.fixture.server()
        """

    """ TODO(anyone) put back when TestFixture fixed upstream
    def on_post_update(self, _updateInfo, _ecm):
        self.iterations += 1
    """

    def run(self, _iterations):
        return asyncio.run(self._run(_iterations))

    async def _run(self, _iterations):
        # TODO(anyone) workaround until TestFixture is fixed upstream (see below)
        req = RunServer.Request()
        req.iterations = _iterations
        future = self.fixture_client.call_async(req)
        await future
        self.run_status = future.result().success
        self.iterations += future.result().iterations

        """ TODO(anyone) Expect the following to work but .run() with blocking = True
                         doesn't return.
                         Also, even with blocking = False, cannot call .run() more than once. The
                         second call returns False.
        blocking = True
        paused = False
        self.run_status = self.server.run(blocking, _iterations, paused)
        """

    # TODO(anyone) remove when TestFixture fixed upstream
    def stop(self):
        self.run(0)


class BuoySCPyTests(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.test_helper = TestHelper()
        self.node = BuoySCInterface()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.test_helper)
        self.executor_thread = Thread(target=self.executor.spin)
        self.executor_thread.daemon = True
        self.executor_thread.start()
        # TODO(anyone) put back when TestFixture fixed upstream
        # self.node.start()
        # set_verbosity(3)

    def tearDown(self):
        rclpy.shutdown()
        self.assertFalse(rclpy.ok())
        self.executor.shutdown()
        self.executor_thread.join()
        # TODO(anyone) put back when TestFixture fixed upstream
        # self.node.stop()


@launch_testing.post_shutdown_test()
class BuoySCPyTestAfterShutdown(unittest.TestCase):

    def test_exit_code_gazebo(self, gazebo_test_fixture, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            gazebo_test_fixture
        )

    """  TODO(anyone) error when trying this:
                      AttributeError: 'ProcessStarted' object has no attribute 'returncode'
    def test_exit_code_bridge(self, bridge, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            bridge
        )
    """
