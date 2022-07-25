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
import time
import unittest

from buoy_msgs.interface import Interface
from buoy_msgs.msg import SCRecord
from buoy_msgs.srv import PumpCommand, ValveCommand

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


def generate_test_description():

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


class BuoySCPyTests(Interface):

    def __init__(self):
        super().__init__('test_sc_inputs_py', wait_for_services=True)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.range_finder_ = 0
        self.status_ = 0

    def spring_callback(self, data):
        self.range_finder_ = data.range_finder
        self.status_ = data.status

    def send_pump_command(self):
        return asyncio.run(self._send_pump_command())

    async def _send_pump_command(self):
        request = PumpCommand.Request()
        request.duration_sec = 20

        self.pump_future_ = self.pump_client_.call_async(request)
        self.pump_future_.add_done_callback(self.service_response_callback)
        await self.pump_future_

    def send_valve_command(self):
        request = ValveCommand.Request()
        request.duration_sec = 5

        self.valve_future_ = self.valve_client_.call_async(request)
        self.valve_future_.add_done_callback(self.service_response_callback)

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


class BuoySCPyTest(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.test_helper = TestHelper()
        self.node = BuoySCPyTests()
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

    def test_sc_pump_ros(self, gazebo_test_fixture, proc_info):
        clock = self.node.get_clock()
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, 0)
        self.assertEqual(self.test_helper.iterations, 0)

        preCmdIterations = 15000
        statusCheckIterations = 1000
        postCmdIterations = 20000

        # Run simulation server and allow piston to settle
        self.test_helper.run(preCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(self.test_helper.iterations, preCmdIterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        # Before Pump command
        pre_pump_range_finder = self.node.range_finder_

        # Check status field
        self.assertFalse(self.node.status_ & SCRecord.RELIEF_VALVE_REQUEST,
                         'SC Valve Request should be FALSE')
        self.assertFalse(self.node.status_ & SCRecord.RELIEF_VALVE_STATUS,
                         'SC Valve should be CLOSED')
        self.assertFalse(self.node.status_ & SCRecord.PUMP_REQUEST,
                         'SC Pump Request should be FALSE')
        self.assertFalse(self.node.status_ & SCRecord.PUMP_STATUS,
                         'SC Pump should be OFF')
        self.assertFalse(self.node.status_ & SCRecord.PUMP_TOGGLE,
                         'SC Pump Toggle should be OFF')
        self.assertFalse(self.node.status_ & SCRecord.PUMP_OVER_TEMP)
        self.assertFalse(self.node.status_ & SCRecord.TETHER_POWER_REQUEST)
        self.assertTrue(self.node.status_ & SCRecord.TETHER_POWER_STATUS,
                        'SC Tether Power should be ON')
        self.assertFalse(self.node.status_ & SCRecord.LR_FAULT)
        self.assertFalse(self.node.status_ & SCRecord.LR_FAULT)

        # Now send Pump command to run for 20 seconds
        self.node.send_pump_command()
        self.assertEqual(self.node.pump_future_.result().result.value,
                         self.node.pump_future_.result().result.OK)

        # Run to let Pump start
        self.test_helper.run(500)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 500, self.test_helper.iterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        # Check status field
        self.assertFalse(self.node.status_ & SCRecord.RELIEF_VALVE_REQUEST,
                         'SC Valve Request should be FALSE')
        self.assertFalse(self.node.status_ & SCRecord.RELIEF_VALVE_STATUS,
                         'SC Valve should be CLOSED')
        self.assertTrue(self.node.status_ & SCRecord.PUMP_REQUEST,
                        'SC Pump Request should be TRUE')
        self.assertTrue(self.node.status_ & SCRecord.PUMP_STATUS,
                        'SC Pump should be ON')
        self.assertTrue(self.node.status_ & SCRecord.PUMP_TOGGLE,
                        'SC Pump Toggle should be ON')
        self.assertFalse(self.node.status_ & SCRecord.PUMP_OVER_TEMP)
        self.assertFalse(self.node.status_ & SCRecord.TETHER_POWER_REQUEST)
        self.assertTrue(self.node.status_ & SCRecord.TETHER_POWER_STATUS,
                        'SC Tether Power should be ON')
        self.assertFalse(self.node.status_ & SCRecord.LR_FAULT)
        self.assertFalse(self.node.status_ & SCRecord.LR_FAULT)

        # Check pump toggle
        for n in range(1, 5):
            self.test_helper.run(statusCheckIterations)
            self.assertTrue(self.test_helper.run_status)
            self.assertEqual(preCmdIterations + 500 + n * statusCheckIterations,
                             self.test_helper.iterations)
            time.sleep(0.5)
            t, _ = clock.now().seconds_nanoseconds()
            self.assertEqual(t, self.test_helper.iterations // 1000)

            if n % 2 == 1:
                self.assertFalse(self.node.status_ & SCRecord.PUMP_TOGGLE,
                                 'SC Pump Toggle should be OFF')
            else:
                self.assertTrue(self.node.status_ & SCRecord.PUMP_TOGGLE,
                                'SC Pump Toggle should be ON')

        # Run to allow Pump command to finish
        self.test_helper.run(postCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 500 + 4 * statusCheckIterations + postCmdIterations,
                         self.test_helper.iterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        # Check piston motion
        post_pump_range_finder = self.node.range_finder_

        self.assertGreater(
          post_pump_range_finder,
          pre_pump_range_finder - 2.2 * 0.0254 *
          20.0 * 1.0 / 60.0,
          'Piston should retract 2 inches/min for 20 seconds')

        self.assertLess(
          post_pump_range_finder,
          pre_pump_range_finder - 1.8 * 0.0254 *
          20.0 * 1.0 / 60.0,
          'Piston should retract 2 inches/min for 20 seconds')

        # TODO(anyone) remove once TestFixture is fixed upstream
        self.test_helper.stop()

        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=30)


@launch_testing.post_shutdown_test()
class BuoySCPyTestAfterShutdown(unittest.TestCase):

    def test_exit_code_gazebo(self, gazebo_test_fixture, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            gazebo_test_fixture
        )

    def test_exit_code_bridge(self, bridge, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            bridge
        )
