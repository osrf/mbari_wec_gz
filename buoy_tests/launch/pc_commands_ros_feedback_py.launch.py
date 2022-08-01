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

import time

from buoy_examples.torque_controller import PBTorqueControlPolicy

from testing_utils import BuoyPyTestAfterShutdown  # noqa F401 -- runs if imported
from testing_utils import BuoyPyTests
from testing_utils import default_generate_test_description


def generate_test_description():
    return default_generate_test_description()


class BuoyPCPyTest(BuoyPyTests):

    def test_pc_commands_ros(self, gazebo_test_fixture, proc_info):
        torque_policy_ = PBTorqueControlPolicy()

        time.sleep(0.5)
        clock = self.node.get_clock()
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, 0)
        self.assertEqual(self.test_helper.iterations, 0)

        preCmdIterations = 15000
        feedbackCheckIterations = 100

        #######################################
        # Check Default Winding Current Damping
        self.test_helper.run(feedbackCheckIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(self.test_helper.iterations, feedbackCheckIterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        expected_wind_curr = \
            torque_policy_.winding_current_target(self.node.rpm_,
                                                  self.node.scale_,
                                                  self.node.retract_) + self.node.bias_curr_
        self.assertGreater(self.node.wind_curr_, expected_wind_curr - 0.1)
        self.assertLess(self.node.wind_curr_, expected_wind_curr + 0.1)

        # Run simulation server and allow piston to settle
        self.test_helper.run(preCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(self.test_helper.iterations, preCmdIterations + feedbackCheckIterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        ##################################################
        # Winding Current
        wc = 12.345
        self.assertNotEqual(self.node.wind_curr_, wc)

        # Now send wind curr command
        self.node.send_pc_wind_curr_command(wc)
        self.assertEqual(self.node.pc_wind_curr_future_.result().result.value,
                         self.node.pc_wind_curr_future_.result().result.OK)

        # Run to let wind curr process
        self.test_helper.run(feedbackCheckIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 2 * feedbackCheckIterations,
                         self.test_helper.iterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        self.assertGreater(self.node.wind_curr_, wc - 0.1)
        self.assertLess(self.node.wind_curr_, wc + 0.1)

        ##############################################
        # Scale
        scale = 1.23
        self.assertNotEqual(self.node.scale_, scale)

        # Now send scale command
        self.node.send_pc_scale_command(scale)
        self.assertEqual(self.node.pc_scale_future_.result().result.value,
                         self.node.pc_scale_future_.result().result.OK)

        # Run to let scale process
        self.test_helper.run(feedbackCheckIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 3 * feedbackCheckIterations,
                         self.test_helper.iterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        self.assertGreater(self.node.scale_, scale - 0.01)
        self.assertLess(self.node.scale_, scale + 0.01)

        ##################################################
        # Retract
        retract = 0.75
        self.assertNotEqual(self.node.retract_, retract)

        # Now send scale command
        self.node.send_pc_retract_command(retract)
        self.assertEqual(self.node.pc_retract_future_.result().result.value,
                         self.node.pc_retract_future_.result().result.OK)

        # Run to let scale process
        self.test_helper.run(feedbackCheckIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 4 * feedbackCheckIterations,
                         self.test_helper.iterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        self.assertGreater(self.node.retract_, retract - 0.01)
        self.assertLess(self.node.retract_, retract + 0.01)

        ##################################################
        # Check return to default winding current damping
        torque_timeout_iterations = 2000

        # Run to let winding current finish
        self.test_helper.run(torque_timeout_iterations - 2 * feedbackCheckIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations +
                         2 * feedbackCheckIterations +
                         torque_timeout_iterations,
                         self.test_helper.iterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        expected_wind_curr = \
            torque_policy_.winding_current_target(self.node.rpm_,
                                                  self.node.scale_,
                                                  self.node.retract_) + self.node.bias_curr_
        self.assertGreater(self.node.wind_curr_, expected_wind_curr - 0.1)
        self.assertLess(self.node.wind_curr_, expected_wind_curr + 0.1)

        #############################################################
        # Bias Current
        bc = 7.89
        self.assertNotEqual(self.node.bias_curr_, bc)

        # Now send bias curr command
        self.node.send_pc_bias_curr_command(bc)
        self.assertEqual(self.node.pc_retract_future_.result().result.value,
                         self.node.pc_retract_future_.result().result.OK)

        # Run to let bias curr process
        bias_curr_iterations = 9000
        bias_curr_timeout_iterations = 10000
        self.test_helper.run(bias_curr_iterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 2 * feedbackCheckIterations +
                         torque_timeout_iterations + bias_curr_iterations,
                         self.test_helper.iterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        self.assertGreater(self.node.bias_curr_, bc - 0.1)
        self.assertLess(self.node.bias_curr_, bc + 0.1)

        self.assertLess(self.node.range_finder_, 0.5)  # meters

        self.test_helper.run(bias_curr_timeout_iterations - bias_curr_iterations +
                             feedbackCheckIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 3 * feedbackCheckIterations +
                         torque_timeout_iterations + bias_curr_timeout_iterations,
                         self.test_helper.iterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        self.assertGreater(self.node.bias_curr_, -0.1)
        self.assertLess(self.node.bias_curr_, 0.1)

        # TODO(anyone) remove once TestFixture is fixed upstream
        self.test_helper.stop()

        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=30)
