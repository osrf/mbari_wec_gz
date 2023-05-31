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

from buoy_interfaces.msg import SCRecord

from testing_utils import BuoyPyTestAfterShutdown  # noqa F401 -- runs if imported
from testing_utils import BuoyPyTests
from testing_utils import default_generate_test_description


PHYSICS_STEP = 0.01


def generate_test_description():
    sim_params = dict(inc_wave_spectrum='inc_wave_spectrum_type:None',
                      physics_rtf=11.0,
                      physics_step=PHYSICS_STEP)
    return default_generate_test_description(regen_models=True,
                                             regen_kwargs=sim_params)


class BuoySCValvePyTest(BuoyPyTests):

    def test_sc_valve_ros(self, gazebo_test_fixture, proc_info):
        clock = self.node.get_clock()
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, 0)
        self.assertEqual(self.test_helper.iterations, 0)

        preCmdIterations = int(45 / PHYSICS_STEP)
        statusCheckIterations = int(1 / PHYSICS_STEP)
        postCmdIterations = int(5 / PHYSICS_STEP)

        # Run simulation server and wait for piston to settle
        self.test_helper.run(preCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations, self.test_helper.iterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, int(self.test_helper.iterations * PHYSICS_STEP))

        # Before Valve command
        pre_valve_range_finder = self.node.range_finder_

        # Check status field
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_REQUEST,
                         'SC Valve Request should be FALSE')
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_STATUS,
                         'SC Valve should be CLOSED')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_REQUEST,
                         'SC Pump Request should be TRUE')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_STATUS,
                         'SC Pump should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_TOGGLE,
                         'SC Pump Toggle should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_OVER_TEMP)
        self.assertFalse(self.node.sc_status_ & SCRecord.TETHER_POWER_REQUEST)
        self.assertTrue(self.node.sc_status_ & SCRecord.TETHER_POWER_STATUS,
                        'SC Tether Power should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)

        # Now send Valve command to OPEN for 5 seconds
        self.node.send_valve_command(5)
        self.assertEqual(self.node.valve_future_.result().result.value,
                         self.node.valve_future_.result().result.OK)

        # Run a bit for Valve to open and Status to be set
        self.test_helper.run(statusCheckIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + statusCheckIterations, self.test_helper.iterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, int(self.test_helper.iterations * PHYSICS_STEP))

        # Check status field
        self.assertTrue(self.node.sc_status_ & SCRecord.RELIEF_VALVE_REQUEST,
                        'SC Valve Request should be FALSE')
        self.assertTrue(self.node.sc_status_ & SCRecord.RELIEF_VALVE_STATUS,
                        'SC Valve should be CLOSED')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_REQUEST,
                         'SC Pump Request should be TRUE')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_STATUS,
                         'SC Pump should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_TOGGLE,
                         'SC Pump Toggle should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_OVER_TEMP)
        self.assertFalse(self.node.sc_status_ & SCRecord.TETHER_POWER_REQUEST)
        self.assertTrue(self.node.sc_status_ & SCRecord.TETHER_POWER_STATUS,
                        'SC Tether Power should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)

        # Check that pump command fails (controller returns BUSY)
        self.node.send_pump_command(2)
        self.assertEqual(self.node.pump_future_.result().result.value,
                         self.node.pump_future_.result().result.BUSY)

        # Run to allow Valve command to finish
        self.test_helper.run(postCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + statusCheckIterations + postCmdIterations,
                         self.test_helper.iterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, int(self.test_helper.iterations * PHYSICS_STEP))

        # Check Status goes back to normal
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_REQUEST,
                         'SC Valve Request should be FALSE')
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_STATUS,
                         'SC Valve should be CLOSED')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_REQUEST,
                         'SC Pump Request should be TRUE')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_STATUS,
                         'SC Pump should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_TOGGLE,
                         'SC Pump Toggle should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_OVER_TEMP)
        self.assertFalse(self.node.sc_status_ & SCRecord.TETHER_POWER_REQUEST)
        self.assertTrue(self.node.sc_status_ & SCRecord.TETHER_POWER_STATUS,
                        'SC Tether Power should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)

        # Check piston motion
        post_valve_range_finder = self.node.range_finder_

        self.assertGreater(post_valve_range_finder,
                           pre_valve_range_finder + 0.8 * 0.0254 * 5.0,
                           'Piston should extend 1 inch/sec for 5 seconds' +
                           ' -- increase valve absement')

        self.assertLess(post_valve_range_finder,
                        pre_valve_range_finder + 1.2 * 0.0254 * 5.0,
                        'Piston should extend 1 inch/sec for 5 seconds' +
                        ' -- decrease valve absement')

        # TODO(anyone) remove once TestFixture is fixed upstream
        self.test_helper.stop()

        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=30)
