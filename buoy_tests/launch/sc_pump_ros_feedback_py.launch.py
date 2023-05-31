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
    return default_generate_test_description(enable_rosbag=True,
                                             rosbag_name='rosbag2_sc_pump_py',
                                             regen_models=True,
                                             regen_kwargs=sim_params)


class BuoySCPumpPyTest(BuoyPyTests):

    def test_sc_pump_ros(self, gazebo_test_fixture, proc_info):
        clock = self.node.get_clock()
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, 0)
        self.assertEqual(self.test_helper.iterations, 0)

        preCmdIterations = int(45 / PHYSICS_STEP)
        statusCheckIterations = int(1 / PHYSICS_STEP)
        postCmdIterations = int(60 / PHYSICS_STEP)

        # Run simulation server and allow piston to settle
        self.test_helper.run(preCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(self.test_helper.iterations, preCmdIterations)

        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, int(self.test_helper.iterations * PHYSICS_STEP))

        # Before Pump command
        pre_pump_range_finder = self.node.range_finder_

        # Check status field
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_REQUEST,
                         'SC Valve Request should be FALSE')
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_STATUS,
                         'SC Valve should be CLOSED')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_REQUEST,
                         'SC Pump Request should be FALSE')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_STATUS,
                         'SC Pump should be OFF')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_TOGGLE,
                         'SC Pump Toggle should be OFF')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_OVER_TEMP)
        self.assertFalse(self.node.sc_status_ & SCRecord.TETHER_POWER_REQUEST)
        self.assertTrue(self.node.sc_status_ & SCRecord.TETHER_POWER_STATUS,
                        'SC Tether Power should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)

        # Now send Pump command to run for 1 minute
        self.node.send_pump_command(1.0)
        self.assertEqual(self.node.pump_future_.result().result.value,
                         self.node.pump_future_.result().result.OK)

        # Run to let Pump start
        self.test_helper.run(500)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 500, self.test_helper.iterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, int(self.test_helper.iterations * PHYSICS_STEP))

        # Check status field
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_REQUEST,
                         'SC Valve Request should be FALSE')
        self.assertFalse(self.node.sc_status_ & SCRecord.RELIEF_VALVE_STATUS,
                         'SC Valve should be CLOSED')
        self.assertTrue(self.node.sc_status_ & SCRecord.PUMP_REQUEST,
                        'SC Pump Request should be TRUE')
        self.assertTrue(self.node.sc_status_ & SCRecord.PUMP_STATUS,
                        'SC Pump should be ON')
        self.assertTrue(self.node.sc_status_ & SCRecord.PUMP_TOGGLE,
                        'SC Pump Toggle should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_OVER_TEMP)
        self.assertFalse(self.node.sc_status_ & SCRecord.TETHER_POWER_REQUEST)
        self.assertTrue(self.node.sc_status_ & SCRecord.TETHER_POWER_STATUS,
                        'SC Tether Power should be ON')
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)
        self.assertFalse(self.node.sc_status_ & SCRecord.LR_FAULT)

        # Check pump toggle
        for n in range(1, 5):
            self.test_helper.run(statusCheckIterations)
            self.assertTrue(self.test_helper.run_status)
            self.assertEqual(preCmdIterations + 500 + n * statusCheckIterations,
                             self.test_helper.iterations)
            time.sleep(0.5)
            t, _ = clock.now().seconds_nanoseconds()
            self.assertEqual(t, int(self.test_helper.iterations * PHYSICS_STEP))

            if n % 2 == 1:
                self.assertFalse(self.node.sc_status_ & SCRecord.PUMP_TOGGLE,
                                 f'SC Pump Toggle should be OFF: (0x{self.node.sc_status_:x})')
            else:
                self.assertTrue(self.node.sc_status_ & SCRecord.PUMP_TOGGLE,
                                f'SC Pump Toggle should be ON: (0x{self.node.sc_status_:x})')

        # Check that valve command fails (controller returns BUSY)
        self.node.send_valve_command(2)
        self.assertEqual(self.node.valve_future_.result().result.value,
                         self.node.valve_future_.result().result.BUSY)

        # Run to allow Pump command to finish
        self.test_helper.run(postCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations + 500 + 4 * statusCheckIterations + postCmdIterations,
                         self.test_helper.iterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, int(self.test_helper.iterations * PHYSICS_STEP))

        # Check piston motion
        post_pump_range_finder = self.node.range_finder_

        self.assertGreater(post_pump_range_finder,
                           pre_pump_range_finder - 2.2 * 0.0254,
                           'Piston should retract 2 inches/min for 1 minute' +
                           ' -- decrease pump absement')

        self.assertLess(post_pump_range_finder,
                        pre_pump_range_finder - 1.8 * 0.0254,
                        'Piston should retract 2 inches/min for 1 minute' +
                        ' -- increase pump absement')

        # TODO(anyone) remove once TestFixture is fixed upstream
        self.test_helper.stop()

        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=600)
