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
import time

from ament_index_python.packages import get_package_share_directory

from buoy_api.examples.bias_damping import NLBiasDampingPolicy

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from testing_utils import BuoyPyTestAfterShutdown  # noqa F401 -- runs if imported
from testing_utils import BuoyPyTests
from testing_utils import default_generate_test_description


def generate_test_description():
    pkg_buoy_examples = get_package_share_directory('buoy_examples')

    bias_damping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buoy_examples, 'launch', 'bias_damping_py.launch.py'),
        ),
    )

    ld, locals_ = default_generate_test_description(server='fixture_server_sinusoidal_piston')
    ld.add_entity(bias_damping)

    return ld, {**locals_, 'bias_damping': bias_damping}


class BuoyPCBiasDampingPyTest(BuoyPyTests):

    def test_pc_bias_damping_ros(self, gazebo_test_fixture, proc_info):
        time.sleep(0.5)
        clock = self.node.get_clock()
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, 0)
        self.assertEqual(self.test_helper.iterations, 0)

        preCmdIterations = 1000
        testIterations = 20000
        feedbackCheckIterations = 500

        # set PC feedback rate and let server process
        # and let bias damping node load up
        self.node.set_pc_pack_rate_param(50.0)
        self.test_helper.run(preCmdIterations)
        self.assertTrue(self.test_helper.run_status)
        self.assertEqual(preCmdIterations,
                         self.test_helper.iterations)
        time.sleep(0.5)
        t, _ = clock.now().seconds_nanoseconds()
        self.assertEqual(t, self.test_helper.iterations // 1000)

        bias_policy_ = NLBiasDampingPolicy()
        params = self.test_helper.get_params_from_node('pb_nl_bias_damping',
                                                       ['bias_damping.bias',
                                                        'bias_damping.position_breaks',
                                                        'bias_damping.position_deadzone'])
        bias_policy_.bias = params.values[0].double_array_value
        bias_policy_.breaks = params.values[1].double_array_value
        bias_policy_.deadzone = params.values[2].double_array_value
        bias_policy_.update_params()
        self.assertIsNotNone(bias_policy_.bias_interp1d)
        policy_config = f"""bias: {bias_policy_.bias}
breaks: {bias_policy_.breaks}
deadzone: {bias_policy_.deadzone}"""
        print(policy_config)

        # check bias current at intervals while
        # sinusoidal force plugin is running in fixture
        prev_is_None = False
        for idx, _ in enumerate(range(0, testIterations, feedbackCheckIterations)):
            self.test_helper.run(feedbackCheckIterations)
            self.assertTrue(self.test_helper.run_status)
            self.assertEqual(preCmdIterations + (idx + 1) * feedbackCheckIterations,
                             self.test_helper.iterations)
            time.sleep(0.5)
            t, _ = clock.now().seconds_nanoseconds()
            self.assertEqual(t, self.test_helper.iterations // 1000)

            expected_bias_curr = bias_policy_.bias_current_target(self.node.range_finder_)
            if expected_bias_curr is not None:
                if prev_is_None:
                    prev_is_None = False
                    continue
                self.assertGreater(self.node.bias_curr_, expected_bias_curr - 2.0)
                self.assertLess(self.node.bias_curr_, expected_bias_curr + 2.0)
            else:
                self.assertTrue(bias_policy_.deadzone[0] <
                                self.node.range_finder_ <
                                bias_policy_.deadzone[1])
                prev_is_None = True

        # TODO(anyone) remove once TestFixture is fixed upstream
        self.test_helper.stop()

        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=30)
