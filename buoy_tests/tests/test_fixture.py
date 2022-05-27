# Copyright 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# If you compiled Gazebo from source you should modify your
# `PYTHONPATH`:
#
# export PYTHONPATH=$PYTHONPATH:<path to ws>/install/lib/python
#
# Now you can run the example:
#
# python3 examples/scripts/python_api/helperFixture.py

import os
import time
import unittest

from gz.common import set_verbosity
from gz.gazebo import TestFixture, World, world_entity

import pytest


class PyTestFixture(unittest.TestCase):

    def setUp(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))

        self.helper = TestFixture(os.path.join(file_path, 'gravity.sdf'))

        self.post_iterations = 0
        self.iterations = 0
        self.pre_iterations = 0
        self.first_iteration = True

        self.helper.on_post_update(self.on_post_udpate_cb)
        self.helper.on_update(self.on_udpate_cb)
        self.helper.on_pre_update(self.on_pre_udpate_cb)
        self.helper.finalize()

    def on_pre_udpate_cb(self, _info, _ecm):
        self.pre_iterations += 1
        if self.first_iteration:
            self.first_iteration = False
            world_e = world_entity(_ecm)
            print('World entity is ', world_e)
            w = World(world_e)
            """
            v = w.gravity(_ecm)
            ^^ fails with:
            [224.080s] 6: E           TypeError: Unable to convert function return value to a
                                      Python type! The signature was
            [224.080s] 6: E                 (self: gz.sim.World,
                                             arg0: gz.sim.EntityComponentManager)
                                            -> Optional[gz::math::v6::Vector3<double>]
            """
            # print('Gravity ', v)
            modelEntity = w.model_by_name(_ecm, 'falling')
            print('Entity for falling model is: ', modelEntity)

    def on_udpate_cb(self, _info, _ecm):
        self.iterations += 1

    def on_post_udpate_cb(self, _info, _ecm):
        self.post_iterations += 1
        if _info.sim_time.seconds == 1:
            print('Post update sim time: ', _info.sim_time)

    @pytest.mark.xfail(reason='wait for fix upstream')
    def test_server_run(self):
        self.server = self.helper.server()
        blocking = False
        paused = False
        self.assertTrue(self.server.run(blocking, 1000, paused))
        while self.server.is_running():
            time.sleep(0.5)
        # TODO(anyone) This version does not return in docker; wait for upstream
        # self.server.run(True, 1000, False)

        self.assertTrue(self.server.run(blocking, 1000, paused))
        while self.server.is_running():
            time.sleep(0.5)

        print('iterations ', self.iterations)
        print('post_iterations ', self.post_iterations)
        print('pre_iterations ', self.pre_iterations)

        self.assertEqual(self.iterations, 2000)
