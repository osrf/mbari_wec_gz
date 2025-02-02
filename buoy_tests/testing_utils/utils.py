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
import os
from threading import Thread
import unittest

from ament_index_python.packages import get_package_share_directory

from buoy_api import Interface

from buoy_tests.srv import RunServer

from em import invoke as empy

# TODO(anyone) Put back when fixed upstream
# from gz.common import set_verbosity
# from gz.sim import TestFixture

import launch
from launch.substitutions import LaunchConfiguration
import launch.actions
from launch.actions import OpaqueFunction

from launch_ros.actions import Node as launchNode

import launch_testing
import launch_testing.actions

from rcl_interfaces.srv import GetParameters

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as rclpyNode
from rclpy.parameter import Parameter


def regenerate_models(context, *args, **kwargs):
    """
    Regenerate models in buoy_description to set parameters default unless otherwise specified.

    For use as an OpaqueFunction:
        dependent_nodes = [gazebo,
                           bridge,
                           robot_state_publisher,
                           rviz]
        override_params = dict(door_state='open',
                               inc_wave_spectrum=\
                                   'inc_wave_spectrum_type:MonoChromatic;A:1.0;T:12.0')
        launch.actions.OpaqueFunction(function=regenerate_models,
                                      args=dependent_nodes,
                                      kwargs=override_params)
    """
    print('Sim Parameter Overrides:', kwargs)
    # Find packages
    pkg_buoy_gazebo = get_package_share_directory('buoy_gazebo')
    pkg_buoy_description = get_package_share_directory('buoy_description')

    # Set templates to values
    # Find model templates
    model_dir = 'mbari_wec_base'
    empy_base_sdf_file = os.path.join(pkg_buoy_description,
                                      'models', model_dir, 'model.sdf.em')
    base_sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf')

    model_dir = 'mbari_wec'
    empy_sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf.em')
    sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf')

    # Find world file template
    empy_world_file = os.path.join(pkg_buoy_gazebo, 'worlds', 'mbari_wec.sdf.em')
    world_file = os.path.join(pkg_buoy_gazebo, 'worlds', 'mbari_wec_test.sdf')

    # fill mbari_wec_base model template with params
    supported_mbari_wec_base_params = ['door_state']
    mbari_wec_base_params = []
    for wec_base_param in supported_mbari_wec_base_params:
        if wec_base_param in kwargs:
            mbari_wec_base_params.extend(['-D',
                                          f"{wec_base_param} = '{kwargs[wec_base_param]}'"])

    mbari_wec_base_params.extend(['-o', base_sdf_file,
                                  empy_base_sdf_file])
    empy(mbari_wec_base_params)

    # fill mbari_wec world template with params
    supported_mbari_wec_world_params = ['physics_step',
                                        'physics_rtf',
                                        'initial_buoy_height']
    mbari_wec_world_params = []
    for world_param in supported_mbari_wec_world_params:
        if world_param in kwargs:
            mbari_wec_world_params.extend(['-D',
                                           f"{world_param} = {kwargs[world_param]}"])
    mbari_wec_world_params.extend(['-o', world_file,
                                   empy_world_file])
    empy(mbari_wec_world_params)

    # fill mbari_wec model template with params
    supported_mbari_wec_model_params = ['scale_factor',
                                        'inc_wave_seed',
                                        'battery_soc',
                                        'battery_emf',
                                        'initial_piston_position',
                                        'x_mean_pos',
                                        'inc_wave_spectrum']
    mbari_wec_model_params = []
    for world_param in supported_mbari_wec_model_params:
        if world_param in kwargs:
            if 'inc_wave_spectrum_type' in str(kwargs[world_param]):
                inc_wave_spectrum = kwargs[world_param].split(';')
                no_params = len(inc_wave_spectrum) > 1
                inc_wave_spectrum_type = inc_wave_spectrum[0].split(':')
                no_type = \
                    len(inc_wave_spectrum_type) < 2 or \
                    'None' in inc_wave_spectrum_type[1]
                if len(inc_wave_spectrum_type) == 2:
                    mbari_wec_model_params.extend(['-D',
                                                   f'{inc_wave_spectrum_type[0]} =' +
                                                   f"'{inc_wave_spectrum_type[1]}'"])
                else:
                    mbari_wec_model_params.extend(['-D',
                                                   f'{inc_wave_spectrum_type[0]} =' +
                                                   "''"])
                if not no_params and not no_type:
                    for spectrum_param in inc_wave_spectrum[1:]:
                        spectrum_param = spectrum_param.split(':')
                        if len(spectrum_param) < 2 or 'default' in spectrum_param[1]:
                            pass  # just default
                        elif len(spectrum_param) == 2:
                            name, value = spectrum_param[0], float(spectrum_param[1])
                            mbari_wec_model_params.extend(['-D',
                                                           f'{name} =' +
                                                           f'{value}'])
                        else:  # Custom Spectrum
                            name, values = spectrum_param[0], spectrum_param[1:]
                            values = [float(v) for v in values]
                            values_str_arr = ','.join([str(v) for v in values])
                            mbari_wec_model_params.extend(['-D',
                                                           f'{name} =' +
                                                           f"[{values_str_arr}]"])
            else:
                mbari_wec_model_params.extend(['-D',
                                               f"{world_param} = '{kwargs[world_param]}'"])
    mbari_wec_model_params.extend(['-o', sdf_file,
                                   empy_sdf_file])
    empy(mbari_wec_model_params)

    return args


def regenerate_default_models(context, *args):
    """
    Regenerate models in buoy_description to set parameters back to defaults.

    For use as an OpaqueFunction:
        dependent_nodes = [gazebo,
                           bridge,
                           robot_state_publisher,
                           rviz]
        launch.actions.OpaqueFunction(function=regenerate_default_models,
                                      args=dependent_nodes)
    """
    # Find packages
    pkg_buoy_gazebo = get_package_share_directory('buoy_gazebo')
    pkg_buoy_description = get_package_share_directory('buoy_description')

    # Set templates to default values
    regenerate_models = LaunchConfiguration('regenerate_models').perform(context)
    if regenerate_models != 'false':
        # Find model templates
        model_dir = 'mbari_wec_base'
        empy_base_sdf_file = os.path.join(pkg_buoy_description,
                                          'models', model_dir, 'model.sdf.em')
        base_sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf')

        model_dir = 'mbari_wec'
        empy_sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf.em')
        sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf')

        # Find world file template
        empy_world_file = os.path.join(pkg_buoy_gazebo, 'worlds', 'mbari_wec.sdf.em')
        world_file = os.path.join(pkg_buoy_gazebo, 'worlds', 'mbari_wec_test.sdf')

        # Return all files to defaults
        empy(['-o', base_sdf_file,
              empy_base_sdf_file])
        empy(['-o', sdf_file,
              empy_sdf_file])
        empy(['-o', world_file,
              empy_world_file])

        return args


def default_generate_test_description(server='fixture_server',
                                      enable_rosbag=False,
                                      rosbag_name=None,
                                      regen_models=False,
                                      regen_kwargs=None):

    # Test fixture
    gazebo_test_fixture = launchNode(
        package='buoy_tests',
        executable=server,
        output='screen',
        on_exit=launch.actions.Shutdown()
    )

    bridge = launchNode(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                        output='screen')

    if enable_rosbag:
        if rosbag_name is not None:
            rosbag = launch.actions.ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', rosbag_name, '-a'],
                output='screen'
            )
        else:
            rosbag = launch.actions.ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-a'],
                output='screen'
            )

        nodes = [gazebo_test_fixture,
                 bridge,
                 rosbag]
    else:
        nodes = [gazebo_test_fixture,
                 bridge]

    if regen_models:
        ld = launch.LaunchDescription([
            OpaqueFunction(function=regenerate_models,
                           args=nodes,
                           kwargs=regen_kwargs),
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest()
        ])
    else:
        ld = launch.LaunchDescription(nodes + [
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest()
        ])

    return ld, locals()


class BuoyPyTestInterface(Interface):

    def __init__(self, node_name='test_interface_py', wait_for_services=True, **kwargs):
        super().__init__(node_name, wait_for_services=True, **kwargs)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Spring data
        self.range_finder_ = 0.0
        self.sc_status_ = 0

        # Power data
        self.rpm_ = 0.0
        self.wind_curr_ = 0.0
        self.bias_curr_ = 0.0
        self.scale_ = 1.0
        self.retract_ = 0.6
        self.pc_status_ = 0

    def spring_callback(self, data):
        self.range_finder_ = data.range_finder
        self.sc_status_ = data.status

    def power_callback(self, data):
        self.rpm_ = data.rpm
        self.wind_curr_ = data.wcurrent
        self.bias_curr_ = data.bias_current
        self.scale_ = data.scale
        self.retract_ = data.retract
        self.pc_status_ = data.status

    """  TODO(anyone) put back when TestFixture fixed upstream
    def start(self):
        self.thread = Thread(target=rclpy.spin, args=(self,))
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.thread.join()
    """


class TestHelper(rclpyNode):

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

    def get_params_from_node(self, node_name, params):
        return asyncio.run(self._get_params_from_node(node_name, params))

    async def _get_params_from_node(self, node_name, params):
        srv_name = node_name + '/get_parameters'
        client = self.create_client(GetParameters, srv_name)
        while rclpy.ok() and not client.wait_for_service(0.1):
            pass
        req = GetParameters.Request()
        req.names = params
        future = client.call_async(req)
        await future
        resp = None
        if future.result is not None:
            resp = future.result()
        return resp


class BuoyPyTests(unittest.TestCase):

    NODE_NAME = 'test_interface_py'
    CLI_ARGS = None

    def setUp(self):
        rclpy.init()
        self.test_helper = TestHelper()
        self.node = BuoyPyTestInterface(self.NODE_NAME, cli_args=self.CLI_ARGS,
                                        automatically_declare_parameters_from_overrides=True)
        self.node.get_logger().info(repr(self.CLI_ARGS))
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
class BuoyPyTestAfterShutdown(unittest.TestCase):

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
