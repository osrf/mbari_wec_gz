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

"""Launch Gazebo world with a buoy."""

import os
from pathlib import Path
import time

import numpy as np

from ament_index_python.packages import get_package_share_directory

from em import invoke as empy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def spreading_factor_from_spotter_spread_deg(spread_deg):
    """Convert Spotter/SWIFT first-moment directional spread [deg] to cos^(2s) exponent."""
    spread_deg = np.asarray(spread_deg, dtype=float)
    scalar_input = np.ndim(spread_deg) == 0

    max_spread_deg = np.degrees(np.sqrt(2.0))

    if np.any(spread_deg < 0):
        raise ValueError('spread_deg must be >= 0 (use 0 to disable directional spreading)')

    if np.any(spread_deg[spread_deg > 0] > max_spread_deg):
        raise ValueError(
            f'spread_deg must be <= {max_spread_deg:.3f} deg for this mapping'
        )

    # 0 degrees means no directional spreading; return None for scalar.
    if scalar_input and spread_deg.item() == 0.0:
        return None

    spread_rad = np.radians(spread_deg)
    return 2.0 / (spread_rad ** 2) - 1.0


def regenerate_models(context, *args, **kwargs):
    """
    Regenerate models in buoy_description to set parameters default unless otherwise specified.

    For use as an OpaqueFunction:
        dependent_nodes = [gazebo,
                           bridge,
                           robot_state_publisher,
                           rviz]
        launch.actions.OpaqueFunction(function=regenerate_models,
                                      args=dependent_nodes)

    Will grab overridden parameters from launch arguments via context object
    """
    use_rosbag2 = LaunchConfiguration('rosbag2').perform(context)
    if use_rosbag2 == 'true':
        rosbag2 = generate_latest_rosbag2_symlink(context)
        args = args + (rosbag2,)

    regenerate_models = LaunchConfiguration('regenerate_models').perform(context)
    if regenerate_models == 'false':
        return args
    regenerate_default_models(context)

    supported_mbari_wec_base_params = ['door_state']
    supported_mbari_wec_world_params = ['physics_step',
                                        'physics_rtf']
    supported_mbari_wec_model_params = ['scale_factor',
                                        'inc_wave_seed',
                                        'inc_wave_dir',
                                        'battery_soc',
                                        'battery_emf',
                                        'x_mean_pos',
                                        'inc_wave_spectrum']
    supported_mbari_wec_ros_params = ['inc_wave_height_points']
    float_params = ['physics_step',
                    'physics_rtf',
                    'scale_factor',
                    'inc_wave_seed',
                    'inc_wave_dir',
                    'battery_soc',
                    'battery_emf',
                    'x_mean_pos']
    all_params = supported_mbari_wec_base_params \
        + supported_mbari_wec_world_params \
        + supported_mbari_wec_model_params \
        + supported_mbari_wec_ros_params
    override_params = {param: LaunchConfiguration(param).perform(context) for param in all_params}
    override_params = {k: v for k, v in override_params.items() if v != 'None'}
    print('Sim Parameter Overrides:', override_params)

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

    model_dir = 'mbari_wec_ros'
    empy_ros_sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf.em')
    ros_sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf')

    # Find world file template
    empy_world_file = os.path.join(pkg_buoy_gazebo, 'worlds', 'mbari_wec.sdf.em')
    world_file = os.path.join(pkg_buoy_gazebo, 'worlds', 'mbari_wec.sdf')

    # fill mbari_wec_base model template with params
    mbari_wec_base_params = []
    for wec_base_param in supported_mbari_wec_base_params:
        if wec_base_param in override_params:
            mbari_wec_base_params.extend(['-D',
                                          f'{wec_base_param}'
                                          + f" = '{override_params[wec_base_param]}'"])

    mbari_wec_base_params.extend(['-o', base_sdf_file,
                                  empy_base_sdf_file])
    empy(mbari_wec_base_params)

    # fill mbari_wec_ros model template with params
    mbari_wec_ros_params = []
    for wec_ros_param in supported_mbari_wec_ros_params:
        if wec_ros_param in override_params:
            inc_wave_height_points_ = override_params[wec_ros_param].split(';')
            inc_wave_height_points = []
            for point_str in inc_wave_height_points_:
                point = point_str.split(':')
                inc_wave_height_points.append([float(p) for p in point])
            mbari_wec_ros_params.extend(['-D',
                                          f'{wec_ros_param}'
                                          + f' = {inc_wave_height_points}'])

    mbari_wec_ros_params.extend(['-o', ros_sdf_file,
                                  empy_ros_sdf_file])
    print(f'{mbari_wec_ros_params = }')
    empy(mbari_wec_ros_params)

    # fill mbari_wec world template with params
    mbari_wec_world_params = []
    for world_param in supported_mbari_wec_world_params:
        if world_param in override_params:
            mbari_wec_world_params.extend(['-D',
                                           f'{world_param}'
                                           + f' = {float(override_params[world_param])}'])
    mbari_wec_world_params.extend(['-o', world_file,
                                   empy_world_file])
    empy(mbari_wec_world_params)

    # fill mbari_wec model template with params
    mbari_wec_model_params = []
    for world_param in supported_mbari_wec_model_params:
        if world_param in override_params:
            print(f'{world_param = }\n{override_params[world_param] = }')  # noqa: E202, E251
            if 'inc_wave_spectrum_type' in override_params[world_param]:
                inc_wave_spectrum = override_params[world_param].split(';')
                has_params = len(inc_wave_spectrum) > 1
                inc_wave_spectrum_type = inc_wave_spectrum[0].split(':')
                no_type = \
                    len(inc_wave_spectrum_type) < 2 \
                    or 'None' in inc_wave_spectrum_type[1]
                if len(inc_wave_spectrum_type) == 2:
                    mbari_wec_model_params.extend(['-D',
                                                   f'{inc_wave_spectrum_type[0]} = '
                                                   + f"'{inc_wave_spectrum_type[1]}'"])
                else:
                    mbari_wec_model_params.extend(['-D',
                                                   f'{inc_wave_spectrum_type[0]} ='
                                                   + "''"])
                if has_params and not no_type:
                    int_spectrum_params = {'n_phases'}
                    for spectrum_param in inc_wave_spectrum[1:]:
                        spectrum_param = spectrum_param.split(':')
                        if len(spectrum_param) < 2 or 'default' in spectrum_param[1]:
                            pass  # just default
                        elif len(spectrum_param) == 2:
                            name = spectrum_param[0]
                            if name == 'spreading_deg':
                                sf = spreading_factor_from_spotter_spread_deg(
                                    float(spectrum_param[1])
                                )
                                # spreading_deg=0 → sf=None → no directional spreading
                                value = None if sf is None else float(sf)
                                name = 'spreading_factor'
                            else:
                                value = int(spectrum_param[1]) if name in int_spectrum_params \
                                    else float(spectrum_param[1])
                            mbari_wec_model_params.extend(['-D',
                                                           f'{name} = '
                                                           + repr(value)])
                        else:  # Custom Spectrum
                            name, values = spectrum_param[0], spectrum_param[1:]
                            values = [float(v) for v in values]
                            values_str_arr = ','.join([str(v) for v in values])
                            mbari_wec_model_params.extend(['-D',
                                                           f'{name} = '
                                                           + f'[{values_str_arr}]'])
            else:
                mbari_wec_model_params.extend(['-D',
                                               f'{world_param}'
                                               + (f' = {float(override_params[world_param])}'
                                                  if world_param in float_params else
                                                  f" = '{override_params[world_param]}'")
                                               ])
    mbari_wec_model_params.extend(['-o', sdf_file,
                                   empy_sdf_file])
    empy(mbari_wec_model_params)

    return args


def regenerate_default_models(context, *args, **kwargs):
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
        world_file = os.path.join(pkg_buoy_gazebo, 'worlds', 'mbari_wec.sdf')

        # Return all files to defaults
        empy(['-o', base_sdf_file,
              empy_base_sdf_file])
        empy(['-o', sdf_file,
              empy_sdf_file])
        empy(['-o', world_file,
              empy_world_file])

    return args


def generate_latest_rosbag2_symlink(context):

    pbloghome = LaunchConfiguration('pbloghome').perform(context)
    pbloghome = Path(pbloghome)
    rosbag2_home = pbloghome / 'rosbag2'

    rosbag2_dir = 'rosbag2_' + str(time.strftime('%Y%m%d%H%M%S'))
    rosbag2_dir = rosbag2_home / rosbag2_dir
    rosbag2_dir = str(rosbag2_dir)

    # python workaround for 'ln -sf'
    os.symlink(rosbag2_dir,
               str(pbloghome / 'latest_rosbag_temp'), target_is_directory=True)
    os.replace(str(pbloghome / 'latest_rosbag_temp'),
               str(pbloghome / 'latest_rosbag'))

    # record all topics with rosbag2
    rosbag2 = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-s', 'mcap',
            '-o', rosbag2_dir,
            '-a'
        ],
        output='screen',
        # Probably don't need this since we won't even enter this func if rosbag2:=false
        condition=IfCondition(LaunchConfiguration('rosbag2'))
    )

    return rosbag2


def generate_launch_description():

    gazebo_world_file_launch_arg = DeclareLaunchArgument(
        'world_file', default_value=['mbari_wec.sdf'],
        description='Gazebo world filename.sdf'
    )

    gazebo_world_name_launch_arg = DeclareLaunchArgument(
        'world_name', default_value=['mbari_wec_world'],
        description='Gazebo <world name>'
    )

    # rviz_launch_arg = DeclareLaunchArgument(
    #     'rviz', default_value='false',
    #     description='Open RViz.'
    # )

    gazebo_debugger_arg = DeclareLaunchArgument(
        'debugger', default_value='false',
        description='run gazebo in gdb'
    )

    extra_gz_args = DeclareLaunchArgument(
        'extra_gz_args', default_value='',
        description='extra_gz_args'
    )

    regenerate_models_arg = DeclareLaunchArgument(
        'regenerate_models', default_value='true',
        description='regenerate template models using defaults'
    )
    supported_params = {'door_state': 'open or closed',
                        'physics_step': 'step size in seconds',
                        'physics_rtf': 'sim real-time factor',
                        'scale_factor': 'target winding current scale factor',
                        'inc_wave_seed': 'random seed for incident wave computation',
                        'inc_wave_height_points': 'points where wave height is reported defined as'
                                                + ' x1:y1;x2:y2;...'
                                                + ' (x, y: local cartesian coords relative to buoy'
                                                + ' origin in meters)',
                        'inc_wave_dir': 'incident wave direction (compass deg True, waves FROM)',
                        'battery_soc': 'initial battery state of charge as pct (0-1)',
                        'battery_emf': 'initial battery emf in Volts',
                        'x_mean_pos': 'desired mean piston position in meters',
                        'inc_wave_spectrum': 'incident wave spectrum defined as'
                                             + ' inc_wave_spectrum_type:type;'
                                             + 'p1:v1[:v2...];p2:v1[:v2...]'
                                             + ' (e.g. Bretschneider supports Hs, Tp,'
                                             + ' n_phases, spreading_deg -> spreading_factor)'}
    supported_params_args = []
    for param in supported_params:
        supported_params_args.append(
            DeclareLaunchArgument(
                param, default_value='None',
                description=supported_params[param]
            )
        )

    home_path = Path.home()
    pblog_home_default = home_path / '.pblogs'

    pblog_loghome_launch_arg = DeclareLaunchArgument(
        'pbloghome', default_value=[str(pblog_home_default)],
        description='root pblog directory'
    )

    pblog_logdir_launch_arg = DeclareLaunchArgument(
        'pblogdir', default_value=[''],
        description='specific pblog directory in pbloghome'
    )

    rosbag2_launch_arg = DeclareLaunchArgument(
        'rosbag2', default_value='false',
        description='save rosbags to <pblogdir>/rosbag2/rosbag2_<datetime>'
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_buoy_gazebo = get_package_share_directory('buoy_gazebo')
    pkg_pblog = get_package_share_directory('sim_pblog')
    # pkg_buoy_description = get_package_share_directory('buoy_description')
    # model_dir = 'mbari_wec_ros'
    model_name = 'MBARI_WEC_ROS'
    # ros_sdf_file = os.path.join(pkg_buoy_description, 'models', model_dir, 'model.sdf')

    # with open(ros_sdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [
            LaunchConfiguration('extra_gz_args'), PathJoinSubstitution([' ']),
            PathJoinSubstitution([
                pkg_buoy_gazebo,
                'worlds',
                LaunchConfiguration('world_file')
                ])
            ],
            'debugger': LaunchConfiguration('debugger'),
            'on_exit_shutdown': 'True'}.items(),
    )

    # Bridge to forward tf and joint states to ros2
    link_pose_gz_topic = '/model/' + model_name + '/pose'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            ['/world/', LaunchConfiguration('world_name'), '/model/', model_name, '/joint_state',
             '@', 'sensor_msgs/msg/JointState', '[', 'gz.msgs.Model'],
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            (['/world/', LaunchConfiguration('world_name'), '/model/', model_name, '/joint_state'],
                'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    pblog = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pblog, 'launch', 'sim_pblog.launch.py'),
        ),
        launch_arguments={'loghome': LaunchConfiguration('pbloghome'),
                          'logdir': LaunchConfiguration('pblogdir')}.items(),
    )

    # TODO(andermi) get this running again?
    # # Get the parser plugin convert sdf to urdf using robot_description topic
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': robot_desc},
    #     ]
    # )

    # # Launch rviz
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(pkg_buoy_gazebo, 'rviz', 'mbari_wec.rviz')],
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     parameters=[
    #         {'use_sim_time': True},
    #     ]
    # )

    # Generate files before running any nodes
    dependent_nodes = [gazebo,
                       bridge,
                       pblog]
    # robot_state_publisher,
    # rviz]

    return LaunchDescription(supported_params_args + [
        gazebo_world_file_launch_arg,
        gazebo_world_name_launch_arg,
        pblog_loghome_launch_arg,
        pblog_logdir_launch_arg,
        rosbag2_launch_arg,
        # rviz_launch_arg,
        gazebo_debugger_arg,
        extra_gz_args,
        regenerate_models_arg,
        OpaqueFunction(function=lambda context, *args, **kwargs: args,
                       args=supported_params_args),
        OpaqueFunction(function=regenerate_models,
                       args=dependent_nodes),  # generate files before running nodes
    ])
