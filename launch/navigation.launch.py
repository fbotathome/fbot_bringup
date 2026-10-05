"""Nav2 (AMCL + planners + costmaps) or SLAM, on top of an ALREADY RUNNING robot.

This launch does NOT start the robot description or ros2_control. Use
robot.launch.py use_navigation:=true to start everything at once.

TEMPORARY COMPAT: the old launch also started lasers, IMU and EKF. Until every
fbot_behavior task is migrated to robot.launch.py, with_robot_support defaults
to true so old task launches keep their sensors. robot.launch.py passes false.
Flip the default to false (then delete the arg) when the migration is done.

  ros2 launch fbot_bringup navigation.launch.py map_file:=lab_2026_2.yaml
  ros2 launch fbot_bringup navigation.launch.py use_slam:=true
  ros2 launch fbot_bringup navigation.launch.py use_keepout_zones:=true

The robot footprint is taken from fbot_description/config/footprint.yaml and
written into the Nav2 params (leaf key `footprint`).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def _launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('fbot_navigation')

    use_keepout = LaunchConfiguration('use_keepout_zones').perform(context).lower() == 'true'
    params_name = 'nav2_params_keepout.yaml' if use_keepout else 'nav2_params.yaml'
    params_file = os.path.join(nav_share, 'param', params_name)

    footprint_file = os.path.join(get_package_share_directory('fbot_description'), 'config', 'footprint.yaml')
    with open(footprint_file) as f:
        footprint = yaml.safe_load(f)['footprint']

    params_with_footprint = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'footprint': footprint},
        convert_types=False,
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'nav.launch.py')),
        launch_arguments={
            'use_slam': LaunchConfiguration('use_slam'),
            'use_keepout': LaunchConfiguration('use_keepout_zones'),
            'map_file': LaunchConfiguration('map_file'),
            'params_file': params_with_footprint,
            'use_rviz': LaunchConfiguration('use_navigation_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    share = get_package_share_directory('fbot_bringup')
    support = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            condition=IfCondition(LaunchConfiguration('with_robot_support')),
        )
        for name in ('sensors.launch.py', 'localization.launch.py')
    ]
    return support + [nav]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='false', description='Run slam_toolbox instead of AMCL + map'),
        DeclareLaunchArgument('map_file', default_value='lab_2026_2.yaml',
                              description='Map yaml inside fbot_navigation/maps (or an absolute path)'),
        DeclareLaunchArgument('use_keepout_zones', default_value='false', description='Enable keepout zone filter'),
        DeclareLaunchArgument('use_navigation_rviz', default_value='false', description='Start RViz2 with the nav config'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('with_robot_support', default_value='true',
                              description='TEMPORARY: also start sensors + EKF (old behaviour). robot.launch.py sets false'),
        DeclareLaunchArgument('use_description', default_value='false',
                              description='DEPRECATED, ignored: navigation never starts the robot description'),
        OpaqueFunction(function=_launch_setup),
    ])
