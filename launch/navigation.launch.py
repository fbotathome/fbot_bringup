"""Nav2 (AMCL + planners + costmaps) or SLAM, on top of an ALREADY RUNNING robot.

This launch does NOT start the robot description, ros2_control, sensors or EKF.
Use robot.launch.py use_navigation:=true to start everything at once.


  ros2 launch fbot_bringup navigation.launch.py map_file:=lab_2026_2.yaml
  ros2 launch fbot_bringup navigation.launch.py use_slam:=true
  ros2 launch fbot_bringup navigation.launch.py use_keepout_zones:=true

The robot footprint is taken from fbot_description/config/robot/<robot_version>.yaml
(key `footprint`) and written into the Nav2 params (leaf key `footprint`).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def _launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('fbot_navigation')

    use_keepout = LaunchConfiguration('use_keepout_zones').perform(context).lower() == 'true'
    params_name = 'nav2_params_keepout.yaml' if use_keepout else 'nav2_params.yaml'
    params_file = os.path.join(nav_share, 'param', params_name)

    robot_version = LaunchConfiguration('robot_version').perform(context)
    footprint_file = os.path.join(get_package_share_directory('fbot_description'), 'config', 'robot', f'{robot_version}.yaml')
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
    return [nav]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='false', description='Run slam_toolbox instead of AMCL + map'),
        DeclareLaunchArgument('map_file', default_value='lab_2026_2.yaml',
                              description='Map yaml inside fbot_navigation/maps (or an absolute path)'),
        DeclareLaunchArgument('use_keepout_zones', default_value='false', description='Enable keepout zone filter'),
        DeclareLaunchArgument('use_navigation_rviz', default_value='false', description='Start RViz2 with the nav config'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_version', default_value='v1', description='BORIS version (footprint source)'),
        OpaqueFunction(function=_launch_setup),
    ])
