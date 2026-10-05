"""Interbotix arm (optional; BORIS no longer ships with the WX200 by default).

  ros2 launch fbot_bringup arm.launch.py robot_model:=wx200

Needs the description built with use_arm_mount:=true (robot.launch.py does this
automatically when use_arm:=true): the arm base is attached to arm_mount_link.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('interbotix_xsarm_control'), 'launch', 'xsarm_control.launch.py')
        ),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'hardware_type': LaunchConfiguration('hardware_type'),
            'use_sim': LaunchConfiguration('use_sim'),
            'use_rviz': 'false',
            'use_world_frame': 'false',
        }.items(),
    )

    # arm base pose on the mounting plate (same transform BORIS used before)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_arm_mount_to_arm_base',
        arguments=['0.1', '0', '0.0235', '0', '0', '0', 'arm_mount_link', 'wx200/base_link'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='wx200'),
        DeclareLaunchArgument('hardware_type', default_value='actual'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        arm,
        static_tf,
    ])
