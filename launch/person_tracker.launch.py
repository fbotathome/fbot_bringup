#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tracker_type_arg = DeclareLaunchArgument(
        'tracker_type', default_value='classic',
        description='Person tracker to launch: classic (yolo_tracker_recognition) or fusion (camera_lidar_person_tracker)')

    use_remote_arg = DeclareLaunchArgument(
        'use_remote', default_value='false', description='Run classic tracker remotely (only applies to classic)')

    config_arg = DeclareLaunchArgument(
        'config', default_value=os.path.join(get_package_share_directory('fbot_recognition'), 'config', 'yolo_tracker_recognition.yaml'),
        description='Path to tracker parameter YAML (shared)')

    tracker_type = LaunchConfiguration('tracker_type')

    # Classic YOLO tracker (backwards compatible)
    classic_node = Node(
        package='fbot_recognition',
        executable='yolo_tracker_recognition',
        name='yolo_tracker_recognition',
        parameters=[LaunchConfiguration('config')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('tracker_type'), "' == 'classic'"]))
    )

    # Fusion tracker (camera + lidar) new implementation
    fusion_node = Node(
        package='fbot_recognition',
        executable='camera_lidar_person_tracker',
        name='camera_lidar_person_tracker',
        parameters=[
            {'model_file': 'yolo11n-pose'},  # override if needed
            {'fusion.lidar.min_points': 15},
            {'fusion.lidar.use': True},
            {'fusion.priority': 'lidar'}
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('tracker_type'), "' == 'fusion'"]))
    )

    return LaunchDescription([
        tracker_type_arg,
        use_remote_arg,
        config_arg,
        classic_node,
        fusion_node
    ])
