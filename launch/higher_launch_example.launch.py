from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

"""
@package fbot_bringup
@file higher_launch_example.launch.py
@brief Launch file for including another launch file with configuration.

This launch file is responsible for including the 'super_version_launch_poc.launch.py' 
from the 'py_pubsub' package and passing the path to the configuration file as an argument.

Functions:
    generate_launch_description(): Generates and returns the launch description.

@return LaunchDescription: The launch description including the specified launch file and configuration.
"""

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('py_pubsub')
    config_file = str(Path(package_share_dir) / 'config/config.yaml')

    # Return the launch description
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('py_pubsub'), 'super_version_launch_poc.launch.py']),
            launch_arguments=[('config_file_path', config_file)]
        )
    ])
