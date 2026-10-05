"""Wheel odometry + IMU fusion (robot_localization EKF).

The EKF is the single owner of the odom -> base_footprint transform
(fbot_description/config/ekf.yaml, controller has enable_odom_tf: false).

  ros2 launch fbot_bringup localization.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(get_package_share_directory('fbot_description'), 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
