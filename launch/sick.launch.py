"""Sick LMS1xx laser (sick_scan_xd). Publishes /scan in frame sick_laser.

Normally started by sensors.launch.py use_sick:=true. Driver settings (IP,
angles, ...) are in config/sick_lms_1xx.launch, the sick_scan_xd launch format.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_config = os.path.join(get_package_share_directory('fbot_bringup'), 'config', 'sick_lms_1xx.launch')

    return LaunchDescription([
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            output='screen',
            # frame_id must be the URDF link of the Sick (fbot_description urdf/sensors/sick_lms1xx.xacro)
            arguments=[driver_config, 'frame_id:=sick_laser', 'laserscan_topic:=scan'],
        ),
    ])
