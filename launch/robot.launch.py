"""Start the BORIS robot body. THE entry point for hardware bringup.

  ros2 launch fbot_bringup robot.launch.py                          # base + lasers + IMU + EKF
  ros2 launch fbot_bringup robot.launch.py use_navigation:=true map_file:=lab_2026_2.yaml
  ros2 launch fbot_bringup robot.launch.py use_neck:=true use_navigation:=true
  ros2 launch fbot_bringup robot.launch.py base_version:=v2         # new Shark base
  ros2 launch fbot_bringup robot.launch.py use_arm:=true            # wx200 (optional)
  ros2 launch fbot_bringup robot.launch.py use_slam:=true use_navigation:=true   # map while driving

Task launches (fbot_behavior) should include this ONCE instead of separate
description / navigation / neck launches.

  robot.launch.py
   |- base.launch.py          description + ros2_control (hoverboard) + diff drive
   |- sensors.launch.py       Hokuyo x2, IMU            (use_lasers, use_imu, use_sick)
   |- localization.launch.py  EKF                       (use_localization)
   |- navigation.launch.py    Nav2 / SLAM               (use_navigation, use_slam)
   |- neck.launch.py          neck_controller + face    (use_neck)
   '- arm.launch.py           Interbotix arm            (use_arm)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('fbot_bringup')

    def include(name, condition=None, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments=(args or {}).items(),
            condition=condition,
        )

    lc = LaunchConfiguration

    declared = [
        DeclareLaunchArgument('base_version', default_value='v1',
                              description='Shark base version (fbot_description/config/base/<v>.yaml)'),
        DeclareLaunchArgument('use_lasers', default_value='true', description='Hokuyo lasers (/scan2, /scan3)'),
        DeclareLaunchArgument('use_imu', default_value='true', description='BNO055 IMU'),
        DeclareLaunchArgument('use_sick', default_value='false', description='Sick LMS (publishes /scan)'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB2', description='IMU serial port'),
        DeclareLaunchArgument('use_localization', default_value='true', description='EKF (odom -> base_footprint)'),
        DeclareLaunchArgument('use_navigation', default_value='false', description='Nav2 (or SLAM with use_slam)'),
        DeclareLaunchArgument('use_slam', default_value='false', description='slam_toolbox instead of AMCL + map'),
        DeclareLaunchArgument('use_keepout_zones', default_value='false', description='Keepout zone filter'),
        DeclareLaunchArgument('map_file', default_value='lab_2026_2.yaml', description='Map in fbot_navigation/maps'),
        DeclareLaunchArgument('use_navigation_rviz', default_value='false', description='RViz2 with the nav config'),
        DeclareLaunchArgument('use_neck', default_value='false', description='Neck controller, face + neck in the URDF'),
        DeclareLaunchArgument('use_arm', default_value='false', description='Interbotix arm (adds the arm plate to the URDF)'),
        DeclareLaunchArgument('arm_z_position', default_value='0.34', description='Arm plate height on the torso [m]'),
    ]

    base = include('base.launch.py', args={
        'base_version': lc('base_version'),
        'use_neck': lc('use_neck'),
        'use_arm_mount': lc('use_arm'),
        'arm_z_position': lc('arm_z_position'),
    })
    sensors = include('sensors.launch.py', args={
        'use_lasers': lc('use_lasers'),
        'use_imu': lc('use_imu'),
        'use_sick': lc('use_sick'),
        'imu_port': lc('imu_port'),
    })
    localization = include('localization.launch.py', condition=IfCondition(lc('use_localization')))
    navigation = include('navigation.launch.py', condition=IfCondition(lc('use_navigation')), args={
        'use_slam': lc('use_slam'),
        'use_keepout_zones': lc('use_keepout_zones'),
        'map_file': lc('map_file'),
        'use_navigation_rviz': lc('use_navigation_rviz'),
    })
    neck = include('neck.launch.py', condition=IfCondition(lc('use_neck')))
    arm = include('arm.launch.py', condition=IfCondition(lc('use_arm')))

    return LaunchDescription(declared + [base, sensors, localization, navigation, neck, arm])
