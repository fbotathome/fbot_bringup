"""Start the BORIS robot body. THE entry point for hardware bringup.

  ros2 launch fbot_bringup robot.launch.py                          # base + lasers + IMU + EKF
  ros2 launch fbot_bringup robot.launch.py use_navigation:=true map_file:=lab_2026_2.yaml
  ros2 launch fbot_bringup robot.launch.py use_neck:=true use_navigation:=true
  ros2 launch fbot_bringup robot.launch.py robot_version:=v1        # old BORIS (v2 is the default)
  ros2 launch fbot_bringup robot.launch.py use_slam:=true use_navigation:=true   # map while driving
  ros2 launch fbot_bringup robot.launch.py use_navigation:=true use_scan_watchdog:=false  # no /scan banners

Task launches (fbot_behavior) include this ONCE (instead of separate description /
navigation / neck launches), plus manipulator.launch.py when they use the arm.

  robot.launch.py
   |- base.launch.py          description + ros2_control (hoverboard) + diff drive
   |- sensors.launch.py       Hokuyo x2, IMU            (use_lasers, use_imu, use_sick)
   |- localization.launch.py  EKF                       (use_localization)
   |- navigation.launch.py    Nav2 / SLAM               (use_navigation, use_slam)
   |- scan_watchdog           warns if /scan is missing  (use_navigation + use_scan_watchdog)
   '- neck.launch.py          neck_controller + face    (use_neck)

The arm is NOT started here: tasks include manipulator.launch.py, which attaches
it to arm_mount_link (present by default, use_arm_mount).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
        DeclareLaunchArgument('robot_version', default_value='v2',
                              description='BORIS version (fbot_description/config/robot/<v>.yaml)'),
        DeclareLaunchArgument('use_lasers', default_value='true', description='Hokuyo lasers (/scan2, /scan3)'),
        DeclareLaunchArgument('use_imu', default_value='true', description='BNO055 IMU'),
        DeclareLaunchArgument('use_sick', default_value='false', description='Sick LMS (publishes /scan)'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyIMU', description='IMU serial port (udev symlink)'),
        DeclareLaunchArgument('use_localization', default_value='true', description='EKF (odom -> base_footprint)'),
        DeclareLaunchArgument('use_navigation', default_value='false', description='Nav2 (or SLAM with use_slam)'),
        DeclareLaunchArgument('use_slam', default_value='false', description='slam_toolbox instead of AMCL + map'),
        DeclareLaunchArgument('use_keepout_zones', default_value='false', description='Keepout zone filter'),
        DeclareLaunchArgument('map_file', default_value='lab_2026_2.yaml', description='Map in fbot_navigation/maps'),
        DeclareLaunchArgument('use_navigation_rviz', default_value='false', description='RViz2 with the nav config'),
        DeclareLaunchArgument('use_scan_watchdog', default_value='true',
                              description='With navigation: banner when /scan (Sick) is missing or the robot is not localized'),
        DeclareLaunchArgument('use_neck', default_value='false', description='Neck controller, face + neck in the URDF'),
        DeclareLaunchArgument('use_arm_mount', default_value='true',
                              description='Arm mounting plate in the URDF (parent of the arm, see manipulator.launch.py)'),
        DeclareLaunchArgument('arm_z_position', default_value='0.315', description='Arm plate height on the torso [m]'),
    ]

    base = include('base.launch.py', args={
        'robot_version': lc('robot_version'),
        'use_neck': lc('use_neck'),
        'use_arm_mount': lc('use_arm_mount'),
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
        'robot_version': lc('robot_version'),
    })
    neck = include('neck.launch.py', condition=IfCondition(lc('use_neck')))
    scan_watchdog = Node(
        package='fbot_bringup', executable='scan_watchdog', output='screen',
        condition=IfCondition(PythonExpression([
            "'", lc('use_navigation'), "'.lower() == 'true' and '", lc('use_scan_watchdog'), "'.lower() == 'true'",
        ])),
    )

    return LaunchDescription(declared + [base, sensors, localization, navigation, neck, scan_watchdog])
