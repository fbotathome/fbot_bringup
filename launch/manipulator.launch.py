"""Bring up the BORIS manipulator stack for a selected arm.

Single entry point that starts, for the chosen ``arm_type``:
  1. the arm's MoveIt bring-up (move_group + hardware driver), and
  2. the matching fbot_manipulator interface (motion-primitive node + MTC task server).

Examples:
    # WidowX 200 on BORIS (real hardware) -- the default
    ros2 launch fbot_bringup manipulator.launch.py arm_type:=wx200

    # UFACTORY xArm6 (real hardware)
    ros2 launch fbot_bringup manipulator.launch.py arm_type:=xarm6 robot_ip:=192.168.1.185
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'arm_type',
            default_value='wx200',
            choices=['wx200', 'xarm6'],
            description="Which manipulator to bring up: 'wx200' (Interbotix, BORIS arm) "
                        "or 'xarm6' (UFACTORY).",
        ),
        DeclareLaunchArgument(
            'hardware_type',
            default_value='actual',
            description="wx200 only: Interbotix hardware_type (actual / fake / gz_classic).",
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.208',
            description="xarm6 only: IP address of the xArm control box. Override to match "
                        "your robot.",
        ),
    ]

    arm_type = LaunchConfiguration('arm_type')
    hardware_type = LaunchConfiguration('hardware_type')
    robot_ip = LaunchConfiguration('robot_ip')

    is_wx200 = IfCondition(PythonExpression(["'", arm_type, "' == 'wx200'"]))
    is_xarm6 = IfCondition(PythonExpression(["'", arm_type, "' == 'xarm6'"]))

    wx200_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py',
        ])),
        launch_arguments={
            'robot_model': 'wx200',
            'hardware_type': hardware_type,
            'use_moveit_rviz': 'false',
            'use_world_frame': 'false',
        }.items(),
        condition=is_wx200,
    )

    wx200_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('fbot_manipulator'), 'launch', 'manipulator_interface_wx200.launch.py',
        ])),
        condition=is_wx200,
    )

    xarm6_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'), 'launch', 'xarm6_moveit_realmove.launch.py',
        ])),
        launch_arguments={
            'robot_ip': robot_ip,
            'add_mtc': 'true',
        }.items(),
        condition=is_xarm6,
    )

    xarm6_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('fbot_manipulator'), 'launch', 'manipulator_interface.launch.py',
        ])),
        launch_arguments={'arm_type': 'xarm6'}.items(),
        condition=is_xarm6,
    )

    return LaunchDescription(declared_arguments + [
        wx200_moveit,
        wx200_interface,
        xarm6_moveit,
        xarm6_interface,
    ])
