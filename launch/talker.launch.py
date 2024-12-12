from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
@file talker.launch.py
@brief Launch file for the 'talker' node in the 'py_pubsub' package.

This launch file declares launch arguments:
- 'use_talker': A boolean to determine whether to launch the 'talker' node.
- 'name': The name to assign to the 'talker' node.
- 'namespace': The namespace to assign to the 'talker' node.

The 'talker' node is launched with the specified name and namespace if 'use_talker' is set to true.

@return LaunchDescription object containing the launch configuration.
"""

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument('use_talker', default_value='true'),
    DeclareLaunchArgument('name', default_value='talker'),
    DeclareLaunchArgument('namespace', default_value='namespace'),
    Node(
      package='py_pubsub',
      executable='talker',
      name=LaunchConfiguration('name'),
      namespace = LaunchConfiguration('namespace'),
      condition=IfCondition(LaunchConfiguration('use_talker'))
    ),
  ])