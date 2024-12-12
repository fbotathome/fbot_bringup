from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
@file listener.launch.py
@brief Launch file for the listener node in the py_pubsub package.

This launch file declares two launch arguments:
- 'use_listener': A boolean flag to determine whether to launch the listener node.
- 'namespace': The namespace under which the listener node will be launched.

The listener node is launched with the following parameters:
- package: 'py_pubsub'
- executable: 'listener'
- name: 'listener'
- output: 'screen'
- namespace: The value of the 'namespace' launch argument
- condition: The listener node is launched only if the 'use_listener' launch argument is set to true.

@return LaunchDescription object containing the launch configuration.
"""

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument('use_listener', default_value='true'),
    DeclareLaunchArgument('namespace', default_value='namespace'),
    Node(
      package='py_pubsub',
      executable='listener',
      name='listener',
      output= 'screen',
      namespace= LaunchConfiguration('namespace'),
      condition=IfCondition(LaunchConfiguration('use_listener'))
    ),
  ])
