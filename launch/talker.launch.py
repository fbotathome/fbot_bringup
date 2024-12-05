from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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