from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
