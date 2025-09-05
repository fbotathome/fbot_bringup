import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file_name',
            default_value='pose',
            description='Name of the world configuration file'
        )
    )

    config_file_name = LaunchConfiguration('config_file_name')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_world"), 'launch', 'pose.launch.py')
        ),
        launch_arguments={
            'config_file_name': config_file_name
        }.items()
    )

    return LaunchDescription([
        *declared_arguments,
        world,
    ])
