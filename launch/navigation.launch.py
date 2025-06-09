import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'navigation.launch.py')

        ),
        launch_arguments={
            'use_description': 'false',
            'use_rviz': 'false'
        }.items()
    )

    return LaunchDescription([
        navigation,
    ])