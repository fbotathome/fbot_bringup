import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("interbotix_xsarm_control"), 'launch', 'xsarm_control.launch.py')
        ),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model', default='wx200'),
            'hardware_type': LaunchConfiguration('hardware_type', default='actual'),
            'use_sim': LaunchConfiguration('use_sim', default='false'),
        }.items()
    )

    return LaunchDescription([
        arm,
    ])