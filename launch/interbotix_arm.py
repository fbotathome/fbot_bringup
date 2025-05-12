import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("interbotix_xsarm_moveit"), 'launch', 'xsarm_moveit.launch.py')
        ),
        launch_arguments={
            'robot_model': 'wx200',
            'hardware_type': 'fake',
        }.items()
    )

    return LaunchDescription([
        arm,
    ])