import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

#TODO: Need to rethink the structure of the package

def generate_launch_description():

    yolov8_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_recognition"), 'launch', 'yolov8_object_recognition.launch.py')
        ),
        launch_arguments={
            'use_remote': 'true',
            'use_realsense': 'false',
        }.items()
    )

    return LaunchDescription([
        yolov8_recognition,
    ])
