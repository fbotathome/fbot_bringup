import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    face_recog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_recognition"), 'launch', 'face_recognition.launch.py')
        
        ),
        launch_arguments={
            'use_remote':'true'
            'use_realsense':'false'
        }.items()
    )

    return LaunchDescription([
        face_recog_launch
    ])