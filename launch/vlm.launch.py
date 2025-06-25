import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    vlm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_vlm"), 'launch', 'vlm.launch.py')
        ),
        launch_arguments={
            'vlm_api_type': 'ollama',
            'vlm_api_model': 'llama3.2-vision',
            'vlm_api_host': 'http://192.168.4.102:11434'
        }.items()
    )

    return LaunchDescription([
        vlm,
    ])