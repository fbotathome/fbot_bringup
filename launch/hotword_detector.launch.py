import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    hotword_config_file = DeclareLaunchArgument(
        'hotword_config_file',
        default_value='fbot_hotword_detection.yaml',
        description="If should run the node on remote"
    )
    
    hotword_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_hri_bringup"), 'hotword_detector.launch.py')
            ),
        launch_arguments={
            'hotword_config_file': LaunchConfiguration("hotword_config_file"),
        }.items())

    return LaunchDescription([
        hotword_config_file,
        hotword_detector,
    ])