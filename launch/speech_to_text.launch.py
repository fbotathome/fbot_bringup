import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    speech_to_text_arg = DeclareLaunchArgument(
        'stt_config_file',
        default_value='fbot_stt_quiz.yaml',
        description="The configuration file for the speech to text challenge. "
    )

    speech_to_text = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_hri_bringup"), 'speech_to_text.launch.py')
        ),
        launch_arguments={
            'stt_config_file': LaunchConfiguration('stt_config_file'),
        }.items()
    )

    return LaunchDescription([
        speech_to_text_arg,
        speech_to_text,
    ])
