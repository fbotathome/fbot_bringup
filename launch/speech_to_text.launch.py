import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

#THIS LAUNCHFILE IS FOR TESTING PURPOSES ONLY, IT SHOULD NOT BE USED FOR NOW.

def generate_launch_description():

    synthesizer_speech_arg = DeclareLaunchArgument(
        'challenge_config',
        default_value='fbot_stt_quiz',
        description="The configuration file for the speech to text challenge. "
    )

    synthesizer_speech = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_hri_bringup"), 'speech_to_text.launch.py')
        ),
        launch_arguments={
            'challenge_config': LaunchConfiguration('challenge_config', default='fbot_stt_quiz.yaml'),
        }.items()
    )

    return LaunchDescription([
        synthesizer_speech_arg,
        synthesizer_speech,
    ])