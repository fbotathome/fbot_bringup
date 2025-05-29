import os

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

#THIS LAUNCHFILE IS FOR TESTING PURPOSES ONLY, IT SHOULD NOT BE USED FOR NOW.

def generate_launch_description():

    synthesizer_speech_arg = DeclareLaunchArgument(
        'launch_speec_to_text',
        default_value='true',
        description="If should launch the synthesizer speech node"
    )

    synthesizer_speech = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_hri_bringup"), 'speech_to_text.launch.py')
        
        ),
        launch_arguments={
            'use_remote': 'false',
        }.items()
    )

    return LaunchDescription([
        synthesizer_speech_arg,
        synthesizer_speech,
    ])