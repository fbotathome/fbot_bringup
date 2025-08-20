import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_remote_arg = DeclareLaunchArgument(
        'use_remote',
        default_value='true',
        description="If should run the node on remote"
    )

    synthesizer_speech = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_hri_bringup"), 'asr_riva.launch.py')
        
        ),
        launch_arguments={
            'use_remote': LaunchConfiguration("use_remote"),
        }.items()
    )

    return LaunchDescription([
        config_remote_arg,
        synthesizer_speech,
    ])