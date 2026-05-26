import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_remote_arg = DeclareLaunchArgument(
        'use_remote',
        default_value='false',
        description="If should run the node on remote"
    )

    launch_realsense_arg = DeclareLaunchArgument(
        'use_realsense',
        default_value='false',
        description="If should launch the realsense node"
    )

    launch_femtobolt_arg = DeclareLaunchArgument(
        'use_femtobolt',
        default_value='false',
        description="If should launch the femtobolt node"
    )

    face_recog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_recognition"), 'launch', 'face_recognition.launch.py')
        
        ),
        launch_arguments={
            'use_remote': LaunchConfiguration("use_remote"),
            'use_realsense':LaunchConfiguration("use_realsense"),
            'use_femtobolt':LaunchConfiguration("use_femtobolt"),
        }.items()
    )

    return LaunchDescription([
        config_remote_arg,
        launch_realsense_arg,
        launch_femtobolt_arg,
        face_recog_launch
    ])