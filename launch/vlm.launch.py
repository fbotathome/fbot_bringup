import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_api_model',
            default_value='gemini/gemini-3.1-flash-lite',
            description='Model to use for the VLM API. Example: gemma3:4b, gemini-1.5-pro:8b, etc.'
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_api_host',
            default_value='http://localhost:11434',
            description='Host URL for the VLM API. Must be set for openai and ollama.'
        ))

    vlm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_vlm'), 'launch', 'vlm.launch.py')
        ),
        launch_arguments={
            'vlm_api_model': LaunchConfiguration('vlm_api_model'),
            'vlm_api_host': LaunchConfiguration('vlm_api_host'),
        }.items()
    )

    return LaunchDescription([
        *declared_arguments,
        vlm,
    ])