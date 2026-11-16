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
            default_value='http://jetson:11434',
            description='Host URL for the VLM API. Must be set for openai and ollama.'
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_api_timeout',
            default_value='10',
            description='Seconds (integer) to wait for the primary VLM API before using the fallback.'
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_fallback_model',
            default_value='gemma4:latest',
            description="Local Ollama model used when the primary model fails ('none' disables it)."
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_fallback_host',
            default_value='http://jetson:11434',
            description='Ollama server for the fallback model (Jetson).'
        ))

    vlm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_vlm'), 'launch', 'vlm.launch.py')
        ),
        launch_arguments={
            'vlm_api_model': LaunchConfiguration('vlm_api_model'),
            'vlm_api_host': LaunchConfiguration('vlm_api_host'),
            'vlm_api_timeout': LaunchConfiguration('vlm_api_timeout'),
            'vlm_fallback_model': LaunchConfiguration('vlm_fallback_model'),
            'vlm_fallback_host': LaunchConfiguration('vlm_fallback_host'),
        }.items()
    )

    return LaunchDescription([
        *declared_arguments,
        vlm,
    ])
