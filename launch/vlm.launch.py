import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_api_type',
            default_value='ollama',
            description='Type of the VLM API. Must be one of: [openai, ollama, google-genai]'
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_api_model',
            default_value='qwen3:14b',
            description='Model to use for the VLM API. Example: gemma3:4b, gemini-1.5-pro:8b, etc.'
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vlm_api_host',
            default_value='http://192.168.0.102:11434',
            description='Host URL for the VLM API. Must be set for openai and ollama.'
        ))

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