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
            'use_sim',
            default_value='false',
            description='Use tools_test implementations for simulation.'
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            'llm_model',
            default_value='gemini/gemini-3.1-flash-lite',
            description='LLM model identifier for the agent.'
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/femtobolt/color/image_raw',
            description='Camera image topic for perception.'
        ))

    agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_agent'), 'launch', 'fbot_agent.launch.py')
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'llm_model': LaunchConfiguration('llm_model'),
            'camera_topic': LaunchConfiguration('camera_topic'),
        }.items()
    )

    return LaunchDescription([
        *declared_arguments,
        agent,
    ])