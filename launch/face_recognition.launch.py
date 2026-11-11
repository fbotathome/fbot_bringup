"""Bringup of face recognition: launches the recognition node and, optionally, the camera.

camera.launch.py is only included with use_camera:=true (default false), so that a task
that already includes camera.launch.py does not start it (and the point cloud
reconstruction) twice. use_remote applies to both the recognition node and the camera:
when true they run on the Jetson, and camera.launch.py runs the point cloud
reconstruction here. use_realsense / use_femtobolt select which camera driver
camera.launch.py starts (off by default, the cameras already run as a service on the Jetson).
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_remote',
            default_value='true',
            description='If should run the recognition (and the camera, with use_camera) on the remote Jetson machine'
        ),
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='If camera.launch.py (use_camera) should start the RealSense driver'
        ),
        DeclareLaunchArgument(
            'use_femtobolt',
            default_value='false',
            description='If camera.launch.py (use_camera) should start the Femto Bolt driver'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            description='If should include camera.launch.py (camera drivers and, with use_remote, the point cloud reconstruction)'
        ),
    ]

    face_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_recognition'), 'launch', 'face_recognition.launch.py')
        ),
        launch_arguments={
            'use_remote': LaunchConfiguration('use_remote'),
        }.items()
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_bringup'), 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'use_remote': LaunchConfiguration('use_remote'),
            'use_realsense': LaunchConfiguration('use_realsense'),
            'use_femtobolt': LaunchConfiguration('use_femtobolt'),
            'validate_config': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_camera'))
    )

    return LaunchDescription([
        *declared_arguments,
        face_recognition,
        camera,
    ])
