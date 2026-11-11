"""Bringup of YOLOv8 object recognition: launches the recognition node and, optionally, the camera.

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
            description='Run the recognition on the RealSense; with use_camera, also start its driver'
        ),
        DeclareLaunchArgument(
            'use_femtobolt',
            default_value='false',
            description='Run the recognition on the Femto Bolt (default when none is selected); with use_camera, also start its driver'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            description='If should include camera.launch.py (camera drivers and, with use_remote, the point cloud reconstruction)'
        ),
    ]

    # use_femtobolt / use_realsense also select which recognition nodes run (one per camera,
    # Femto Bolt when none is selected); this holds even with use_camera:=false
    yolo_object_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_recognition'), 'launch', 'yolov8_object_recognition.launch.py')
        ),
        launch_arguments={
            'use_remote': LaunchConfiguration('use_remote'),
            'use_femtobolt': LaunchConfiguration('use_femtobolt'),
            'use_realsense': LaunchConfiguration('use_realsense'),
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
        yolo_object_recognition,
        camera,
    ])
