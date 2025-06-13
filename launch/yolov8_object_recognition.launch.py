import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_remote_ssh import FindPackageShareRemote
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

#TODO: Need to rethink the structure of the package

def generate_launch_description():
    config_file_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_recognition'),
        'config',
        'yolov8_object_recognition.yaml']
    )

    config_file_path = PathJoinSubstitution([
        get_package_share_directory('fbot_recognition'),
        'config',
        'yolov8_object_recognition.yaml']
    )

    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value=config_file_path,
        description='Path to the parameter file'
    )

    config_file_remote_arg = DeclareLaunchArgument(
        'remote_config',
        default_value=config_file_path_remote,
        description='Path to the parameter file'
    )

    config_remote_arg = DeclareLaunchArgument(
        'use_remote',
        default_value='false',
        description="If should run the node on remote"
    )

    launch_realsense_arg = DeclareLaunchArgument(
        'use_realsense',
        default_value='false',
        description="If should launch the camera node"
    )

    yolo_object_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_recognition"), 'launch', 'yolov8_object_recognition.launch.py')
        ),
        launch_arguments={
            'use_remote': LaunchConfiguration("use_remote"),
            'use_realsense': LaunchConfiguration('use_realsense'),
            'remote_config':LaunchConfiguration("remote_config"),
            "config": LaunchConfiguration("config")
        }.items()
    )

    return LaunchDescription([
        launch_realsense_arg,
        config_remote_arg,
        config_file_arg,
        config_file_remote_arg,
        yolo_object_recognition,
    ])