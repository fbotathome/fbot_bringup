import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_remote_ssh import FindPackageShareRemote
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_recognition'),
        'config',
        'yolo_ros.yaml']
    )

    config_file_path = PathJoinSubstitution([
        get_package_share_directory('fbot_recognition'),
        'config',
        'yolo_ros.yaml']
    )

    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value=config_file_path,
        description='Path to the parameter file'
    )

    config_file_remote_arg = DeclareLaunchArgument(
        'remote_config',
        default_value=config_file_path_remote,
        description='Path to the remote parameter file'
    )

    config_remote_arg = DeclareLaunchArgument(
        'use_remote',
        default_value='true',
        description="If should run the nodes on remote"
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

    use_tracking_arg = DeclareLaunchArgument(
        'use_tracking',
        default_value='False',
        description="Whether to activate tracking"
    )

    use_3d_arg = DeclareLaunchArgument(
        'use_3d',
        default_value='True',
        description="Whether to activate 3D detections"
    )

    yolo_ros_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_recognition"), 'launch', 'yolo_ros.launch.py')
        ),
        launch_arguments={
            'use_remote': LaunchConfiguration("use_remote"),
            'use_realsense': LaunchConfiguration('use_realsense'),
            'use_femtobolt': LaunchConfiguration('use_femtobolt'),
            'use_tracking': LaunchConfiguration('use_tracking'),
            'use_3d': LaunchConfiguration('use_3d'),
            'remote_config': LaunchConfiguration("remote_config"),
            'config': LaunchConfiguration("config"),
        }.items()
    )

    return LaunchDescription([
        config_remote_arg,
        launch_realsense_arg,
        launch_femtobolt_arg,
        use_tracking_arg,
        use_3d_arg,
        config_file_arg,
        config_file_remote_arg,
        yolo_ros_recognition,
    ])
