import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def validate_camera_config(context):
    if context.launch_configurations.get('use_realsense', 'false') == 'false':
        raise Exception("All the camera nodes are disabled. Please enable at least one camera node.")

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='If should launch the realsense'
        )
    )

    realsense2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': 'fbot_vision',
            'camera_name': 'camera',
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_realsense'))
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=validate_camera_config),
        realsense2_node,
    ])