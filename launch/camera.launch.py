from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

def validate_camera_config(context):
    if context.launch_configurations.get('use_realsense', 'false') == 'false':
        raise Exception("All the camera nodes are disabled. Please enable at least one camera node.")
    
def get_declared_arguments():

    realsense_config_file_path = PathJoinSubstitution([
        get_package_share_directory('fbot_bringup'),
        'config',
        'realsense.yaml']
    )

    return [
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='If should launch the realsense'
        ),
        DeclareLaunchArgument(
            'realsense_config',
            default_value=realsense_config_file_path,
            description='If should run the node on remote'
        ),
        DeclareLaunchArgument(
            'name',
            default_value='camera',
            description='Camera node name'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='camera',
            description='Camera node namespace'
        ),
    ]

def generate_launch_description():

    declared_arguments = get_declared_arguments()

    realsense2_node = Node(
        package='realsense2_camera',
        namespace=LaunchConfiguration('namespace'),
        name=LaunchConfiguration('name'),
        executable='realsense2_camera_node',
        parameters=[LaunchConfiguration('realsense_config'),],
        remappings=[
            ('/camera/camera/color/image_raw', '/fbot_vision/color/image_raw'),
            ('/camera/camera/color/camera_info', 'fbot_vision/color/camera_info'),
            ('/camera/camera/color/metadata', '/fbot_vision/color/metadata'),
            ('/camera/camera/aligned_depth_to_color/image_raw', '/fbot_vision/aligned_depth_to_color/image_raw'),
            ('/camera/camera/aligned_depth_to_color/camera_info', '/fbot_vision/aligned_depth_to_color/camera_info'),
        ],
        emulate_tty=True,
        condition =IfCondition(LaunchConfiguration('use_realsense')),
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=validate_camera_config),
        realsense2_node,
    ])