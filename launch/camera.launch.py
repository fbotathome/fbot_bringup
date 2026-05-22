import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory
from launch_remote_ssh import LaunchRemoteSSH

JETSON_USER = 'jetson'
JETSON_MACHINE = 'jetson'
JETSON_SETUP_BASH = '/home/jetson/jetson_ws/install/setup.bash'


def validate_camera_config(context):
    if (
        context.launch_configurations.get('validate_config', 'true') == 'true'
        and context.launch_configurations.get('use_realsense', 'false') == 'false'
        and context.launch_configurations.get('use_femtobolt', 'false') == 'false'
    ):
        raise Exception("All the camera nodes are disabled. Please enable at least one camera node.")


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_remote',
            default_value='true',
            description='If should run the camera nodes on the remote Jetson machine'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='If should launch the realsense'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_femtobolt',
            default_value='false',
            description='If should launch the femtobolt'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'validate_config',
            default_value='true',
            description='If should validate the camera configuration (at least one camera should be enabled)'
        )
    )

    realsense_launch_args = {
        'camera_namespace': 'fbot_vision',
        'camera_name': 'realsense',
        'enable_rgbd': 'true',
        'enable_sync': 'true',
        'align_depth.enable': 'true',
        'enable_color': 'true',
        'enable_depth': 'true',
        'pointcloud.enable': 'true',
    }

    femtobolt_launch_args = {
        'camera_namespace': 'fbot_vision',
        'camera_name': 'femtobolt',
        'enable_color': 'true',
        'enable_depth': 'true',
        'enable_ir': 'true',
        'depth_registration': 'true',
        'align_mode': 'SW',
        'enable_frame_sync': 'true',
        'enable_point_cloud': 'true',
        'enable_colored_point_cloud': 'true',
    }

    # ── Local nodes ────────────────────────────────────────────────────────────

    realsense_local = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments=realsense_launch_args.items(),
        condition=IfCondition(
            PythonExpression([
                "'true' if '", LaunchConfiguration('use_realsense'), "' == 'true' and '",
                LaunchConfiguration('use_remote'), "' == 'false' else 'false'"
            ])
        )
    )

    femtobolt_local = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'femto_bolt.launch.py')
        ),
        launch_arguments=femtobolt_launch_args.items(),
        condition=IfCondition(
            PythonExpression([
                "'true' if '", LaunchConfiguration('use_femtobolt'), "' == 'true' and '",
                LaunchConfiguration('use_remote'), "' == 'false' else 'false'"
            ])
        )
    )

    # ── Remote nodes (Jetson) ──────────────────────────────────────────────────

    realsense_remote = LaunchRemoteSSH(
        package='realsense2_camera',
        file='rs_launch.py',
        user=JETSON_USER,
        machine=JETSON_MACHINE,
        source_paths=[JETSON_SETUP_BASH],
        launch_arguments=realsense_launch_args.items(),
        condition=IfCondition(
            PythonExpression([
                "'true' if '", LaunchConfiguration('use_realsense'), "' == 'true' and '",
                LaunchConfiguration('use_remote'), "' == 'true' else 'false'"
            ])
        )
    )

    femtobolt_remote = LaunchRemoteSSH(
        package='orbbec_camera',
        file='femto_bolt.launch.py',
        user=JETSON_USER,
        machine=JETSON_MACHINE,
        source_paths=[JETSON_SETUP_BASH],
        launch_arguments=femtobolt_launch_args.items(),
        condition=IfCondition(
            PythonExpression([
                "'true' if '", LaunchConfiguration('use_femtobolt'), "' == 'true' and '",
                LaunchConfiguration('use_remote'), "' == 'true' else 'false'"
            ])
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=validate_camera_config),
        realsense_local,
        femtobolt_local,
        realsense_remote,
        femtobolt_remote,
    ])