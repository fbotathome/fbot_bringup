"""Robot base: robot_description + ros2_control (hoverboard) + diff drive controller.

Normally included by boris.launch.py. Run it alone to test the base only:

  ros2 launch fbot_bringup base.launch.py
  ros2 launch fbot_bringup base.launch.py robot_version:=v1 use_neck:=false

Geometry (wheel radius / separation) comes ONLY from
fbot_description/config/robot/<robot_version>.yaml. It is merged into the controller
parameters here, so boris_controllers.yaml does not contain it.

Topics: /cmd_vel (in), /odom (out), /joint_states (BORIS joints: wheels + neck).

Everything ros2_control-related runs in the `base` namespace
(/base/controller_manager, /base/hoverboard_base_controller, /base/joint_states) so
it can run next to the arm's own controller manager (manipulator.launch.py).
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


BASE_NS = 'base'
HOVERBOARD_TOPICS = [
    'emergency_button',
    'hoverboard/connected',
    'hoverboard/battery_voltage',
    'hoverboard/temperature',
] + [f'hoverboard/{side}_wheel/{field}' for side in ('left', 'right') for field in ('velocity', 'position', 'cmd')]
CONTROLLER_MANAGER = f'/{BASE_NS}/controller_manager'


def _merged_controller_params(robot_version: str) -> str:
    """boris_controllers.yaml + wheel geometry of the selected base -> temp yaml path."""
    description_share = get_package_share_directory('fbot_description')
    base_cfg_path = os.path.join(description_share, 'config', 'robot', f'{robot_version}.yaml')
    if not os.path.isfile(base_cfg_path):
        raise RuntimeError(f"Unknown robot_version '{robot_version}': {base_cfg_path} does not exist")
    with open(base_cfg_path) as f:
        base = yaml.safe_load(f)

    ctrl_path = os.path.join(description_share, 'config', 'boris_controllers.yaml')
    with open(ctrl_path) as f:
        params = yaml.safe_load(f)

    controller = params['hoverboard_base_controller']['ros__parameters']
    controller['wheel_radius'] = float(base['wheel']['radius'])
    controller['wheel_separation'] = float(base['wheel']['separation'])

    # the controller manager runs in the `base` namespace: use fully-qualified node names
    params = {f'/{BASE_NS}/{node}': value for node, value in params.items()}

    out = tempfile.NamedTemporaryFile('w', prefix=f'fbot_controllers_{robot_version}_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def _launch_setup(context, *args, **kwargs):
    robot_version = LaunchConfiguration('robot_version').perform(context)
    controllers_file = _merged_controller_params(robot_version)

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                PathJoinSubstitution([FindPackageShare('fbot_description'), 'urdf', 'boris.urdf.xacro']),
                ' robot_version:=', robot_version,
                ' use_neck:=', LaunchConfiguration('use_neck'),
                ' use_arm_mount:=', LaunchConfiguration('use_arm_mount'),
                ' arm_z_position:=', LaunchConfiguration('arm_z_position'),
            ]),
            value_type=str,
        )
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=BASE_NS,
        parameters=[robot_description, controllers_file],
        output='both',
        remappings=[
            (f'/{BASE_NS}/hoverboard_base_controller/cmd_vel_unstamped', '/cmd_vel'),
            (f'/{BASE_NS}/hoverboard_base_controller/odom', '/odom'),
            ('~/robot_description', '/robot_description'),
            # the hoverboard_driver plugin publishes its own status topics; keep them in the
            # root namespace (neck_controller listens to /emergency_button:
            # true = released, false = pressed/stopped)
            *[(f'/{BASE_NS}/{t}', f'/{t}') for t in HOVERBOARD_TOPICS],
            # joint_state_broadcaster publishes /base/joint_states; the BORIS
            # joint_state_publisher below merges it with the neck into /joint_states.
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='boris_joint_state_publisher',   # the arm stack has its own joint_state_publisher
        output='both',
        parameters=[{
            'source_list': [f'/{BASE_NS}/joint_states', '/boris_head/joint_states'],
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', CONTROLLER_MANAGER],
    )

    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hoverboard_base_controller', '--controller-manager', CONTROLLER_MANAGER],
    )

    # start the base controller only after the broadcaster is up
    delay_base_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[base_controller_spawner],
        )
    )

    return [
        control_node,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_broadcaster_spawner,
        delay_base_controller,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_version', default_value='v2',
                              description='BORIS version: fbot_description/config/robot/<robot_version>.yaml'),
        DeclareLaunchArgument('use_neck', default_value='true',
                              description='Include the neck + camera mount in the robot description'),
        DeclareLaunchArgument('use_arm_mount', default_value='true',
                              description='Include the empty arm mounting plate in the description'),
        DeclareLaunchArgument('arm_z_position', default_value='0.315',
                              description='v1: height of the arm plate on the torso [m] (v2: arm on the base, from the yaml)'),
        OpaqueFunction(function=_launch_setup),
    ])
