import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_nav = LaunchConfiguration('use_nav', default='true')
    # [ADICIONADO] Argumento opcional para abrir a interface grafica do braco (default: true)
    use_rqt = LaunchConfiguration('use_rqt', default='true')

    # [ADICIONADO] Configura o IGN_GAZEBO_RESOURCE_PATH automaticamente apenas durante a simulacao
    home_dir = os.path.expanduser('~')
    ign_resource_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join([
            os.path.join(home_dir, 'fbot_ws/install/interbotix_xsarm_descriptions/share'),
            os.path.join(home_dir, 'fbot_ws/install/sensors_description/share'),
            os.path.join(home_dir, 'fbot_ws/install/shark_description/share'),
            os.path.join(home_dir, 'fbot_ws/install/boris_head_description/share'),
            os.path.join(home_dir, 'fbot_ws/install/fbot_simulation/share/fbot_simulation/models'),
            os.path.join(home_dir, 'fbot_ws/src/fbot_simulation/models'),
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ]),
    )

    world_file = os.path.expanduser('~/fbot_ws/src/fbot_simulation/worlds/arena_env.world')
    bridge_file = os.path.expanduser('~/fbot_bridge_teste.yaml')
    nav_pkg = get_package_share_directory('fbot_navigation')
    # map_file = os.path.join(nav_pkg, 'maps', 'arena_home_OPL_V2.yaml')
    map_file = os.path.join(nav_pkg, 'maps', 'arena_gazebo.yaml')
    param_file = os.path.join(nav_pkg, 'param', 'nav2_params_sim.yaml')
    rviz_config_dir = os.path.join(nav_pkg, 'rviz', 'navigation.rviz')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('boris_description'), 'urdf', 'boris_description.xacro']
            ),
            ' ',
            'use_sim:=true arm_z_position:=0.23 robot_name:=wx200 use_world_frame:=false hardware_type:=gz_classic use_gripper:=true show_gripper_fingers:=true',
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('shark_description'), 'config', 'shark_controllers_sim.yaml']
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'boris',
            '-x', '-0.2',
            '-y', '1.2',
            '-z', '0.25',
            '-Y', '-1.57',
            '-allow_renaming', 'true',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--switch-timeout', '30.0'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hoverboard_base_controller',
            '--param-file', robot_controllers,
            '--switch-timeout', '30.0',
        ],
        remappings=[
            ('/hoverboard_base_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/hoverboard_base_controller/odom', '/odom'),
        ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'interbotix_arm_controller',
            'interbotix_gripper_controller',
            '--param-file', robot_controllers,
            '--switch-timeout', '30.0',
        ],
    )
    
    # [ADICIONADO] Ponte da odometria para o Nav2 conseguir ler a velocidade do robô e frear
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/hoverboard_base_controller/odom', '/odom'],
        parameters=[{'use_sim_time': True}],
    )

    # [ADICIONADO] Ponte de /cmd_vel (padrao do robo real, Nav2 e tasks) para o controlador da simulacao
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/hoverboard_base_controller/cmd_vel_unstamped'],
        parameters=[{'use_sim_time': True}],
    )

    # [ADICIONADO] Abre a interface grafica rqt para manipular o braco e a garra
    rqt_arm_node = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        condition=IfCondition(use_rqt),
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['-0.2', '1.2', '0.0', '-1.57', '0.0', '0.0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_navigation'), 'launch', 'navigation_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'map': map_file,
            'params_file': param_file,
        }.items(),
        condition=IfCondition(use_nav),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_nav),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments=[('gz_args', f' -r -v 1 {world_file}')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_nav', default_value='true'),
        DeclareLaunchArgument('use_rqt', default_value='true'),
        ign_resource_env,
        gz_sim,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        cmd_vel_relay,
        odom_relay,
        # static_map_to_odom,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner, arm_controller_spawner, rqt_arm_node],
            )
        ),
        nav2_bringup_launch,
        rviz_node,
    ])
