"""Bring up the BORIS manipulator: arm driver + MoveIt + fbot_manipulator interface.

Task launches that manipulate include this next to boris.launch.py:

    ros2 launch fbot_bringup manipulator.launch.py                         # xArm6, real arm
    ros2 launch fbot_bringup manipulator.launch.py robot_ip:=192.168.1.185
    ros2 launch fbot_bringup manipulator.launch.py xarm_fake:=true          # no arm needed (fake controllers)
    ros2 launch fbot_bringup manipulator.launch.py arm_type:=wx200          # Interbotix WidowX 200 (option)

Starts, for the chosen arm_type:
  1. the arm's MoveIt bring-up (move_group + hardware driver),
  2. the matching fbot_manipulator interface (motion primitives + MTC task server),
  3. one static transform attaching the arm to BORIS: arm_mount_link -> <arm root>
     (xarm6: world, wx200: wx200/base_link). Set the pose with mount_xyz / mount_rpy.
  4. xarm6 + use_wrist_camera: the UFACTORY RealSense stand on the wrist (link_eef mesh, also
     a MoveIt collision body) and one static transform link_eef -> realsense_link.
     realsense_link is the root of the RealSense driver tree (camera.launch.py,
     camera_name:=realsense, publish_tf on), so its optical frames follow the arm. The xArm
     copies of those frames stay off (add_d435i_links:=false): one publisher per frame, with
     the camera's factory calibration. The Femto Bolt (main camera) is on the neck in the
     BORIS URDF (fbot_description urdf/v2/neck.xacro). v1 has realsense_link on the neck:
     use use_wrist_camera:=false there.

Coexistence with the BORIS base (boris.launch.py): the xArm stack runs in the root
namespace with its own /controller_manager and joint_state_publisher; BORIS uses
/base/controller_manager and boris_joint_state_publisher. The xArm robot_description
topic is remapped to /xarm/robot_description so BORIS keeps /robot_description.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

# Arm root frame and its default pose on arm_mount_link. v2: arm_mount_link is the bottom
# centre of the xArm base from the Onshape export (fbot_description config/robot/v2.yaml),
# which is also the xArm's own base origin, so the offset is zero. The CAD tool does not
# extract the arm's yaw (assumed forward); correct it with mount_rpy if needed.
ARMS = {
    'xarm6': {'root': 'world', 'xyz': '0 0 0', 'rpy': '0 0 0'},
    'wx200': {'root': 'wx200/base_link', 'xyz': '0.1 0 0.0235', 'rpy': '0 0 0'},
}

# link_eef -> realsense_link on the UFACTORY D435i stand
# (xarm_description/urdf/camera/realsense_d435i.urdf.xacro, realsense_link_joint)
WRIST_CAMERA_TF = {'xyz': '0.06746 -0.0175 0.0237', 'rpy': '3.141592653589793 -1.5707963267948966 0'}


def _include(package, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=args.items(),
    )


def _mount_tf(context, arm_type):
    arm = ARMS[arm_type]
    x, y, z = (LaunchConfiguration('mount_xyz').perform(context) or arm['xyz']).split()
    roll, pitch, yaw = (LaunchConfiguration('mount_rpy').perform(context) or arm['rpy']).split()
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arm_mount_tf',
        arguments=['--x', x, '--y', y, '--z', z, '--roll', roll, '--pitch', pitch, '--yaw', yaw,
                   '--frame-id', 'arm_mount_link', '--child-frame-id', arm['root']],
        output='screen',
    )


def _wrist_camera_tf():
    x, y, z = WRIST_CAMERA_TF['xyz'].split()
    roll, pitch, yaw = WRIST_CAMERA_TF['rpy'].split()
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wrist_camera_tf',
        arguments=['--x', x, '--y', y, '--z', z, '--roll', roll, '--pitch', pitch, '--yaw', yaw,
                   '--frame-id', 'link_eef', '--child-frame-id', 'realsense_link'],
        output='screen',
    )


def _launch_setup(context, *args, **kwargs):
    arm_type = LaunchConfiguration('arm_type').perform(context)

    if arm_type == 'xarm6':
        fake = LaunchConfiguration('xarm_fake').perform(context).lower() == 'true'
        moveit_args = {'add_mtc': 'true', 'add_gripper': LaunchConfiguration('add_gripper')}
        wrist_camera = LaunchConfiguration('use_wrist_camera').perform(context).lower() == 'true'
        if wrist_camera:
            moveit_args.update({'add_realsense_d435i': 'true', 'add_d435i_links': 'false'})
        if not fake:
            moveit_args['robot_ip'] = LaunchConfiguration('robot_ip')
        arm = GroupAction(scoped=True, actions=[
            SetRemap(src='/robot_description', dst='/xarm/robot_description'),
            _include('xarm_moveit_config',
                     'xarm6_moveit_fake.launch.py' if fake else 'xarm6_moveit_realmove.launch.py',
                     moveit_args),
            _include('fbot_manipulator', 'manipulator_interface.launch.py',
                     {'arm_type': 'xarm6', 'enable_surfaces': LaunchConfiguration('enable_surfaces')}),
        ])
        return [arm, _mount_tf(context, arm_type)] + ([_wrist_camera_tf()] if wrist_camera else [])
    else:  # wx200: the Interbotix stack is namespaced (/wx200), no remap needed
        arm = GroupAction(scoped=True, actions=[
            _include('interbotix_xsarm_moveit', 'xsarm_moveit.launch.py', {
                'robot_model': 'wx200',
                'hardware_type': LaunchConfiguration('hardware_type'),
                'use_moveit_rviz': 'false',
                'use_world_frame': 'false',
            }),
            # requires fbot_manipulator with the wx200 interface (branch feat/add_wx200)
            _include('fbot_manipulator', 'manipulator_interface_wx200.launch.py', {}),
        ])

    return [arm, _mount_tf(context, arm_type)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_type', default_value='xarm6', choices=list(ARMS),
                              description="'xarm6' (UFACTORY, current arm) or 'wx200' (Interbotix, option)"),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.208',
                              description='xarm6: IP of the xArm control box'),
        DeclareLaunchArgument('xarm_fake', default_value='false',
                              description='xarm6: MoveIt with fake controllers (no arm connected)'),
        DeclareLaunchArgument('add_gripper', default_value='true',
                              description='xarm6: include the xArm gripper (fbot_manipulator MTC uses it)'),
        DeclareLaunchArgument('use_wrist_camera', default_value='true',
                              description='xarm6: RealSense stand on the wrist + link_eef -> realsense_link'),
        DeclareLaunchArgument('enable_surfaces', default_value='true',
                              description='xarm6: collision surfaces around objects in MTC tasks'),
        DeclareLaunchArgument('hardware_type', default_value='actual',
                              description='wx200: Interbotix hardware_type (actual / fake / gz_classic)'),
        DeclareLaunchArgument('mount_xyz', default_value='',
                              description="Arm pose on arm_mount_link 'x y z' [m] (empty: per-arm default)"),
        DeclareLaunchArgument('mount_rpy', default_value='',
                              description="Arm pose on arm_mount_link 'roll pitch yaw' [rad] (empty: per-arm default)"),
        OpaqueFunction(function=_launch_setup),
    ])
