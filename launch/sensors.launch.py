"""Navigation sensors: two Hokuyo lasers + BNO055 IMU (+ optional Sick LMS).

  ros2 launch fbot_bringup sensors.launch.py
  ros2 launch fbot_bringup sensors.launch.py imu_port:=/dev/sensors/imu use_imu:=false

Topics: /scan2 (ground Hokuyo), /scan3 (back Hokuyo), /bno055/imu.
/scan is only published by the Sick LMS (use_sick:=true); AMCL and slam_toolbox
read /scan. Parameters live in fbot_description/config/sensors/.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensors_cfg = os.path.join(get_package_share_directory('fbot_description'), 'config', 'sensors')

    use_lasers = LaunchConfiguration('use_lasers')
    use_imu = LaunchConfiguration('use_imu')
    use_sick = LaunchConfiguration('use_sick')

    urg_ground = Node(
        package='urg_node', executable='urg_node_driver', name='urg_node_ground', output='screen',
        parameters=[os.path.join(sensors_cfg, 'urg_ground.yaml')],
        remappings=[('scan', 'scan2')],
        condition=IfCondition(use_lasers),
    )

    urg_back = Node(
        package='urg_node', executable='urg_node_driver', name='urg_node_back', output='screen',
        parameters=[os.path.join(sensors_cfg, 'urg_back.yaml')],
        remappings=[('scan', 'scan3')],
        condition=IfCondition(use_lasers),
    )

    imu = Node(
        package='bno055', executable='bno055', name='bno055',
        parameters=[os.path.join(sensors_cfg, 'bno055.yaml'), {'uart_port': LaunchConfiguration('imu_port')}],
        condition=IfCondition(use_imu),
    )

    sick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_bringup'), 'launch', 'sick.launch.py')
        ),
        condition=IfCondition(use_sick),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lasers', default_value='true', description='Start the two Hokuyo lasers'),
        DeclareLaunchArgument('use_imu', default_value='true', description='Start the BNO055 IMU'),
        DeclareLaunchArgument('use_sick', default_value='false', description='Start the Sick LMS (publishes /scan)'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB2',
                              description='IMU serial port (prefer a udev symlink)'),
        urg_ground,
        urg_back,
        imu,
        sick,
    ])
