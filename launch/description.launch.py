import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_z_position',
            default_value='0.23',
            description='Z position of the arm in meters'
        )
    )

    arm_z_position = LaunchConfiguration('arm_z_position')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("boris_description"), 'launch', 'boris_description.launch.py')
        ),
        launch_arguments={
            'arm_z_position': arm_z_position
        }.items()
    )

    return LaunchDescription([
        *declared_arguments,
        robot_launch,
    ])