import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_keepout_arg = DeclareLaunchArgument(
            'use_keepout_zones',
            default_value='false',
            description='Parameter to include or not keepout zones on the navigation'
        )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'navigation.launch.py')

        ),
        launch_arguments={
            'use_description': 'false',
            'use_rviz': 'false'
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_keepout_zones')),

    )

    navigation_keepout = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'navigation_keepout.launch.py')
        ),
        launch_arguments={
            'use_description': 'false',
            'use_rviz': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_keepout_zones')),
    )

    return LaunchDescription([
        use_keepout_arg,
        navigation,
        navigation_keepout
    ])