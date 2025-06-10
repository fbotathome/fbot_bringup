import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

#THIS LAUNCHFILE IS FOR TESTING PURPOSES ONLY, IT SHOULD NOT BE USED FOR NOW.

def generate_launch_description():
    hotword_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_hri_bringup"), 'hotword_detector.launch.py')))

    return LaunchDescription([
        hotword_detector,
    ])