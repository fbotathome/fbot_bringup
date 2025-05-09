import os

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_object_recognition = DeclareLaunchArgument(
        'object_recognition',
        default_value='false',
        description="If should launch the camera node"
    )

    launch_tracker = DeclareLaunchArgument(
        'launch_tracker',
        default_value='false',
        description="If should launch the tracker node"
    )

    object_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_recognition"), 'launch', 'yolov8_object_recognition.launch.py')
        
        ),
        launch_arguments={
            'launch_realsense': 'false',
            'use_remote': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('object_recognition') )
    )

    yolo_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_recognition"), 'launch', 'yolo_tracker_recognition.launch.py')
        ),
        launch_arguments={
            'launch_realsense': 'false',
            'use_remote': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_tracker') )
    )

    realsense2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_realsense'))
    )

    return LaunchDescription([
        launch_object_recognition,
        launch_tracker,
        object_recognition,
        realsense2_node,
        yolo_tracker
    ])