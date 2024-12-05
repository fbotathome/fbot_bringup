from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
import yaml
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def yaml_to_dict(config_file_path):
    with open(config_file_path, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
    
def launchFactory(launchConfiguration: dict):
    return IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution(
            [FindPackageShare(launchConfiguration["package_name"]), launchConfiguration["executable"]]
        ),
        launch_arguments=[
            (key, str(value)) for key, value in launchConfiguration["parameters"].items()
        ],
    )
def generateMultipleLaunchesInstances(context: LaunchContext, config_file_path: LaunchConfiguration):

    # Resolve the path to the configuration file
    resolved_path = context.perform_substitution(config_file_path)
    launchesInformations = yaml_to_dict(resolved_path)  # Load YAML configuration

    launchList = []
    for _, values in launchesInformations.items():
        if values['enable'] == 'true':
            launchList.append(launchFactory(values))
    return launchList

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file_path",
        default_value="base_config.yaml",
        description="Which configuration file should be used",
    )

    # Generate the launch description
    return LaunchDescription(
        [
            config_file_arg,
            OpaqueFunction(function=generateMultipleLaunchesInstances, args=[LaunchConfiguration("config_file_path")]),
        ]
    )