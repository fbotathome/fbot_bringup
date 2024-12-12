from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
import yaml
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

"""
@file full_bringup.launch.py
@brief Launch file for bringing up multiple ROS nodes based on a YAML configuration file.
This launch file reads a YAML configuration file and launches multiple ROS nodes as specified in the file.
Each node's launch configuration is defined in the YAML file, including the package name, executable, and parameters.
Functions:
    yaml_to_dict(config_file_path)
        Reads a YAML file and converts it to a dictionary.
    launchFactory(launchConfiguration)
        Creates an IncludeLaunchDescription action based on the provided launch configuration dictionary.
    generateMultipleLaunchesInstances(context, config_file_path)
        Generates a list of launch descriptions for nodes that are enabled in the YAML configuration file.
    generate_launch_description()
        Generates the main launch description, including the declaration of launch arguments and the opaque function to generate multiple launch instances.
@param config_file_path: Path to the YAML configuration file.
@type config_file_path: str
@return LaunchDescription
"""

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