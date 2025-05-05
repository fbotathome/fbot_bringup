from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution, LocalSubstitution
from launch.substitution import Substitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from launch import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
import os


def nome_arquivo (texto):
    return texto


def dict2list(dictio):
  
    lista = []
    for key, value in dictio.items():
        if isinstance(value, dict):
            for sub_key, sub_value in value.items():
               if isinstance(sub_value, dict):
                    for sub_sub_key, sub_sub_value in sub_value.items():
                        lista.append([f"{key}.{sub_key}.{sub_sub_key}", sub_sub_value])
               else:
                lista.append([f"{key}.{sub_key}", sub_value])
        else:
            lista.append([key, value])
    return lista


def launch_setup(context: LaunchContext):
    teste = LaunchConfiguration('teste').perform(context) # Here you'll get the runtime config value
    print(f'LaunchConfiguration: {teste}')
    node1 = Node(package = 'fbot_bringup',
                 executable = 'test',
                 parameters =[{'teste': nome_arquivo(teste)}],)

    return [node1]


def generate_launch_description():
    #Definir node_name vindo do behavior]


    opfunc = OpaqueFunction(function = launch_setup)
    
    return LaunchDescription([
        opfunc
        ])