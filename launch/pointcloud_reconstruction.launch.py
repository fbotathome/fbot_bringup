"""
Reconstrói a nuvem de pontos colorida a partir das imagens comprimidas
publicadas por uma câmera em outra máquina.

Saídas (em /<camera>/reconstructed/):
    depth/image_raw   profundidade descomprimida
    color/image_raw   cor descomprimida (só com compressed_color:=true)
    points            nuvem XYZRGB

Exemplos:
    ros2 launch <pacote> pointcloud_reconstruction.launch.py camera_name:=femtobolt
    ros2 launch <pacote> pointcloud_reconstruction.launch.py camera_name:=realsense
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Tópico da profundidade ALINHADA à cor, relativo a /<camera>
DEFAULT_DEPTH_TOPICS = {
    'femtobolt': 'depth/image_raw',                    # com depth_registration:=true
    'realsense': 'aligned_depth_to_color/image_raw',   # com align_depth.enable:=true
}


def launch_setup(context):
    cfg = context.launch_configurations
    cam = cfg['camera_name'].strip('/')
    compressed_color = cfg['compressed_color'].lower() == 'true'
    depth_rel = cfg['depth_topic'].strip('/') or DEFAULT_DEPTH_TOPICS.get(cam)

    if not depth_rel:
        raise RuntimeError(
            f"Não sei o tópico de profundidade alinhada da câmera '{cam}'. "
            "Passe depth_topic:=<tópico relativo a /<camera>>."
        )

    base = f'/{cam}'
    out = f'{base}/reconstructed'

    actions = [
        Node(
            package='image_transport', executable='republish',
            name=f'{cam}_depth_decompress',
            arguments=['compressedDepth', 'raw'],
            remappings=[
                ('in/compressedDepth', f'{base}/{depth_rel}/compressedDepth'),
                ('out', f'{out}/depth/image_raw'),
            ],
        ),
    ]

    if compressed_color:
        actions.append(
            Node(
                package='image_transport', executable='republish',
                name=f'{cam}_color_decompress',
                arguments=['compressed', 'raw'],
                remappings=[
                    ('in/compressed', f'{base}/color/image_raw/compressed'),
                    ('out', f'{out}/color/image_raw'),
                ],
            )
        )
        color_topic = f'{out}/color/image_raw'
    else:
        color_topic = f'{base}/color/image_raw'

    actions.append(
        ComposableNodeContainer(
            name=f'{cam}_reconstruction_container', namespace='',
            package='rclcpp_components', executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name=f'{cam}_point_cloud_xyzrgb',
                    remappings=[
                        ('rgb/camera_info', f'{base}/color/camera_info'),
                        ('rgb/image_rect_color', color_topic),
                        ('depth_registered/image_rect', f'{out}/depth/image_raw'),
                        ('points', f'{out}/points'),
                    ],
                ),
            ],
            output='screen',
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='femtobolt',
            description='Nome da câmera (femtobolt ou realsense)'),
        DeclareLaunchArgument(
            'compressed_color', default_value='true',
            description='Usa a cor comprimida (true) ou a crua (false)'),
        DeclareLaunchArgument(
            'depth_topic', default_value='',
            description='Tópico da profundidade alinhada, relativo à câmera (vazio = padrão da câmera)'),
        OpaqueFunction(function=launch_setup),
    ])