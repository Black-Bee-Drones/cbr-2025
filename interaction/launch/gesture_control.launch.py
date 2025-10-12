from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Lança o nó integrado de reconhecimento e controle de gestos do pacote interaction.
    """
    return LaunchDescription([
        Node(
            package='interaction',
            executable='gesture_recognizer',
            name='gesture_recognizer',
            output='screen'
        ),
    ])
