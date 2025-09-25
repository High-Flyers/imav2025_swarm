import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []
    for drone_id in range(1, 4):
        ns = f'/px4_{drone_id}'
        nodes.append(
            Node(
                package='imav2025_swarm',
                executable='position_publisher',
                namespace=ns,
                name='position_publisher',
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='imav2025_swarm',
                executable='swarm_control',
                namespace=ns,
                name='swarm_control',
                output='screen',
            )
        )
    return LaunchDescription(nodes)
