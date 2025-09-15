from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(name='uxrce_dds', cmd=['MicroXRCEAgent', 'udp4', '-p', '8888']),
    ])
