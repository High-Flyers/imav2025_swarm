from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace"
        ),
        Node(
            package="flight_control",
            executable="mission_control",
            namespace=namespace
        )
    ])