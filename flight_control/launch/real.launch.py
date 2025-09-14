from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    package_path = get_package_share_path("flight_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_file_path=(
                        package_path / "launch" / "px4_comm.launch.py"
                    ).as_posix()
                )
            ),
            Node(
                package="flight_control",
                executable="basic_mission",
                namespace=namespace,
            ),
        ]
    )
