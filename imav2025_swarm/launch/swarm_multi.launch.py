from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    imav_package_path = get_package_share_path("imav2025_swarm")

    swarm_count = LaunchConfiguration("swarm_count")
    swarm_count_arg = DeclareLaunchArgument("swarm_count")

    def actions_with_context(context):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    (imav_package_path / "launch" / "swarm_member.launch.py").as_posix()
                ),
                launch_arguments={"drone_id": str(drone_id + 1)}.items(),
            )
            for drone_id in range(int(swarm_count.perform(context)))
        ]

    ld = LaunchDescription()

    ld.add_action(swarm_count_arg)
    ld.add_action(OpaqueFunction(function=actions_with_context))

    return ld
