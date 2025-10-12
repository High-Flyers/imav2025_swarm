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

    ns_prefix = LaunchConfiguration("ns_prefix")
    swarm_count = LaunchConfiguration("swarm_count")
    params_file = LaunchConfiguration("params_file")

    ns_prefix_arg = DeclareLaunchArgument("ns_prefix", default_value="uav")
    swarm_count_arg = DeclareLaunchArgument("swarm_count", default_value="3")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=(
            imav_package_path / "params" / "imav_sim_params.yaml"
        ).as_posix(),
    )

    def actions_with_context(context):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    (imav_package_path / "launch" / "swarm_member.launch.py").as_posix()
                ),
                launch_arguments={
                    "ns_prefix": ns_prefix,
                    "drone_id": str(drone_id + 1),
                    "params_file": params_file,
                }.items(),
            )
            for drone_id in range(int(swarm_count.perform(context)))
        ]

    ld = LaunchDescription()

    ld.add_action(ns_prefix_arg)
    ld.add_action(swarm_count_arg)
    ld.add_action(params_file_arg)
    ld.add_action(OpaqueFunction(function=actions_with_context))

    return ld
