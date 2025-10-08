from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    imav_package_path = get_package_share_path("imav2025_swarm")

    drone_id = LaunchConfiguration("drone_id")
    params_file = LaunchConfiguration("params_file")

    drone_id_arg = DeclareLaunchArgument("drone_id")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=(
            imav_package_path / "params" / "imav_sim_params.yaml"
        ).as_posix(),
    )

    def actions_with_context(context):
        namespace = f"/px4_{drone_id.perform(context)}"

        position_publisher = Node(
            package="imav2025_swarm",
            executable="position_publisher",
            namespace=namespace,
            output="screen",
        )

        swarm_control = Node(
            package="imav2025_swarm",
            executable="swarm_control",
            namespace=namespace,
            output="screen",
            parameters=[params_file]
        )

        return [position_publisher, swarm_control]

    ld = LaunchDescription()

    ld.add_action(drone_id_arg)
    ld.add_action(params_file_arg)
    ld.add_action(OpaqueFunction(function=actions_with_context))

    return ld
