from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drone_id = LaunchConfiguration("drone_id")
    drone_id_arg = DeclareLaunchArgument("drone_id")

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
        )

        return [position_publisher, swarm_control]

    ld = LaunchDescription()

    ld.add_action(drone_id_arg)
    ld.add_action(OpaqueFunction(function=actions_with_context))

    return ld
