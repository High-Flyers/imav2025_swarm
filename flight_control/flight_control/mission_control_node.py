import rclpy
from rclpy.node import Node

from flight_control.offboard_control import OffboardControl


class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control_node")
        self._offboard_control = OffboardControl(self)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)


def main():
    rclpy.init()
    node = MissionControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
