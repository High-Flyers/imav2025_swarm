import rclpy
from rclpy.node import Node


class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control_node")

    def run(self):
        pass


def main():
    try:
        rclpy.init()
        node = MissionControlNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
