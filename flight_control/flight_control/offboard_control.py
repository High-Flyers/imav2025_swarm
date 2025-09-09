from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode

from qos_profiles import PX4_QOS


class OffboardControl:
    HEARTBEAT_THRESHOLD = 10

    def __init__(self, node: Node):
        self._node = node

        self._offboard_control_mode_pub = self._node.create_publisher(
            OffboardControl, "fmu/in/offboard_control_mode", PX4_QOS
        )
        self._heartbeat = self._node.create_timer(0.1, self.__heartbeat_cb)

        self._heartbeat_counter = 0

    def __publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self._node.get_clock().now().nanoseconds / 1000)
        self._offboard_control_mode_pub.publish(msg)

    def __heartbeat_cb(self):
        self.__publish_offboard_control_heartbeat_signal()
        if self._heartbeat_counter < self.HEARTBEAT_THRESHOLD:
            self._heartbeat_counter += 1

