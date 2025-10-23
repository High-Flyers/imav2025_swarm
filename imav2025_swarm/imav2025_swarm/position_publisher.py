import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PointStamped, TransformStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from flight_control.utils.frame_transforms import ned_to_enu, lla_to_enu


class PositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")
        self.drone_id = self.get_namespace().strip("/")
        self.id = self.drone_id[-1]
        self.ns_prefix = self.get_namespace()[:-2]
        self.publisher_ = self.create_publisher(
            PointStamped, "/imav/swarm_positions", 0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.local_position_cb,
            qos_profile_sub,
        )

        self.reference_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{self.ns_prefix}_1/fmu/out/vehicle_local_position",
            self.reference_position_cb,
            qos_profile_sub,
        )

        self.reference_local_position: VehicleLocalPosition = None
        self.reference_transform_sent = False

        self.reference_frame_static_broadcaster = StaticTransformBroadcaster(self)
        self.current_position_broadcaster = TransformBroadcaster(self)

    def local_position_cb(self, msg: VehicleLocalPosition):
        if self.reference_local_position is None:
            return

        if self.id != "1" and not self.reference_transform_sent:
            self.make_reference_transform(msg)
            self.reference_transform_sent = True
            return

        self.make_current_transform(msg)

        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = f"{self.drone_id}"
        enu = ned_to_enu(msg.x, msg.y, msg.z)
        pos_msg.point.x = enu[0]
        pos_msg.point.y = enu[1]
        pos_msg.point.z = enu[2]
        self.publisher_.publish(pos_msg)
        self.get_logger().debug(f"Published position for {pos_msg.header.frame_id}")

    def reference_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.reference_local_position = msg

    def make_reference_transform(self, local_position: VehicleLocalPosition) -> None:
        if self.reference_local_position is None:
            return

        ts = TransformStamped()

        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = f"{self.ns_prefix}_1_ref"
        ts.child_frame_id = f"{self.drone_id}_ref"

        enu = lla_to_enu(
            local_position.ref_lat,
            local_position.ref_lon,
            local_position.ref_alt,
            self.reference_local_position.ref_lat,
            self.reference_local_position.ref_lon,
            self.reference_local_position.ref_alt,
        )

        ts.transform.translation.x = enu[0]
        ts.transform.translation.y = enu[1]
        ts.transform.translation.z = enu[2]

        self.reference_frame_static_broadcaster.sendTransform(ts)

    def make_current_transform(self, local_position: VehicleLocalPosition) -> None:
        ts = TransformStamped()

        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = f"{self.drone_id}_ref"
        ts.child_frame_id = f"{self.drone_id}"

        enu = ned_to_enu(local_position.x, local_position.y, local_position.z)

        ts.transform.translation.x = enu[0]
        ts.transform.translation.y = enu[1]
        ts.transform.translation.z = enu[2]

        self.current_position_broadcaster.sendTransform(ts)


def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
