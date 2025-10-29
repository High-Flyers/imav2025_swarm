import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from flight_control.utils.frame_transforms import ned_to_enu, lla_to_enu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


class PositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")
        self.drone_id = self.get_namespace().strip("/")
        self.id = self.drone_id[-1]
        self.ns_prefix = self.get_namespace()[:-2]
        self.publisher_ = self.create_publisher(Odometry, '/imav/swarm_positions', 0)

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
        self.should_sent_reference_transform = self.id != "1"

        self.reference_frame_static_broadcaster = StaticTransformBroadcaster(self)
        self.current_position_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def local_position_cb(self, msg: VehicleLocalPosition):
        if self.reference_local_position is None:
            return

        if self.should_sent_reference_transform:
            self.make_reference_transform(msg)
            self.should_sent_reference_transform = False
            return
        else:
            self.make_current_transform(msg)

        try:
            trans = self.tf_buffer.lookup_transform(
                f"{self.ns_prefix.strip('/')}_1_ref",
                f"{self.drone_id}",
                rclpy.time.Time()
            )
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = f"{self.drone_id}"
            odom_msg.pose.pose.position.x = trans.transform.translation.x
            odom_msg.pose.pose.position.y = trans.transform.translation.y
            odom_msg.pose.pose.position.z = trans.transform.translation.z

            vel_enu = ned_to_enu(msg.vx, msg.vy, msg.vz)
            odom_msg.twist.twist.linear.x = vel_enu[0]
            odom_msg.twist.twist.linear.y = vel_enu[1]
            odom_msg.twist.twist.linear.z = vel_enu[2]
            self.publisher_.publish(odom_msg)
            self.get_logger().debug(f'Published odometry from tf lookup for {odom_msg.header.frame_id}')
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

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
