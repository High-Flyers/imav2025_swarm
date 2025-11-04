import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from flight_control.utils.frame_transforms import ned_to_enu, lla_to_enu
from nav_msgs.msg import Odometry


class PositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")
        self.drone_id = self.get_namespace().strip("/")
        self.id = self.drone_id[-1]
        self.ns_prefix = self.get_namespace()[:-2]
        self.publisher_ = self.create_publisher(Odometry, '/imav/swarm_positions', 0)
        self.init_enu = None

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

    def local_position_cb(self, msg: VehicleLocalPosition):
        if self.reference_local_position is None:
            return

        if self.should_sent_reference_transform:
            self.init_enu = self.make_reference_transform(msg)
            self.should_sent_reference_transform = False
            return
        else:
            enu = self.make_current_transform(msg)

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = f"{self.drone_id}"
            odom_msg.pose.pose.position.x = enu[0]
            odom_msg.pose.pose.position.y = enu[1]
            odom_msg.pose.pose.position.z = enu[2]

            vel_enu = ned_to_enu(msg.vx, msg.vy, msg.vz)
            odom_msg.twist.twist.linear.x = vel_enu[0]
            odom_msg.twist.twist.linear.y = vel_enu[1]
            odom_msg.twist.twist.linear.z = vel_enu[2]
            self.publisher_.publish(odom_msg)
            self.get_logger().debug(f'Published odometry from tf lookup for {odom_msg.header.frame_id}')

    def reference_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.reference_local_position = msg
        self.local_position_sub = None

    def make_reference_transform(self, local_position: VehicleLocalPosition) -> tuple[float, float, float]:
        if self.reference_local_position is None:
            return

        enu = lla_to_enu(
            local_position.ref_lat,
            local_position.ref_lon,
            local_position.ref_alt,
            self.reference_local_position.ref_lat,
            self.reference_local_position.ref_lon,
            self.reference_local_position.ref_alt,
        )

        return enu

    def make_current_transform(self, local_position: VehicleLocalPosition) -> tuple[float, float, float]:
        enu = ned_to_enu(local_position.x, local_position.y, local_position.z)

        if self.should_sent_reference_transform:
            enu[0] += self.init_enu[0]
            enu[1] += self.init_enu[1]
            enu[2] += self.init_enu[2]

        return enu

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
