import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from flight_control.utils.frame_transforms import ned_to_enu

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.drone_id = self.get_namespace().strip('/')
        self.publisher_ = self.create_publisher(PointStamped, '/imav/swarm_positions', 0)

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )
        
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile_sub
        )

    def local_position_callback(self, msg: VehicleLocalPosition):
        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = f"{self.drone_id}"
        enu = ned_to_enu(msg.x, msg.y, msg.z)
        pos_msg.point.x = enu[0]
        pos_msg.point.y = enu[1]
        pos_msg.point.z = enu[2]
        self.publisher_.publish(pos_msg)
        self.get_logger().debug(f'Published position for {pos_msg.header.frame_id}')

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

if __name__ == '__main__':
    main()