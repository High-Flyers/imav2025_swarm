import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
import numpy as np

from imav2025_swarm.waypoint_tracker import WaypointTracker
from flight_control.offboard_control import OffboardControl

LEADER_ID = 1
VELOCITY_LIMIT = 3.0

class SwarmControlNode(Node):
    STATES = ["INIT", "TAKING_OFF", "IN_AIR", "SWARMING"]

    def __init__(self):
        super().__init__("swarm_control")

        self.declare_parameter("ns_prefix", value="uav")
        self.ns_prefix = (
            self.get_parameter("ns_prefix").get_parameter_value().string_value
        )

        self.positions = {}  # drone_id: np.array([x, y, z])
        self.states = {}  # drone_id: state string
        self.local_id = self.get_namespace().strip("/")
        self.id = int(self.local_id.replace(f"{self.ns_prefix}_", ""))
        self.get_logger().info(f"Drone ID: {self.id}")
        self.state = "IDLE"
        self.last_state = None
        self.target_takeoff_height = 0.5
        self.position_sub = self.create_subscription(
            PointStamped, "/imav/swarm_positions", self.position_callback, 10
        )
        self.state_pub = self.create_publisher(String, "/imav/swarm_states", 10)
        self.state_sub = self.create_subscription(
            String, "/imav/swarm_states", self.state_callback, 10
        )
        self.offboard = OffboardControl(self, self.id)
        if self.id == 1:
            self.waypoint_tracker = WaypointTracker(self, self.offboard)
            self.swarm_start_srv = self.create_service(
                Trigger, "swarm_start", self.swarm_start_callback
            )

        self.mission_start_srv = self.create_service(
            Trigger, "mission_start", self.mission_start_callback
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.spring_constant = 1.0
        self.graph = None

    def mission_start_callback(self, _, response):
        self.state = "INIT"
        response.success = True
        response.message = f"Mission started on drone: {self.local_id}"
        return response

    def swarm_start_callback(self, _, response):
        for drone_id in self.states.keys():
            mission_start_client = self.create_client(
                Trigger, f"/{drone_id}/mission_start"
            )
            mission_start_client.call_async(Trigger.Request())

        response.success = True
        response.message = "Swarm started"
        return response

    def position_callback(self, msg: PointStamped):
        drone_id = msg.header.frame_id
        self.positions[drone_id] = np.array([msg.point.x, msg.point.y, msg.point.z])

    def state_callback(self, msg: String):
        # msg.data format: "drone_id:STATE"
        try:
            drone_id, state = msg.data.split(":")
            self.states[drone_id] = state
        except Exception:
            pass

    def compute_graph(self):
        ids = sorted(self.positions.keys())
        n = len(ids)
        graph = np.zeros((n, n))
        for i, id_i in enumerate(ids):
            for j, id_j in enumerate(ids):
                if i != j:
                    graph[i, j] = np.linalg.norm(self.positions[id_i][:2] - self.positions[id_j][:2])
        return graph, ids

    def control_loop(self):
        # Publish our state
        self.state_pub.publish(String(data=f"{self.local_id}:{self.state}"))

        if self.last_state != self.state:
            self.get_logger().info(f"State changed: {self.last_state} -> {self.state}")
            self.last_state = self.state

        # Wait for offboard ready and our position
        if not self.offboard.is_ready or self.local_id not in self.positions:
            return

        # INIT -> TAKING_OFF
        if self.state == "INIT":
            self.offboard.set_offboard_mode()
            self.offboard.arm()
            self.state = "TAKING_OFF"
            return

        # TAKING_OFF: climb to target height
        if self.state == "TAKING_OFF":
            my_pos = self.positions[self.local_id]
            self.offboard.fly_point(my_pos[0], my_pos[1], self.target_takeoff_height)
            if abs(my_pos[2] - self.target_takeoff_height) < 0.2:
                self.state = "IN_AIR"
            return

        # Wait for all drones to be IN_AIR
        if self.state == "IN_AIR":
            if all(
                state == "IN_AIR" for state in self.states.values() if state is not None
            ) and len(self.states) == len(self.positions):
                self.state = "SWARMING"
            return

        # SWARMING: run spring control
        if self.state == "SWARMING":
            if self.id == LEADER_ID:
                return  # Leader hovers in place

            if self.graph is None or len(self.positions) != self.graph.shape[0]:
                self.graph, self.id_list = self.compute_graph()

            idx = self.id_list.index(self.local_id)
            my_pos = self.positions[self.local_id]
            velocity = np.zeros(3)

            for j, other_id in enumerate(self.id_list):
                if other_id == self.local_id:
                    continue
                displacement = self.positions[other_id][:2] - my_pos[:2]
                distance = np.linalg.norm(displacement)
                direction = displacement / distance if distance != 0 else np.zeros(2)
                spring_length = self.graph[idx, j]
                velocity_magnitude = (
                    (distance - spring_length) / self.spring_constant
                ) ** 2
                if distance < spring_length:
                    velocity_magnitude = -velocity_magnitude
                velocity_2d = velocity_magnitude * direction
                velocity[0] += velocity_2d[0]
                velocity[1] += velocity_2d[1]

            if len(self.id_list) > 1:
                velocity /= (len(self.id_list) - 1)

            leader_pos = self.positions["px4_" + str(LEADER_ID)]
            altitude_diff = leader_pos[2] - my_pos[2]
            velocity[2] = (altitude_diff / self.spring_constant) ** 2

            if altitude_diff < 0:
                velocity[2] = -velocity[2]

            self.get_logger().info(f"[{self.id}] Velocity command: {velocity}, Position: {my_pos}, Leader: {leader_pos}", throttle_duration_sec=1)
            self.velocity = np.clip(velocity, -VELOCITY_LIMIT, VELOCITY_LIMIT)
            self.offboard.fly_vel(velocity[0], velocity[1], velocity[2])


def main(args=None):
    rclpy.init(args=args)
    node = SwarmControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
