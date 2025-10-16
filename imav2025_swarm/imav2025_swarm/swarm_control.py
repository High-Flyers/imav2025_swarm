import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
import numpy as np

from imav2025_swarm.waypoint_tracker import WaypointTracker
from flight_control.offboard_control import OffboardControl


class SwarmControlNode(Node):
    STATES = ["INIT", "TAKING_OFF", "IN_AIR", "SWARMING"]

    def __init__(self):
        super().__init__("swarm_control")
        self.positions = {}  # drone_id: np.array([x, y, z])
        self.states = {}  # drone_id: state string
        self.local_id = self.get_namespace().strip("/")
        self.id = int(self.local_id.replace("px4_", ""))
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
        self.spring_constant = 0.5
        self.graph = None

    def mission_start_callback(self, _, response):
        self.state = "INIT"
        response.success = True
        response.message = f"Mission started on drone: {self.local_id}"
        return response
    
    def swarm_start_callback(self, _, response):
        for drone_id in self.states.keys():
            mission_start_client = self.create_client(Trigger, f"/{drone_id}/mission_start")
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
                    graph[i, j] = np.linalg.norm(
                        self.positions[id_i] - self.positions[id_j]
                    )
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
            if self.id == 1:
                return  # Leader hovers in place
            if self.graph is None or len(self.positions) != self.graph.shape[0]:
                self.graph, self.id_list = self.compute_graph()
            idx = self.id_list.index(self.local_id)
            my_pos = self.positions[self.local_id]
            velocity = np.zeros(3)
            for j, other_id in enumerate(self.id_list):
                if other_id == self.local_id:
                    continue
                displacement = self.positions[other_id] - my_pos
                distance = np.linalg.norm(displacement)
                direction = displacement / distance if distance != 0 else np.zeros(3)
                spring_length = self.graph[idx, j]
                velocity_magnitude = (
                    (distance - spring_length) / self.spring_constant
                ) ** 2
                if distance < spring_length:
                    velocity_magnitude = -velocity_magnitude
                velocity += velocity_magnitude * direction
            if len(self.id_list) > 1:
                velocity /= len(self.id_list) - 1
            target = my_pos + velocity * 0.1
            self.offboard.fly_point(target[0], target[1], target[2])


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
