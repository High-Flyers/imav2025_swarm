import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
import numpy as np

from imav2025_swarm.waypoint_tracker import WaypointTracker
from flight_control.offboard_control import OffboardControl

LEADER_ID = 1
VELOCITY_LIMIT = 2.0
TAKEOFF_HEIGHT = 1.5
LANDING_HEIGHT = 1.5
MIN_VELOCITY = 0.1
SPRING_CONSTANT = 0.5


class SwarmControlNode(Node):
    STATES = ["INIT", "ARMING", "TAKING_OFF", "IN_AIR", "SWARMING", "HOVER", "LANDING"]

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
        self.leader_drone_id = f"{self.ns_prefix}_{LEADER_ID}"
        self.is_leader = self.id == LEADER_ID
        self.get_logger().info(f"Drone ID: {self.id}")
        self.state = "IDLE"
        self.last_state = None
        self.position_sub = self.create_subscription(
            Odometry, "/imav/swarm_positions", self.position_callback, 10
        )
        self.state_pub = self.create_publisher(String, "/imav/swarm_states", 10)
        self.state_sub = self.create_subscription(
            String, "/imav/swarm_states", self.state_callback, 10
        )
        self.offboard = OffboardControl(self, self.id)
        if self.is_leader:
            self.waypoint_tracker = WaypointTracker(self, self.offboard)
            self.swarm_start_srv = self.create_service(
                Trigger, "swarm_start", self.swarm_start_callback
            )

        self.mission_start_srv = self.create_service(
            Trigger, "mission_start", self.mission_start_callback
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.graph = None
        self.leader_horizontal_velocities = None

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

    def position_callback(self, msg: Odometry):
        drone_id = msg.header.frame_id
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        self.positions[drone_id] = np.array([position.x, position.y, position.z])

        if drone_id == self.leader_drone_id:
            vx = velocity.x if abs(velocity.x) > MIN_VELOCITY else 0.0
            vy = velocity.y if abs(velocity.y) > MIN_VELOCITY else 0.0
            self.leader_horizontal_velocities = np.array([vx, vy])

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
                        self.positions[id_i][:2] - self.positions[id_j][:2]
                    )
        return graph, ids

    def control_loop(self):
        # Publish our state
        self.state_pub.publish(String(data=f"{self.local_id}:{self.state}"))

        if self.last_state != self.state:
            self.get_logger().info(f"State changed: {self.last_state} -> {self.state}")
            self.last_state = self.state

        # Wait for offboard ready and our position
        if (
            not self.offboard.is_ready
            or self.local_id not in self.positions
            or (self.is_leader and self.leader_horizontal_velocities is None)
        ):
            self.offboard.set_offboard_mode()
            return

        # INIT -> TAKING_OFF
        if self.state == "INIT":
            self.offboard.arm()
            if self.offboard.is_armed:
                self.state = "ARMING"
            return

        if self.state == "ARMING":
            if all(
                state == "ARMING" for state in self.states.values() if state is not None
            ):
                self.state = "TAKING_OFF"
            return

        # TAKING_OFF: climb to target height
        if self.state == "TAKING_OFF":
            my_pos = self.positions[self.local_id]
            self.offboard.fly_point(my_pos[0], my_pos[1], TAKEOFF_HEIGHT)
            if abs(my_pos[2] - TAKEOFF_HEIGHT) < 0.1:
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
            if self.is_leader:
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
                velocity_magnitude = ((distance - spring_length) / SPRING_CONSTANT) ** 2
                if distance < spring_length:
                    velocity_magnitude = -velocity_magnitude
                velocity_2d = velocity_magnitude * direction
                velocity[0] += velocity_2d[0]
                velocity[1] += velocity_2d[1]

            if len(self.id_list) > 1:
                velocity /= len(self.id_list) - 1

            leader_pos = self.positions[self.ns_prefix + "_" + str(LEADER_ID)]
            altitude_diff = leader_pos[2] - my_pos[2]
            velocity[2] = (altitude_diff / SPRING_CONSTANT) ** 2

            if altitude_diff < 0:
                velocity[2] = -velocity[2]

            horizontal_speed = np.linalg.norm(velocity[:2])
            if horizontal_speed == 0:
                velocity[:2] = np.zeros(2)
            else:
                velocity[:2] = (velocity[:2] / horizontal_speed) * min(
                    horizontal_speed, VELOCITY_LIMIT
                )
            velocity[:2] += self.leader_horizontal_velocities
            self.get_logger().info(
                f"[{self.id}] Velocity command: {velocity}, Position: {my_pos}, Leader: {leader_pos}, Velocity: {self.leader_horizontal_velocities}",
                throttle_duration_sec=1,
            )
            self.offboard.fly_vel(velocity[0], velocity[1], velocity[2])

            x, y, z = self.offboard.enu.position()
            if z <= LANDING_HEIGHT:
                self.offboard.fly_point(x, y, LANDING_HEIGHT)
                self.state = "HOVER"
                return

        # HOVER: wait for all drones to HOVER
        if self.state == "HOVER":
            if all(
                state == "HOVER" for state in self.states.values() if state is not None
            ) and len(self.states) == len(self.positions):
                self.state = "LANDING"
            return

        # LANDING: land
        if self.state == "LANDING":
            self.offboard.land()
            return


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
