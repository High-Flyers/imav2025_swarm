from rclpy.node import Node
from std_msgs.msg import String

from flight_control.offboard_control import OffboardControl
from flight_control.utils.frame_transforms import lla_to_enu


class WaypointTracker:
    def __init__(self, node: Node, offboard_control: OffboardControl) -> None:
        self._node = node
        self._offboard = offboard_control
        self._target_position = None
        self._states = {}

        self._node.declare_parameter("latitude", value=0.0)
        self._node.declare_parameter("longitude", value=0.0)

        self._swarm_state_sub = self._node.create_subscription(
            String, "/imav/swarm_states", self.__state_callback, 10
        )

        self._control_timer = self._node.create_timer(0.1, self.__control_loop)

    @property
    def is_swarming(self):
        return all(state == "SWARMING" for state in self._states.values())

    def __state_callback(self, msg: String) -> None:
        # msg.data format: "drone_id:STATE"
        try:
            drone_id, state = msg.data.split(":")
            self._states[drone_id] = state
        except Exception:
            pass

    def __control_loop(self):
        if self._target_position is None and self._offboard.enu is not None:
            self.__initialize_target_position()

        if not self.is_swarming or self._target_position is None:
            return

        if self._offboard.is_point_reached(
            self._target_position[0], self._target_position[1], 5.0
        ):
            self._offboard.land()

        self._offboard.fly_point(
            self._target_position[0], self._target_position[1], 5.0
        )

    def __initialize_target_position(self):
        latitude = (
            self._node.get_parameter("latitude").get_parameter_value().double_value
        )
        longitude = (
            self._node.get_parameter("longitude").get_parameter_value().double_value
        )
        enu = self._offboard.enu
        self._target_position = lla_to_enu(
            latitude, longitude, enu.ref_alt, enu.ref_lat, enu.ref_lon, enu.ref_alt
        )
