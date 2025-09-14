from enum import Enum, auto

import rclpy
from rclpy.node import Node

from flight_control.offboard_control import OffboardControl


class State(Enum):
    IDLE = 1
    ARM = 2
    TAKEOFF = 3
    TRANSPORT = 4
    LAND = 5
    END = 6


class BasicMissionExampleNode(Node):
    TAKEOFF_HEIGHT = 3.0
    TRANSPORT_TARGET = (0.0, -10.0, 3.0)

    def __init__(self):
        super().__init__("mission_control_node")
        self._offboard_control = OffboardControl(self)
        self._state = State.IDLE
        self._vehicle_command_send = False

    def run(self):
        while rclpy.ok():
            if self._state == State.END:
                return
            state = self._state
            self.__step()
            if state != self._state:
                self.get_logger().info(f"STATE: {self._state}")
            rclpy.spin_once(self)

    def __step(self):
        match self._state:
            case State.IDLE:
                if (
                    self._offboard_control.is_in_offboard
                    and self._offboard_control.is_ready
                ):
                    self._state = State.ARM
                    return
                self._offboard_control.set_offboard_mode()
            case State.ARM:
                if self._offboard_control.is_armed:
                    self._state = State.TAKEOFF
                    return
                if not self._vehicle_command_send:
                    self._offboard_control.arm()
                    self._vehicle_command_send = True
            case State.TAKEOFF:
                position = self._offboard_control.enu.position()
                if self._offboard_control.is_point_reached(
                    position[0], position[1], self.TAKEOFF_HEIGHT
                ):
                    self._state = State.TRANSPORT
                    return
                self._offboard_control.fly_point(
                    position[0], position[1], self.TAKEOFF_HEIGHT
                )
            case State.TRANSPORT:
                if self._offboard_control.is_point_reached(*self.TRANSPORT_TARGET):
                    self._state = State.LAND
                    return
                self._offboard_control.fly_point(*self.TRANSPORT_TARGET)
            case State.LAND:
                self._offboard_control.land()
                self._state = State.END
                return


def main():
    rclpy.init()
    node = BasicMissionExampleNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
