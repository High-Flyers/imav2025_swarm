from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from qos_profiles import PX4_QOS


def offboard_command(func):
    def wrapper(self, *args, **kwargs):
        if self.is_in_offboard:
            return func(*args, **kwargs)

    return wrapper


class OffboardControl:
    HEARTBEAT_THRESHOLD = 10

    def __init__(self, node: Node) -> None:
        self._node = node

        self._vehicle_local_position = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()

        self._vehicle_local_position_sub = self._node.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.__vehicle_local_position_cb,
            PX4_QOS,
        )

        self._vehicle_status_sb = self._node.create_subscription(
            VehicleStatus, "fmu/out/vehicle_status", self.__vehicle_status_cb, PX4_QOS
        )

        self._vehicle_command_pub = self._node.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", PX4_QOS
        )
        self._offboard_control_mode_pub = self._node.create_publisher(
            OffboardControl, "fmu/in/offboard_control_mode", PX4_QOS
        )
        self._heartbeat = self._node.create_timer(0.1, self.__heartbeat_cb)

        self._heartbeat_counter = 0

    @property
    def is_in_offboard(self):
        return self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def is_armed(self):
        return self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def arm(self):
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def disarm(self):
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )

    def land(self):
        self.__publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_to_launch(self):
        self.__publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def set_offboard_mode(self):
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def set_hold_mode(self):
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=2.0
        )

    def __vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self._vehicle_local_position = msg

    def __vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def __publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.__timestamp_now()
        self._vehicle_command_pub.publish(msg)

    @offboard_command
    def __publish_offboard_control_heartbeat_signal(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.__timestamp_now()
        self._offboard_control_mode_pub.publish(msg)

    def __heartbeat_cb(self):
        self.__publish_offboard_control_heartbeat_signal()
        if self._heartbeat_counter < self.HEARTBEAT_THRESHOLD:
            self._heartbeat_counter += 1

    def __timestamp_now(self):
        return int(self._node.get_clock().now().nanoseconds / 1000)
