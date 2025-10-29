from rclpy.node import Node

import message_filters as mf

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude,
    TrajectorySetpoint,
)

from flight_control.utils.qos_profiles import PX4_QOS
from flight_control.utils.frame_transforms import (
    ENULocalOdometry,
    enu_to_ned,
    enu_to_ned_heading,
)


class OffboardControl:
    HEARTBEAT_THRESHOLD = 10

    def __init__(self, node: Node, id: int = 0) -> None:
        self._id = id
        self._enu: ENULocalOdometry = None

        self._node = node

        self._vehicle_local_position = VehicleLocalPosition()
        self._vehicle_attitude = VehicleAttitude()
        self._vehicle_status = VehicleStatus()

        self._vehicle_odometry_ts = mf.ApproximateTimeSynchronizer(
            [
                mf.Subscriber(
                    self._node,
                    VehicleLocalPosition,
                    "fmu/out/vehicle_local_position",
                    qos_profile=PX4_QOS,
                ),
                mf.Subscriber(
                    self._node,
                    VehicleAttitude,
                    "fmu/out/vehicle_attitude",
                    qos_profile=PX4_QOS,
                ),
            ],
            slop=0.1,
            queue_size=5,
            allow_headerless=True,
        )
        self._vehicle_odometry_ts.registerCallback(self.__vehicle_odom_ts_cb)

        self._vehicle_status_sub = self._node.create_subscription(
            VehicleStatus, "fmu/out/vehicle_status", self.__vehicle_status_cb, PX4_QOS
        )

        self._vehicle_command_pub = self._node.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", PX4_QOS
        )
        self._offboard_control_mode_pub = self._node.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", PX4_QOS
        )
        self._trejctory_setpoint_pub = self._node.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", PX4_QOS
        )
        self._heartbeat = self._node.create_timer(0.1, self.__heartbeat_cb)

        self._heartbeat_counter = 0

    @property
    def enu(self) -> ENULocalOdometry:
        return self._enu

    @property
    def local_position(self) -> VehicleLocalPosition:
        return self._vehicle_local_position

    @property
    def attitude(self) -> VehicleAttitude:
        return self._vehicle_attu

    @property
    def is_ready(self) -> bool:
        return (
            self._heartbeat_counter >= self.HEARTBEAT_THRESHOLD
            and self.is_in_offboard
            and self._enu is not None
        )

    @property
    def is_in_offboard(self) -> bool:
        return self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def is_armed(self) -> bool:
        return self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def arm(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def disarm(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )

    def land(self) -> None:
        self.__publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_to_launch(self) -> None:
        self.__publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def set_offboard_mode(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def set_hold_mode(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=2.0
        )

    def is_point_reached(self, x: float, y: float, z: float, epsilon=0.1) -> bool:
        """
        Check if the vehicle reached coordinates specified by ENU x, y, z.
        """
        current = self._enu.position()
        target = (x, y, z)
        return all(abs(c - t) < epsilon for c, t in zip(current, target))

    def fly_point(self, x: float, y: float, z: float, heading=None) -> None:
        """
        Send command to fly to point specified by x, y, z coordinates (in ENU convention).
        """
        if not self.is_in_offboard:
            return

        msg = TrajectorySetpoint()
        msg.position = enu_to_ned(x, y, z)
        msg.yaw = enu_to_ned_heading(
            heading if heading is not None else self._enu.heading
        )
        msg.timestamp = self.__timestamp_now()
        self._trejctory_setpoint_pub.publish(msg)

    def __vehicle_odom_ts_cb(
        self, local_position: VehicleLocalPosition, attitude: VehicleAttitude
    ) -> None:
        self._vehicle_local_position = local_position
        self._vehicle_attitude = attitude
        self._enu = ENULocalOdometry.from_px4(local_position, attitude)

    def __vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def __publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.target_system = self._id + 1
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.__timestamp_now()
        self._vehicle_command_pub.publish(msg)

    def __publish_offboard_control_heartbeat_signal(self) -> None:
        if not self.is_in_offboard:
            return

        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.__timestamp_now()
        self._offboard_control_mode_pub.publish(msg)

    def __heartbeat_cb(self) -> None:
        self.__publish_offboard_control_heartbeat_signal()
        if self._heartbeat_counter < self.HEARTBEAT_THRESHOLD:
            self._heartbeat_counter += 1

    def __timestamp_now(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1000)
