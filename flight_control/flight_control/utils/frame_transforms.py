from dataclasses import dataclass

import numpy as np
from quaternion import from_euler_angles, as_euler_angles

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


def ned_to_enu(n, e, d):
    return (float(e), float(n), float(-d))


def enu_to_ned(e, n, u):
    return (float(n), float(e), float(-u))


def ned_to_enu_heading(heading):
    return heading + (np.pi / 2)


def enu_to_ned_heading(heading):
    return heading - (np.pi / 2)


def frd_to_flu_quaternion(x, y, z, w):
    q_enu = np.quaternion(float(w), float(x), float(-y), float(-z))
    q_90 = from_euler_angles((0, 0, np.pi / 2))

    result = q_90 * q_enu
    return (result.x, result.y, result.z, result.w)


def heading_from_quaternion(x, y, z, w):
    q = np.quaternion(float(w), float(x), float(y), float(z))
    angles = as_euler_angles(q)

    return angles[0]


@dataclass
class ENULocalOdometry:
    """
    Vehicle position and orientation data stored in the ENU format.
    """

    x: float
    y: float
    z: float

    vx: float
    vy: float
    vz: float

    qx: float
    qy: float
    qz: float
    qw: float

    heading: float

    @classmethod
    def from_px4(
        self, local_position: VehicleLocalPosition, attitude: VehicleAttitude
    ) -> None:
        """
        Initialize position and orientation data based on vehicle local position and attitude reported by px4 autopilot.
        """
        self.x, self.y, self.z = ned_to_enu(
            local_position.x, local_position.y, local_position.z
        )
        self.vx, self.vy, self.vz = ned_to_enu(
            local_position.vx, local_position.vy, local_position.vz
        )
        self.qx, self.qy, self.qz, self.qw = frd_to_flu_quaternion(
            *attitude.q[1:], w=attitude.q[0]
        )
        self.heading = -local_position.heading + (np.pi / 2.0)

    def position(self):
        return self.x, self.y, self.z

    def velocity(self):
        return self.vx, self.vy, self.vz

    def orientation(self):
        return self.qx, self.qy, self.qz, self.qw
