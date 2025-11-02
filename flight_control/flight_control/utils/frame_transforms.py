from dataclasses import dataclass

import math
import numpy as np
from quaternion import from_euler_angles, as_euler_angles

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


# WGS84 ellipsoid constants
a = 6378137.0  # semi-major axis
f = 1 / 298.257223563  # flattening
b = a * (1 - f)
e_sq = f * (2 - f)


def lla_to_ecef(lat, lon, alt):
    """Convert latitude, longitude, altitude to ECEF coordinates."""
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    N = a / math.sqrt(1 - e_sq * (math.sin(lat_rad) ** 2))

    x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (N * (1 - e_sq) + alt) * math.sin(lat_rad)

    return np.array([x, y, z])


def ecef_to_enu(x, y, z, lat_ref, lon_ref, alt_ref):
    """Convert ECEF coordinates to local ENU frame."""
    x_ref, y_ref, z_ref = lla_to_ecef(lat_ref, lon_ref, alt_ref)

    lat_rad = math.radians(lat_ref)
    lon_rad = math.radians(lon_ref)

    # Rotation matrix from ECEF to ENU
    R = np.array(
        [
            [-math.sin(lon_rad), math.cos(lon_rad), 0],
            [
                -math.sin(lat_rad) * math.cos(lon_rad),
                -math.sin(lat_rad) * math.sin(lon_rad),
                math.cos(lat_rad),
            ],
            [
                math.cos(lat_rad) * math.cos(lon_rad),
                math.cos(lat_rad) * math.sin(lon_rad),
                math.sin(lat_rad),
            ],
        ]
    )

    ecef_vector = np.array([x - x_ref, y - y_ref, z - z_ref])
    enu = R @ ecef_vector

    return enu  # [East, North, Up]


def lla_to_enu(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """Directly convert LLA to ENU using reference."""
    x, y, z = lla_to_ecef(lat, lon, alt)
    return ecef_to_enu(x, y, z, lat_ref, lon_ref, alt_ref)


def ned_to_enu(n, e, d):
    return (float(e), float(n), float(-d))


def enu_to_ned(e, n, u):
    return (float(n), float(e), float(-u))


def ned_to_enu_heading(heading):
    return -heading + (np.pi / 2.0)


def enu_to_ned_heading(heading):
    return -heading + (np.pi / 2.0)


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

    ref_lat: float
    ref_lon: float
    ref_alt: float

    @classmethod
    def from_px4(cls, local_position: VehicleLocalPosition, attitude: VehicleAttitude):
        """
        Initialize position and orientation data based on vehicle local position and attitude reported by px4 autopilot.
        """
        x, y, z = ned_to_enu(local_position.x, local_position.y, local_position.z)
        vx, vy, vz = ned_to_enu(local_position.vx, local_position.vy, local_position.vz)
        qx, qy, qz, qw = frd_to_flu_quaternion(*attitude.q[1:], w=attitude.q[0])
        heading = ned_to_enu_heading(local_position.heading)
        return ENULocalOdometry(
            x,
            y,
            z,
            vx,
            vy,
            vz,
            qx,
            qy,
            qz,
            qw,
            heading,
            local_position.ref_lat,
            local_position.ref_lon,
            local_position.ref_alt,
        )

    def position(self):
        return self.x, self.y, self.z

    def velocity(self):
        return self.vx, self.vy, self.vz

    def orientation(self):
        return self.qx, self.qy, self.qz, self.qw
