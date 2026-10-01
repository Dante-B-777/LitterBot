from __future__ import annotations

from dataclasses import dataclass


@dataclass
class IMUObservation:
    yaw: float | None
    wz: float
    ax: float
    ay: float


class IMUObservationModel:
    """
    IMU model for a planar mobile robot.

    Expected conventions:
    - wz: angular velocity around +Z in base frame
    - ax, ay: linear acceleration in robot/body frame
    - yaw: optional absolute yaw if an upstream IMU filter provides orientation
    """

    def from_ros_components(
        self,
        wz: float,
        ax: float,
        ay: float,
        yaw: float | None = None,
    ) -> IMUObservation:
        return IMUObservation(yaw=yaw, wz=wz, ax=ax, ay=ay)

