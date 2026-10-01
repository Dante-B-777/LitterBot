from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LidarPoseObservation:
    x: float
    y: float
    yaw: float


class LidarObservationModel:
    """
    Absolute pose correction model.

    This is intended to consume pose estimates produced by LiDAR-based mapping
    or localization, such as slam_toolbox localization mode.
    """

    def from_pose(self, x: float, y: float, yaw: float) -> LidarPoseObservation:
        return LidarPoseObservation(x=x, y=y, yaw=yaw)

