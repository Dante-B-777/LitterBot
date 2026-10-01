from __future__ import annotations

import math
from dataclasses import dataclass

from litterbot_bridge.localization_config import LitterBotPhysicalConfig


@dataclass
class BodyTwist2D:
    vx: float
    vy: float
    wz: float


class MecanumMotionModel:
    """
    Predicts robot motion for a 2D mecanum base.

    State layout:
    [x, y, yaw, vx, vy, wz]
    """

    def __init__(self, physical: LitterBotPhysicalConfig) -> None:
        self._physical = physical
        self._tick_to_rad = (2.0 * math.pi) / physical.encoder_ticks_per_rev

    def body_twist_from_encoder_deltas(
        self,
        ticks_fl: int,
        ticks_fr: int,
        ticks_rl: int,
        ticks_rr: int,
        dt: float,
    ) -> BodyTwist2D:
        if dt <= 0.0:
            return BodyTwist2D(0.0, 0.0, 0.0)

        wheel_radius = self._physical.wheel_radius_m
        l_plus_w = self._physical.l_plus_w

        d_fl = ticks_fl * self._tick_to_rad * wheel_radius
        d_fr = ticks_fr * self._tick_to_rad * wheel_radius
        d_rl = ticks_rl * self._tick_to_rad * wheel_radius
        d_rr = ticks_rr * self._tick_to_rad * wheel_radius

        vx = (d_fl + d_fr + d_rl + d_rr) / (4.0 * dt)
        vy = (-d_fl + d_fr + d_rl - d_rr) / (4.0 * dt)
        wz = (-d_fl + d_fr - d_rl + d_rr) / (4.0 * l_plus_w * dt)
        return BodyTwist2D(vx=vx, vy=vy, wz=wz)

    def predict_state(
        self,
        state: list[float],
        dt: float,
        control: BodyTwist2D,
    ) -> list[float]:
        x, y, yaw, _, _, _ = state
        vx = control.vx
        vy = control.vy
        wz = control.wz

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_vx = (vx * cos_yaw) - (vy * sin_yaw)
        world_vy = (vx * sin_yaw) + (vy * cos_yaw)

        yaw_next = self.normalize_angle(yaw + (wz * dt))
        return [
            x + (world_vx * dt),
            y + (world_vy * dt),
            yaw_next,
            vx,
            vy,
            wz,
        ]

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

