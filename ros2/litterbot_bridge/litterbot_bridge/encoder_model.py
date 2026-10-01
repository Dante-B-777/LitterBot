from __future__ import annotations

from dataclasses import dataclass

from litterbot_bridge.mecanum_motion_model import BodyTwist2D, MecanumMotionModel


@dataclass
class EncoderObservation:
    vx: float
    vy: float
    wz: float


class EncoderObservationModel:
    def __init__(self, motion_model: MecanumMotionModel) -> None:
        self._motion_model = motion_model
        self._last_ticks: list[int] | None = None
        self._last_stamp_s: float | None = None

    def update(self, ticks: list[int], stamp_s: float) -> EncoderObservation | None:
        if len(ticks) < 4:
            return None

        if self._last_ticks is None or self._last_stamp_s is None:
            self._last_ticks = ticks[:4]
            self._last_stamp_s = stamp_s
            return None

        dt = stamp_s - self._last_stamp_s
        if dt <= 0.0:
            return None

        deltas = [ticks[i] - self._last_ticks[i] for i in range(4)]
        self._last_ticks = ticks[:4]
        self._last_stamp_s = stamp_s

        twist = self._motion_model.body_twist_from_encoder_deltas(
            deltas[0], deltas[1], deltas[2], deltas[3], dt,
        )
        return EncoderObservation(vx=twist.vx, vy=twist.vy, wz=twist.wz)

    @staticmethod
    def as_control(obs: EncoderObservation) -> BodyTwist2D:
        return BodyTwist2D(vx=obs.vx, vy=obs.vy, wz=obs.wz)

