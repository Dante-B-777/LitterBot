from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from litterbot_bridge.localization_config import EKFNoiseConfig
from litterbot_bridge.mecanum_motion_model import BodyTwist2D, MecanumMotionModel


@dataclass
class EKFState:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float

    def as_vector(self) -> np.ndarray:
        return np.array([[self.x], [self.y], [self.yaw], [self.vx], [self.vy], [self.wz]], dtype=float)

    @classmethod
    def from_vector(cls, vector: np.ndarray) -> 'EKFState':
        return cls(
            x=float(vector[0, 0]),
            y=float(vector[1, 0]),
            yaw=float(vector[2, 0]),
            vx=float(vector[3, 0]),
            vy=float(vector[4, 0]),
            wz=float(vector[5, 0]),
        )


class ExtendedKalmanFilter2D:
    """
    Inspectable EKF for planar localization.

    State vector:
    x = [x, y, yaw, vx, vy, wz]^T
    """

    STATE_DIM = 6

    def __init__(self, motion_model: MecanumMotionModel, noise: EKFNoiseConfig) -> None:
        self.motion_model = motion_model
        self.noise = noise
        self.state = EKFState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.covariance = np.diag(noise.initial_covariance_diag)
        self._last_predict_dt = 0.0

    def predict(self, dt: float, control: BodyTwist2D) -> None:
        if dt <= 0.0:
            return

        x_prev = self.state.as_vector()
        x_pred = np.array(self.motion_model.predict_state(self.state_as_list(), dt, control), dtype=float).reshape((-1, 1))

        yaw = float(x_prev[2, 0])
        vx = control.vx
        vy = control.vy
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        f_jacobian = np.eye(self.STATE_DIM)
        f_jacobian[0, 2] = (-vx * sin_yaw - vy * cos_yaw) * dt
        f_jacobian[1, 2] = (vx * cos_yaw - vy * sin_yaw) * dt

        q = np.diag([value * max(dt, 1e-3) for value in self.noise.process_noise_diag])
        self.covariance = (f_jacobian @ self.covariance @ f_jacobian.T) + q
        self.state = EKFState.from_vector(x_pred)
        self.state.yaw = self.motion_model.normalize_angle(self.state.yaw)
        self._last_predict_dt = dt

    def update_encoder_velocity(self, vx: float, vy: float, wz: float) -> None:
        z = np.array([[vx], [vy], [wz]], dtype=float)
        h = np.array([[self.state.vx], [self.state.vy], [self.state.wz]], dtype=float)
        h_jacobian = np.array([
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ], dtype=float)
        r = np.diag(self.noise.encoder_velocity_noise)
        self._apply_update(z=z, h=h, h_jacobian=h_jacobian, r=r)

    def update_imu(self, wz: float, ax: float, ay: float, yaw: float | None = None) -> None:
        if yaw is not None:
            z = np.array([[yaw], [wz]], dtype=float)
            h = np.array([[self.state.yaw], [self.state.wz]], dtype=float)
            h_jacobian = np.array([
                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ], dtype=float)
            r = np.diag([self.noise.imu_yaw_noise, self.noise.imu_wz_noise])
            self._apply_update(z=z, h=h, h_jacobian=h_jacobian, r=r, angle_indices={0})
        else:
            z = np.array([[wz]], dtype=float)
            h = np.array([[self.state.wz]], dtype=float)
            h_jacobian = np.array([
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ], dtype=float)
            r = np.array([[self.noise.imu_wz_noise]], dtype=float)
            self._apply_update(z=z, h=h, h_jacobian=h_jacobian, r=r)

        dt = self._last_predict_dt
        if dt > 0.0:
            self.state.vx += ax * dt
            self.state.vy += ay * dt

    def update_lidar_pose(self, x: float, y: float, yaw: float) -> None:
        z = np.array([[x], [y], [yaw]], dtype=float)
        h = np.array([[self.state.x], [self.state.y], [self.state.yaw]], dtype=float)
        h_jacobian = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        ], dtype=float)
        r = np.diag(self.noise.lidar_pose_noise)
        self._apply_update(z=z, h=h, h_jacobian=h_jacobian, r=r, angle_indices={2})

    def state_as_list(self) -> list[float]:
        return [self.state.x, self.state.y, self.state.yaw, self.state.vx, self.state.vy, self.state.wz]

    def _apply_update(
        self,
        z: np.ndarray,
        h: np.ndarray,
        h_jacobian: np.ndarray,
        r: np.ndarray,
        angle_indices: set[int] | None = None,
    ) -> None:
        innovation = z - h
        if angle_indices:
            for idx in angle_indices:
                innovation[idx, 0] = self.motion_model.normalize_angle(float(innovation[idx, 0]))

        s = (h_jacobian @ self.covariance @ h_jacobian.T) + r
        k = self.covariance @ h_jacobian.T @ np.linalg.inv(s)
        x_new = self.state.as_vector() + (k @ innovation)
        identity = np.eye(self.STATE_DIM)
        self.covariance = (identity - (k @ h_jacobian)) @ self.covariance
        self.state = EKFState.from_vector(x_new)
        self.state.yaw = self.motion_model.normalize_angle(self.state.yaw)

