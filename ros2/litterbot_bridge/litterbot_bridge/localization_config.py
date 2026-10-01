from __future__ import annotations

from dataclasses import dataclass, field
import json
import os


CALIBRATION_DIR = os.path.join(os.path.expanduser('~'), '.litterbot')
CALIBRATION_FILE = os.path.join(CALIBRATION_DIR, 'localization_calibration.json')


@dataclass
class LitterBotPhysicalConfig:
    """
    Robot-specific physical values for localization.

    Variables you should verify for your bot:
    - wheel_radius_m: driven wheel radius in metres
    - wheelbase_m: front axle to rear axle centre distance
    - track_width_m: left wheel centre to right wheel centre distance
    - encoder_ticks_per_rev: output shaft encoder ticks per full wheel revolution
    - base_frame: robot body frame used in TF/odometry
    - odom_frame: local drift frame
    - map_frame: global corrected frame
    - camera_*: RealSense D455f position relative to base_link
    - lidar_*: RPLidar position relative to base_link
    """

    wheel_radius_m: float = 0.052
    wheelbase_m: float = 0.33655
    track_width_m: float = 0.41275
    encoder_ticks_per_rev: float = 537.7
    base_frame: str = 'base_link'
    odom_frame: str = 'odom'
    map_frame: str = 'map'
    camera_frame: str = 'camera_link'
    camera_x_m: float = 0.0
    camera_y_m: float = 0.0
    camera_z_m: float = 0.60325
    lidar_frame: str = 'laser'
    lidar_x_m: float = -0.127
    lidar_y_m: float = 0.0
    lidar_z_m: float = 0.3937

    @property
    def l_plus_w(self) -> float:
        return self.wheelbase_m + self.track_width_m


@dataclass
class EKFNoiseConfig:
    """
    Default covariance values.

    Tune these on the robot:
    - process_noise_diag: how much the state is allowed to drift each predict step
    - encoder_velocity_noise: wheel odom velocity observation variance
    - imu_yaw_noise: yaw observation variance if absolute yaw is available
    - imu_wz_noise: IMU yaw-rate observation variance
    - imu_accel_noise: IMU linear acceleration observation variance
    - lidar_pose_noise: LiDAR/SLAM absolute pose observation variance
    """

    process_noise_diag: list[float] = field(default_factory=lambda: [
        0.02,   # x
        0.02,   # y
        0.01,   # yaw
        0.10,   # vx
        0.10,   # vy
        0.08,   # wz
    ])
    encoder_velocity_noise: list[float] = field(default_factory=lambda: [
        0.05, 0.05, 0.04,
    ])
    imu_yaw_noise: float = 0.20
    imu_wz_noise: float = 0.03
    imu_accel_noise: list[float] = field(default_factory=lambda: [
        0.40, 0.40,
    ])
    lidar_pose_noise: list[float] = field(default_factory=lambda: [
        0.05, 0.05, 0.03,
    ])
    initial_covariance_diag: list[float] = field(default_factory=lambda: [
        0.10, 0.10, 0.10, 0.20, 0.20, 0.20,
    ])


@dataclass
class LocalizationTopicConfig:
    encoder_topic: str = '/encoder_raw'
    imu_topic: str = '/imu/data_raw'
    lidar_pose_topic: str = '/slam_toolbox/pose'
    output_odom_topic: str = '/odometry/ekf'
    publish_tf: bool = True
    lidar_pose_enabled: bool = True


def load_calibration_overrides() -> dict:
    if not os.path.exists(CALIBRATION_FILE):
        return {}

    try:
        with open(CALIBRATION_FILE, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {}

    if not isinstance(data, dict):
        return {}
    return data


def save_calibration_overrides(data: dict) -> None:
    os.makedirs(CALIBRATION_DIR, exist_ok=True)
    with open(CALIBRATION_FILE, 'w', encoding='utf-8') as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
