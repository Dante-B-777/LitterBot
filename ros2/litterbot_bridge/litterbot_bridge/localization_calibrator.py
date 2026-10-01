from __future__ import annotations

import math
from statistics import mean

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

from litterbot_bridge.localization_config import (
    CALIBRATION_FILE,
    LocalizationTopicConfig,
    save_calibration_overrides,
)


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
    cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))
    return math.atan2(siny_cosp, cosy_cosp)


def _variance(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    mu = mean(values)
    return sum((value - mu) ** 2 for value in values) / float(len(values) - 1)


class LocalizationCalibrator(Node):
    """
    Stationary preflight calibration.

    Assumes:
    - robot is not moving
    - LiDAR / slam pose is stable if enabled
    - IMU is streaming
    """

    def __init__(self) -> None:
        super().__init__('localization_calibrator')

        topics = LocalizationTopicConfig()
        self.declare_parameter('encoder_topic', topics.encoder_topic)
        self.declare_parameter('imu_topic', topics.imu_topic)
        self.declare_parameter('lidar_pose_topic', topics.lidar_pose_topic)
        self.declare_parameter('lidar_pose_enabled', topics.lidar_pose_enabled)
        self.declare_parameter('duration_s', 20.0)

        self._encoder_topic = str(self.get_parameter('encoder_topic').value)
        self._imu_topic = str(self.get_parameter('imu_topic').value)
        self._lidar_pose_topic = str(self.get_parameter('lidar_pose_topic').value)
        self._lidar_pose_enabled = bool(self.get_parameter('lidar_pose_enabled').value)
        self._duration_s = float(self.get_parameter('duration_s').value)

        self._encoder_samples: list[list[int]] = []
        self._imu_wz_samples: list[float] = []
        self._imu_ax_samples: list[float] = []
        self._imu_ay_samples: list[float] = []
        self._imu_yaw_samples: list[float] = []
        self._lidar_x_samples: list[float] = []
        self._lidar_y_samples: list[float] = []
        self._lidar_yaw_samples: list[float] = []

        self._done = False
        self._start_time = self.get_clock().now()
        self._last_progress_second = -1

        self.create_subscription(Int32MultiArray, self._encoder_topic, self._on_encoder, 50)
        self.create_subscription(Imu, self._imu_topic, self._on_imu, 100)
        if self._lidar_pose_enabled:
            self.create_subscription(PoseWithCovarianceStamped, self._lidar_pose_topic, self._on_lidar_pose, 20)

        self.create_timer(0.2, self._on_timer)
        self.get_logger().info(
            'Localization calibration started. Keep the robot stationary until completion.'
        )

    @property
    def done(self) -> bool:
        return self._done

    def _on_encoder(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 4:
            self._encoder_samples.append(list(msg.data[:4]))

    def _on_imu(self, msg: Imu) -> None:
        self._imu_wz_samples.append(float(msg.angular_velocity.z))
        self._imu_ax_samples.append(float(msg.linear_acceleration.x))
        self._imu_ay_samples.append(float(msg.linear_acceleration.y))
        if msg.orientation_covariance and msg.orientation_covariance[0] >= 0.0:
            self._imu_yaw_samples.append(quaternion_to_yaw(msg.orientation))

    def _on_lidar_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._lidar_x_samples.append(float(msg.pose.pose.position.x))
        self._lidar_y_samples.append(float(msg.pose.pose.position.y))
        self._lidar_yaw_samples.append(quaternion_to_yaw(msg.pose.pose.orientation))

    def _on_timer(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        elapsed_sec = int(elapsed)
        if elapsed_sec != self._last_progress_second and elapsed_sec <= int(self._duration_s):
            self._last_progress_second = elapsed_sec
            self.get_logger().info(
                f'Calibration progress: {elapsed:.1f}/{self._duration_s:.1f}s'
            )

        if elapsed >= self._duration_s:
            self._finalize()

    def _finalize(self) -> None:
        if self._done:
            return
        self._done = True

        encoder_velocity_noise = self._estimate_encoder_velocity_noise()
        imu_yaw_noise = max(_variance(self._imu_yaw_samples), 1e-6) if self._imu_yaw_samples else 0.20
        imu_wz_noise = max(_variance(self._imu_wz_samples), 1e-6) if self._imu_wz_samples else 0.03
        imu_accel_noise = [
            max(_variance(self._imu_ax_samples), 1e-6) if self._imu_ax_samples else 0.40,
            max(_variance(self._imu_ay_samples), 1e-6) if self._imu_ay_samples else 0.40,
        ]
        lidar_pose_noise = self._estimate_lidar_pose_noise()

        calibration = {
            'imu_bias': {
                'wz': mean(self._imu_wz_samples) if self._imu_wz_samples else 0.0,
                'ax': mean(self._imu_ax_samples) if self._imu_ax_samples else 0.0,
                'ay': mean(self._imu_ay_samples) if self._imu_ay_samples else 0.0,
                'yaw': mean(self._imu_yaw_samples) if self._imu_yaw_samples else 0.0,
            },
            'ekf_noise': {
                'encoder_velocity_noise': encoder_velocity_noise,
                'imu_yaw_noise': imu_yaw_noise,
                'imu_wz_noise': imu_wz_noise,
                'imu_accel_noise': imu_accel_noise,
                'lidar_pose_noise': lidar_pose_noise,
            },
            'sample_counts': {
                'encoder': len(self._encoder_samples),
                'imu': len(self._imu_wz_samples),
                'lidar_pose': len(self._lidar_x_samples),
            },
        }
        save_calibration_overrides(calibration)
        self.get_logger().info(f'Calibration saved to {CALIBRATION_FILE}')
        self.get_logger().info(f'Computed noise: {calibration["ekf_noise"]}')

    def _estimate_encoder_velocity_noise(self) -> list[float]:
        if len(self._encoder_samples) < 2:
            return [0.05, 0.05, 0.04]

        deltas = []
        last = self._encoder_samples[0]
        for current in self._encoder_samples[1:]:
            deltas.append([current[i] - last[i] for i in range(4)])
            last = current

        flat = [item for sample in deltas for item in sample]
        tick_var = max(_variance([float(value) for value in flat]), 1e-6) if flat else 1e-6
        velocity_var = max(tick_var * 1e-3, 1e-6)
        return [velocity_var, velocity_var, velocity_var]

    def _estimate_lidar_pose_noise(self) -> list[float]:
        if not self._lidar_pose_enabled or len(self._lidar_x_samples) < 2:
            return [0.05, 0.05, 0.03]
        return [
            max(_variance(self._lidar_x_samples), 1e-6),
            max(_variance(self._lidar_y_samples), 1e-6),
            max(_variance(self._lidar_yaw_samples), 1e-6),
        ]


def run_calibration() -> None:
    rclpy.init()
    node = LocalizationCalibrator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    del args
    run_calibration()
