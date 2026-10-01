from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformBroadcaster

from litterbot_bridge.ekf_core import ExtendedKalmanFilter2D
from litterbot_bridge.encoder_model import EncoderObservationModel
from litterbot_bridge.imu_model import IMUObservationModel
from litterbot_bridge.lidar_model import LidarObservationModel
from litterbot_bridge.localization_calibrator import run_calibration
from litterbot_bridge.localization_config import (
    EKFNoiseConfig,
    LitterBotPhysicalConfig,
    LocalizationTopicConfig,
    load_calibration_overrides,
)
from litterbot_bridge.mecanum_motion_model import BodyTwist2D, MecanumMotionModel


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
    cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))
    return math.atan2(siny_cosp, cosy_cosp)


class EKFLocalizationNode(Node):
    """
    Modular EKF localization node.

    Modules:
    - MecanumMotionModel: robot kinematics and predict model
    - EncoderObservationModel: raw wheel ticks -> body twist
    - IMUObservationModel: IMU data -> yaw/wz/accel observation
    - LidarObservationModel: SLAM/LiDAR pose -> absolute correction
    - ExtendedKalmanFilter2D: prediction and update math
    """

    def __init__(self) -> None:
        super().__init__('ekf_localization')

        physical = LitterBotPhysicalConfig()
        topics = LocalizationTopicConfig()
        noise = EKFNoiseConfig()

        self.declare_parameter('wheel_radius_m', physical.wheel_radius_m)
        self.declare_parameter('wheelbase_m', physical.wheelbase_m)
        self.declare_parameter('track_width_m', physical.track_width_m)
        self.declare_parameter('encoder_ticks_per_rev', physical.encoder_ticks_per_rev)
        self.declare_parameter('base_frame', physical.base_frame)
        self.declare_parameter('odom_frame', physical.odom_frame)
        self.declare_parameter('map_frame', physical.map_frame)
        self.declare_parameter('encoder_topic', topics.encoder_topic)
        self.declare_parameter('imu_topic', topics.imu_topic)
        self.declare_parameter('lidar_pose_topic', topics.lidar_pose_topic)
        self.declare_parameter('output_odom_topic', topics.output_odom_topic)
        self.declare_parameter('publish_tf', topics.publish_tf)
        self.declare_parameter('lidar_pose_enabled', topics.lidar_pose_enabled)
        self.declare_parameter('process_noise_diag', noise.process_noise_diag)
        self.declare_parameter('encoder_velocity_noise', noise.encoder_velocity_noise)
        self.declare_parameter('imu_yaw_noise', noise.imu_yaw_noise)
        self.declare_parameter('imu_wz_noise', noise.imu_wz_noise)
        self.declare_parameter('imu_accel_noise', noise.imu_accel_noise)
        self.declare_parameter('lidar_pose_noise', noise.lidar_pose_noise)
        self.declare_parameter('initial_covariance_diag', noise.initial_covariance_diag)

        calibration = load_calibration_overrides()
        ekf_noise = calibration.get('ekf_noise', {})
        imu_bias = calibration.get('imu_bias', {})

        self.physical = LitterBotPhysicalConfig(
            wheel_radius_m=float(self.get_parameter('wheel_radius_m').value),
            wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            track_width_m=float(self.get_parameter('track_width_m').value),
            encoder_ticks_per_rev=float(self.get_parameter('encoder_ticks_per_rev').value),
            base_frame=str(self.get_parameter('base_frame').value),
            odom_frame=str(self.get_parameter('odom_frame').value),
            map_frame=str(self.get_parameter('map_frame').value),
        )
        self.topics = LocalizationTopicConfig(
            encoder_topic=str(self.get_parameter('encoder_topic').value),
            imu_topic=str(self.get_parameter('imu_topic').value),
            lidar_pose_topic=str(self.get_parameter('lidar_pose_topic').value),
            output_odom_topic=str(self.get_parameter('output_odom_topic').value),
            publish_tf=bool(self.get_parameter('publish_tf').value),
            lidar_pose_enabled=bool(self.get_parameter('lidar_pose_enabled').value),
        )
        self.noise = EKFNoiseConfig(
            process_noise_diag=list(self.get_parameter('process_noise_diag').value),
            encoder_velocity_noise=list(ekf_noise.get('encoder_velocity_noise', self.get_parameter('encoder_velocity_noise').value)),
            imu_yaw_noise=float(ekf_noise.get('imu_yaw_noise', self.get_parameter('imu_yaw_noise').value)),
            imu_wz_noise=float(ekf_noise.get('imu_wz_noise', self.get_parameter('imu_wz_noise').value)),
            imu_accel_noise=list(ekf_noise.get('imu_accel_noise', self.get_parameter('imu_accel_noise').value)),
            lidar_pose_noise=list(ekf_noise.get('lidar_pose_noise', self.get_parameter('lidar_pose_noise').value)),
            initial_covariance_diag=list(self.get_parameter('initial_covariance_diag').value),
        )
        self._imu_bias_wz = float(imu_bias.get('wz', 0.0))
        self._imu_bias_ax = float(imu_bias.get('ax', 0.0))
        self._imu_bias_ay = float(imu_bias.get('ay', 0.0))
        self._imu_bias_yaw = float(imu_bias.get('yaw', 0.0))

        self.motion_model = MecanumMotionModel(self.physical)
        self.encoder_model = EncoderObservationModel(self.motion_model)
        self.imu_model = IMUObservationModel()
        self.lidar_model = LidarObservationModel()
        self.ekf = ExtendedKalmanFilter2D(self.motion_model, self.noise)

        self._odom_pub = self.create_publisher(Odometry, self.topics.output_odom_topic, 10)
        self._status_pub = self.create_publisher(String, '/localization/status', 10)
        self._tf_pub = TransformBroadcaster(self) if self.topics.publish_tf else None

        self.create_subscription(Int32MultiArray, self.topics.encoder_topic, self._on_encoder, 30)
        self.create_subscription(Imu, self.topics.imu_topic, self._on_imu, 50)
        if self.topics.lidar_pose_enabled:
            self.create_subscription(PoseWithCovarianceStamped, self.topics.lidar_pose_topic, self._on_lidar_pose, 10)

        self._last_predict_time_s: float | None = None
        self.get_logger().info(
            'ekf_localization started - '
            f'encoder={self.topics.encoder_topic} imu={self.topics.imu_topic} '
            f'lidar_pose={self.topics.lidar_pose_topic} out={self.topics.output_odom_topic}'
        )

    def _stamp_to_seconds(self, stamp) -> float:
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

    def _predict_if_needed(self, stamp_s: float, control: BodyTwist2D) -> None:
        if self._last_predict_time_s is None:
            self._last_predict_time_s = stamp_s
            return
        dt = stamp_s - self._last_predict_time_s
        if dt <= 0.0:
            return
        self.ekf.predict(dt, control)
        self._last_predict_time_s = stamp_s

    def _on_encoder(self, msg: Int32MultiArray) -> None:
        now = self.get_clock().now().to_msg()
        stamp_s = self._stamp_to_seconds(now)
        obs = self.encoder_model.update(list(msg.data), stamp_s)
        if obs is None:
            return

        control = self.encoder_model.as_control(obs)
        self._predict_if_needed(stamp_s, control)
        self.ekf.update_encoder_velocity(obs.vx, obs.vy, obs.wz)
        self._publish_odometry(now)

    def _on_imu(self, msg: Imu) -> None:
        stamp_s = self._stamp_to_seconds(msg.header.stamp)
        yaw = None
        if msg.orientation_covariance and msg.orientation_covariance[0] >= 0.0:
            yaw = quaternion_to_yaw(msg.orientation) - self._imu_bias_yaw

        obs = self.imu_model.from_ros_components(
            wz=float(msg.angular_velocity.z) - self._imu_bias_wz,
            ax=float(msg.linear_acceleration.x) - self._imu_bias_ax,
            ay=float(msg.linear_acceleration.y) - self._imu_bias_ay,
            yaw=yaw,
        )
        self.ekf.update_imu(wz=obs.wz, ax=obs.ax, ay=obs.ay, yaw=obs.yaw)
        self._publish_odometry(msg.header.stamp)
        self._publish_status(f'imu update @ {stamp_s:.3f}s')

    def _on_lidar_pose(self, msg: PoseWithCovarianceStamped) -> None:
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        obs = self.lidar_model.from_pose(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        )
        self.ekf.update_lidar_pose(obs.x, obs.y, obs.yaw)
        self._publish_odometry(msg.header.stamp)
        self._publish_status(
            f'lidar correction x={obs.x:.3f} y={obs.y:.3f} yaw={obs.yaw:.3f}'
        )

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))

    def _publish_odometry(self, stamp) -> None:
        state = self.ekf.state
        covariance = self.ekf.covariance
        q = yaw_to_quaternion(state.yaw)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.physical.odom_frame
        msg.child_frame_id = self.physical.base_frame
        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = state.vx
        msg.twist.twist.linear.y = state.vy
        msg.twist.twist.angular.z = state.wz

        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        pose_cov[0] = float(covariance[0, 0])
        pose_cov[7] = float(covariance[1, 1])
        pose_cov[35] = float(covariance[2, 2])
        twist_cov[0] = float(covariance[3, 3])
        twist_cov[7] = float(covariance[4, 4])
        twist_cov[35] = float(covariance[5, 5])
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self._odom_pub.publish(msg)

        if self._tf_pub is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.physical.odom_frame
            tf_msg.child_frame_id = self.physical.base_frame
            tf_msg.transform.translation.x = state.x
            tf_msg.transform.translation.y = state.y
            tf_msg.transform.rotation = q
            self._tf_pub.sendTransform(tf_msg)


def main(args: list[str] | None = None) -> None:
    del args

    if sys.stdin.isatty():
        while True:
            answer = input('Calibrate localization sensors before startup? [y/n]: ').strip().lower()
            if answer in {'y', 'yes'}:
                run_calibration()
                break
            if answer in {'n', 'no', ''}:
                break
            print('Please enter y or n.')

    rclpy.init()
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
