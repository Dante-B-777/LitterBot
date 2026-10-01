from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


@dataclass
class ImuSample:
    msg: Imu
    stamp_ns: int


class IMUBridge(Node):
    def __init__(self) -> None:
        super().__init__('imu_bridge')

        self.declare_parameter('passthrough_topic', '/camera/realsense_imu/imu')
        self.declare_parameter('gyro_topic', '/camera/realsense_imu/gyro/sample')
        self.declare_parameter('accel_topic', '/camera/realsense_imu/accel/sample')
        self.declare_parameter('output_topic', '/imu/data_raw')
        self.declare_parameter('frame_id', 'camera_imu_optical_frame')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('sample_timeout_s', 0.25)
        self.declare_parameter('sync_window_s', 0.05)

        self._passthrough_topic = str(self.get_parameter('passthrough_topic').value)
        self._gyro_topic = str(self.get_parameter('gyro_topic').value)
        self._accel_topic = str(self.get_parameter('accel_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._sample_timeout_ns = int(float(self.get_parameter('sample_timeout_s').value) * 1e9)
        self._sync_window_ns = int(float(self.get_parameter('sync_window_s').value) * 1e9)

        self._imu_pub = self.create_publisher(Imu, self._output_topic, 20)
        self._status_pub = self.create_publisher(Bool, '/imu/status', 4)
        self._info_pub = self.create_publisher(String, '/imu/info', 10)

        self.create_subscription(Imu, self._passthrough_topic, self._on_passthrough, qos_profile_sensor_data)
        self.create_subscription(Imu, self._gyro_topic, self._on_gyro, qos_profile_sensor_data)
        self.create_subscription(Imu, self._accel_topic, self._on_accel, qos_profile_sensor_data)

        self._gyro_sample: ImuSample | None = None
        self._accel_sample: ImuSample | None = None
        self._last_publish_ns = 0
        self._last_status_ok = False
        self._last_status_msg = 'waiting for RealSense IMU topics'

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f'imu_bridge started - passthrough={self._passthrough_topic} '
            f'gyro={self._gyro_topic} accel={self._accel_topic} out={self._output_topic}'
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _stamp_ns(self, msg: Imu) -> int:
        stamp = msg.header.stamp
        return (stamp.sec * 1_000_000_000) + stamp.nanosec

    def _publish_status(self, ok: bool, text: str) -> None:
        if ok == self._last_status_ok and text == self._last_status_msg:
            return
        self._last_status_ok = ok
        self._last_status_msg = text
        self._status_pub.publish(Bool(data=ok))
        self._info_pub.publish(String(data=text))

    def _normalize_message(self, msg: Imu) -> Imu:
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self._frame_id or msg.header.frame_id
        out.orientation = msg.orientation
        out.orientation_covariance = list(msg.orientation_covariance)
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        return out

    def _on_passthrough(self, msg: Imu) -> None:
        out = self._normalize_message(msg)
        self._imu_pub.publish(out)
        self._last_publish_ns = self._now_ns()
        self._publish_status(True, f'publishing passthrough IMU from {self._passthrough_topic}')

    def _on_gyro(self, msg: Imu) -> None:
        self._gyro_sample = ImuSample(msg=msg, stamp_ns=self._stamp_ns(msg))

    def _on_accel(self, msg: Imu) -> None:
        self._accel_sample = ImuSample(msg=msg, stamp_ns=self._stamp_ns(msg))

    def _on_timer(self) -> None:
        now_ns = self._now_ns()

        if (now_ns - self._last_publish_ns) < self._sample_timeout_ns:
            self._publish_status(True, self._last_status_msg)
            return

        if self._gyro_sample is None or self._accel_sample is None:
            self._publish_status(False, 'waiting for both gyro and accel samples')
            return

        if (now_ns - self._gyro_sample.stamp_ns) > self._sample_timeout_ns:
            self._publish_status(False, 'gyro topic stale')
            return

        if (now_ns - self._accel_sample.stamp_ns) > self._sample_timeout_ns:
            self._publish_status(False, 'accel topic stale')
            return

        if abs(self._gyro_sample.stamp_ns - self._accel_sample.stamp_ns) > self._sync_window_ns:
            self._publish_status(False, 'gyro/accel samples out of sync')
            return

        gyro_msg = self._gyro_sample.msg
        accel_msg = self._accel_sample.msg

        out = Imu()
        out.header = gyro_msg.header
        out.header.frame_id = self._frame_id or gyro_msg.header.frame_id or accel_msg.header.frame_id
        out.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        out.angular_velocity = gyro_msg.angular_velocity
        out.angular_velocity_covariance = list(gyro_msg.angular_velocity_covariance)
        out.linear_acceleration = accel_msg.linear_acceleration
        out.linear_acceleration_covariance = list(accel_msg.linear_acceleration_covariance)

        self._imu_pub.publish(out)
        self._last_publish_ns = now_ns
        self._publish_status(True, 'publishing fused gyro+accel IMU samples')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IMUBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
