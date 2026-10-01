"""
odom_publisher.py - Publish /odom for the LitterBot web UI and ROS graph.

This node prefers raw wheel encoder ticks from /encoder_raw. If encoder data is
not available yet, it falls back to integrating the commanded /cmd_vel stream so
the rest of the stack still gets a live odometry estimate.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS_M = 0.052
WHEELBASE_M = 0.168
TRACK_WIDTH_M = 0.2025
L_PLUS_W = WHEELBASE_M + TRACK_WIDTH_M
TICKS_PER_REV = 537.7
TICK_TO_RAD = (2.0 * math.pi) / TICKS_PER_REV


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomPublisher(Node):
    """Publish nav_msgs/Odometry from encoder ticks or cmd_vel fallback."""

    def __init__(self) -> None:
        super().__init__('odom_publisher')

        self.declare_parameter('odom_rate_hz', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)

        self._odom_rate_hz = float(self.get_parameter('odom_rate_hz').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._cmd_vel_timeout_s = float(self.get_parameter('cmd_vel_timeout_s').value)

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_pub = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(Int32MultiArray, '/encoder_raw', self._on_encoder_raw, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_update_time = self.get_clock().now()
        self._last_cmd_vel_time = self.get_clock().now()
        self._last_encoder_time = None
        self._last_encoder_ticks: list[int] | None = None
        self._using_encoder_mode = False

        period = 1.0 / max(self._odom_rate_hz, 1.0)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f'odom_publisher started - rate={self._odom_rate_hz:.1f} Hz '
            f'odom_frame={self._odom_frame} base_frame={self._base_frame}'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._vx = float(msg.linear.x)
        self._vy = float(msg.linear.y)
        self._wz = float(msg.angular.z)
        self._last_cmd_vel_time = self.get_clock().now()

    def _on_encoder_raw(self, msg: Int32MultiArray) -> None:
        ticks = list(msg.data)
        if len(ticks) < 4:
            return

        now = self.get_clock().now()
        if self._last_encoder_ticks is None or self._last_encoder_time is None:
            self._last_encoder_ticks = ticks[:4]
            self._last_encoder_time = now
            return

        dt = (now - self._last_encoder_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        deltas = [ticks[i] - self._last_encoder_ticks[i] for i in range(4)]
        self._last_encoder_ticks = ticks[:4]
        self._last_encoder_time = now
        self._using_encoder_mode = True

        wheel_dists = [delta * TICK_TO_RAD * WHEEL_RADIUS_M for delta in deltas]
        fl, fr, rl, rr = wheel_dists

        vx = (fl + fr + rl + rr) / (4.0 * dt)
        vy = (-fl + fr + rl - rr) / (4.0 * dt)
        wz = (-fl + fr - rl + rr) / (4.0 * L_PLUS_W * dt)

        self._integrate(vx, vy, wz, dt)
        self._publish_odom(now)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        encoder_age = None
        if self._last_encoder_time is not None:
            encoder_age = (now - self._last_encoder_time).nanoseconds / 1e9

        if encoder_age is not None and encoder_age < 0.25:
            return

        cmd_age = (now - self._last_cmd_vel_time).nanoseconds / 1e9
        vx = self._vx if cmd_age <= self._cmd_vel_timeout_s else 0.0
        vy = self._vy if cmd_age <= self._cmd_vel_timeout_s else 0.0
        wz = self._wz if cmd_age <= self._cmd_vel_timeout_s else 0.0
        self._using_encoder_mode = False

        self._integrate(vx, vy, wz, dt)
        self._publish_odom(now)

    def _integrate(self, vx: float, vy: float, wz: float, dt: float) -> None:
        self._last_update_time = self.get_clock().now()
        self._yaw += wz * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        world_vx = vx * cos_yaw - vy * sin_yaw
        world_vy = vx * sin_yaw + vy * cos_yaw
        self._x += world_vx * dt
        self._y += world_vy * dt
        self._vx = vx
        self._vy = vy
        self._wz = wz

    def _publish_odom(self, stamp) -> None:
        q = yaw_to_quaternion(self._yaw)

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = self._vx
        msg.twist.twist.linear.y = self._vy
        msg.twist.twist.angular.z = self._wz
        self._odom_pub.publish(msg)

        if self._tf_pub is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self._odom_frame
            tf_msg.child_frame_id = self._base_frame
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.rotation = q
            self._tf_pub.sendTransform(tf_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
