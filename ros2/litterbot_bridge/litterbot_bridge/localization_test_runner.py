from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(orientation) -> float:
    siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y))
    cosy_cosp = 1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)))
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class MotionSegment:
    name: str
    duration_s: float
    linear_x: float
    linear_y: float
    angular_z: float
    description: str


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class TestSample:
    time_s: float
    ideal_x: float
    ideal_y: float
    ideal_yaw: float
    ideal_vx: float
    ideal_vy: float
    ideal_wz: float
    actual_x: float
    actual_y: float
    actual_yaw: float
    actual_vx: float
    actual_vy: float
    actual_wz: float
    error_x: float
    error_y: float
    error_pos: float
    error_yaw: float


class LocalizationTestRunner(Node):
    def __init__(self) -> None:
        super().__init__('localization_test_runner')

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/ekf')
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('startup_delay_s', 3.0)
        self.declare_parameter('output_root', str(Path.home() / '.litterbot' / 'localization_tests'))

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._command_rate_hz = max(float(self.get_parameter('command_rate_hz').value), 1.0)
        self._startup_delay_s = max(float(self.get_parameter('startup_delay_s').value), 0.0)
        self._output_root = Path(str(self.get_parameter('output_root').value))

        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 30)

        self._actual_pose: Pose2D | None = None
        self._start_actual_pose: Pose2D | None = None

        self.get_logger().info(
            f'localization_test_runner started - cmd={self._cmd_topic} odom={self._odom_topic}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._actual_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=quaternion_to_yaw(msg.pose.pose.orientation),
            vx=float(msg.twist.twist.linear.x),
            vy=float(msg.twist.twist.linear.y),
            wz=float(msg.twist.twist.angular.z),
        )

    def run_interactive(self) -> None:
        print('Localization Test Runner')
        print('This script commands the robot and compares /odometry/ekf to an ideal trajectory.')
        print('')
        segments = self._prompt_segments()
        if not segments:
            print('No motion segments entered. Exiting.')
            return

        print('')
        print('Planned trajectory:')
        total_duration_s = 0.0
        for index, segment in enumerate(segments, start=1):
            total_duration_s += segment.duration_s
            print(f'  {index}. {segment.description} ({segment.duration_s:.2f} s)')
        print(f'Total commanded motion time: {total_duration_s:.2f} s')
        print('')

        self._wait_for_odometry()
        self._countdown()
        samples, output_dir, summary = self._execute_segments(segments)
        self._write_outputs(samples, output_dir, summary, segments)
        print('')
        print(f'Test output saved to: {output_dir}')
        print(f'Position RMSE: {summary["rmse_position_m"]:.4f} m')
        print(f'Max position error: {summary["max_position_error_m"]:.4f} m')
        print(f'Yaw RMSE: {summary["rmse_yaw_rad"]:.4f} rad')

    def _prompt_segments(self) -> list[MotionSegment]:
        segments: list[MotionSegment] = []

        while True:
            print('Choose a motion segment to append:')
            print('  circle  - drive a circle with constant linear speed and yaw rate')
            print('  line    - drive straight or strafe')
            print('  rotate  - rotate in place')
            print('  done    - start the test')
            choice = input('Segment type [circle/line/rotate/done]: ').strip().lower()
            if choice in {'', 'done'}:
                return segments
            if choice == 'circle':
                segment = self._prompt_circle_segment()
            elif choice == 'line':
                segment = self._prompt_line_segment()
            elif choice == 'rotate':
                segment = self._prompt_rotate_segment()
            else:
                print('Unsupported segment. Please choose circle, line, rotate, or done.')
                print('')
                continue

            segments.append(segment)
            print(f'Added: {segment.description}')
            print('')

    def _prompt_circle_segment(self) -> MotionSegment:
        radius_m = self._prompt_positive_float('Circle radius in meters', 0.5)
        speed_mps = self._prompt_positive_float('Linear speed in m/s', 0.20)
        laps = self._prompt_positive_float('Number of laps', 1.0)
        direction = input('Direction [ccw/cw] (default ccw): ').strip().lower() or 'ccw'
        sign = 1.0 if direction != 'cw' else -1.0
        angular_z = sign * (speed_mps / radius_m)
        duration_s = (2.0 * math.pi * radius_m * laps) / speed_mps
        return MotionSegment(
            name='circle',
            duration_s=duration_s,
            linear_x=speed_mps,
            linear_y=0.0,
            angular_z=angular_z,
            description=(
                f'Circle radius={radius_m:.3f} m speed={speed_mps:.3f} m/s '
                f'laps={laps:.2f} direction={direction}'
            ),
        )

    def _prompt_line_segment(self) -> MotionSegment:
        axis = input('Axis [forward/strafe] (default forward): ').strip().lower() or 'forward'
        distance_m = self._prompt_positive_float('Distance in meters', 1.0)
        speed_mps = self._prompt_positive_float('Linear speed in m/s', 0.20)
        direction = input('Direction sign [+/-] (default +): ').strip() or '+'
        sign = -1.0 if direction.startswith('-') else 1.0
        duration_s = distance_m / speed_mps
        linear_x = sign * speed_mps if axis != 'strafe' else 0.0
        linear_y = sign * speed_mps if axis == 'strafe' else 0.0
        return MotionSegment(
            name='line',
            duration_s=duration_s,
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=0.0,
            description=(
                f'Line axis={axis} distance={distance_m:.3f} m '
                f'speed={sign * speed_mps:.3f} m/s'
            ),
        )

    def _prompt_rotate_segment(self) -> MotionSegment:
        angle_deg = self._prompt_positive_float('Rotation angle in degrees', 360.0)
        yaw_rate = self._prompt_positive_float('Yaw rate in rad/s', 0.5)
        direction = input('Direction [ccw/cw] (default ccw): ').strip().lower() or 'ccw'
        sign = 1.0 if direction != 'cw' else -1.0
        angular_z = sign * yaw_rate
        duration_s = math.radians(angle_deg) / yaw_rate
        return MotionSegment(
            name='rotate',
            duration_s=duration_s,
            linear_x=0.0,
            linear_y=0.0,
            angular_z=angular_z,
            description=(
                f'Rotate angle={angle_deg:.1f} deg yaw_rate={yaw_rate:.3f} rad/s '
                f'direction={direction}'
            ),
        )

    def _prompt_positive_float(self, label: str, default: float) -> float:
        while True:
            raw = input(f'{label} [{default}]: ').strip()
            if raw == '':
                return default
            try:
                value = float(raw)
            except ValueError:
                print('Please enter a numeric value.')
                continue
            if value <= 0.0:
                print('Value must be positive.')
                continue
            return value

    def _wait_for_odometry(self) -> None:
        print(f'Waiting for localization odometry on {self._odom_topic}...')
        while rclpy.ok() and self._actual_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._actual_pose is None:
            raise RuntimeError(f'No odometry received on {self._odom_topic}.')
        print('Localization stream detected.')

    def _countdown(self) -> None:
        delay_s = int(math.ceil(self._startup_delay_s))
        if delay_s <= 0:
            return
        print('Place the robot at the desired start pose and clear the area.')
        for remaining in range(delay_s, 0, -1):
            print(f'Starting in {remaining}...')
            self._publish_cmd(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=1.0)

    def _execute_segments(
        self,
        segments: list[MotionSegment],
    ) -> tuple[list[TestSample], Path, dict[str, float]]:
        output_dir = self._output_root / datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir.mkdir(parents=True, exist_ok=True)

        samples: list[TestSample] = []
        ideal_pose = Pose2D()
        self._start_actual_pose = self._actual_pose

        timer_period_s = 1.0 / self._command_rate_hz
        elapsed_s = 0.0

        for segment in segments:
            segment_elapsed_s = 0.0
            while rclpy.ok() and segment_elapsed_s < segment.duration_s:
                dt = min(timer_period_s, segment.duration_s - segment_elapsed_s)
                self._publish_cmd(segment.linear_x, segment.linear_y, segment.angular_z)
                rclpy.spin_once(self, timeout_sec=dt)

                ideal_pose = self._integrate_ideal_pose(
                    ideal_pose,
                    segment.linear_x,
                    segment.linear_y,
                    segment.angular_z,
                    dt,
                )
                actual_pose = self._relative_actual_pose()
                samples.append(self._make_sample(elapsed_s + dt, ideal_pose, actual_pose))
                elapsed_s += dt
                segment_elapsed_s += dt

        self._publish_cmd(0.0, 0.0, 0.0)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        summary = self._summarize(samples)
        return samples, output_dir, summary

    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _integrate_ideal_pose(
        self,
        pose: Pose2D,
        vx: float,
        vy: float,
        wz: float,
        dt: float,
    ) -> Pose2D:
        mid_yaw = pose.yaw + (0.5 * wz * dt)
        world_vx = (vx * math.cos(mid_yaw)) - (vy * math.sin(mid_yaw))
        world_vy = (vx * math.sin(mid_yaw)) + (vy * math.cos(mid_yaw))
        return Pose2D(
            x=pose.x + (world_vx * dt),
            y=pose.y + (world_vy * dt),
            yaw=normalize_angle(pose.yaw + (wz * dt)),
            vx=vx,
            vy=vy,
            wz=wz,
        )

    def _relative_actual_pose(self) -> Pose2D:
        if self._actual_pose is None or self._start_actual_pose is None:
            return Pose2D()

        dx = self._actual_pose.x - self._start_actual_pose.x
        dy = self._actual_pose.y - self._start_actual_pose.y
        cos_yaw = math.cos(self._start_actual_pose.yaw)
        sin_yaw = math.sin(self._start_actual_pose.yaw)

        rel_x = (cos_yaw * dx) + (sin_yaw * dy)
        rel_y = (-sin_yaw * dx) + (cos_yaw * dy)
        return Pose2D(
            x=rel_x,
            y=rel_y,
            yaw=normalize_angle(self._actual_pose.yaw - self._start_actual_pose.yaw),
            vx=self._actual_pose.vx,
            vy=self._actual_pose.vy,
            wz=self._actual_pose.wz,
        )

    def _make_sample(self, time_s: float, ideal: Pose2D, actual: Pose2D) -> TestSample:
        error_x = actual.x - ideal.x
        error_y = actual.y - ideal.y
        error_pos = math.hypot(error_x, error_y)
        error_yaw = normalize_angle(actual.yaw - ideal.yaw)
        return TestSample(
            time_s=time_s,
            ideal_x=ideal.x,
            ideal_y=ideal.y,
            ideal_yaw=ideal.yaw,
            ideal_vx=ideal.vx,
            ideal_vy=ideal.vy,
            ideal_wz=ideal.wz,
            actual_x=actual.x,
            actual_y=actual.y,
            actual_yaw=actual.yaw,
            actual_vx=actual.vx,
            actual_vy=actual.vy,
            actual_wz=actual.wz,
            error_x=error_x,
            error_y=error_y,
            error_pos=error_pos,
            error_yaw=error_yaw,
        )

    def _summarize(self, samples: list[TestSample]) -> dict[str, float]:
        if not samples:
            return {
                'sample_count': 0,
                'rmse_position_m': 0.0,
                'rmse_x_m': 0.0,
                'rmse_y_m': 0.0,
                'rmse_yaw_rad': 0.0,
                'max_position_error_m': 0.0,
            }

        count = float(len(samples))
        rmse_x = math.sqrt(sum(sample.error_x ** 2 for sample in samples) / count)
        rmse_y = math.sqrt(sum(sample.error_y ** 2 for sample in samples) / count)
        rmse_pos = math.sqrt(sum(sample.error_pos ** 2 for sample in samples) / count)
        rmse_yaw = math.sqrt(sum(sample.error_yaw ** 2 for sample in samples) / count)
        max_pos = max(sample.error_pos for sample in samples)
        return {
            'sample_count': len(samples),
            'rmse_position_m': rmse_pos,
            'rmse_x_m': rmse_x,
            'rmse_y_m': rmse_y,
            'rmse_yaw_rad': rmse_yaw,
            'max_position_error_m': max_pos,
        }

    def _write_outputs(
        self,
        samples: list[TestSample],
        output_dir: Path,
        summary: dict[str, float],
        segments: list[MotionSegment],
    ) -> None:
        self._write_csv(samples, output_dir / 'trajectory_comparison.csv')
        self._write_summary(summary, segments, output_dir / 'summary.json')
        self._plot_samples(samples, output_dir)

    def _write_csv(self, samples: list[TestSample], path: Path) -> None:
        with path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(asdict(samples[0]).keys()))
            writer.writeheader()
            for sample in samples:
                writer.writerow(asdict(sample))

    def _write_summary(
        self,
        summary: dict[str, float],
        segments: list[MotionSegment],
        path: Path,
    ) -> None:
        payload = {
            'summary': summary,
            'segments': [asdict(segment) for segment in segments],
        }
        with path.open('w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2)

    def _plot_samples(self, samples: list[TestSample], output_dir: Path) -> None:
        if not samples:
            return

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print('matplotlib is not installed; skipping plot generation.')
            return

        times = [sample.time_s for sample in samples]
        ideal_x = [sample.ideal_x for sample in samples]
        ideal_y = [sample.ideal_y for sample in samples]
        actual_x = [sample.actual_x for sample in samples]
        actual_y = [sample.actual_y for sample in samples]
        ideal_yaw = [sample.ideal_yaw for sample in samples]
        actual_yaw = [sample.actual_yaw for sample in samples]
        error_pos = [sample.error_pos for sample in samples]

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(ideal_x, ideal_y, label='ideal', linewidth=2.0)
        ax.plot(actual_x, actual_y, label='localization', linewidth=2.0)
        ax.set_title('Localization Path vs Ideal Path')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_dir / 'xy_path.png', dpi=150)
        plt.close(fig)

        fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        axes[0].plot(times, ideal_x, label='ideal x')
        axes[0].plot(times, actual_x, label='actual x')
        axes[0].plot(times, ideal_y, label='ideal y', linestyle='--')
        axes[0].plot(times, actual_y, label='actual y', linestyle='--')
        axes[0].set_ylabel('position (m)')
        axes[0].grid(True)
        axes[0].legend()

        axes[1].plot(times, ideal_yaw, label='ideal yaw')
        axes[1].plot(times, actual_yaw, label='actual yaw')
        axes[1].set_ylabel('yaw (rad)')
        axes[1].grid(True)
        axes[1].legend()

        axes[2].plot(times, error_pos, label='position error')
        axes[2].set_ylabel('error (m)')
        axes[2].set_xlabel('time (s)')
        axes[2].grid(True)
        axes[2].legend()

        fig.tight_layout()
        fig.savefig(output_dir / 'coordinates_vs_time.png', dpi=150)
        plt.close(fig)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalizationTestRunner()
    try:
        node.run_interactive()
    finally:
        node._publish_cmd(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
