"""
ui_server.py — LitterBot Command Center WebSocket Bridge
========================================================

Overview
--------
This node runs a FastAPI web server alongside the ROS2 graph. It serves the
command center frontend (HTML/JS) and bridges ROS2 topics to the browser over
a WebSocket connection.

Responsibilities:
  - Serve the frontend HTML at GET /
  - Accept WebSocket connections at ws://host:<port>/ws
  - Subscribe to /cmd_vel, /stm32_tx, /encoder_raw, /odom and forward
    log entries to all connected browser clients in real time
  - Receive keyboard teleop commands from the browser and publish to /cmd_vel
  - Receive e-stop signals from the browser and publish to /emergency_stop

Threading Model
---------------
uvicorn (ASGI) runs on the main thread using asyncio.
rclpy.spin() runs in a background daemon thread.
ROS2 callbacks push log entries into the asyncio event loop using
loop.call_soon_threadsafe() so they can be safely broadcast to WebSocket clients.

WebSocket Message Protocol (JSON)
----------------------------------
Server -> Browser (log entry):
  {"type": "log", "channel": "stm32_tx|stm32_rx|node", "ts": 1234.56, "msg": "..."}

Browser -> Server (teleop):
  {"type": "cmd_vel", "vx": 0.3, "vy": 0.0, "wz": 0.0}

Browser -> Server (e-stop):
  {"type": "estop", "active": true|false}

Browser -> Server (deploy):
  {"type": "deploy"}

Browser -> Server (return to standby):
  {"type": "standby"}

Browser -> Server (dev mode):
  {"type": "dev_mode", "active": true|false}

Server -> Browser (state update):
  {"type": "state", "state": "STANDBY|DEPLOYING|ACTIVE", "dev_mode": true|false, "ts": 1234.56}

Topics
------
  Subscribed:
    /cmd_vel      (geometry_msgs/Twist)        — echoed to "node" log panel
    /stm32_tx     (std_msgs/String)            — echoed to "stm32_tx" log panel
    /stm32_rx     (std_msgs/String)            — echoed to "stm32_rx" log panel
    /stm32_state  (std_msgs/String)            — deploy state updates
    /encoder_raw  (std_msgs/Int32MultiArray)   — echoed to "stm32_rx" log panel
    /odom         (nav_msgs/Odometry)          — echoed to "node" log panel
    /odometry/ekf (nav_msgs/Odometry)          — localization stream for UI
    /localization/status (std_msgs/String)     — localization status text
  Published:
    /cmd_vel        (geometry_msgs/Twist)      — teleop commands from browser
    /emergency_stop (std_msgs/Bool)            — e-stop from browser
    /deploy_cmd     (std_msgs/String)          — deploy/standby from browser
    /dev_mode_cmd   (std_msgs/Bool)            — dev mode toggle from browser

Parameters
----------
  host         (str,  default '0.0.0.0') — bind address
  port         (int,  default 8765)       — HTTP + WebSocket port
  log_history  (int,  default 100)        — max entries kept per log channel
"""

import asyncio
import base64
import math
import json
import os
import re
import shutil
import subprocess
import threading
import time
import tempfile
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, Int32MultiArray, String

import uvicorn
from fastapi import HTTPException
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse


# --------------------------------------------------------------------------- #
# Log entry helper
# --------------------------------------------------------------------------- #

def _log_entry(channel: str, msg: str) -> dict:
    """Build a log entry dict ready for JSON serialisation."""
    return {"type": "log", "channel": channel, "ts": time.time(), "msg": msg}


def _export_bag_to_mp4(bag_dir: str, topic: str = '/camera/image/compressed') -> str:
    """Export a compressed camera topic from a ROS2 bag to <bag>/camera.mp4."""
    import cv2
    import numpy as np
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    bag_path = Path(os.path.expanduser(bag_dir)).resolve()
    output_path = bag_path / 'camera.mp4'

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    msg_type_name = topic_types.get(topic)
    if not msg_type_name:
        raise RuntimeError(f'Topic not found in bag: {topic}')

    msg_type = get_message(msg_type_name)
    frames = []
    stamps = []
    while reader.has_next():
        current_topic, data, timestamp = reader.read_next()
        if current_topic != topic:
            continue
        msg = deserialize_message(data, msg_type)
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        frames.append(image)
        stamps.append(timestamp)

    if not frames:
        raise RuntimeError(f'No decodable frames found on {topic}')

    if len(stamps) >= 2:
        duration_s = max((stamps[-1] - stamps[0]) / 1e9, 1e-3)
        fps = max(1.0, min(60.0, len(stamps) / duration_s))
    else:
        fps = 10.0

    height, width = frames[0].shape[:2]
    def _normalize_frames() -> list[np.ndarray]:
        normalized: list[np.ndarray] = []
        for frame in frames:
            if frame.shape[1] != width or frame.shape[0] != height:
                frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            normalized.append(frame)
        return normalized

    normalized_frames = _normalize_frames()

    def _write_with_gstreamer() -> bool:
        if shutil.which('gst-launch-1.0') is None:
            return False

        with tempfile.TemporaryDirectory(prefix='litterbot_mp4_') as temp_dir:
            temp_path = Path(temp_dir)
            for idx, frame in enumerate(normalized_frames):
                frame_path = temp_path / f'frame_{idx:06d}.jpg'
                if not cv2.imwrite(str(frame_path), frame):
                    raise RuntimeError(f'Failed to write temp frame {frame_path}')

            gst_cmd = [
                'gst-launch-1.0',
                '-q',
                'multifilesrc',
                f'location={temp_path / "frame_%06d.jpg"}',
                'index=0',
                f'caps=image/jpeg,framerate={int(round(max(1.0, fps) * 1000))}/1000',
                '!',
                'jpegdec',
                '!',
                'videoconvert',
                '!',
                'x264enc',
                'tune=zerolatency',
                'speed-preset=ultrafast',
                'key-int-max=30',
                '!',
                'h264parse',
                '!',
                'mp4mux',
                'faststart=true',
                '!',
                'filesink',
                f'location={output_path}',
            ]
            subprocess.run(gst_cmd, check=True)
        return output_path.exists() and output_path.stat().st_size > 0

    try:
        if not _write_with_gstreamer():
            raise RuntimeError('GStreamer export unavailable')
    except Exception:
        writer = cv2.VideoWriter(
            str(output_path),
            cv2.VideoWriter_fourcc(*'mp4v'),
            fps,
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f'Failed to open video writer for {output_path}')
        for frame in normalized_frames:
            writer.write(frame)
        writer.release()
    return str(output_path)


def _load_recording_detail(bag_dir: str) -> dict[str, Any]:
    """Load LiDAR and odom playback data from a ROS2 bag."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    bag_path = Path(os.path.expanduser(bag_dir)).resolve()
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    scan_type = get_message(topic_types['/scan']) if '/scan' in topic_types else None
    odom_type = get_message(topic_types['/odom']) if '/odom' in topic_types else None

    first_ts: int | None = None
    last_ts: int | None = None
    last_scan_emit = -1.0
    last_odom_emit = -1.0
    scans: list[dict[str, Any]] = []
    odom: list[dict[str, Any]] = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if first_ts is None:
            first_ts = timestamp
        last_ts = timestamp
        rel_t = (timestamp - first_ts) / 1e9 if first_ts is not None else 0.0

        if topic == '/scan' and scan_type is not None:
            if rel_t - last_scan_emit < 0.35:
                continue
            msg = deserialize_message(data, scan_type)
            sample_count = len(msg.ranges)
            step = max(1, sample_count // 120) if sample_count else 1
            plot_points: list[list[float]] = []
            angle = msg.angle_min
            for idx, rng in enumerate(msg.ranges):
                if idx % step != 0:
                    angle += msg.angle_increment
                    continue
                if msg.range_min <= rng <= msg.range_max:
                    plot_points.append([angle, float(rng)])
                angle += msg.angle_increment
            scans.append({
                't': rel_t,
                'points': sample_count,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'plot_points': plot_points,
            })
            last_scan_emit = rel_t

        elif topic == '/odom' and odom_type is not None:
            if rel_t - last_odom_emit < 0.10:
                continue
            msg = deserialize_message(data, odom_type)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            odom.append({
                't': rel_t,
                'x': float(p.x),
                'y': float(p.y),
                'yaw_deg': math.degrees(math.atan2(siny_cosp, cosy_cosp)),
                'vx': float(msg.twist.twist.linear.x),
                'vy': float(msg.twist.twist.linear.y),
                'wz': float(msg.twist.twist.angular.z),
            })
            last_odom_emit = rel_t

    duration = max(0.0, ((last_ts - first_ts) / 1e9) if first_ts is not None and last_ts is not None else 0.0)
    return {
        'duration_sec': duration,
        'scan_frames': scans,
        'odom_frames': odom,
        'has_scan': bool(scans),
        'has_odom': bool(odom),
    }


# --------------------------------------------------------------------------- #
# ROS2 Node
# --------------------------------------------------------------------------- #

class UIServer(Node):
    """
    ROS2 node that bridges the graph to a browser via WebSocket.

    Subscribes to relevant topics and forwards formatted log entries to all
    connected WebSocket clients. Publishes /cmd_vel and /emergency_stop in
    response to browser messages.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__('ui_server')

        # asyncio event loop — used to safely push messages from ROS2 callbacks
        self._loop = loop

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('host',        '0.0.0.0')
        self.declare_parameter('port',        8765)
        self.declare_parameter('log_history', 25)
        self.declare_parameter('recording_dir', str(Path.home() / 'litterbot_bags'))

        self._host        = self.get_parameter('host').value
        self._port        = self.get_parameter('port').value
        self._max_history = self.get_parameter('log_history').value
        self._recording_dir = str(self.get_parameter('recording_dir').value)

        # ------------------------------------------------------------------ #
        # WebSocket client registry and log history
        # Accessed from the asyncio thread only — no lock needed.
        # ------------------------------------------------------------------ #
        self._ws_clients: set[WebSocket] = set()
        self.history: dict[str, list[dict]] = {
            'stm32_tx': [],
            'stm32_rx': [],
            'node':     [],
        }

        # ------------------------------------------------------------------ #
        # Deploy/standby state (mirrors what stm32_bridge reports)
        # ------------------------------------------------------------------ #
        self._stm32_state: str = 'STANDBY'
        self._dev_mode: bool = False
        self._camera_connected: bool = False
        self._lidar_connected: bool = False
        self._recording: bool = False
        self._recording_path: str = ''
        self._record_proc: subprocess.Popen | None = None
        self._playback: bool = False
        self._playback_path: str = ''
        self._play_proc: subprocess.Popen | None = None
        self._exporting_video: bool = False
        self._exporting_path: str = ''
        self._last_exported_video: str = ''
        self._last_camera_frame_ts: float = 0.0
        self._last_camera_broadcast_ts: float = 0.0
        self._last_scan_broadcast_ts: float = 0.0
        self._last_encoder_log_ts: float = 0.0
        self._last_odom_log_ts: float = 0.0
        self._last_odom: dict[str, float] = {
            'x': 0.0,
            'y': 0.0,
            'yaw_deg': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'wz': 0.0,
            'ts': 0.0,
        }
        self._last_localization: dict[str, float | str] = {
            'x': 0.0,
            'y': 0.0,
            'yaw_deg': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'wz': 0.0,
            'source': '/odometry/ekf',
            'ts': 0.0,
        }
        self._last_localization_status: str = 'Localization idle'
        self.create_timer(1.0, self._poll_background_processes)

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._cmd_vel_pub  = self.create_publisher(Twist,  '/cmd_vel', 10)
        self._estop_pub    = self.create_publisher(Bool,   '/emergency_stop', 10)
        self._pause_pub    = self.create_publisher(Bool,   '/pause_cmd', 10)
        self._scoop_pub    = self.create_publisher(String, '/scoop_cmd', 10)
        self._cal_cmd_pub  = self.create_publisher(String, '/cal_cmd', 10)
        self._stepper_pub  = self.create_publisher(String, '/stepper_cmd', 10)
        self._servo_pub    = self.create_publisher(String, '/servo_cmd', 10)
        self._deploy_pub   = self.create_publisher(String, '/deploy_cmd', 10)
        self._dev_mode_pub = self.create_publisher(Bool,   '/dev_mode_cmd', 10)

        # ------------------------------------------------------------------ #
        # Subscribers — forward each message to the browser log
        # ------------------------------------------------------------------ #
        self.create_subscription(String,          '/stm32_tx',    self._on_stm32_tx,    10)
        self.create_subscription(String,          '/stm32_rx',    self._on_stm32_rx,    10)
        self.create_subscription(String,          '/stm32_state', self._on_stm32_state, 10)
        self.create_subscription(Int32MultiArray, '/encoder_raw', self._on_encoder_raw, 10)
        self.create_subscription(Twist,           '/cmd_vel',     self._on_cmd_vel,     10)
        self.create_subscription(Odometry,        '/odom',        self._on_odom,        10)
        self.create_subscription(Odometry,        '/odometry/ekf', self._on_localization_odom, 10)
        self.create_subscription(String,          '/localization/status', self._on_localization_status, 10)
        self.create_subscription(CompressedImage, '/camera/image/compressed', self._on_camera_frame, 2)
        self.create_subscription(Bool,            '/camera/status', self._on_camera_status, 2)
        self.create_subscription(String,          '/camera/info', self._on_camera_info, 10)
        self.create_subscription(LaserScan,       '/scan', self._on_scan, 5)

        self.get_logger().info(
            f'ui_server ready — http://{self._host}:{self._port}/'
        )

    # ---------------------------------------------------------------------- #
    # ROS2 topic callbacks
    # All run in the rclpy thread — use call_soon_threadsafe to reach asyncio.
    # ---------------------------------------------------------------------- #

    def _on_stm32_tx(self, msg: String) -> None:
        """Forward a /stm32_tx message to the Jetson->STM32 log panel."""
        entry = _log_entry('stm32_tx', msg.data)
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_encoder_raw(self, msg: Int32MultiArray) -> None:
        """Forward a /encoder_raw message to the STM32->Jetson log panel."""
        now = time.time()
        if (now - self._last_encoder_log_ts) < 0.25:
            return
        self._last_encoder_log_ts = now
        vals = ' '.join(str(v) for v in msg.data)
        entry = _log_entry('stm32_rx', f'ENC {vals}')
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Forward a /cmd_vel message to the Node<->Node log panel."""
        text = (
            f'/cmd_vel  vx={msg.linear.x:.2f}  '
            f'vy={msg.linear.y:.2f}  '
            f'wz={msg.angular.z:.2f}'
        )
        entry = _log_entry('node', text)
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_odom(self, msg: Odometry) -> None:
        """Forward a /odom message to the Node<->Node log panel."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._last_odom = {
            'x': float(p.x),
            'y': float(p.y),
            'yaw_deg': math.degrees(yaw),
            'vx': float(msg.twist.twist.linear.x),
            'vy': float(msg.twist.twist.linear.y),
            'wz': float(msg.twist.twist.angular.z),
            'ts': time.time(),
        }
        odom_msg = {'type': 'odom', **self._last_odom}
        self._loop.call_soon_threadsafe(self._broadcast_dict_sync, odom_msg)
        now = time.time()
        if (now - self._last_odom_log_ts) < 0.25:
            return
        self._last_odom_log_ts = now
        text = (
            f'/odom  x={p.x:.3f}  y={p.y:.3f}  '
            f'yaw={self._last_odom["yaw_deg"]:.1f}'
        )
        entry = _log_entry('node', text)
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_localization_odom(self, msg: Odometry) -> None:
        """Forward the filtered localization odometry stream to the browser."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._last_localization = {
            'x': float(p.x),
            'y': float(p.y),
            'yaw_deg': math.degrees(yaw),
            'vx': float(msg.twist.twist.linear.x),
            'vy': float(msg.twist.twist.linear.y),
            'wz': float(msg.twist.twist.angular.z),
            'source': msg.header.frame_id or '/odometry/ekf',
            'ts': time.time(),
        }
        localization_msg = {'type': 'localization', **self._last_localization}
        self._loop.call_soon_threadsafe(self._broadcast_dict_sync, localization_msg)

    def _on_localization_status(self, msg: String) -> None:
        self._last_localization_status = msg.data
        self._loop.call_soon_threadsafe(
            self._broadcast_dict_sync,
            {'type': 'localization_status', 'status': msg.data, 'ts': time.time()},
        )

    def _on_stm32_rx(self, msg: String) -> None:
        """Forward raw STM32 UART responses to the STM32->Jetson log panel."""
        entry = _log_entry('stm32_rx', msg.data)
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_stm32_state(self, msg: String) -> None:
        """Handle STM32 state updates — broadcast to all browser clients."""
        # Parse "STATE ACTIVE DEV" etc.
        parts = msg.data.split()
        if len(parts) >= 2:
            self._stm32_state = parts[1]
            self._dev_mode = 'DEV' in parts
        # Send as a state message (not a log) so the UI can update its overlay
        state_msg = {
            "type": "state",
            "state": self._stm32_state,
            "dev_mode": self._dev_mode,
            "ts": time.time(),
        }
        self._loop.call_soon_threadsafe(self._broadcast_dict_sync, state_msg)
        # Also log it
        entry = _log_entry('node', f'STM32 state: {msg.data}')
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_camera_status(self, msg: Bool) -> None:
        if self._camera_connected != bool(msg.data):
            self._camera_connected = bool(msg.data)
            self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)

    def _on_camera_info(self, msg: String) -> None:
        entry = _log_entry('node', f'CAMERA {msg.data}')
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_camera_frame(self, msg: CompressedImage) -> None:
        was_connected = self._camera_connected
        self._camera_connected = True
        now = time.time()
        self._last_camera_frame_ts = now
        if (now - self._last_camera_broadcast_ts) < 0.5:
            return
        self._last_camera_broadcast_ts = now
        payload = {
            'type': 'camera_frame',
            'ts': now,
            'format': msg.format,
            'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
        }
        self._loop.call_soon_threadsafe(self._broadcast_dict_sync, payload)
        # Only broadcast sensor_state on connection transition — doing it per
        # frame was calling _list_bags() (filesystem scan) at camera/lidar rate
        # and was the main source of UI lag.
        if not was_connected:
            self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)

    def _on_scan(self, msg: LaserScan) -> None:
        was_connected = self._lidar_connected
        self._lidar_connected = True
        now = time.time()
        if (now - self._last_scan_broadcast_ts) < 0.5:
            return
        self._last_scan_broadcast_ts = now
        sample_count = len(msg.ranges)
        step = max(1, sample_count // 90) if sample_count else 1
        plot_points: list[list[float]] = []
        angle = msg.angle_min
        for idx, rng in enumerate(msg.ranges):
            if idx % step != 0:
                angle += msg.angle_increment
                continue
            if msg.range_min <= rng <= msg.range_max:
                plot_points.append([angle, float(rng)])
            angle += msg.angle_increment
        payload = {
            'type': 'lidar_scan',
            'ts': now,
            'points': sample_count,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'plot_points': plot_points,
        }
        self._loop.call_soon_threadsafe(self._broadcast_dict_sync, payload)
        if not was_connected:
            self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)

    # ---------------------------------------------------------------------- #
    # Asyncio-thread helpers (called via call_soon_threadsafe)
    # ---------------------------------------------------------------------- #

    def _broadcast_sync(self, entry: dict) -> None:
        """
        Schedule a broadcast coroutine on the asyncio event loop.
        Called from call_soon_threadsafe — must not be a coroutine itself.
        """
        self._loop.create_task(self._broadcast(entry))

    def _broadcast_dict_sync(self, data: dict) -> None:
        """Broadcast an arbitrary dict to all WebSocket clients."""
        self._loop.create_task(self._broadcast_raw(data))

    def _broadcast_sensor_state_sync(self) -> None:
        self._loop.create_task(self._broadcast_raw({
            'type': 'sensor_status',
            'camera_connected': self._camera_connected,
            'lidar_connected': self._lidar_connected,
            'recording': self._recording,
            'recording_path': self._recording_path,
            'playback': self._playback,
            'playback_path': self._playback_path,
            'exporting_video': self._exporting_video,
            'exporting_path': self._exporting_path,
            'last_exported_video': self._last_exported_video,
            'available_bags': self._list_bags(),
            'ts': time.time(),
        }))

    def _list_bags(self) -> list[str]:
        out_dir = Path(os.path.expanduser(self._recording_dir))
        if not out_dir.exists():
            return []

        return [
            str(path)
            for path in sorted(
                out_dir.iterdir(),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            if path.is_dir() and (path / 'metadata.yaml').exists()
        ]

    def _poll_background_processes(self) -> None:
        if self._record_proc is not None and self._record_proc.poll() is not None:
            self._record_proc = None
            self._recording = False
            self.get_logger().info(f'Recording exited: {self._recording_path}')
            self._loop.call_soon_threadsafe(
                self._broadcast_sync,
                _log_entry('node', f'RECORDING EXIT {self._recording_path}')
            )
            self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)

        if self._play_proc is not None and self._play_proc.poll() is not None:
            last_path = self._playback_path
            self._play_proc = None
            self._playback = False
            self._playback_path = ''
            self.get_logger().info(f'Playback finished: {last_path}')
            self._loop.call_soon_threadsafe(
                self._broadcast_sync,
                _log_entry('node', f'PLAYBACK STOP {last_path}')
            )
            self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)

    def _start_video_export(self, bag_dir: str) -> None:
        if not bag_dir or self._exporting_video:
            return

        def worker() -> None:
            try:
                output = _export_bag_to_mp4(bag_dir)
                self._last_exported_video = output
                self.get_logger().info(f'MP4 export complete: {output}')
                self._loop.call_soon_threadsafe(
                    self._broadcast_sync,
                    _log_entry('node', f'MP4 READY {output}')
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'MP4 export failed for {bag_dir}: {exc}')
                self._loop.call_soon_threadsafe(
                    self._broadcast_sync,
                    _log_entry('node', f'MP4 ERR {bag_dir}: {exc}')
                )
            finally:
                self._exporting_video = False
                self._exporting_path = ''
                self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)

        self._exporting_video = True
        self._exporting_path = bag_dir
        self._loop.call_soon_threadsafe(
            self._broadcast_sync,
            _log_entry('node', f'MP4 EXPORT START {bag_dir}')
        )
        self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)
        threading.Thread(target=worker, daemon=True).start()

    async def _broadcast_raw(self, data: dict) -> None:
        """Send a raw JSON dict to all connected WebSocket clients."""
        dead: set[WebSocket] = set()
        payload = json.dumps(data)
        for ws in list(self._ws_clients):
            try:
                await ws.send_text(payload)
            except Exception:  # noqa: BLE001
                dead.add(ws)
        self._ws_clients -= dead

    async def _broadcast(self, entry: dict) -> None:
        """Send a log entry to all connected WebSocket clients."""
        channel = entry.get('channel', 'node')
        hist = self.history.setdefault(channel, [])
        hist.append(entry)
        if len(hist) > self._max_history:
            hist.pop(0)

        dead: set[WebSocket] = set()
        payload = json.dumps(entry)
        for ws in list(self._ws_clients):
            try:
                await ws.send_text(payload)
            except Exception:  # noqa: BLE001
                dead.add(ws)
        self._ws_clients -= dead

    # ---------------------------------------------------------------------- #
    # Browser -> ROS2 message handlers
    # ---------------------------------------------------------------------- #

    def handle_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """Publish a Twist from keyboard teleop data received over WebSocket."""
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = float(wz)
        self._cmd_vel_pub.publish(msg)

    def handle_estop(self, active: bool) -> None:
        """Publish an emergency stop command received from the browser."""
        msg = Bool()
        msg.data = bool(active)
        self._estop_pub.publish(msg)
        level = 'ACTIVATED' if active else 'cleared'
        self.get_logger().warn(f'Emergency stop {level} from UI.')

    def handle_pause(self, active: bool) -> None:
        """Publish a pause/resume command to /pause_cmd."""
        msg = Bool()
        msg.data = bool(active)
        self._pause_pub.publish(msg)
        self.get_logger().info(f'{"Pause" if active else "Resume"} from UI.')

    def handle_scoop(self) -> None:
        """Publish a scoop trigger to /scoop_cmd."""
        msg = String()
        msg.data = 'SCOOP'
        self._scoop_pub.publish(msg)
        self.get_logger().info('Scoop command sent.')

    def handle_cal(self, cmd: str) -> None:
        """Publish a calibration command to /cal_cmd."""
        msg = String()
        msg.data = cmd
        self._cal_cmd_pub.publish(msg)
        self.get_logger().info(f'Cal command sent: {cmd}')

    def handle_stepper_jog(self, direction: str, steps: int, delay_us: int) -> None:
        """
        Publish a stepper jog burst to /stepper_cmd (legacy burst form).

        Kept for any UI path that still sends a single one-shot burst.
        The hold-to-jog I/K keys now use handle_stepper_jog_start /
        handle_stepper_jog_stop, which map to the firmware's continuous
        blocking jog loop and avoid burst-to-burst gap artifacts.
        """
        direction = direction.upper()
        if direction not in ('CW', 'CCW'):
            self.get_logger().warn(f'Bad stepper jog direction: {direction!r}')
            return

        msg = String()
        msg.data = f'STEP {direction} {int(steps)} {int(delay_us)}'
        self._stepper_pub.publish(msg)

    def handle_stepper_jog_start(self, direction: str, delay_us: int) -> None:
        """Publish a continuous jog-start command to /stepper_cmd."""
        direction = direction.upper()
        if direction not in ('CW', 'CCW'):
            self.get_logger().warn(f'Bad stepper jog direction: {direction!r}')
            return
        if delay_us < 5:
            delay_us = 5
        msg = String()
        msg.data = f'JOG_START {direction} {int(delay_us)}'
        self._stepper_pub.publish(msg)

    def handle_stepper_jog_stop(self) -> None:
        """Publish a continuous jog-stop command to /stepper_cmd."""
        msg = String()
        msg.data = 'JOG_STOP'
        self._stepper_pub.publish(msg)

    def handle_servo_jog_start(self, sign: str) -> None:
        """
        Publish a continuous servo jog-start command to /servo_cmd.

        Sent once on keydown. Firmware launches a smooth glide toward the
        corresponding endpoint (0 or 4095) and the servo's internal
        controller runs the motion — no further messages until keyup.
        """
        if sign not in ('+', '-'):
            self.get_logger().warn(f'Bad servo sign: {sign!r}')
            return
        msg = String()
        msg.data = f'JOG_START {sign}'
        self._servo_pub.publish(msg)
        self.get_logger().info(f'Servo jog start: {msg.data}')

    def handle_servo_jog_stop(self) -> None:
        """Publish a continuous servo jog-stop command to /servo_cmd."""
        msg = String()
        msg.data = 'JOG_STOP'
        self._servo_pub.publish(msg)

    def handle_autonomy(self, active: bool) -> None:
        """Log autonomy mode change. Future: publish to /autonomy_mode."""
        mode = 'AUTONOMOUS' if active else 'TELEOP'
        self.get_logger().info(f'Mode changed to {mode} from UI.')
        entry = _log_entry('node', f'MODE → {mode}')
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def handle_deploy(self) -> None:
        """Publish a deploy command to /deploy_cmd."""
        msg = String()
        msg.data = 'deploy'
        self._deploy_pub.publish(msg)
        self.get_logger().info('Deploy command sent from UI.')

    def handle_return_to_standby(self) -> None:
        """Publish a standby command to /deploy_cmd."""
        msg = String()
        msg.data = 'standby'
        self._deploy_pub.publish(msg)
        self.get_logger().info('Return-to-standby sent from UI.')

    def handle_dev_mode(self, active: bool) -> None:
        """Publish a dev mode toggle to /dev_mode_cmd."""
        msg = Bool()
        msg.data = bool(active)
        self._dev_mode_pub.publish(msg)
        self.get_logger().info(f'Dev mode {"ON" if active else "OFF"} from UI.')

    def _sanitize_run_name(self, name: str) -> str:
        cleaned = re.sub(r'[^A-Za-z0-9_-]+', '_', name.strip())
        cleaned = re.sub(r'_+', '_', cleaned).strip('_')
        return cleaned[:48]

    def handle_recording(self, active: bool, run_name: str = '') -> None:
        """Start or stop rosbag recording for camera/LiDAR/robot topics."""
        if active:
            if self._record_proc is not None:
                return

            out_dir = Path(os.path.expanduser(self._recording_dir))
            out_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime('%Y%m%d_%H%M%S')
            label = self._sanitize_run_name(run_name)
            bag_dir = out_dir / (f'{label}_{stamp}' if label else f'litterbot_run_{stamp}')
            cmd = [
                'ros2', 'bag', 'record',
                '-o', str(bag_dir),
                '/camera/image/compressed',
                '/scan',
                '/odom',
                '/cmd_vel',
                '/stm32_tx',
                '/stm32_rx',
                '/stm32_state',
            ]
            self._record_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self._recording = True
            self._recording_path = str(bag_dir)
            self.get_logger().info(f'Recording started: {bag_dir}')
            self._loop.call_soon_threadsafe(
                self._broadcast_sync,
                _log_entry('node', f'RECORDING START {bag_dir}')
            )
            self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)
            return

        if self._record_proc is None:
            return

        self._record_proc.terminate()
        try:
            self._record_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._record_proc.kill()
            self._record_proc.wait(timeout=5.0)
        self._record_proc = None
        self._recording = False
        self.get_logger().info(f'Recording stopped: {self._recording_path}')
        self._loop.call_soon_threadsafe(
            self._broadcast_sync,
            _log_entry('node', f'RECORDING STOP {self._recording_path}')
        )
        self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)
        self._start_video_export(self._recording_path)

    def handle_playback(self, active: bool, bag_path: str = '') -> None:
        """Start or stop rosbag playback for a recorded bag."""
        if active:
            if self._play_proc is not None:
                return
            if self._record_proc is not None:
                self.get_logger().warn('Playback requested while recording is active.')
                self._loop.call_soon_threadsafe(
                    self._broadcast_sync,
                    _log_entry('node', 'PLAYBACK ERR recording still active')
                )
                return

            resolved_path = bag_path.strip()
            if not resolved_path:
                bags = self._list_bags()
                if not bags:
                    self.get_logger().warn('Playback requested but no bag directories were found.')
                    self._loop.call_soon_threadsafe(
                        self._broadcast_sync,
                        _log_entry('node', 'PLAYBACK ERR no bag directories found')
                    )
                    return
                resolved_path = bags[0]

            bag_dir = Path(os.path.expanduser(resolved_path))
            if not bag_dir.exists() or not bag_dir.is_dir():
                self.get_logger().warn(f'Playback requested for missing bag: {bag_dir}')
                self._loop.call_soon_threadsafe(
                    self._broadcast_sync,
                    _log_entry('node', f'PLAYBACK ERR missing bag {bag_dir}')
                )
                return

            self._play_proc = subprocess.Popen(
                [
                    'ros2', 'bag', 'play', str(bag_dir),
                    '--remap',
                    '/cmd_vel:=/playback/cmd_vel',
                    '/stm32_tx:=/playback/stm32_tx',
                    '/stm32_rx:=/playback/stm32_rx',
                    '/stm32_state:=/playback/stm32_state',
                    '/camera/image/compressed:=/playback/camera/image/compressed',
                    '/scan:=/playback/scan',
                    '/odom:=/playback/odom',
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self._playback = True
            self._playback_path = str(bag_dir)
            self.get_logger().info(f'Playback started: {bag_dir}')
            self._loop.call_soon_threadsafe(
                self._broadcast_sync,
                _log_entry('node', f'PLAYBACK START {bag_dir}')
            )
            self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)
            return

        if self._play_proc is None:
            return

        last_path = self._playback_path
        self._play_proc.terminate()
        try:
            self._play_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._play_proc.kill()
            self._play_proc.wait(timeout=5.0)
        self._play_proc = None
        self._playback = False
        self._playback_path = ''
        self.get_logger().info(f'Playback stopped: {last_path}')
        self._loop.call_soon_threadsafe(
            self._broadcast_sync,
            _log_entry('node', f'PLAYBACK STOP {last_path}')
        )
        self._loop.call_soon_threadsafe(self._broadcast_sensor_state_sync)

    def destroy_node(self) -> bool:
        if self._record_proc is not None:
            self.handle_recording(False)
        if self._play_proc is not None:
            self.handle_playback(False)
        return super().destroy_node()


# --------------------------------------------------------------------------- #
# FastAPI application factory
# --------------------------------------------------------------------------- #

def build_app(node: UIServer) -> FastAPI:
    """
    Construct and return the FastAPI application.

    Parameters
    ----------
    node : UIServer
        The live ROS2 node — used by endpoint handlers to publish/subscribe.
    """
    app = FastAPI(title='LitterBot Command Center')

    # The live UI may run from either the source tree or an installed package.
    # Resolve both so clean Jetson rebuilds still serve the latest frontend.
    def _load_live_ui_html() -> str:
        resolved = Path(__file__).resolve()
        candidates = [resolved.parent / 'ui' / 'index.html']
        for parent in resolved.parents:
            candidates.append(parent / 'src' / 'litterbot_bridge' / 'litterbot_bridge' / 'ui' / 'index.html')

        seen: set[str] = set()
        unique_candidates: list[Path] = []
        for candidate in candidates:
            key = str(candidate)
            if key not in seen:
                seen.add(key)
                unique_candidates.append(candidate)

        for candidate in unique_candidates:
            if candidate.exists():
                return candidate.read_text(encoding='utf-8')
        raise FileNotFoundError(
            'Unable to locate live UI index.html. Checked: '
            + ', '.join(str(path) for path in unique_candidates)
        )

    def _recordings_page_html() -> str:
        return """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>LitterBot Recordings</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0d0d0d;
    color: #e0e0e0;
    font-family: 'Segoe UI', system-ui, sans-serif;
    min-height: 100vh;
  }
  #topbar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
    padding: 14px 18px;
    background: #161616;
    border-bottom: 1px solid #2a2a2a;
  }
  #title { font-size: 16px; font-weight: 700; letter-spacing: 0.05em; }
  .link-btn, button {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    padding: 8px 12px;
    border: 1px solid #444;
    border-radius: 6px;
    background: #171717;
    color: #ddd;
    text-decoration: none;
    font-size: 12px;
    font-weight: 700;
    cursor: pointer;
  }
  #page {
    padding: 18px;
    display: grid;
    grid-template-columns: 320px minmax(0, 1fr);
    gap: 18px;
  }
  #sidebar, #viewer {
    border: 1px solid #2a2a2a;
    border-radius: 12px;
    background: #111;
  }
  #sidebar { padding: 14px; display: flex; flex-direction: column; gap: 12px; }
  #viewer { padding: 14px; display: flex; flex-direction: column; gap: 14px; }
  #status { color: #9aa0a6; font-size: 12px; }
  #run-list { display: flex; flex-direction: column; gap: 8px; max-height: 75vh; overflow: auto; }
  .run-btn {
    width: 100%;
    text-align: left;
    justify-content: flex-start;
    flex-direction: column;
    align-items: flex-start;
    gap: 4px;
    background: #0f141a;
    border-color: #243447;
  }
  .run-btn.active { border-color: #4fc3f7; color: #fff; }
  .run-name { font-size: 13px; font-weight: 700; }
  .run-meta { font-size: 11px; color: #9aa0a6; }
  .muted { color: #7a7f85; font-size: 12px; }
  #viewer-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 10px;
    flex-wrap: wrap;
  }
  #run-title { font-size: 15px; font-weight: 700; }
  #run-meta {
    font-family: 'Cascadia Code', 'Consolas', monospace;
    font-size: 11px;
    color: #9aa0a6;
    white-space: pre-wrap;
  }
  #player-grid {
    display: grid;
    grid-template-columns: minmax(0, 1.25fr) minmax(320px, 0.9fr);
    gap: 14px;
    align-items: start;
  }
  video {
    width: 100%;
    max-height: 420px;
    border: 1px solid #243447;
    border-radius: 8px;
    background: #000;
  }
  #lidar-card, #odom-card {
    border: 1px solid #243447;
    border-radius: 10px;
    background: #0b1015;
    padding: 10px;
  }
  #lidar-card { display: flex; flex-direction: column; gap: 8px; }
  #lidar-title, #odom-title {
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
  }
  #lidar-title { color: #66bb6a; }
  #odom-title { color: #4fc3f7; }
  #lidar-canvas {
    width: 100%;
    height: 240px;
    border: 1px solid #233423;
    border-radius: 6px;
    background:
      radial-gradient(circle at center, rgba(102,187,106,0.08) 0, rgba(102,187,106,0.04) 35%, rgba(0,0,0,0) 36%),
      linear-gradient(180deg, #091109, #050805);
    display: block;
  }
  #lidar-info {
    color: #bdbdbd;
    font-family: 'Cascadia Code', 'Consolas', monospace;
    white-space: pre-wrap;
    font-size: 11px;
  }
  #odom-grid {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 8px;
  }
  .odom-cell {
    background: #0b0f14;
    border: 1px solid #1d2a38;
    border-radius: 6px;
    padding: 8px;
  }
  .odom-label {
    font-size: 10px;
    color: #6d859d;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    margin-bottom: 4px;
  }
  .odom-value {
    font-family: 'Cascadia Code', 'Consolas', monospace;
    font-size: 15px;
    color: #d7ecff;
  }
  #timeline-wrap {
    display: flex;
    align-items: center;
    gap: 10px;
  }
  #timeline {
    flex: 1;
  }
  #time-label {
    min-width: 84px;
    text-align: right;
    font-family: 'Cascadia Code', 'Consolas', monospace;
    font-size: 11px;
    color: #9aa0a6;
  }
  @media (max-width: 1100px) {
    #page { grid-template-columns: 1fr; }
    #player-grid { grid-template-columns: 1fr; }
  }
</style>
</head>
<body>
  <div id="topbar">
    <div id="title">LitterBot Recordings</div>
    <a class="link-btn" href="/">Back To Live UI</a>
  </div>
  <div id="page">
    <div id="sidebar">
      <div style="display:flex; justify-content:space-between; align-items:center; gap:10px;">
        <div style="font-weight:700;">Test Runs</div>
        <button onclick="loadRuns()">Refresh</button>
      </div>
      <div id="status">Loading recordings...</div>
      <div id="run-list"></div>
    </div>
    <div id="viewer">
      <div id="viewer-header">
        <div>
          <div id="run-title">Select a test run</div>
          <div id="run-meta" class="muted">No run selected yet.</div>
        </div>
        <div id="run-links" class="muted"></div>
      </div>
      <div id="timeline-wrap">
        <input id="timeline" type="range" min="0" max="1000" value="0">
        <div id="time-label">0.00 s</div>
      </div>
      <div id="player-grid">
        <div>
          <video id="video" controls preload="metadata"></video>
        </div>
        <div style="display:flex; flex-direction:column; gap:14px;">
          <div id="lidar-card">
            <div id="lidar-title">LiDAR</div>
            <canvas id="lidar-canvas" width="480" height="240"></canvas>
            <div id="lidar-info" class="muted">No LiDAR data loaded.</div>
          </div>
          <div id="odom-card">
            <div id="odom-title">Odometry</div>
            <div id="odom-grid">
              <div class="odom-cell"><div class="odom-label">X</div><div class="odom-value" id="odom-x">0.000 m</div></div>
              <div class="odom-cell"><div class="odom-label">Y</div><div class="odom-value" id="odom-y">0.000 m</div></div>
              <div class="odom-cell"><div class="odom-label">Heading</div><div class="odom-value" id="odom-yaw">0.0 deg</div></div>
              <div class="odom-cell"><div class="odom-label">Vx</div><div class="odom-value" id="odom-vx">0.000 m/s</div></div>
              <div class="odom-cell"><div class="odom-label">Vy</div><div class="odom-value" id="odom-vy">0.000 m/s</div></div>
              <div class="odom-cell"><div class="odom-label">Wz</div><div class="odom-value" id="odom-wz">0.000 rad/s</div></div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
<script>
let runs = [];
let selectedRun = null;
let runDetail = null;
let syncingTimeline = false;

function fmtSec(v) { return `${(v || 0).toFixed(2)} s`; }

function renderLidarFrame(frame) {
  const canvas = document.getElementById('lidar-canvas');
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height, cx = w / 2, cy = h - 18;
  const maxRadius = Math.min(w * 0.46, h - 24);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#071007';
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = '#173117';
  ctx.lineWidth = 1;
  for (let i = 1; i <= 4; i++) {
    const r = (maxRadius * i) / 4;
    ctx.beginPath(); ctx.arc(cx, cy, r, Math.PI, 2 * Math.PI); ctx.stroke();
  }
  for (let deg = -90; deg <= 90; deg += 30) {
    const ang = (deg * Math.PI) / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.sin(ang) * maxRadius, cy - Math.cos(ang) * maxRadius);
    ctx.stroke();
  }
  ctx.strokeStyle = '#66bb6a';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx - maxRadius, cy); ctx.lineTo(cx + maxRadius, cy); ctx.stroke();
  if (!frame || !Array.isArray(frame.plot_points) || !frame.plot_points.length) {
    document.getElementById('lidar-info').textContent = 'No LiDAR data in this run.';
    return;
  }
  const denom = Math.max(0.1, frame.range_max || 12);
  ctx.fillStyle = '#8df58d';
  for (const pt of frame.plot_points) {
    const angle = pt[0], range = pt[1];
    const r = Math.min(1.0, range / denom) * maxRadius;
    const x = cx + Math.sin(angle) * r;
    const y = cy - Math.cos(angle) * r;
    ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
  }
  document.getElementById('lidar-info').textContent =
    `t=${fmtSec(frame.t)}\\nPoints: ${frame.points}\\nAngle: ${frame.angle_min.toFixed(2)} to ${frame.angle_max.toFixed(2)}\\nRange: ${frame.range_min.toFixed(2)} to ${frame.range_max.toFixed(2)}`;
}

function renderOdomFrame(frame) {
  const set = (id, value) => { const el = document.getElementById(id); if (el) el.textContent = value; };
  if (!frame) {
    set('odom-x', 'n/a');
    set('odom-y', 'n/a');
    set('odom-yaw', 'n/a');
    set('odom-vx', 'n/a');
    set('odom-vy', 'n/a');
    set('odom-wz', 'n/a');
    return;
  }
  set('odom-x', `${(frame.x || 0).toFixed(3)} m`);
  set('odom-y', `${(frame.y || 0).toFixed(3)} m`);
  set('odom-yaw', `${(frame.yaw_deg || 0).toFixed(1)} deg`);
  set('odom-vx', `${(frame.vx || 0).toFixed(3)} m/s`);
  set('odom-vy', `${(frame.vy || 0).toFixed(3)} m/s`);
  set('odom-wz', `${(frame.wz || 0).toFixed(3)} rad/s`);
}

function nearestFrame(frames, t) {
  if (!Array.isArray(frames) || !frames.length) return null;
  let best = frames[0];
  for (const frame of frames) {
    if (frame.t <= t) best = frame;
    else break;
  }
  return best;
}

function syncViewToTime(t) {
  if (!runDetail) return;
  document.getElementById('time-label').textContent = fmtSec(t);
  renderLidarFrame(nearestFrame(runDetail.scan_frames, t));
  renderOdomFrame(nearestFrame(runDetail.odom_frames, t));
}

async function loadRuns() {
  const status = document.getElementById('status');
  const list = document.getElementById('run-list');
  status.textContent = 'Loading recordings...';
  list.innerHTML = '';
  const resp = await fetch('/api/recordings');
  const data = await resp.json();
  runs = Array.isArray(data.recordings) ? data.recordings : [];
  status.textContent = `${runs.length} test run(s) found`;
  if (!runs.length) {
    list.innerHTML = '<div class="muted">No recordings found yet.</div>';
    return;
  }
  for (const item of runs) {
    const btn = document.createElement('button');
    btn.className = 'run-btn';
    btn.innerHTML = `<div class="run-name">${item.name}</div><div class="run-meta">${item.modified}</div>`;
    btn.onclick = () => selectRun(item.name);
    if (selectedRun && selectedRun.name === item.name) btn.classList.add('active');
    list.appendChild(btn);
  }
  if (!selectedRun) selectRun(runs[0].name);
}

async function selectRun(name) {
  selectedRun = runs.find((r) => r.name === name) || null;
  document.querySelectorAll('.run-btn').forEach((el) => {
    el.classList.toggle('active', el.querySelector('.run-name')?.textContent === name);
  });
  const resp = await fetch(`/api/recordings/${encodeURIComponent(name)}`);
  runDetail = await resp.json();
  document.getElementById('run-title').textContent = runDetail.name;
  document.getElementById('run-meta').textContent =
    `Path: ${runDetail.path}\\nUpdated: ${runDetail.modified}\\nDuration: ${fmtSec(runDetail.duration_sec)}`;
  document.getElementById('run-links').innerHTML =
    `${runDetail.mp4_url ? `<a class="link-btn" href="${runDetail.mp4_url}" target="_blank" rel="noopener">Open MP4</a>` : '<span class="muted">MP4 pending export</span>'}
     ${runDetail.metadata_url ? `<a class="link-btn" href="${runDetail.metadata_url}" target="_blank" rel="noopener">Metadata</a>` : ''}`;

  const video = document.getElementById('video');
  if (runDetail.mp4_url) {
    video.src = runDetail.mp4_url;
    video.style.display = 'block';
  } else {
    video.removeAttribute('src');
    video.load();
    video.style.display = 'none';
  }
  document.getElementById('timeline').value = 0;
  syncViewToTime(0);
}

const video = document.getElementById('video');
const timeline = document.getElementById('timeline');
video.addEventListener('timeupdate', () => {
  if (!runDetail) return;
  syncingTimeline = true;
  const duration = Math.max(runDetail.duration_sec || video.duration || 0.001, 0.001);
  timeline.value = Math.round((video.currentTime / duration) * 1000);
  syncViewToTime(video.currentTime);
  syncingTimeline = false;
});
timeline.addEventListener('input', () => {
  if (!runDetail) return;
  const duration = Math.max(runDetail.duration_sec || video.duration || 0.001, 0.001);
  const t = (Number(timeline.value) / 1000) * duration;
  syncViewToTime(t);
  if (!syncingTimeline && video.src) {
    video.currentTime = Math.min(t, duration);
  }
});

loadRuns();
</script>
</body>
</html>"""

    def _recording_items() -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        for bag in node._list_bags():
            bag_path = Path(bag)
            mp4_path = bag_path / 'camera.mp4'
            metadata_path = bag_path / 'metadata.yaml'
            items.append({
                'name': bag_path.name,
                'path': str(bag_path),
                'modified': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(bag_path.stat().st_mtime)),
                'mp4_url': f'/recordings/media/{bag_path.name}/camera.mp4' if mp4_path.exists() else '',
                'metadata_url': f'/recordings/media/{bag_path.name}/metadata.yaml' if metadata_path.exists() else '',
            })
        return items

    def _recording_item_by_name(bag_name: str) -> dict[str, Any] | None:
        for item in _recording_items():
            if item['name'] == bag_name:
                return item
        return None

    _NO_CACHE_HEADERS = {
        'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
        'Pragma': 'no-cache',
        'Expires': '0',
    }

    @app.get('/', response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        """Serve the command center frontend."""
        html = _load_live_ui_html()
        return HTMLResponse(content=html, headers=_NO_CACHE_HEADERS)

    @app.get('/recordings', response_class=HTMLResponse)
    async def recordings_page() -> HTMLResponse:
        """Serve the recordings browser page."""
        return HTMLResponse(content=_recordings_page_html(), headers=_NO_CACHE_HEADERS)

    @app.get('/api/recordings', response_class=JSONResponse)
    async def recordings_api() -> JSONResponse:
        """Return recording inventory for the recordings page."""
        return JSONResponse({'recordings': _recording_items()})

    @app.get('/api/recordings/{bag_name}', response_class=JSONResponse)
    async def recording_detail_api(bag_name: str) -> JSONResponse:
        """Return test-run detail, including LiDAR and odom playback data."""
        item = _recording_item_by_name(bag_name)
        if item is None:
            raise HTTPException(status_code=404, detail='Recording not found')
        detail = _load_recording_detail(item['path'])
        return JSONResponse({**item, **detail})

    @app.get('/recordings/media/{bag_name}/{filename}')
    async def recording_media(bag_name: str, filename: str) -> FileResponse:
        """Serve recording artifacts from the bag directory."""
        safe_filename = Path(filename).name
        target = Path(os.path.expanduser(node._recording_dir)) / bag_name / safe_filename
        if not target.exists() or not target.is_file():
            raise HTTPException(status_code=404, detail='Recording file not found')
        return FileResponse(target)

    @app.websocket('/ws')
    async def websocket_endpoint(ws: WebSocket) -> None:
        """
        Handle a WebSocket connection from the command center browser.

        On connect: register client and send full log history so the browser
        immediately shows recent entries without waiting for new messages.

        On message: parse JSON and dispatch to cmd_vel or estop handlers.

        On disconnect: deregister client cleanly.
        """
        await ws.accept()
        node._ws_clients.add(ws)
        node.get_logger().info(f'Browser connected — {len(node._ws_clients)} client(s)')

        # Send current deploy state so the browser knows immediately
        await ws.send_text(json.dumps({
            "type": "state",
            "state": node._stm32_state,
            "dev_mode": node._dev_mode,
            "ts": time.time(),
        }))
        await ws.send_text(json.dumps({
            "type": "sensor_status",
            "camera_connected": node._camera_connected,
            "lidar_connected": node._lidar_connected,
            "recording": node._recording,
            "recording_path": node._recording_path,
            "playback": node._playback,
            "playback_path": node._playback_path,
            "exporting_video": node._exporting_video,
            "exporting_path": node._exporting_path,
            "last_exported_video": node._last_exported_video,
            "available_bags": node._list_bags(),
            "ts": time.time(),
        }))
        await ws.send_text(json.dumps({
            "type": "odom",
            **node._last_odom,
        }))
        await ws.send_text(json.dumps({
            "type": "localization",
            **node._last_localization,
        }))
        await ws.send_text(json.dumps({
            "type": "localization_status",
            "status": node._last_localization_status,
            "ts": time.time(),
        }))

        # Keep refresh/reconnect cheap. The live UI is more important than
        # replaying old log spam into a browser that is already under load.

        try:
            while True:
                raw = await ws.receive_text()
                try:
                    data: dict[str, Any] = json.loads(raw)
                except json.JSONDecodeError:
                    node.get_logger().warn(f'Bad JSON from browser: {raw!r}')
                    continue

                msg_type = data.get('type')

                if msg_type == 'cmd_vel':
                    node.handle_cmd_vel(
                        data.get('vx', 0.0),
                        data.get('vy', 0.0),
                        data.get('wz', 0.0),
                    )

                elif msg_type == 'estop':
                    node.handle_estop(bool(data.get('active', True)))

                elif msg_type == 'pause':
                    node.handle_pause(bool(data.get('active', True)))

                elif msg_type == 'scoop':
                    node.handle_scoop()

                elif msg_type == 'cal_arm':
                    node.handle_cal('cal_arm')

                elif msg_type == 'cal_servo_start':
                    node.handle_cal('cal_servo_start')

                elif msg_type == 'cal_servo_stop':
                    node.handle_cal('cal_servo_stop')

                elif msg_type == 'cal_save_home':
                    node.handle_cal('save_pose_1')

                elif msg_type == 'cal_save_open':
                    node.handle_cal('save_pose_2')

                elif msg_type == 'cal_save_closed':
                    node.handle_cal('save_pose_3')

                elif msg_type == 'cal_save_dump':
                    node.handle_cal('save_pose_4')

                elif msg_type == 'cal_save_pose':
                    try:
                        pose_index = int(data.get('index', 1))
                    except (TypeError, ValueError):
                        pose_index = 1
                    pose_index = max(1, min(10, pose_index))
                    node.handle_cal(f'save_pose_{pose_index}')

                elif msg_type == 'stepper_jog':
                    node.handle_stepper_jog(
                        str(data.get('dir', 'CW')),
                        int(data.get('steps', 40)),
                        int(data.get('delay_us', 600)),
                    )

                elif msg_type == 'stepper_jog_start':
                    node.handle_stepper_jog_start(
                        str(data.get('dir', 'CW')),
                        int(data.get('delay_us', 85)),
                    )

                elif msg_type == 'stepper_jog_stop':
                    node.handle_stepper_jog_stop()

                elif msg_type == 'servo_jog_start':
                    node.handle_servo_jog_start(str(data.get('sign', '+')))

                elif msg_type == 'servo_jog_stop':
                    node.handle_servo_jog_stop()

                elif msg_type == 'autonomy':
                    node.handle_autonomy(bool(data.get('active', False)))

                elif msg_type == 'deploy':
                    node.handle_deploy()

                elif msg_type == 'standby':
                    node.handle_return_to_standby()

                elif msg_type == 'dev_mode':
                    node.handle_dev_mode(bool(data.get('active', False)))

                elif msg_type == 'record':
                    node.handle_recording(
                        bool(data.get('active', False)),
                        str(data.get('run_name', '')),
                    )

                elif msg_type == 'playback':
                    node.handle_playback(
                        bool(data.get('active', False)),
                        str(data.get('bag_path', '')),
                    )

                elif msg_type == 'refresh_bags':
                    node._loop.call_soon_threadsafe(node._broadcast_sensor_state_sync)

                else:
                    node.get_logger().debug(f'Unknown message type: {msg_type}')

        except WebSocketDisconnect:
            node._ws_clients.discard(ws)
            node.get_logger().info(f'Browser disconnected — {len(node._ws_clients)} client(s)')

    return app


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(args=None) -> None:
    """
    Start ui_server: spin rclpy in a background thread, run uvicorn on main thread.

    The asyncio event loop is created first and passed into the ROS2 node so that
    topic callbacks can safely schedule coroutines on it using call_soon_threadsafe.
    """
    rclpy.init(args=args)

    # Grab the asyncio loop that uvicorn will use
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    node = UIServer(loop=loop)
    app  = build_app(node)

    # Spin rclpy in a background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    config = uvicorn.Config(
        app,
        host=node._host,
        port=node._port,
        log_level='info',
    )
    server = uvicorn.Server(config)

    try:
        # IMPORTANT: Use loop.run_until_complete(server.serve()), NOT server.run().
        # server.run() creates its own event loop, which would differ from node._loop
        # and break call_soon_threadsafe. Serving directly on our loop ensures ROS2
        # callbacks can safely schedule coroutines via call_soon_threadsafe.
        loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == '__main__':
    main()
