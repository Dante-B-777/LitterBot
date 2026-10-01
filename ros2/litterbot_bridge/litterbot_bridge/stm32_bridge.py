"""
stm32_bridge.py — LitterBot ROS2 <-> STM32 Bridge Node
=======================================================

Overview
--------
This node bridges the ROS2 graph with the LitterBot STM32 microcontroller
over UART serial (LPUART1, 115200 baud on the STM32 side).

Responsibilities:
  1. Subscribe to /cmd_vel (geometry_msgs/Twist)
  2. Apply mecanum wheel inverse kinematics to produce four wheel speeds
  3. Scale speeds to [-1000, 1000] firmware units (firmware maps to %)
  4. Format and send the motor command string to the STM32:
       "M <fl> <fr> <rl> <rr>\\n"
  5. Publish /encoder_raw (std_msgs/Int32MultiArray) — stub for future use
     when the STM32 firmware sends back encoder tick data

Mock Serial Stub
----------------
When `mock_serial` parameter is True (default), no real serial port is opened.
Instead, the node logs what it *would* send. This allows full development and
testing without any hardware attached.

Set `mock_serial:=false` and provide `serial_port` to use real hardware:
  ros2 run litterbot_bridge stm32_bridge --ros-args \\
      -p mock_serial:=false -p serial_port:=/dev/ttyUSB0

Mecanum Wheel Kinematics
------------------------
Given body velocity (vx, vy, wz):

  FL = (1/r) * (vx - vy - (L+W)*wz)
  FR = (1/r) * (vx + vy + (L+W)*wz)
  RL = (1/r) * (vx + vy - (L+W)*wz)
  RR = (1/r) * (vx - vy + (L+W)*wz)

Robot physical parameters (from ROS2 Development.md):
  r      = 0.052 m   (wheel radius)
  L      = 0.168 m   (wheelbase, front-to-rear)
  W      = 0.2025 m  (track width, left-to-right)
  L+W    = 0.3705 m

Output is scaled to [-1000, 1000] firmware units via MAX_WHEEL_SPEED_RAD_S.

Serial Protocol
---------------
Command (Jetson -> STM32):
  "M <fl> <fr> <rl> <rr>\\n"
  Values are integers in range [-1000, 1000].
  Example: "M 500 500 500 500\\n" — all wheels forward at 50%

Encoder (STM32 -> Jetson):  [TBD — not yet implemented on STM32 side]

Topics
------
  Subscribed:
    /cmd_vel        (geometry_msgs/Twist)      — desired body velocity
    /emergency_stop (std_msgs/Bool)            — True=halt and block, False=resume
    /deploy_cmd     (std_msgs/String)          — "deploy" or "standby"
    /dev_mode_cmd   (std_msgs/Bool)            — True=enable dev mode, False=disable
  Published:
    /encoder_raw    (std_msgs/Int32MultiArray) — raw encoder ticks [FL, FR, RL, RR]
    /stm32_tx       (std_msgs/String)          — raw UART frame sent to STM32
    /stm32_rx       (std_msgs/String)          — raw UART lines received from STM32
    /stm32_state    (std_msgs/String)          — current STM32 deploy state

Parameters
----------
  serial_port  (str,  default '/dev/ttyUSB0') — serial device path
  baud_rate    (int,  default 115200)         — must match STM32 UART config
  mock_serial  (bool, default True)           — bypass real serial for dev/test
"""

import math
import re
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32MultiArray, String

from litterbot_bridge.device_discovery import detect_serial_device

# ---------------------------------------------------------------------------
# Robot physical constants (see ROS2 Development.md — Mecanum Wheel Kinematics)
# ---------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.052        # metres
WHEELBASE_M    = 0.168        # metres  (L — front to rear axle)
TRACK_WIDTH_M  = 0.2025       # metres  (W — left to right wheel centre)
L_PLUS_W       = WHEELBASE_M + TRACK_WIDTH_M  # 0.3705 m

# Maximum wheel angular speed used for normalising to percent.
# At 100% duty cycle the motors run at roughly 3 rev/s (~18.8 rad/s).
# Adjust this if motor characterisation data becomes available.
MAX_WHEEL_SPEED_RAD_S = 18.85  # rad/s  (3 rev/s × 2π)


class STM32Bridge(Node):
    """
    ROS2 node that bridges /cmd_vel Twist commands to the STM32 over UART.

    In mock mode (default) the serial write is replaced with a log message
    so the node can be developed and tested without physical hardware.
    """

    def __init__(self) -> None:
        super().__init__('stm32_bridge')

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('baud_rate',   115200)
        self.declare_parameter('mock_serial', True)

        self._port      = str(self.get_parameter('serial_port').value)
        self._baud      = self.get_parameter('baud_rate').value
        self._mock      = self.get_parameter('mock_serial').value

        if not self._mock and self._port.strip() in ('', 'auto', 'AUTO', 'Auto'):
            resolution = detect_serial_device('stm32')
            if resolution.path:
                self._port = resolution.path
                self.get_logger().info(
                    f'Auto-detected STM32 serial port: {self._port} '
                    f'({resolution.source})'
                )
            else:
                self.get_logger().warn(
                    'STM32 serial_port=auto but no matching device was found. '
                    'Falling back to /dev/ttyACM0.'
                )
                self._port = '/dev/ttyACM0'

        # ------------------------------------------------------------------ #
        # Serial connection (real or mock)
        # ------------------------------------------------------------------ #
        self._serial = None
        if self._mock:
            self.get_logger().info(
                'mock_serial=True — serial port will NOT be opened. '
                'Motor commands will be logged instead.'
            )
        else:
            self._open_serial()

        # ------------------------------------------------------------------ #
        # Subscriber: /cmd_vel
        # ------------------------------------------------------------------ #
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,  # QoS history depth
        )

        # ------------------------------------------------------------------ #
        # Publisher: /encoder_raw
        # Publishes [FL, FR, RL, RR] encoder tick counts from the STM32.
        # Currently a stub — the STM32 firmware does not yet send ticks.
        # ------------------------------------------------------------------ #
        self._encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/encoder_raw',
            10,
        )

        # ------------------------------------------------------------------ #
        # Publisher: /stm32_tx
        # Publishes the raw UART frame string each time a motor command is
        # sent. Consumed by ui_server to populate the "Jetson->STM32" log panel.
        # ------------------------------------------------------------------ #
        self._stm32_tx_pub = self.create_publisher(
            String,
            '/stm32_tx',
            10,
        )

        # ------------------------------------------------------------------ #
        # Subscriber: /emergency_stop
        # Bool True  -> send X\n to STM32, block all future motor commands
        # Bool False -> clear the block, resume normal operation
        # ------------------------------------------------------------------ #
        self._estop_active: bool = False
        self._estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self._emergency_stop_callback,
            10,
        )

        # ------------------------------------------------------------------ #
        # Subscriber: /pause_cmd
        # Bool True  -> send M 0 0 0 0\n, block all motor/scoop commands
        # Bool False -> clear pause, resume normal operation
        # Unlike e-stop, pause does NOT send X\n — motors are zeroed via a
        # normal motor command so the STM32 watchdog stays satisfied.
        # ------------------------------------------------------------------ #
        self._paused: bool = False
        self._pause_sub = self.create_subscription(
            Bool,
            '/pause_cmd',
            self._pause_callback,
            10,
        )

        # ------------------------------------------------------------------ #
        # Subscriber: /scoop_cmd
        # Triggers the coordinated stepper+servo scoop sequence on the STM32.
        # Sends "S SCOOP\n" over UART — STM32 firmware executes the full sequence.
        # Blocked during e-stop or pause.
        # ------------------------------------------------------------------ #
        self._scoop_sub = self.create_subscription(
            String,
            '/scoop_cmd',
            self._scoop_callback,
            10,
        )

        self._cal_sub = self.create_subscription(
            String,
            '/cal_cmd',
            self._cal_callback,
            10,
        )

        # ------------------------------------------------------------------ #
        # Subscriber: /stepper_cmd
        # Carries raw stepper jog bursts from the UI. Message format is
        # "STEP <CW|CCW> <steps> <delay_us>" — bridge wraps it as the
        # corresponding TEST STEP UART command. Used for the I/K jog keys.
        # ------------------------------------------------------------------ #
        self._stepper_sub = self.create_subscription(
            String,
            '/stepper_cmd',
            self._stepper_callback,
            10,
        )

        # ------------------------------------------------------------------ #
        # Subscriber: /servo_cmd
        # Bucket servo jog. Message format is "JOG <+|-> [delta]" — bridge
        # wraps it as "SERVO JOG <+|-> <delta>\n" for the STM32. Each call
        # issues one relative HX65 MovePosition; the UI repeats ~20 Hz.
        # ------------------------------------------------------------------ #
        self._servo_sub = self.create_subscription(
            String,
            '/servo_cmd',
            self._servo_callback,
            10,
        )

        # ------------------------------------------------------------------ #
        # Deploy/Standby system
        # STM32 owns the state machine (STANDBY → DEPLOYING → ACTIVE).
        # This node relays deploy/dev-mode commands and publishes STM32 state.
        # ------------------------------------------------------------------ #
        self._stm32_state: str = 'STANDBY'  # Last known STM32 state
        self._dev_mode: bool = False          # Last known dev mode flag

        # Publisher: /stm32_state — current STM32 deploy state string
        self._stm32_state_pub = self.create_publisher(
            String, '/stm32_state', 10,
        )

        # Publisher: /stm32_rx — raw lines received from STM32 UART
        self._stm32_rx_pub = self.create_publisher(
            String, '/stm32_rx', 10,
        )

        # Subscriber: /deploy_cmd — "deploy" or "standby"
        self._deploy_sub = self.create_subscription(
            String, '/deploy_cmd', self._deploy_callback, 10,
        )

        # Subscriber: /dev_mode_cmd — Bool True=enable, False=disable
        self._dev_mode_sub = self.create_subscription(
            Bool, '/dev_mode_cmd', self._dev_mode_callback, 10,
        )

        # ------------------------------------------------------------------ #
        # Serial read thread (for STM32 responses)
        # ------------------------------------------------------------------ #
        self._serial_read_thread = None
        self._serial_read_running = False
        if not self._mock and self._serial and self._serial.is_open:
            self._start_serial_read_thread()
            # Brief delay so STM32 can recover from USB connection glitch
            # before we send the initial state query. Without this, Q arrives
            # during UART error recovery and gets dropped.
            import time as _time
            _time.sleep(0.5)
            self._send_raw(b'Q\n')

        self.get_logger().info(
            f'stm32_bridge started — port={self._port}  baud={self._baud}  '
            f'mock={self._mock}'
        )

    # ---------------------------------------------------------------------- #
    # Serial helpers
    # ---------------------------------------------------------------------- #

    def _open_serial(self) -> None:
        """Open the UART serial port to the STM32."""
        try:
            import serial  # pyserial — not installed in mock mode
            self._serial = serial.Serial(self._port, self._baud, timeout=1.0)
            self.get_logger().info(
                f'Serial port opened: {self._port} @ {self._baud} baud'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'Failed to open serial port {self._port}: {exc}\n'
                'Falling back to mock mode.'
            )
            self._mock = True

    def _send_raw(self, data: bytes) -> None:
        """Send raw bytes to STM32 (real or mock)."""
        if self._mock:
            self.get_logger().debug(f'[MOCK SERIAL TX] {data!r}')
        else:
            if self._serial and self._serial.is_open:
                self._serial.write(data)
            else:
                self.get_logger().warn('Serial port not open — command dropped.')

    def _start_serial_read_thread(self) -> None:
        """Start a daemon thread that reads lines from the STM32 UART."""
        self._serial_read_running = True
        self._serial_read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True
        )
        self._serial_read_thread.start()

    def _serial_read_loop(self) -> None:
        """Background thread: read lines from STM32 and publish to /stm32_rx."""
        while self._serial_read_running:
            try:
                if not (self._serial and self._serial.is_open):
                    break
                line = self._serial.readline()
                if not line:
                    continue
                text = line.decode('ascii', errors='replace').strip()
                if not text:
                    continue

                # Publish raw line
                rx_msg = String()
                rx_msg.data = text
                self._stm32_rx_pub.publish(rx_msg)

                # Parse state responses: "STATE STANDBY", "STATE ACTIVE DEV", etc.
                if text.startswith('STATE '):
                    parts = text.split()
                    if len(parts) >= 2:
                        self._stm32_state = parts[1]
                        self._dev_mode = 'DEV' in parts
                        state_msg = String()
                        state_msg.data = text
                        self._stm32_state_pub.publish(state_msg)
                        self.get_logger().info(f'STM32 state: {text}')

                # Parse encoder telemetry: "ENC <fl> <fr> <rl> <rr>"
                elif text.startswith('ENC '):
                    values = re.findall(r'-?\d+', text)
                    if len(values) >= 4:
                        ticks = [int(v) for v in values[-4:]]
                        enc_msg = Int32MultiArray()
                        enc_msg.data = ticks
                        self._encoder_pub.publish(enc_msg)
                    else:
                        self.get_logger().warn(f'Bad encoder telemetry: {text}')

                # Log deploy results
                elif text.startswith('DEPLOY'):
                    self.get_logger().info(f'STM32: {text}')
                    # After deploy, query state to get updated status
                    if 'OK' in text or 'FAIL' in text:
                        self._send_raw(b'Q\n')

                # Log dev mode acknowledgements
                elif text.startswith('DEV '):
                    self.get_logger().info(f'STM32: {text}')
                    self._send_raw(b'Q\n')

                # Log standby rejections
                elif text.startswith('ERR:STANDBY'):
                    self.get_logger().warn('STM32 rejected command — robot is in STANDBY.')

            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Serial read error: {exc}')
                break

    def _deploy_callback(self, msg: String) -> None:
        """Handle /deploy_cmd — send D DEPLOY or return to standby."""
        cmd = msg.data.strip().lower()
        if cmd == 'deploy':
            self.get_logger().info('Sending D DEPLOY to STM32...')
            self._send_raw(b'D DEPLOY\n')
            tx_msg = String()
            tx_msg.data = 'D DEPLOY'
            self._stm32_tx_pub.publish(tx_msg)
            if self._mock:
                self._stm32_state = 'ACTIVE'
                state_msg = String()
                state_msg.data = 'STATE ACTIVE'
                self._stm32_state_pub.publish(state_msg)
                self.get_logger().info('[MOCK] Auto-transitioning to STATE ACTIVE')
        elif cmd == 'standby':
            # E-stop + re-query forces standby awareness
            self._send_raw(b'X\n')
            self._send_raw(b'Q\n')
            tx_msg = String()
            tx_msg.data = 'X (return to standby)'
            self._stm32_tx_pub.publish(tx_msg)

    def _dev_mode_callback(self, msg: Bool) -> None:
        """Handle /dev_mode_cmd — toggle dev mode on the STM32."""
        if msg.data:
            self._send_raw(b'DEV ON\n')
            tx_msg = String()
            tx_msg.data = 'DEV ON'
            self._stm32_tx_pub.publish(tx_msg)
        else:
            self._send_raw(b'DEV OFF\n')
            tx_msg = String()
            tx_msg.data = 'DEV OFF'
            self._stm32_tx_pub.publish(tx_msg)

    def _send_motor_command(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """
        Format and send a motor command frame to the STM32.

        Frame format: "M <fl> <fr> <rl> <rr>\\n"
        Values must be integers in [-1000, 1000].

        In mock mode the frame is logged at DEBUG level instead of written to
        the serial port.
        """
        frame = f'M {fl} {fr} {rl} {rr}\n'

        # Publish the raw frame so the UI can log what was sent
        tx_msg = String()
        tx_msg.data = frame.strip()
        self._stm32_tx_pub.publish(tx_msg)

        if self._mock:
            self.get_logger().debug(f'[MOCK SERIAL TX] {frame.strip()}')
        else:
            if self._serial and self._serial.is_open:
                self._serial.write(frame.encode('ascii'))
            else:
                self.get_logger().warn('Serial port not open — command dropped.')

    def _emergency_stop_callback(self, msg: Bool) -> None:
        """
        Handle /emergency_stop messages from the command center UI.

        Bool True  — activate e-stop: send X\\n to STM32 UART and block
                     all subsequent /cmd_vel processing until cleared.
        Bool False — deactivate e-stop: resume normal /cmd_vel processing.

        The STM32 firmware X command immediately stops all motors and
        resets its 250 ms watchdog timer.
        """
        if msg.data:
            self._estop_active = True
            # Send X\n — firmware stops all motors immediately
            if self._mock:
                self.get_logger().warn('[MOCK SERIAL TX] X  (EMERGENCY STOP)')
            else:
                if self._serial and self._serial.is_open:
                    self._serial.write(b'X\n')
            # Also publish to /stm32_tx so the UI logs it
            tx_msg = String()
            tx_msg.data = 'X'
            self._stm32_tx_pub.publish(tx_msg)
            self.get_logger().warn('EMERGENCY STOP activated — motor commands blocked.')
        else:
            self._estop_active = False
            # Publish so the UI can show the estop-cleared event in the log
            clear_msg = String()
            clear_msg.data = 'ESTOP CLEAR'
            self._stm32_tx_pub.publish(clear_msg)
            self.get_logger().info('Emergency stop cleared — resuming normal operation.')

    # ---------------------------------------------------------------------- #
    # Kinematics
    # ---------------------------------------------------------------------- #

    @staticmethod
    def _twist_to_wheel_units(
        vx: float, vy: float, wz: float
    ) -> tuple[int, int, int, int]:
        """
        Apply mecanum wheel inverse kinematics to a body velocity command.

        Parameters
        ----------
        vx : float  — forward  velocity  (m/s), positive = forward
        vy : float  — sideways velocity  (m/s), positive = left
        wz : float  — yaw rate           (rad/s), positive = counter-clockwise

        Returns
        -------
        (fl, fr, rl, rr) — wheel speeds as integers in [-1000, 1000].

        Firmware mapping: uart_cmd.c divides by 10 to get percent,
        so 1000 → 100%, 500 → 50%, -1000 → -100%, etc.

        Kinematics reference (see ROS2 Development.md):
          FL = (1/r)(vx - vy - (L+W)*wz)
          FR = (1/r)(vx + vy + (L+W)*wz)
          RL = (1/r)(vx + vy - (L+W)*wz)
          RR = (1/r)(vx - vy + (L+W)*wz)
        """
        inv_r = 1.0 / WHEEL_RADIUS_M

        fl_rad_s = inv_r * (vx - vy - L_PLUS_W * wz)
        fr_rad_s = inv_r * (vx + vy + L_PLUS_W * wz)
        rl_rad_s = inv_r * (vx + vy - L_PLUS_W * wz)
        rr_rad_s = inv_r * (vx - vy + L_PLUS_W * wz)

        def to_units(rad_s: float) -> int:
            """Scale rad/s to [-1000, 1000] firmware units and clamp."""
            units = (rad_s / MAX_WHEEL_SPEED_RAD_S) * 1000.0
            return int(max(-1000, min(1000, math.copysign(math.floor(abs(units)), units))))

        return (
            to_units(fl_rad_s),
            to_units(fr_rad_s),
            to_units(rl_rad_s),
            to_units(rr_rad_s),
        )

    # ---------------------------------------------------------------------- #
    # Subscription callbacks
    # ---------------------------------------------------------------------- #

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Handle an incoming /cmd_vel message.

        Converts the Twist into four wheel speed firmware units ([-1000, 1000]) using mecanum
        inverse kinematics, then sends the motor command to the STM32.

        Parameters
        ----------
        msg : geometry_msgs/Twist
            linear.x  — forward velocity  (m/s)
            linear.y  — sideways velocity (m/s)
            angular.z — yaw rate          (rad/s)
        """
        if self._estop_active or self._paused:
            return  # Dropped while e-stop or pause is active

        fl, fr, rl, rr = self._twist_to_wheel_units(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )
        self._send_motor_command(fl, fr, rl, rr)

    def _pause_callback(self, msg: Bool) -> None:
        """
        Handle /pause_cmd messages.

        Bool True  — pause: send M 0 0 0 0 to zero motors, block commands.
        Bool False — resume: unblock commands.

        Unlike e-stop, pause sends a zero motor command (not X\\n), so the
        STM32 watchdog timer remains satisfied and the STM32 stays responsive.
        E-stop always overrides pause — if _estop_active, pause/resume is ignored.
        """
        if self._estop_active:
            return  # E-stop takes precedence; ignore pause commands

        if msg.data:
            self._paused = True
            self._send_motor_command(0, 0, 0, 0)
            tx_msg = String()
            tx_msg.data = 'PAUSE'
            self._stm32_tx_pub.publish(tx_msg)
            self.get_logger().info('Paused — motors zeroed, commands blocked.')
        else:
            self._paused = False
            tx_msg = String()
            tx_msg.data = 'RESUME'
            self._stm32_tx_pub.publish(tx_msg)
            self.get_logger().info('Resumed — normal operation.')

    def _scoop_callback(self, msg: String) -> None:
        """
        Handle a /scoop_cmd message — trigger the scoop sequence on the STM32.

        Sends "S\\n" over UART. The STM32 firmware executes the full coordinated
        stepper+servo sequence autonomously after receiving this command.
        Silently dropped if e-stop is active.
        """
        if self._estop_active or self._paused:
            self.get_logger().warn('Scoop command ignored — e-stop or pause is active.')
            return

        self.get_logger().info('Sending S SCOOP to STM32...')
        self._send_raw(b'S SCOOP\n')

        tx_msg = String()
        tx_msg.data = 'S SCOOP'
        self._stm32_tx_pub.publish(tx_msg)

    def _cal_callback(self, msg: String) -> None:
        """
        Handle a /cal_cmd message — route calibration commands to STM32 UART.

        Dropped if e-stop is active (cal requires a responsive STM32).
        """
        if self._estop_active:
            self.get_logger().warn('Cal command ignored — e-stop is active.')
            return

        _cal_map = {
            'cal_arm':          b'TEST ARM HOME\n',
            'cal_home':         b'TEST ARM HOME\n',
            'cal_status':       b'TEST ARM STATUS\n',
            'cal_save_steps':   b'TEST ARM SAVE\n',
            'cal_servo_start':  b'CAL SERVO START\n',
            'cal_servo_stop':   b'CAL SERVO STOP\n',
            'cal_save_home':    b'TEST SAVE HOME\n',
            'cal_save_open':    b'TEST SAVE OPEN\n',
            'cal_save_closed':  b'TEST SAVE CLOSED\n',
            'cal_save_dump':    b'TEST SAVE DUMP\n',
        }
        cmd = msg.data.strip()
        if cmd.startswith('save_pose_'):
            try:
                pose_index = int(cmd.split('_')[-1])
            except ValueError:
                pose_index = 0
            if 1 <= pose_index <= 10:
                uart_cmd = f'TEST SAVE P{pose_index}\n'.encode()
            else:
                uart_cmd = None
        else:
            uart_cmd = _cal_map.get(cmd)
        if uart_cmd is None:
            self.get_logger().warn(f'Unknown cal command: {cmd!r}')
            return

        self.get_logger().info(f'Sending: {uart_cmd.decode().strip()}')
        self._send_raw(uart_cmd)

        tx_msg = String()
        tx_msg.data = uart_cmd.decode().strip()
        self._stm32_tx_pub.publish(tx_msg)

    def _stepper_callback(self, msg: String) -> None:
        """
        Handle a /stepper_cmd message — forward to the STM32 as the
        appropriate UART command.

        Supported payload formats:
          * "JOG_START <CW|CCW> <delay_us>"
              Continuous firmware-side jog. Starts a blocking step loop
              in the STM32 that runs until any new UART line arrives.
              This is the preferred path — no burst-to-burst gaps.
          * "JOG_STOP"
              Break out of the continuous jog loop. Maps to the STEPPER
              JOG STOP UART command. The STM32's blocking jog loop
              polls its command-ready flag and exits as soon as this
              arrives over the wire.
          * "STEP <CW|CCW> <steps> <delay_us>"
              Legacy single-burst form (used by calibration/test UI).
              Maps to TEST STEP for one-off positioning. Kept for
              backwards compat; not used by the hold-to-jog keys.

        Dropped if e-stop or pause is active.
        """
        if self._estop_active or self._paused:
            self.get_logger().warn('Stepper jog ignored — e-stop or pause is active.')
            return

        parts = msg.data.strip().split()
        if not parts:
            return

        verb = parts[0]

        # Continuous jog start ---------------------------------------------
        if verb == 'JOG_START':
            if len(parts) != 3:
                self.get_logger().warn(
                    f'Malformed /stepper_cmd: {msg.data!r} '
                    f'(expected "JOG_START <CW|CCW> <delay_us>")'
                )
                return
            direction = parts[1].upper()
            if direction not in ('CW', 'CCW'):
                self.get_logger().warn(f'Bad stepper direction: {direction!r}')
                return
            try:
                delay_us = int(parts[2])
            except ValueError:
                self.get_logger().warn(f'Bad stepper delay: {msg.data!r}')
                return
            if delay_us < 5:
                delay_us = 5
            uart_cmd = f'STEPPER JOG START {direction} {delay_us}\n'.encode()
            self._send_raw(uart_cmd)
            tx_msg = String()
            tx_msg.data = uart_cmd.decode().strip()
            self._stm32_tx_pub.publish(tx_msg)
            return

        # Continuous jog stop ----------------------------------------------
        if verb == 'JOG_STOP':
            uart_cmd = b'STEPPER JOG STOP\n'
            self._send_raw(uart_cmd)
            tx_msg = String()
            tx_msg.data = 'STEPPER JOG STOP'
            self._stm32_tx_pub.publish(tx_msg)
            return

        # Legacy single-burst form ------------------------------------------
        if verb == 'STEP':
            if len(parts) != 4:
                self.get_logger().warn(
                    f'Malformed /stepper_cmd: {msg.data!r} '
                    f'(expected "STEP <CW|CCW> <steps> <delay_us>")'
                )
                return
            direction = parts[1].upper()
            if direction not in ('CW', 'CCW'):
                self.get_logger().warn(f'Bad stepper direction: {direction!r}')
                return
            try:
                steps    = int(parts[2])
                delay_us = int(parts[3])
            except ValueError:
                self.get_logger().warn(f'Bad stepper numeric args: {msg.data!r}')
                return
            if steps <= 0 or delay_us < 5:
                self.get_logger().warn(
                    f'Bad stepper args: steps={steps} delay_us={delay_us}'
                )
                return
            uart_cmd = f'TEST STEP {direction} {steps} {delay_us}\n'.encode()
            self._send_raw(uart_cmd)
            tx_msg = String()
            tx_msg.data = uart_cmd.decode().strip()
            self._stm32_tx_pub.publish(tx_msg)
            return

        self.get_logger().warn(f'Unknown stepper verb in /stepper_cmd: {verb!r}')

    def _servo_callback(self, msg: String) -> None:
        """
        Handle a /servo_cmd message — forward to the STM32 as continuous
        servo jog commands.

        Payload formats:
          * "JOG_START <+|->"  — start a continuous glide toward the +
                                 (4095) or - (0) endpoint. Firmware sends
                                 one HX65 MovePositionSpeed targeting the
                                 endpoint at a slow glide speed and the
                                 servo's own controller handles the rest.
          * "JOG_STOP"         — freeze the servo at its current estimated
                                 position. Firmware computes position from
                                 start_pos + sign*speed*elapsed and issues
                                 one MovePositionSpeed to hold it there.

        Dropped if e-stop or pause is active.
        """
        if self._estop_active or self._paused:
            self.get_logger().warn('Servo jog ignored — e-stop or pause is active.')
            return

        parts = msg.data.strip().split()
        if not parts:
            return
        verb = parts[0]

        if verb == 'JOG_START':
            if len(parts) < 2 or parts[1] not in ('+', '-'):
                self.get_logger().warn(
                    f'Malformed /servo_cmd: {msg.data!r} '
                    f'(expected "JOG_START <+|->")'
                )
                return
            uart_cmd = f'SERVO JOG START {parts[1]}\n'.encode()
        elif verb == 'JOG_STOP':
            uart_cmd = b'SERVO JOG STOP\n'
        else:
            self.get_logger().warn(f'Unknown servo verb in /servo_cmd: {verb!r}')
            return

        self.get_logger().info(f'Servo UART: {uart_cmd!r}')
        self._send_raw(uart_cmd)
        tx_msg = String()
        tx_msg.data = uart_cmd.decode().strip()
        self._stm32_tx_pub.publish(tx_msg)

    # ---------------------------------------------------------------------- #
    # Cleanup
    # ---------------------------------------------------------------------- #

    def destroy_node(self) -> None:
        """Stop serial read thread and close the serial port on shutdown."""
        self._serial_read_running = False
        if self._serial and self._serial.is_open:
            self._serial.write(b'X\n')
            self._serial.close()
            self.get_logger().info('Serial port closed.')
        if self._serial_read_thread and self._serial_read_thread.is_alive():
            self._serial_read_thread.join(timeout=2.0)
        super().destroy_node()


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(args=None) -> None:
    """Spin the STM32Bridge node until SIGINT."""
    rclpy.init(args=args)
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
