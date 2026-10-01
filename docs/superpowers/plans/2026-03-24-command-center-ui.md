# Command Center UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a browser-based command center for LitterBot with real-time log panels, keyboard teleop, and emergency stop, communicating over WebSocket to a FastAPI ROS2 node.

**Architecture:** FastAPI + WebSocket node (`ui_server`) runs alongside existing ROS2 nodes on the Jetson/WSL2. The browser connects via WebSocket for bidirectional real-time communication. No browser-side installs required — open `http://localhost:8765`.

**Tech Stack:** Python 3.10, ROS2 Humble, FastAPI, uvicorn, vanilla HTML/JS (no framework)

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Modify | `ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py` | Fix speed scaling bug; add `/stm32_tx` publisher; add `/emergency_stop` subscriber |
| Modify | `ros2/litterbot_bridge/setup.py` | Add `ui_server` entry point |
| Modify | `ros2/litterbot_bridge/package.xml` | Add `nav_msgs` exec_depend |
| Create | `ros2/litterbot_bridge/requirements.txt` | pip deps: fastapi, uvicorn |
| Create | `ros2/litterbot_bridge/litterbot_bridge/ui_server.py` | FastAPI + rclpy bridge node |
| Create | `ros2/litterbot_bridge/litterbot_bridge/ui/index.html` | Self-contained frontend |
| Create | `ros2/litterbot_bridge/test/test_kinematics.py` | Unit tests for kinematics scaling fix |

---

## Task 1: Install pip dependencies

**Files:**
- Create: `ros2/litterbot_bridge/requirements.txt`

- [ ] **Step 1: Create requirements.txt**

```
fastapi>=0.110.0
uvicorn[standard]>=0.29.0
```

Save to `ros2/litterbot_bridge/requirements.txt`.

- [ ] **Step 2: Install into WSL Python environment**

```bash
pip install -r /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/ros2/litterbot_bridge/requirements.txt
```

Expected output: `Successfully installed fastapi-... uvicorn-...` (or "already satisfied")

- [ ] **Step 3: Verify imports work**

```bash
python3 -c "import fastapi; import uvicorn; print('OK')"
```

Expected: `OK`

- [ ] **Step 4: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add ros2/litterbot_bridge/requirements.txt
git commit -m "add pip requirements for ui_server (fastapi, uvicorn)"
```

---

## Task 2: Fix stm32_bridge — speed scaling bug

**Files:**
- Create: `ros2/litterbot_bridge/test/test_kinematics.py`
- Modify: `ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py`

**Background:** The firmware (`uart_cmd.c`) accepts values `[-1000, 1000]` and maps them to `[-100%, 100%]` internally. The current code outputs `[-100, 100]`, causing the robot to run at 1/10th speed.

- [ ] **Step 1: Write the failing test**

Create `ros2/litterbot_bridge/test/test_kinematics.py`:

```python
"""
test_kinematics.py — Unit tests for stm32_bridge mecanum kinematics.

These tests verify:
  1. Full-forward Twist produces firmware-range output [-1000, 1000]
  2. Zero Twist produces all zeros
  3. Output is clamped to [-1000, 1000]
  4. All-forward command produces equal positive values for all four wheels
"""
import sys
import os

# Allow import without ROS2 environment
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from litterbot_bridge.stm32_bridge import STM32Bridge  # noqa: E402 — path hack above


def test_zero_twist_gives_zero_output():
    """Stopped robot → all wheel commands are 0."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.0, 0.0, 0.0)
    assert fl == 0 and fr == 0 and rl == 0 and rr == 0


def test_full_forward_output_in_firmware_range():
    """Large forward velocity → values in [-1000, 1000], not [-100, 100]."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(1.0, 0.0, 0.0)
    # All four wheels should be equal and positive for pure forward motion
    assert fl == fr == rl == rr
    assert fl > 0
    # Must be in firmware range — if this is ≤ 100 the bug is still present
    assert fl > 100, f"Expected >100 (firmware range), got {fl} — speed scaling bug!"
    assert fl <= 1000


def test_output_clamped_to_firmware_range():
    """Very large velocity → clamped to ±1000."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(100.0, 0.0, 0.0)
    assert fl == 1000
    assert fr == 1000

    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(-100.0, 0.0, 0.0)
    assert fl == -1000


def test_pure_rotation_left_right_asymmetry():
    """Counter-clockwise rotation → left wheels negative, right wheels positive."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.0, 0.0, 1.0)
    assert fl < 0 and rl < 0, "Left wheels should be negative for CCW rotation"
    assert fr > 0 and rr > 0, "Right wheels should be positive for CCW rotation"
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/ros2/litterbot_bridge
python3 -m pytest test/test_kinematics.py -v 2>&1 | head -40
```

Expected: `AttributeError: type object 'STM32Bridge' has no attribute '_twist_to_wheel_units'` or `AssertionError: Expected >100 (firmware range), got 50`

- [ ] **Step 3: Apply the fix to stm32_bridge.py**

In `ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py`, make these changes:

**a) Update the module docstring** — change the "Clamp speeds to [-100, 100]" line and the Serial Protocol section:
```
  3. Scale speeds to [-1000, 1000] firmware units (firmware maps these to %)
  ...
Serial Protocol
---------------
Command (Jetson -> STM32):
  "M <fl> <fr> <rl> <rr>\\n"
  Values are integers in range [-1000, 1000] (firmware maps to -100%..+100%).
  Example: "M 500 500 500 500\\n" — all wheels forward at 50%
```

**b) Rename the static method** from `_twist_to_wheel_percents` to `_twist_to_wheel_units` and update its docstring and internals:

```python
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

    Firmware mapping: the STM32 uart_cmd.c divides by 10 to get percent,
    so 1000 → 100%, 500 → 50%, etc.

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
```

**c) Update `_send_motor_command` docstring** — change `[-100, 100]` to `[-1000, 1000]`.

**d) Update `_cmd_vel_callback`** — rename the method call:
```python
fl, fr, rl, rr = self._twist_to_wheel_units(
    msg.linear.x,
    msg.linear.y,
    msg.angular.z,
)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/ros2/litterbot_bridge
python3 -m pytest test/test_kinematics.py -v
```

Expected:
```
test/test_kinematics.py::test_zero_twist_gives_zero_output PASSED
test/test_kinematics.py::test_full_forward_output_in_firmware_range PASSED
test/test_kinematics.py::test_output_clamped_to_firmware_range PASSED
test/test_kinematics.py::test_pure_rotation_left_right_asymmetry PASSED
4 passed in ...
```

- [ ] **Step 5: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py \
        ros2/litterbot_bridge/test/test_kinematics.py
git commit -m "fix: scale motor commands to [-1000,1000] to match STM32 firmware

Firmware uart_cmd.c expects values in [-1000, 1000] and divides by 10
internally to get percent. Previous code sent [-100, 100] causing the
robot to run at 1/10th the commanded speed."
```

---

## Task 3: Extend stm32_bridge — /stm32_tx publisher and /emergency_stop subscriber

**Files:**
- Modify: `ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py`

- [ ] **Step 1: Add String import at top of stm32_bridge.py**

In the imports block, add:
```python
from std_msgs.msg import Int32MultiArray, String, Bool
```
(replace the existing `from std_msgs.msg import Int32MultiArray` line)

- [ ] **Step 2: Add /stm32_tx publisher and /emergency_stop subscriber in __init__**

After the existing `self._encoder_pub` block, add:

```python
        # ------------------------------------------------------------------ #
        # Publisher: /stm32_tx
        # Publishes the raw UART frame string each time a motor command is
        # sent. Consumed by ui_server to populate the "Jetson→STM32" log panel.
        # ------------------------------------------------------------------ #
        self._stm32_tx_pub = self.create_publisher(
            String,
            '/stm32_tx',
            10,
        )

        # ------------------------------------------------------------------ #
        # Subscriber: /emergency_stop
        # Bool True  → send X\n to STM32, block all future motor commands
        # Bool False → clear the block, resume normal operation
        # ------------------------------------------------------------------ #
        self._estop_active: bool = False
        self._estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self._emergency_stop_callback,
            10,
        )
```

- [ ] **Step 3: Publish /stm32_tx in _send_motor_command**

After the `frame = f'M {fl} {fr} {rl} {rr}\n'` line, add:
```python
        # Publish the raw frame so the UI can log what was sent
        tx_msg = String()
        tx_msg.data = frame.strip()
        self._stm32_tx_pub.publish(tx_msg)
```

- [ ] **Step 4: Add _emergency_stop_callback method**

Add after `_send_motor_command`, before `_twist_to_wheel_units`:

```python
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
            frame = 'X\n'
            if self._mock:
                self.get_logger().warn('[MOCK SERIAL TX] X  (EMERGENCY STOP)')
            else:
                if self._serial and self._serial.is_open:
                    self._serial.write(frame.encode('ascii'))
            # Also publish to /stm32_tx so the UI logs it
            tx_msg = String()
            tx_msg.data = 'X'
            self._stm32_tx_pub.publish(tx_msg)
            self.get_logger().warn('EMERGENCY STOP activated — motor commands blocked.')
        else:
            self._estop_active = False
            self.get_logger().info('Emergency stop cleared — resuming normal operation.')
```

- [ ] **Step 5: Guard _cmd_vel_callback with e-stop check**

At the top of `_cmd_vel_callback`, add:
```python
        if self._estop_active:
            return  # All motor commands silently dropped while e-stop is active
```

- [ ] **Step 6: Update destroy_node to send X\n instead of M 0 0 0 0**

Replace:
```python
            self._send_motor_command(0, 0, 0, 0)
```
With:
```python
            # Use X command for clean stop rather than M 0 0 0 0
            self._serial.write(b'X\n')
```

Note: this write is already inside `if self._serial and self._serial.is_open:` from the existing code structure — keep that guard in place. In mock mode the block is never reached.

- [ ] **Step 7: Quick smoke test — colcon build still passes**

```bash
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litterbot_bridge 2>&1 | tail -5
```

Expected: `Finished <<< litterbot_bridge`

- [ ] **Step 8: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add ros2/litterbot_bridge/litterbot_bridge/stm32_bridge.py
git commit -m "feat(stm32_bridge): add /stm32_tx publisher and /emergency_stop subscriber

- Publish raw UART frame to /stm32_tx after every motor command so
  ui_server can display it in the log panel
- Subscribe to /emergency_stop (Bool): True sends X\\n to STM32 and
  blocks /cmd_vel; False clears the block
- Use X\\n instead of M 0 0 0 0 in destroy_node for cleaner shutdown"
```

---

## Task 4: Update package metadata

**Files:**
- Modify: `ros2/litterbot_bridge/package.xml`
- Modify: `ros2/litterbot_bridge/setup.py`

- [ ] **Step 1: Add nav_msgs to package.xml**

After the existing `<exec_depend>std_msgs</exec_depend>` line, add:
```xml
  <exec_depend>nav_msgs</exec_depend>
```

Also update the comment block at the top to mention `nav_msgs`:
```xml
    - nav_msgs  : Odometry message for /odom (subscribed by ui_server)
```

- [ ] **Step 2: Add ui_server entry point to setup.py**

In the `console_scripts` list, add:
```python
            # ui_server: command center WebSocket bridge
            #   ros2 run litterbot_bridge ui_server
            #   ros2 run litterbot_bridge ui_server --ros-args -p port:=8765
            'ui_server = litterbot_bridge.ui_server:main',
```

- [ ] **Step 3: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add ros2/litterbot_bridge/package.xml ros2/litterbot_bridge/setup.py
git commit -m "chore: add nav_msgs dep and ui_server entry point to package metadata"
```

---

## Task 5: Create ui_server.py

**Files:**
- Create: `ros2/litterbot_bridge/litterbot_bridge/ui_server.py`

**Threading model:**
- `uvicorn` runs FastAPI on the main thread (asyncio event loop)
- `rclpy.spin()` runs in a `threading.Thread` (background)
- ROS2 topic callbacks bridge into asyncio using `loop.call_soon_threadsafe()`
- All WebSocket clients are tracked in a `set`; log messages are broadcast to all

- [ ] **Step 1: Create ui_server.py**

Create `ros2/litterbot_bridge/litterbot_bridge/ui_server.py` with the following content:

```python
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
Server → Browser (log entry):
  {"type": "log", "channel": "stm32_tx|stm32_rx|node", "ts": 1234.56, "msg": "..."}

Browser → Server (teleop):
  {"type": "cmd_vel", "vx": 0.3, "vy": 0.0, "wz": 0.0}

Browser → Server (e-stop):
  {"type": "estop", "active": true|false}

Topics
------
  Subscribed:
    /cmd_vel      (geometry_msgs/Twist)        — echoed to "node" log panel
    /stm32_tx     (std_msgs/String)            — echoed to "stm32_tx" log panel
    /encoder_raw  (std_msgs/Int32MultiArray)   — echoed to "stm32_rx" log panel
    /odom         (nav_msgs/Odometry)          — echoed to "node" log panel
  Published:
    /cmd_vel      (geometry_msgs/Twist)        — teleop commands from browser
    /emergency_stop (std_msgs/Bool)            — e-stop from browser

Parameters
----------
  host         (str,  default '0.0.0.0') — bind address
  port         (int,  default 8765)       — HTTP + WebSocket port
  log_history  (int,  default 100)        — max entries kept per log channel
"""

import asyncio
import json
import math
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32MultiArray, String

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse


# --------------------------------------------------------------------------- #
# Log entry helper
# --------------------------------------------------------------------------- #

def _log_entry(channel: str, msg: str) -> dict:
    """Build a log entry dict ready for JSON serialisation."""
    return {"type": "log", "channel": channel, "ts": time.time(), "msg": msg}


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
        self.declare_parameter('log_history', 100)

        self._host        = self.get_parameter('host').value
        self._port        = self.get_parameter('port').value
        self._max_history = self.get_parameter('log_history').value

        # ------------------------------------------------------------------ #
        # WebSocket client registry and log history
        # Accessed from the asyncio thread only — no lock needed.
        # ------------------------------------------------------------------ #
        self.clients: set[WebSocket] = set()
        self.history: dict[str, list[dict]] = {
            'stm32_tx': [],
            'stm32_rx': [],
            'node':     [],
        }

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._estop_pub   = self.create_publisher(Bool, '/emergency_stop', 10)

        # ------------------------------------------------------------------ #
        # Subscribers — forward each message to the browser log
        # ------------------------------------------------------------------ #
        self.create_subscription(String,         '/stm32_tx',    self._on_stm32_tx,    10)
        self.create_subscription(Int32MultiArray, '/encoder_raw', self._on_encoder_raw, 10)
        self.create_subscription(Twist,          '/cmd_vel',     self._on_cmd_vel,     10)
        self.create_subscription(Odometry,       '/odom',        self._on_odom,        10)

        self.get_logger().info(
            f'ui_server ready — http://{self._host}:{self._port}/'
        )

    # ---------------------------------------------------------------------- #
    # ROS2 topic callbacks
    # All run in the rclpy thread — use call_soon_threadsafe to reach asyncio.
    # ---------------------------------------------------------------------- #

    def _on_stm32_tx(self, msg: String) -> None:
        """Forward a /stm32_tx message to the 'Jetson→STM32' log panel."""
        entry = _log_entry('stm32_tx', msg.data)
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_encoder_raw(self, msg: Int32MultiArray) -> None:
        """Forward a /encoder_raw message to the 'STM32→Jetson' log panel."""
        vals = ' '.join(str(v) for v in msg.data)
        entry = _log_entry('stm32_rx', f'ENC {vals}')
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Forward a /cmd_vel message to the 'Node↔Node' log panel."""
        text = (
            f'/cmd_vel  vx={msg.linear.x:.2f}  '
            f'vy={msg.linear.y:.2f}  '
            f'wz={msg.angular.z:.2f}'
        )
        entry = _log_entry('node', text)
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    def _on_odom(self, msg: Odometry) -> None:
        """Forward a /odom message to the 'Node↔Node' log panel."""
        p = msg.pose.pose.position
        text = f'/odom  x={p.x:.3f}  y={p.y:.3f}'
        entry = _log_entry('node', text)
        self._loop.call_soon_threadsafe(self._broadcast_sync, entry)

    # ---------------------------------------------------------------------- #
    # Asyncio-thread helpers (called via call_soon_threadsafe)
    # ---------------------------------------------------------------------- #

    def _broadcast_sync(self, entry: dict) -> None:
        """
        Schedule a broadcast coroutine on the asyncio event loop.
        Called from call_soon_threadsafe — must not be a coroutine itself.
        """
        asyncio.ensure_future(self._broadcast(entry))

    async def _broadcast(self, entry: dict) -> None:
        """Send a log entry to all connected WebSocket clients."""
        channel = entry.get('channel', 'node')
        hist = self.history.setdefault(channel, [])
        hist.append(entry)
        if len(hist) > self._max_history:
            hist.pop(0)

        dead: set[WebSocket] = set()
        payload = json.dumps(entry)
        for ws in list(self.clients):
            try:
                await ws.send_text(payload)
            except Exception:  # noqa: BLE001
                dead.add(ws)
        self.clients -= dead

    # ---------------------------------------------------------------------- #
    # Browser → ROS2 message handlers
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

    # Path to the bundled frontend
    _ui_dir = Path(__file__).parent / 'ui'

    @app.get('/', response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        """Serve the command center frontend."""
        html = (_ui_dir / 'index.html').read_text(encoding='utf-8')
        return HTMLResponse(content=html)

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
        node.clients.add(ws)
        node.get_logger().info(f'Browser connected — {len(node.clients)} client(s)')

        # Send history so the browser populates immediately on connect
        for channel, entries in node.history.items():
            for entry in entries:
                await ws.send_text(json.dumps(entry))

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

                else:
                    node.get_logger().debug(f'Unknown message type: {msg_type}')

        except WebSocketDisconnect:
            node.clients.discard(ws)
            node.get_logger().info(f'Browser disconnected — {len(node.clients)} client(s)')

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
        loop='none',   # We provide our own event loop
        log_level='info',
    )
    server = uvicorn.Server(config)

    try:
        loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verify the file is importable (no syntax errors)**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/ros2/litterbot_bridge
python3 -c "
import sys; sys.path.insert(0, '.')
# We can't fully import without ROS2, but we can check syntax
import ast, pathlib
src = pathlib.Path('litterbot_bridge/ui_server.py').read_text()
ast.parse(src)
print('Syntax OK')
"
```

Expected: `Syntax OK`

- [ ] **Step 3: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add ros2/litterbot_bridge/litterbot_bridge/ui_server.py
git commit -m "feat: add ui_server FastAPI+rclpy WebSocket bridge node

Serves command center frontend at GET / and handles WebSocket connections
at /ws. Bridges /cmd_vel, /stm32_tx, /encoder_raw, /odom to browser log
panels. Publishes /cmd_vel (teleop) and /emergency_stop from browser input."
```

---

## Task 6: Create the frontend (index.html)

**Files:**
- Create: `ros2/litterbot_bridge/litterbot_bridge/ui/index.html`

- [ ] **Step 1: Create the ui/ directory and index.html**

```bash
mkdir -p /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/ros2/litterbot_bridge/litterbot_bridge/ui
```

Create `ros2/litterbot_bridge/litterbot_bridge/ui/index.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>LitterBot Command Center</title>
<style>
  /* ------------------------------------------------------------------ */
  /* Reset + base                                                         */
  /* ------------------------------------------------------------------ */
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: #0d0d0d;
    color: #e0e0e0;
    font-family: 'Segoe UI', system-ui, sans-serif;
    font-size: 14px;
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }

  /* ------------------------------------------------------------------ */
  /* Top bar                                                              */
  /* ------------------------------------------------------------------ */
  #topbar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 0 16px;
    height: 48px;
    background: #161616;
    border-bottom: 1px solid #2a2a2a;
    flex-shrink: 0;
  }

  #topbar h1 {
    font-size: 15px;
    font-weight: 600;
    letter-spacing: 0.04em;
    color: #f0f0f0;
  }

  #topbar-right {
    display: flex;
    align-items: center;
    gap: 12px;
  }

  #conn-status {
    font-size: 12px;
    color: #888;
    transition: color 0.2s;
  }
  #conn-status.connected { color: #66bb6a; }
  #conn-status.disconnected { color: #ef5350; }

  #estop-btn {
    padding: 6px 18px;
    border: none;
    border-radius: 4px;
    font-size: 13px;
    font-weight: 700;
    cursor: pointer;
    letter-spacing: 0.05em;
    transition: background 0.15s, color 0.15s;
    background: #c0392b;
    color: #fff;
  }
  #estop-btn.active { background: #7f0000; }
  #estop-btn:hover  { background: #e74c3c; }
  #estop-btn.active:hover { background: #66bb6a; color: #000; }

  /* ------------------------------------------------------------------ */
  /* Main layout: left (logs) + right (camera + teleop)                  */
  /* ------------------------------------------------------------------ */
  #main {
    display: flex;
    flex: 1;
    overflow: hidden;
  }

  /* Left column: three stacked log panels */
  #log-col {
    display: flex;
    flex-direction: column;
    flex: 1.4;
    border-right: 1px solid #2a2a2a;
    overflow: hidden;
  }

  .log-panel {
    display: flex;
    flex-direction: column;
    flex: 1;
    border-bottom: 1px solid #2a2a2a;
    overflow: hidden;
  }
  .log-panel:last-child { border-bottom: none; }

  .log-header {
    padding: 4px 10px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    background: #111;
    flex-shrink: 0;
  }
  .log-header.stm32-tx  { color: #4fc3f7; border-left: 3px solid #4fc3f7; }
  .log-header.stm32-rx  { color: #81c784; border-left: 3px solid #81c784; }
  .log-header.node      { color: #ce93d8; border-left: 3px solid #ce93d8; }

  .log-body {
    flex: 1;
    overflow-y: auto;
    padding: 4px 8px;
    font-family: 'Cascadia Code', 'Consolas', 'Courier New', monospace;
    font-size: 11.5px;
    line-height: 1.5;
    background: #0d0d0d;
  }

  .log-entry { display: flex; gap: 8px; }
  .log-ts    { color: #555; flex-shrink: 0; }
  .log-msg   { color: #ccc; word-break: break-all; }

  /* Right column */
  #right-col {
    display: flex;
    flex-direction: column;
    flex: 1;
    overflow: hidden;
  }

  /* Camera placeholder */
  #camera-panel {
    flex: 1.4;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #111;
    border-bottom: 1px solid #2a2a2a;
    color: #333;
    font-size: 13px;
    letter-spacing: 0.05em;
  }

  /* Teleop panel */
  #teleop-panel {
    flex: 1;
    padding: 14px 18px;
    background: #0d0d0d;
    overflow-y: auto;
  }

  #teleop-panel h3 {
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: #888;
    margin-bottom: 12px;
  }

  #key-grid {
    display: grid;
    grid-template-columns: repeat(3, 40px);
    grid-template-rows: repeat(3, 36px);
    gap: 4px;
    margin-bottom: 14px;
  }

  .key {
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid #333;
    border-radius: 4px;
    font-size: 12px;
    font-weight: 600;
    color: #aaa;
    background: #1a1a1a;
    transition: background 0.08s, color 0.08s, border-color 0.08s;
    user-select: none;
  }
  .key.active {
    background: #1a3a5c;
    border-color: #4fc3f7;
    color: #fff;
  }
  .key.empty { border: none; background: transparent; }

  #space-key {
    grid-column: span 3;
    width: 100%;
    height: 32px;
    margin-top: 4px;
    display: flex;
    align-items: center;
    justify-content: center;
    border: 1px solid #333;
    border-radius: 4px;
    font-size: 11px;
    font-weight: 600;
    color: #aaa;
    background: #1a1a1a;
    transition: background 0.08s, color 0.08s;
    user-select: none;
  }
  #space-key.active { background: #1a3a5c; border-color: #4fc3f7; color: #fff; }

  #teleop-legend {
    font-size: 11px;
    color: #555;
    line-height: 1.7;
    font-family: monospace;
  }

  #teleop-disabled-overlay {
    display: none;
    margin-top: 8px;
    padding: 6px 10px;
    background: #3a0000;
    border: 1px solid #c0392b;
    border-radius: 4px;
    color: #ef5350;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.04em;
  }
</style>
</head>
<body>

<!-- -------------------------------------------------------------------- -->
<!-- Top bar                                                               -->
<!-- -------------------------------------------------------------------- -->
<div id="topbar">
  <h1>LitterBot Command Center</h1>
  <div id="topbar-right">
    <span id="conn-status" class="disconnected">DISCONNECTED</span>
    <button id="estop-btn" onclick="toggleEstop()">STOP</button>
  </div>
</div>

<!-- -------------------------------------------------------------------- -->
<!-- Main                                                                  -->
<!-- -------------------------------------------------------------------- -->
<div id="main">

  <!-- Log panels -->
  <div id="log-col">

    <div class="log-panel" id="panel-stm32-tx">
      <div class="log-header stm32-tx">Jetson to STM32</div>
      <div class="log-body" id="log-stm32-tx"></div>
    </div>

    <div class="log-panel" id="panel-stm32-rx">
      <div class="log-header stm32-rx">STM32 to Jetson</div>
      <div class="log-body" id="log-stm32-rx"></div>
    </div>

    <div class="log-panel" id="panel-node">
      <div class="log-header node">Node to Node</div>
      <div class="log-body" id="log-node"></div>
    </div>

  </div>

  <!-- Right column -->
  <div id="right-col">

    <div id="camera-panel">
      [ Camera — not yet connected ]
    </div>

    <div id="teleop-panel">
      <h3>Keyboard Teleop</h3>

      <!-- WASD key grid: Q _ E / A S D -->
      <div id="key-grid">
        <div class="key" id="key-q">Q</div>
        <div class="key empty"></div>
        <div class="key" id="key-e">E</div>
        <div class="key empty"></div>
        <div class="key" id="key-w">W</div>
        <div class="key empty"></div>
        <div class="key" id="key-a">A</div>
        <div class="key" id="key-s">S</div>
        <div class="key" id="key-d">D</div>
      </div>
      <div id="space-key">SPACE — Stop</div>

      <div id="teleop-legend">
        W / S &nbsp;— forward / back<br>
        A / D &nbsp;— rotate left / right<br>
        Q / E &nbsp;— strafe left / right
      </div>

      <div id="teleop-disabled-overlay">TELEOP DISABLED — E-STOP ACTIVE</div>
    </div>

  </div>
</div>

<script>
// ========================================================================
// Constants
// ========================================================================
const MAX_LOG = 100;       // Max entries per panel
const TELEOP_HZ = 10;      // Command publish rate while keys held (Hz)
const TELEOP_SPEED = {     // Body velocity magnitudes
  vx: 0.3,   // m/s  forward / back
  vy: 0.3,   // m/s  strafe
  wz: 0.5,   // rad/s rotate
};

// ========================================================================
// State
// ========================================================================
let ws = null;
let estopActive = false;
let teleopInterval = null;

// Keys currently held: map from key → {vx, vy, wz} contribution
const KEY_MAP = {
  'w': { vx:  TELEOP_SPEED.vx, vy: 0,               wz: 0               },
  's': { vx: -TELEOP_SPEED.vx, vy: 0,               wz: 0               },
  'a': { vx: 0,                vy: 0,                wz:  TELEOP_SPEED.wz },
  'd': { vx: 0,                vy: 0,                wz: -TELEOP_SPEED.wz },
  'q': { vx: 0,                vy:  TELEOP_SPEED.vy, wz: 0               },
  'e': { vx: 0,                vy: -TELEOP_SPEED.vy, wz: 0               },
};
const heldKeys = new Set();

// ========================================================================
// WebSocket connection
// ========================================================================
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const url   = `${proto}://${location.host}/ws`;

  ws = new WebSocket(url);

  ws.onopen = () => {
    setConnStatus(true);
    startTeleopLoop();
  };

  ws.onclose = () => {
    setConnStatus(false);
    stopTeleopLoop();
    // Reconnect after 2 seconds
    setTimeout(connect, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      if (data.type === 'log') {
        appendLog(data.channel, data.ts, data.msg);
      }
    } catch (_) { /* ignore malformed */ }
  };
}

// ========================================================================
// Connection status
// ========================================================================
function setConnStatus(connected) {
  const el = document.getElementById('conn-status');
  el.textContent   = connected ? 'CONNECTED' : 'DISCONNECTED';
  el.className     = connected ? 'connected'  : 'disconnected';
}

// ========================================================================
// Log panel helpers
// ========================================================================
const CHANNEL_TO_ID = {
  stm32_tx: 'log-stm32-tx',
  stm32_rx: 'log-stm32-rx',
  node:     'log-node',
};

function appendLog(channel, ts, msg) {
  const bodyId = CHANNEL_TO_ID[channel];
  if (!bodyId) return;

  const body = document.getElementById(bodyId);
  const entry = document.createElement('div');
  entry.className = 'log-entry';

  const tsEl  = document.createElement('span');
  tsEl.className = 'log-ts';
  tsEl.textContent = new Date(ts * 1000).toLocaleTimeString('en-US', { hour12: false });

  const msgEl = document.createElement('span');
  msgEl.className = 'log-msg';
  msgEl.textContent = msg;

  entry.appendChild(tsEl);
  entry.appendChild(msgEl);
  body.appendChild(entry);

  // Trim to MAX_LOG
  while (body.childElementCount > MAX_LOG) {
    body.removeChild(body.firstElementChild);
  }

  // Auto-scroll to bottom
  body.scrollTop = body.scrollHeight;
}

// ========================================================================
// Emergency stop
// ========================================================================
function toggleEstop() {
  estopActive = !estopActive;
  applyEstopState();
}

function applyEstopState() {
  const btn     = document.getElementById('estop-btn');
  const overlay = document.getElementById('teleop-disabled-overlay');

  if (estopActive) {
    btn.textContent = 'RESUME';
    btn.classList.add('active');
    overlay.style.display = 'block';
    heldKeys.clear();
    updateKeyDisplay();
  } else {
    btn.textContent = 'STOP';
    btn.classList.remove('active');
    overlay.style.display = 'none';
  }

  sendEstop(estopActive);
}

function sendEstop(active) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'estop', active }));
  }
}

// ========================================================================
// Keyboard teleop
// ========================================================================
document.addEventListener('keydown', (e) => {
  if (estopActive) return;
  const k = e.key.toLowerCase();

  if (k === ' ') {
    e.preventDefault();
    // Immediate stop — clear all keys and send zero command
    heldKeys.clear();
    updateKeyDisplay();
    sendCmdVel(0, 0, 0);
    return;
  }

  if (KEY_MAP[k] && !heldKeys.has(k)) {
    heldKeys.add(k);
    updateKeyDisplay();
  }
});

document.addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  if (heldKeys.delete(k)) {
    updateKeyDisplay();
  }
});

function computeCmdVel() {
  let vx = 0, vy = 0, wz = 0;
  for (const k of heldKeys) {
    const contrib = KEY_MAP[k];
    if (contrib) { vx += contrib.vx; vy += contrib.vy; wz += contrib.wz; }
  }
  // Clamp
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  return {
    vx: clamp(vx, -TELEOP_SPEED.vx, TELEOP_SPEED.vx),
    vy: clamp(vy, -TELEOP_SPEED.vy, TELEOP_SPEED.vy),
    wz: clamp(wz, -TELEOP_SPEED.wz, TELEOP_SPEED.wz),
  };
}

function sendCmdVel(vx, vy, wz) {
  if (ws && ws.readyState === WebSocket.OPEN && !estopActive) {
    ws.send(JSON.stringify({ type: 'cmd_vel', vx, vy, wz }));
  }
}

function startTeleopLoop() {
  stopTeleopLoop();
  teleopInterval = setInterval(() => {
    if (heldKeys.size > 0 && !estopActive) {
      const { vx, vy, wz } = computeCmdVel();
      sendCmdVel(vx, vy, wz);
    }
  }, 1000 / TELEOP_HZ);
}

function stopTeleopLoop() {
  if (teleopInterval !== null) {
    clearInterval(teleopInterval);
    teleopInterval = null;
  }
}

function updateKeyDisplay() {
  const ids = { q:'key-q', w:'key-w', e:'key-e', a:'key-a', s:'key-s', d:'key-d' };
  for (const [k, id] of Object.entries(ids)) {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('active', heldKeys.has(k));
  }
}

// ========================================================================
// Boot
// ========================================================================
connect();
</script>
</body>
</html>
```

- [ ] **Step 2: Verify the file exists**

```bash
ls -la /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/ros2/litterbot_bridge/litterbot_bridge/ui/
```

Expected: `index.html` listed

- [ ] **Step 3: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add ros2/litterbot_bridge/litterbot_bridge/ui/index.html
git commit -m "feat: add command center frontend (index.html)

Dark-themed single-page app with three log panels (Jetson->STM32,
STM32->Jetson, Node<->Node), WASD keyboard teleop with 10 Hz publish
loop, emergency stop button with teleop lockout, camera placeholder,
and auto-reconnecting WebSocket client."
```

---

## Task 7: Wire the HTML into setup.py data_files

**Files:**
- Modify: `ros2/litterbot_bridge/setup.py`

The HTML file must be included in the installed package so `ros2 run` can find it.

- [ ] **Step 1: Add ui/index.html to data_files in setup.py**

In `setup.py`, update `data_files` to:

```python
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
```

**Wait** — the HTML is loaded by `ui_server.py` using `Path(__file__).parent / 'ui' / 'index.html'`, which resolves relative to the installed Python package location. For an `ament_python` package, `find_packages()` installs the `litterbot_bridge/` directory including subdirectories. Check that `ui/` is picked up.

Verify by checking that `find_packages()` will include `litterbot_bridge.ui` — it won't because `ui/` has no `__init__.py`. The HTML must be included via `package_data` instead.

Update `setup.py`:

```python
    packages=find_packages(exclude=['test']),
    package_data={
        'litterbot_bridge': ['ui/index.html'],
    },
```

- [ ] **Step 2: Verify path resolves correctly after build**

```bash
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litterbot_bridge 2>&1 | tail -3
source install/setup.bash
python3 -c "
from litterbot_bridge import ui_server
import pathlib
p = pathlib.Path(ui_server.__file__).parent / 'ui' / 'index.html'
assert p.exists(), f'index.html NOT found at {p}'
print(f'OK — found at {p}')
"
```

Expected: `OK — found at .../litterbot_bridge/ui/index.html`

- [ ] **Step 3: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add ros2/litterbot_bridge/setup.py
git commit -m "fix: include ui/index.html in package_data so ui_server can load it"
```

---

## Task 8: Build and smoke test

- [ ] **Step 1: colcon build**

```bash
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litterbot_bridge 2>&1
```

Expected: `Finished <<< litterbot_bridge` with no errors.

- [ ] **Step 2: Source the install and verify both nodes are registered**

```bash
source ~/litterbot_ws/install/setup.bash
ros2 pkg executables litterbot_bridge
```

Expected:
```
litterbot_bridge stm32_bridge
litterbot_bridge ui_server
```

- [ ] **Step 3: Run stm32_bridge in mock mode (Terminal 1)**

```bash
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash
ros2 run litterbot_bridge stm32_bridge --ros-args --log-level debug
```

Expected: `stm32_bridge started — port=/dev/ttyUSB0  baud=115200  mock=True`

- [ ] **Step 4: Run ui_server (Terminal 2)**

```bash
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash
ros2 run litterbot_bridge ui_server
```

Expected: `ui_server ready — http://0.0.0.0:8765/`

- [ ] **Step 5: Open browser and verify UI loads**

Open `http://localhost:8765` in Windows browser.

Expected: Command center UI renders with three log panels, camera placeholder, key grid, STOP button. Status shows DISCONNECTED briefly then CONNECTED.

- [ ] **Step 6: Inject a test /cmd_vel (Terminal 3)**

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Expected:
- stm32_bridge terminal logs: `[MOCK SERIAL TX] M <some> <values>`
- "Jetson to STM32" panel in browser shows the M command
- "Node to Node" panel shows `/cmd_vel vx=0.30 vy=0.00 wz=0.50`

- [ ] **Step 7: Test e-stop button**

Click STOP in the browser. Expected: button changes to RESUME, teleop overlay appears.

Run another `ros2 topic pub /cmd_vel` — the stm32_bridge should NOT log any motor command (e-stop blocks it). The "Jetson to STM32" panel should show `X` (the stop frame).

- [ ] **Step 8: Commit smoke test confirmation**

No code change — just a commit message to mark the milestone:

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git commit --allow-empty -m "chore: smoke test passed — ui_server + stm32_bridge working end-to-end"
```

---

## Task 9: Update documentation

**Files:**
- Modify: `Litterbot_Obsidian/Technical Documentation/ROS2 Development.md`

- [ ] **Step 1: Update node status table**

Mark `ui_server` as done in the Planned Nodes table:

| `ui_server` | `litterbot_bridge` | Sub: `/cmd_vel`, `/stm32_tx`, `/encoder_raw`, `/odom`; Pub: `/cmd_vel`, `/emergency_stop` | **Done** |

- [ ] **Step 2: Update Known Bugs table**

Mark the following three rows as fixed (change status column or remove the row):
- Bug 1: "Motor percent values sent as -100..+100 but firmware expects -1000..+1000"
- Bug 2: "E-stop sends M 0 0 0 0 instead of X\n"
- Bug 3: "No /stm32_tx topic published"

- [ ] **Step 3: Update Node Development Log**

Add entries:
```
| 2026-03-24 | `stm32_bridge` | Fixed speed scaling bug (×10): now sends [-1000,1000] to firmware. Added /stm32_tx publisher and /emergency_stop subscriber. E-stop sends X\n. |
| 2026-03-24 | `ui_server` | New node: FastAPI + WebSocket command center bridge. Serves index.html at :8765. Three log panels, WASD teleop at 10Hz, e-stop with lockout. |
```

- [ ] **Step 4: Commit**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git add "Litterbot_Obsidian/Technical Documentation/ROS2 Development.md"
git commit -m "docs: update ROS2 dev log — command center UI complete"
```

---

## Task 10: Final cleanup commit

- [ ] **Step 1: Verify test suite still passes**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/ros2/litterbot_bridge
python3 -m pytest test/test_kinematics.py -v
```

Expected: 4 passed.

- [ ] **Step 2: Check git log is clean**

```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
git log --oneline -10
```

Verify all the commits from this plan are present in order.

- [ ] **Done.** The command center UI is fully implemented.

---

## Quick-Start Reference (after implementation)

```bash
# WSL2 — Terminal 1
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash
ros2 run litterbot_bridge stm32_bridge --ros-args --log-level debug

# WSL2 — Terminal 2
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash
ros2 run litterbot_bridge ui_server

# Windows browser
http://localhost:8765
```
