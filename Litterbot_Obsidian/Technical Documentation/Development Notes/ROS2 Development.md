# ROS2 Development — LitterBot

## Purpose

This document tracks the ROS2 software stack development for LitterBot. The ROS2 layer runs on an **NVIDIA Jetson** (final deployment) and is developed on **WSL2 Ubuntu 22.04** during the build phase. It handles high-level robot logic: teleoperation, odometry, computer vision, and autonomous navigation.

---

## Documentation Structure

This project uses a three-layer documentation system so any AI agent or developer can pick up the project at any point:

| Layer | Files | Purpose | Update Frequency |
|-------|-------|---------|-----------------|
| **Agent briefs** | `CLAUDE.md` files (repo root + each subsystem) | Machine-readable context, auto-loaded by Claude Code. Current status, architecture, key decisions. | Every session |
| **Dev logs** | `Litterbot_Obsidian/Technical Documentation/*.md` | Detailed technical docs: protocols, test procedures, node specs, bugs, math. This file. | After each build/test |
| **Reference** | `docs/`, `LB_FIRM/`, `Report/` | Datasheets, logbooks, original firmware, course deliverables. | Rarely |

**File map:**

```
CLAUDE.md                              ← Master brief (repo root) — read first
ros2/CLAUDE.md                         ← ROS2 subsystem brief
firmware/stm32/LitterBot/CLAUDE.md     ← STM32 subsystem brief
Litterbot_Obsidian/Technical Documentation/
    ROS2 Development.md                ← THIS FILE — ROS2 deep detail
    STM32 System Testing.md            ← STM32 test procedures
```

**Rule:** Any agent starting a new session should read `CLAUDE.md` → subsystem `CLAUDE.md` → this file if working on ROS2.

---

## Development Environment

| Item | Value |
|------|-------|
| Dev OS | WSL2 — Ubuntu 22.04.5 LTS (Jammy) |
| ROS2 Distro | Humble Hawksbill (LTS) |
| Target Platform | NVIDIA Jetson (JetPack 5/6) |
| Workspace | `~/litterbot_ws/` (WSL) |
| Git Repo (Windows) | `C:\Users\benji\Documents\GitHub\LitterBot_STM32\` |
| Git Repo (WSL) | `/mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/` |

### Two-Environment Workflow

```
Windows Claude Code  ──►  STM32 firmware (CubeIDE, .c/.h files, flashing)
WSL2 Claude Code     ──►  ROS2 nodes (Python, colcon, ros2 run/launch)
```

Both environments share the same git repository. ROS2 packages live under `ros2/` in the repo root and are symlinked or copied into `~/litterbot_ws/src/` for building.

---

## Environment Setup

### 1. Install ROS2 Humble (WSL2)

```bash
# Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 apt repo
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

### 2. Create Workspace

```bash
mkdir -p ~/litterbot_ws/src
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 3. Install Claude Code in WSL

```bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt install -y nodejs
npm install -g @anthropic/claude-code
```

### 4. Source ROS2 on Every Session (add to ~/.bashrc)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/litterbot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## System Architecture

```
┌─────────────────────────────────────────────────┐
│                   Jetson / WSL2                  │
│                                                  │
│  /cmd_vel  ──► teleop_twist ──► stm32_bridge     │
│                                      │           │
│                                   UART serial    │
│                                      │           │
│                                   STM32 MCU      │
│                                      │           │
│  /odom  ◄── odom_publisher ◄── encoder ticks     │
│                                                  │
│  /image_raw ──► camera_node ──► /detections      │
│                                      │           │
│                              litter_picker        │
│                            (behavior controller) │
└─────────────────────────────────────────────────┘
```

---

## Node Status

| Node | Package | Status |
|------|---------|--------|
| `stm32_bridge` | `litterbot_bridge` | **Done** — mock + real serial, mecanum kinematics, e-stop, pause, scoop |
| `ui_server` | `litterbot_bridge` | **Done** — FastAPI+WebSocket, full command center UI |
| `odom_publisher` | `litterbot_bridge` | Not started |
| `camera_node` | `litterbot_vision` | Not started |
| `litter_picker` | `litterbot_bringup` | Not started |

---

## STM32 Serial Protocol

The `stm32_bridge` node communicates with the STM32 over UART (LPUART1, 115200 baud, 8N1).

### Commands (Jetson → STM32)

| Command | Format | Gate | Description |
|---------|--------|------|-------------|
| Motor | `M <fl> <fr> <rl> <rr>\n` | GATED | Set 4 wheel speeds, -1000 to +1000. Firmware divides by 10 → % PWM. |
| E-stop | `X\n` | ALWAYS | Immediate hardware stop. |
| Pause | `M 0 0 0 0\n` | GATED | Zeros motors but keeps watchdog satisfied. Resumable. |
| Scoop | `S SCOOP\n` | GATED | Runs full 6-phase scoop sequence (placeholder values). |
| Calibrate | `C CALIBRATE\n` | GATED | Arm calibration with B1 e-stop. |
| Deploy | `D DEPLOY\n` | STANDBY/ACTIVE | Arm homing, transitions to ACTIVE state. |
| Dev mode | `DEV ON\n` / `DEV OFF\n` | ALWAYS | Toggle dev mode (bypasses gate). |
| State query | `Q\n` | ALWAYS | Responds `STATE <state> [DEV]\r\n`. |
| Encoder test | `T\n` | ALWAYS | Prints raw encoder counts via LPUART1 (debug only). |

GATED commands are blocked in STANDBY unless dev mode is enabled. See [[Deploy and Standby System]].

Example: `M 500 500 500 500\n` → all wheels forward at 50% PWM.

> **Note:** Earlier code incorrectly used the range $-100$ to $100$, causing 1/10 speed. Fixed 2026-03-24.

### Telemetry (STM32 → Jetson)

> Not yet implemented on STM32 side. Planned: encoder tick counts at ~50 Hz.

---

## Mecanum Wheel Kinematics

Given a desired body velocity $(v_x, v_y, \omega_z)$, individual wheel speeds are:

$$\omega_{FL} = \frac{1}{r}\left(v_x - v_y - (L+W)\omega_z\right)$$

$$\omega_{FR} = \frac{1}{r}\left(v_x + v_y + (L+W)\omega_z\right)$$

$$\omega_{RL} = \frac{1}{r}\left(v_x + v_y - (L+W)\omega_z\right)$$

$$\omega_{RR} = \frac{1}{r}\left(v_x - v_y + (L+W)\omega_z\right)$$

| Parameter | Value |
|-----------|-------|
| Wheel radius $r$ | 52 mm = 0.052 m |
| Wheelbase $L$ | 168 mm = 0.168 m |
| Track width $W$ | 202.5 mm = 0.2025 m |
| $L + W$ | 0.3705 m |

---

## Odometry

Dead-reckoning pose estimate $(x, y, \theta)$ from encoder ticks:

$$\Delta x = \Delta d \cos\theta, \quad \Delta y = \Delta d \sin\theta$$

$$\Delta\theta = \frac{r}{2(L+W)}\left(\omega_{FR} + \omega_{RR} - \omega_{FL} - \omega_{RL}\right)$$

Encoder specs:
- **537.7 ticks/rev** (28 CPR × 19.2:1 gear ratio × 4x quadrature counting)
- Distance per tick: $\frac{2\pi r}{537.7} = \frac{2\pi \times 0.052}{537.7} \approx 6.076 \times 10^{-4}$ m/tick

---

## Running the Stack

### Recommended — launch file (starts both nodes)

```bash
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash

# Mock mode (no hardware — default)
ros2 launch litterbot_bridge litterbot.launch.py

# Real hardware
ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyUSB0

# Browser — WSL2 dev:
#   http://localhost:8765
# On Jetson (from laptop):
#   http://<jetson-ip>:8765
# Field (Jetson hotspot):
#   http://10.42.0.1:8765
```

### Manual (two terminals)

```bash
# Terminal 1
ros2 run litterbot_bridge stm32_bridge

# Terminal 2
ros2 run litterbot_bridge ui_server
```

### Debug — inject a test cmd_vel

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

> **Note (WSL2 + real hardware):** Pass the USB device through to WSL before connecting:
> `usbipd attach --wsl --busid <id>` on the Windows side.

---

## Package Structure

```
ros2/
├── CLAUDE.md                              ← ROS2 agent brief (read this for quick context)
└── litterbot_bridge/                      ← symlinked to ~/litterbot_ws/src/
    ├── package.xml
    ├── setup.py                           ← entry points for stm32_bridge + ui_server
    ├── setup.cfg
    ├── requirements.txt                   ← fastapi, uvicorn[standard], pyserial
    ├── resource/litterbot_bridge          ← ament resource index marker
    ├── launch/
    │   └── litterbot.launch.py            ← starts both nodes with one command
    ├── test/
    │   ├── conftest.py                    ← shared pytest options (--serial-port flag)
    │   ├── test_kinematics.py             ← 4 unit tests: wheel speed math
    │   ├── test_stm32_bridge_logic.py     ← 8 unit tests: frame format + kinematics
    │   ├── test_stm32_bridge_node.py      ← 7 node tests: e-stop state machine
    │   └── test_firmware_spec.py          ← 19 hardware tests (skipped without --serial-port)
    └── litterbot_bridge/
        ├── __init__.py
        ├── stm32_bridge.py                ← bridge node (mock + real serial)
        ├── ui_server.py                   ← FastAPI + WebSocket ROS2 node
        └── ui/
            └── index.html                 ← self-contained command center frontend
```

**Test counts:** 19 passing (ROS2 unit/node tests), 19 skipped (firmware hardware tests — require `--serial-port /dev/ttyXXX`).

Run tests:
```bash
cd ros2/litterbot_bridge && python3 -m pytest test/ -v
```

---

## Known Bugs / Pending Fixes

All previously known bugs resolved as of 2026-03-24.

| # | File | Issue | Status |
|---|------|-------|--------|
| 1 | `stm32_bridge.py` | Motor values sent as `-100..+100`; firmware expects `-1000..+1000`. Robot ran at 1/10 speed. | **Fixed** — `_twist_to_wheel_units()` now scales ×1000, clamps `[-1000, 1000]`. Unit tests added. |
| 2 | `stm32_bridge.py` | E-stop sent `M 0 0 0 0` instead of `X\n`. | **Fixed** — `destroy_node()` and `/emergency_stop` callback both send `X\n`. |
| 3 | `stm32_bridge.py` | No `/stm32_tx` topic — UI couldn't observe what was sent to STM32. | **Fixed** — `/stm32_tx` `std_msgs/String` publisher added; publishes raw frame on every motor command. |

---

## Command Center UI

Implemented as `ui_server` node in `litterbot_bridge` package. Full design spec at `docs/superpowers/specs/2026-03-24-command-center-ui-design.md`.

### Architecture
- **Backend**: `ui_server.py` — FastAPI + WebSocket ROS2 node. `rclpy.spin()` runs in daemon thread; uvicorn runs on main asyncio loop. `call_soon_threadsafe` bridges ROS2 callbacks into asyncio.
- **Frontend**: `ui/index.html` — self-contained HTML/JS. Open `http://<robot-ip>:8765` in any browser, no installs required.
- **Transport**: WebSocket at `ws://host:8765/ws`. Server pushes JSON log entries; browser sends teleop/e-stop JSON.

### Layout
- Left column: three stacked log panels — Jetson→STM32 (blue), STM32→Jetson (green), Node↔Node (purple)
- Right column: camera placeholder (top), keyboard teleop panel (bottom)
- Header: status bar with 6 indicator dots (WebSocket, E-stop, Pause/Running, STM32 activity, Bridge, Odom/Camera)

### Controls
| Control | What it does |
|---------|-------------|
| **W/S** | Forward / backward (`vx = ±0.5 m/s`) |
| **A/D** | Rotate left / right (`wz = ±0.8 rad/s`) |
| **Q/E** | Strafe left / right (`vy = ±0.5 m/s`, mecanum only) |
| **SPACE** | Emergency stop — always active regardless of mode |
| **F** | Trigger scoop sequence (sends `S SCOOP\n` to STM32) |
| Speed slider | 10–100%, default 60% — scales all BASE_SPEED values |
| PAUSE button | Zeros motors but keeps watchdog alive. Resumable (click RESUME). |
| AUTONOMY button | Disables keyboard teleop when Nav2 is driving |

- Keys publish `/cmd_vel` at **10 Hz** while held (required — STM32 watchdog is 250 ms)
- Multiple keys sum together

### Three Motion States

```
Running ──PAUSE──► Paused ──RESUME──► Running
   │                  │
 SPACE              SPACE
   │                  │
   ▼                  ▼
E-stopped ◄──────────────── (clears pause too)
 (RESUME to recover)
```

- **Running**: keyboard/autonomy sends `/cmd_vel` normally
- **Paused**: `stm32_bridge` sends `M 0 0 0 0\n` to keep watchdog satisfied; `/cmd_vel` blocked
- **E-stopped**: `stm32_bridge` sends `X\n`; all commands blocked; STM32 halts

### Topics
| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `ui_server` | `stm32_bridge` |
| `/stm32_tx` | `std_msgs/String` | `stm32_bridge` | `ui_server` |
| `/emergency_stop` | `std_msgs/Bool` | `ui_server` | `stm32_bridge` |
| `/pause_cmd` | `std_msgs/Bool` | `ui_server` | `stm32_bridge` |
| `/scoop_cmd` | `std_msgs/String` | `ui_server` | `stm32_bridge` |
| `/encoder_raw` | `std_msgs/Int32MultiArray` | `stm32_bridge` (future) | `ui_server` |
| `/odom` | `nav_msgs/Odometry` | `odom_publisher` (future) | `ui_server` |

### pip Dependencies
FastAPI and uvicorn are not in the ROS2 apt repos. Install once in WSL:
```bash
pip install -r ros2/litterbot_bridge/requirements.txt
```
If `pip` is not installed: `curl -sS https://bootstrap.pypa.io/get-pip.py | python3`

---

## Node Development Log

| Date | Node | What Was Done |
|------|------|---------------|
| 2026-03-24 | `stm32_bridge` | Created package `litterbot_bridge`, implemented mecanum kinematics, mock serial stub, `/cmd_vel` sub, `/encoder_raw` pub stub. Clean `colcon build`. |
| 2026-03-24 | `stm32_bridge` | Fixed speed scaling bug (×100→×1000). Added `/stm32_tx` publisher. Added `/emergency_stop` subscriber with `X\n` e-stop. Added 4-test kinematics test suite. |
| 2026-03-24 | `ui_server` | Created FastAPI+WebSocket ROS2 node. Three log panels (stm32_tx/rx/node). WASD+Q/E keyboard teleop at 10 Hz. E-stop button with lock/resume. Auto-reconnect WebSocket. Self-contained `ui/index.html`. |
| 2026-03-25 | `stm32_bridge` | Added `/pause_cmd` subscriber — `True` sends `M 0 0 0 0\n` (watchdog-safe pause), `False` resumes. Added `/scoop_cmd` subscriber — sends `S\n`. Pause blocks `/cmd_vel`; e-stop blocks both. |
| 2026-03-25 | `ui_server` | Added PAUSE button, AUTONOMY toggle, speed slider (10–100%, default 60%, base 0.5 m/s). SPACE changed to always trigger e-stop. F key triggers scoop. Status bar with 6 indicator dots. |
| 2026-03-25 | Launch | Added `launch/litterbot.launch.py` — starts both nodes with one command, supports `mock_serial`/`serial_port`/`ui_port` args. |
| 2026-03-25 | Tests | Added `test_stm32_bridge_logic.py` (8 tests), `test_stm32_bridge_node.py` (7 tests), `test_firmware_spec.py` (19 hardware tests). Total: 19 passing, 19 skipped. |

---

## Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| `ros2: command not found` | ROS2 not sourced — run `source /opt/ros/humble/setup.bash` |
| `ros2 run` says package not found | Workspace not sourced — also run `source ~/litterbot_ws/install/setup.bash` |
| `colcon build` fails on import | Missing `python3-colcon-common-extensions` |
| `import fastapi` fails at ui_server start | Run `pip install -r ros2/litterbot_bridge/requirements.txt` |
| `AttributeError: can't set attribute 'clients'` | rclpy Node has a `clients` property — use `_ws_clients` instead (already fixed) |
| No serial data from STM32 | LPUART1 word length still 9B in CubeMX (needs fix to 8B) |
| WSL can't see serial port | Pass USB device to WSL via `usbipd` (Windows side) |
| Robot moves at 1/10 speed | Speed scaling bug — check `_twist_to_wheel_units` uses ×1000 not ×100 (fixed 2026-03-24) |
