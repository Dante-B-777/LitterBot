# ROS2 Subsystem — Agent Brief

> Read `../../CLAUDE.md` first for full project context. This file covers ROS2 specifics.

---

## Current State (update every session)

**Last updated: 2026-04-01**

| Node | File | Status |
|------|------|--------|
| `stm32_bridge` | `litterbot_bridge/stm32_bridge.py` | **Done** — deploy/dev mode, serial read thread, S SCOOP protocol, state query |
| `ui_server` | `litterbot_bridge/ui_server.py` | **Done** — pane-level deploy overlay (teleop panel only), dev mode gating (overlay hidden in dev mode), 3s deploy hold, LiDAR status dot placeholder |
| Launch file | `launch/litterbot.launch.py` | **Done** — starts both nodes with one command |
| `odom_publisher` | not created yet | Not started |
| `camera_node` | not created yet | Not started |
| `litter_picker` | not created yet | Not started |

---

## How to Build and Run

```bash
# WSL2 — from repo root
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32

# First time only — symlink package into workspace
ln -sf $(pwd)/ros2/litterbot_bridge ~/litterbot_ws/src/litterbot_bridge

# Install pip deps (first time only)
pip install -r ros2/litterbot_bridge/requirements.txt

# Build
cd ~/litterbot_ws && colcon build && source install/setup.bash

# Run everything with one command (recommended)
ros2 launch litterbot_bridge litterbot.launch.py

# Real hardware mode
ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyUSB0

# Or run nodes individually (two terminals)
ros2 run litterbot_bridge stm32_bridge
ros2 run litterbot_bridge ui_server

# Open browser — WSL2 dev:
#   http://localhost:8765
# On Jetson (from laptop):
#   http://<jetson-ip>:8765

# Run tests
cd ros2/litterbot_bridge && python3 -m pytest test/ -v
```

---

## Package Structure

```
ros2/
├── CLAUDE.md                          ← YOU ARE HERE
└── litterbot_bridge/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── requirements.txt               ← fastapi, uvicorn[standard]
    ├── resource/litterbot_bridge
    ├── launch/
    │   └── litterbot.launch.py        ← launch all nodes with one command
    ├── test/
    │   ├── conftest.py                ← shared pytest options (--serial-port)
    │   ├── test_kinematics.py         ← 4 unit tests: wheel speed math
    │   ├── test_stm32_bridge_logic.py ← 8 unit tests: frame format + kinematics
    │   ├── test_stm32_bridge_node.py  ← 7 node tests: e-stop state machine
    │   └── test_firmware_spec.py      ← 19 hardware tests (skipped w/o --serial-port)
    └── litterbot_bridge/
        ├── __init__.py
        ├── stm32_bridge.py            ← bridge node (mock + real serial modes)
        ├── ui_server.py               ← FastAPI+WebSocket command center node
        └── ui/
            └── index.html             ← self-contained frontend
```

---

## Key Design Decisions

- All nodes are **Python** for Jetson portability
- `stm32_bridge` has a `mock_serial` parameter (default `true`) — safe to run without hardware
- `ui_server` runs as a ROS2 node (not a standalone server) — `rclpy.spin()` in daemon thread, uvicorn on main asyncio loop
- Motor speed values: **-1000 to +1000** (firmware divides by 10 internally). Earlier code used -100 to +100 — caused 1/10 speed. Fixed 2026-03-24.
- E-stop sends `X\n` to STM32 (not `M 0 0 0 0`) — hardware stop
- Pause sends `M 0 0 0 0\n` (not `X\n`) — motors zeroed but STM32 watchdog stays satisfied; resumable
- Scoop sends `S SCOOP\n` to STM32 — multi-word command prevents noise trigger (placeholder values in scoop_sequence.h)
- Teleop base speed: `vx=0.5 m/s`, `vy=0.5 m/s`, `wz=0.8 rad/s` (scalable via UI speed slider 10–100%)
- UI server binds to `0.0.0.0` — accessible from laptop browser at `http://<jetson-ip>:8765`
- SPACE bar always triggers e-stop regardless of mode
- **Deploy/Standby system**: STM32 owns the state machine (STANDBY/DEPLOYING/ACTIVE). Robot is inert on boot until 5-second deploy press-and-hold in UI. STM32 gates M/S/C commands in STANDBY. Dev mode bypasses gate.
- `stm32_bridge` has a serial read thread that parses STM32 responses (STATE, DEPLOY, ERR:STANDBY) and publishes to `/stm32_state` and `/stm32_rx`
- On serial connect, bridge sends `Q\n` to query current STM32 state
- UI sends state on WebSocket connect so browser immediately knows deploy status

---

## Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `ui_server` | `stm32_bridge` |
| `/stm32_tx` | `std_msgs/String` | `stm32_bridge` | `ui_server` |
| `/stm32_rx` | `std_msgs/String` | `stm32_bridge` | `ui_server` |
| `/stm32_state` | `std_msgs/String` | `stm32_bridge` | `ui_server` |
| `/encoder_raw` | `std_msgs/Int32MultiArray` | `stm32_bridge` (future) | `ui_server`, `odom_publisher` |
| `/odom` | `nav_msgs/Odometry` | `odom_publisher` (future) | `ui_server` |
| `/emergency_stop` | `std_msgs/Bool` | `ui_server` | `stm32_bridge` |
| `/pause_cmd` | `std_msgs/Bool` | `ui_server` | `stm32_bridge` |
| `/scoop_cmd` | `std_msgs/String` | `ui_server` | `stm32_bridge` |
| `/deploy_cmd` | `std_msgs/String` | `ui_server` | `stm32_bridge` |
| `/dev_mode_cmd` | `std_msgs/Bool` | `ui_server` | `stm32_bridge` |

---

## Mecanum Kinematics

Wheel speed units sent to STM32 (-1000 to +1000):

```
fl = (vx - vy - (L+W)*wz) / v_max * 1000
fr = (vx + vy + (L+W)*wz) / v_max * 1000
rl = (vx + vy - (L+W)*wz) / v_max * 1000
rr = (vx - vy + (L+W)*wz) / v_max * 1000
```

L=0.168m, W=0.2025m, L+W=0.3705m

---

## Detailed Docs

See `../Litterbot_Obsidian/Technical Documentation/ROS2 Development.md` for:
- Full kinematics math with LaTeX
- Odometry equations
- Complete UI design spec
- Bug log
- Run instructions
