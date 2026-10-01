# LitterBot Command Center UI — Design Spec
**Date:** 2026-03-24
**Status:** Approved

---

## Overview

A browser-based command center that runs on the operator's laptop and communicates with the LitterBot robot (Jetson / WSL2 dev) over WebSocket. No software installation required on the laptop — just open a URL in any browser.

---

## Architecture

**Chosen approach:** Custom FastAPI + WebSocket backend (Option B)

```
Laptop browser (HTML/JS)
        │
        │  WebSocket  ws://robot-ip:8765/ws
        │  HTTP       http://robot-ip:8765/
        │
FastAPI ui_server node  (Python, runs on Jetson / WSL2)
        │
        │  ROS2 DDS topics
        │
   ┌────┴────────────────────────────┐
   │  stm32_bridge node (existing)   │
   │  + /stm32_tx publisher (new)    │
   │  + /emergency_stop sub (new)    │
   └────┬────────────────────────────┘
        │  UART  M <fl> <fr> <rl> <rr>\n  or  X\n
        │
    STM32 MCU
```

**Why FastAPI over rosbridge:**
- Full control over which topics are exposed and how they're formatted
- Cleaner place to implement e-stop safety locking logic
- No extra ROS2 package dependency
- Plain Python — easy to read and modify alongside existing nodes

---

## Components

### 1. `ui_server` node  (`litterbot_bridge` package)

New Python node added to the existing `litterbot_bridge` package.

**Responsibilities:**
- Serve the frontend HTML at `GET /`
- Accept WebSocket connections at `ws://host:8765/ws`
- Subscribe to `/cmd_vel`, `/stm32_tx`, `/encoder_raw`, `/odom` and forward formatted log entries to all connected WebSocket clients
- Receive teleop commands from the browser and publish to `/cmd_vel`
- Receive e-stop signal from the browser and publish to `/emergency_stop`

**Threading model:**
- `rclpy` spins in a background thread
- `uvicorn` (FastAPI ASGI server) runs in the main thread
- Topic callbacks push log entries into an `asyncio.Queue` that the WebSocket handler drains

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `host` | `0.0.0.0` | Bind address |
| `port` | `8765` | HTTP + WS port |
| `log_history` | `100` | Max entries per log panel kept in memory |

**Topic subscriptions — future topics:** `ui_server` subscribes to `/encoder_raw` and `/odom` at startup. If the publisher doesn't exist yet, the subscription is harmless and simply receives nothing. This way the UI is wired and ready the moment those nodes come online.

---

### 2. Frontend (`ui/index.html`)

Single self-contained HTML file with embedded CSS and JS. Served by `ui_server` — no build step, no framework.

**Layout (Option A — approved):**
```
┌─────────────────────────────────────────────────────┐
│  LitterBot Command Center        [● CONNECTED] [STOP]│
├──────────────────────────┬──────────────────────────┤
│  Jetson → STM32          │  Camera                  │
│  (blue log panel)        │  [placeholder]            │
├──────────────────────────┤                          │
│  STM32 → Jetson          ├──────────────────────────┤
│  (green log panel)       │  Keyboard Teleop         │
├──────────────────────────┤  W=fwd  S=back           │
│  Node ↔ Node             │  A=rot-L  D=rot-R        │
│  (purple log panel)      │  Q=strafe-L  E=strafe-R  │
│                          │  SPACE=stop               │
└──────────────────────────┴──────────────────────────┘
```

- Dark theme, monospace font in log panels
- Last 100 entries per panel (oldest trimmed)
- Timestamps on every log entry
- Header: no emoji; plain text connection indicator

**WebSocket message protocol (JSON):**

Browser ← Server (log entry):
```json
{"type": "log", "channel": "stm32_tx|stm32_rx|node", "ts": 1234567890.123, "msg": "M 420 420 420 420"}
```

Browser → Server (teleop):
```json
{"type": "cmd_vel", "vx": 0.3, "vy": 0.0, "wz": 0.0}
```

Browser → Server (e-stop):
```json
{"type": "estop", "active": true}
```

---

### 3. Modifications to `stm32_bridge` node

Three changes required:

| # | Change | Detail |
|---|--------|--------|
| 1 | **Fix speed scaling bug** | Currently scales to `[-100, 100]`; firmware expects `[-1000, 1000]`. Multiply `to_percent()` output by 10 and rename range accordingly. |
| 2 | **Add `/stm32_tx` publisher** | Publish `std_msgs/String` of the raw frame each time a motor command is sent, so `ui_server` can log it. |
| 3 | **Add `/emergency_stop` subscriber** | On `True`: send `X\n` to STM32 UART, set an internal `_estop_active` flag that blocks `_cmd_vel_callback` from sending motor commands. On `False`: clear flag and resume normal operation. |

---

## Keyboard Teleop

Keys are captured on `keydown` / `keyup` browser events. While any movement key is held, the browser sends `cmd_vel` messages over WebSocket at **10 Hz** (100 ms interval via `setInterval`). This satisfies the STM32 firmware's 250 ms motor watchdog.

| Key | `vx` | `vy` | `wz` |
|-----|------|------|------|
| W | +0.3 | 0 | 0 |
| S | -0.3 | 0 | 0 |
| A | 0 | 0 | +0.5 |
| D | 0 | 0 | -0.5 |
| Q | 0 | +0.3 | 0 |
| E | 0 | -0.3 | 0 |
| Space | 0 | 0 | 0 (immediate stop) |

Multiple keys can be held; values are summed and clamped.

---

## E-Stop Behavior

1. User clicks STOP button in UI
2. Browser sends `{"type": "estop", "active": true}` over WebSocket
3. `ui_server` publishes `std_msgs/Bool(True)` to `/emergency_stop`
4. `stm32_bridge` receives it: sends `X\n` to STM32 UART; sets `_estop_active = True`
5. All subsequent `/cmd_vel` messages are silently dropped by `stm32_bridge`
6. STM32 firmware stops all motors and resets its own watchdog timer
7. UI shows STOP button in locked/red state; keyboard teleop keys are visually greyed out and input is ignored
8. User must click RESUME to clear — browser sends `{"type": "estop", "active": false}` → `stm32_bridge` clears flag; keyboard re-enables

---

## Topics Summary

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `ui_server` (teleop) | `stm32_bridge` |
| `/stm32_tx` | `std_msgs/String` | `stm32_bridge` | `ui_server` |
| `/encoder_raw` | `std_msgs/Int32MultiArray` | `stm32_bridge` (future) | `ui_server` |
| `/odom` | `nav_msgs/Odometry` | `odom_publisher` (future) | `ui_server` |
| `/emergency_stop` | `std_msgs/Bool` | `ui_server` | `stm32_bridge` |

---

## Package Changes

**`package.xml`** — no changes needed for FastAPI (not in ROS2 apt repos).

**`requirements.txt`** (new file at `ros2/litterbot_bridge/requirements.txt`):
```
fastapi>=0.110.0
uvicorn[standard]>=0.29.0
```
Install once in WSL: `pip install -r ros2/litterbot_bridge/requirements.txt`

**`setup.py`** — add entry point:
```python
'ui_server = litterbot_bridge.ui_server:main',
```

**New files:**
```
ros2/litterbot_bridge/
├── litterbot_bridge/
│   ├── ui_server.py          ← new FastAPI + rclpy node
│   └── ui/
│       └── index.html        ← self-contained frontend
```

---

## Development / Run Instructions

```bash
# Terminal 1: mock stm32_bridge
ros2 run litterbot_bridge stm32_bridge --ros-args --log-level debug

# Terminal 2: ui_server
ros2 run litterbot_bridge ui_server

# Browser (Windows laptop):
# http://localhost:8765
```

For real hardware, add `-p mock_serial:=false -p serial_port:=/dev/ttyUSB0` to `stm32_bridge`.

---

## Out of Scope (this iteration)

- Live camera feed (placeholder only)
- Authentication / access control
- Multi-robot support
- ROS2 bag recording from UI
- SLAM / map visualization
