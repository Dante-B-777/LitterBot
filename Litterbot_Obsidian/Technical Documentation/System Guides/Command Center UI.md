# LitterBot — Command Center UI

> Web-based control interface for the LitterBot robot. Runs on the Jetson, accessed from any browser on the same network.
>
> **Last updated: 2026-04-01 (session 5)**

---

## Overview

The Command Center is a web app that lets you drive and monitor LitterBot from a laptop browser. It requires no software installation on the laptop — just a browser and a WiFi connection.

The web server runs as a ROS2 node on the Jetson. When the Jetson is on, the UI is live automatically (systemd auto-start).

---

## What It Does

| Feature | How to use |
|---------|-----------|
| Drive (omnidirectional) | WASD + QE keys |
| Speed control | Slider (10-100%) |
| Pause | Pause button — holds motors at zero, keeps watchdog alive |
| Emergency Stop | SPACE bar or E-Stop button — sends `X\n` hard stop to STM32 |
| Scoop sequence | F key — triggers `S SCOOP\n` arm motion |
| Deploy | Press-and-hold DEPLOY button for 5 seconds (standby overlay) |
| Dev mode | DEV toggle in topbar — bypasses deploy safety gate |
| Live logs | "Jetson -> STM32", "STM32 -> Jetson", "Node <-> Node" panels |
| Status lights | State dot (STANDBY/DEPLOYING/ACTIVE), WebSocket, E-Stop, STM32, Bridge, Odom, Camera |
| Autonomy toggle | TELEOP/AUTONOMOUS mode switch (future: Nav2 integration) |
| LiDAR pane | Placeholder for LiDAR data visualization (not yet connected) |
| Camera pane | Placeholder for camera feed (not yet connected) |

---

## Architecture

```
Browser (laptop)
    │
    │  HTTP + WebSocket  (port 8765)
    ▼
ui_server.py  ─── ROS2 node on Jetson
    │
    │  ROS2 topics
    ├── /cmd_vel          (geometry_msgs/Twist — drive commands)
    ├── /emergency_stop   (std_msgs/Bool)
    ├── /pause_cmd        (std_msgs/Bool)
    ├── /scoop_cmd        (std_msgs/String)
    ├── /deploy_cmd       (std_msgs/String — "deploy" or "standby")
    ├── /dev_mode_cmd     (std_msgs/Bool — dev mode toggle)
    ├── /stm32_tx         (std_msgs/String — commands sent to STM32)
    ├── /stm32_rx         (std_msgs/String — responses from STM32)
    └── /stm32_state      (std_msgs/String — deploy state updates)
    │
    ▼
stm32_bridge.py  ─── ROS2 node on Jetson
    │
    │  UART 115200 8N1 (/dev/ttyACM0)
    ▼
STM32 firmware  ──► 4x DC motors + scoop mechanism
```

**Key design decision:** The browser never talks to the STM32 directly. All commands go through `ui_server` → ROS2 topics → `stm32_bridge` → UART. This keeps the hardware abstracted and lets other ROS2 nodes (Nav2, autonomous stack) share the same interface.

---

## Source Files

| File | Location | Purpose |
|------|----------|---------|
| `ui_server.py` | `ros2/litterbot_bridge/litterbot_bridge/` | FastAPI + WebSocket server, ROS2 node |
| `stm32_bridge.py` | `ros2/litterbot_bridge/litterbot_bridge/` | UART ↔ ROS2 bridge, serial read thread |
| `index.html` | `ros2/litterbot_bridge/litterbot_bridge/ui/` | Self-contained frontend (served by ui_server) |

---

## Accessing the UI

### Home / Lab (same WiFi router)

Both the laptop and Jetson connect to the same router.

| Item | Value |
|------|-------|
| Jetson IP | `192.168.1.100` (static) |
| UI URL | `http://192.168.1.100:8765` |

Open that URL in any browser. The page loads instantly — no login required.

### Field Deployment (no router — Jetson hotspot)

In the field there is no router. The Jetson acts as its own WiFi access point. The laptop connects directly to the Jetson's network.

```
Normal (home/lab):
  Home router → Jetson at 192.168.1.100 → laptop on same router
  URL: http://192.168.1.100:8765

Field (no router):
  Jetson IS the router → always at 10.42.0.1 → laptop connects to "LitterBot" WiFi
  URL: http://10.42.0.1:8765
```

> Note: While connected to "LitterBot" WiFi, the laptop has no internet access.
> This is expected and fine for field operation.

---

## Setting Up the Jetson Hotspot (one-time)

SSH into the Jetson and run:

```bash
# Create the hotspot
sudo nmcli device wifi hotspot ifname wlan0 ssid "LitterBot" password "litterbot123"

# Give it a stable name
sudo nmcli connection modify Hotspot connection.id LitterBot

# Optional: auto-start hotspot on every boot
# (disable this if you want home WiFi to be the default)
sudo nmcli connection modify LitterBot connection.autoconnect yes

# Verify
nmcli connection show LitterBot
```

> `wlan0` is assumed as the WiFi interface name. Verify with `ip link show` if the hotspot doesn't appear.

When the hotspot is active, the Jetson's IP is always **10.42.0.1** — this is fixed by NetworkManager and does not change between reboots.

### Connecting from the laptop

1. Open WiFi settings on the laptop
2. Connect to **"LitterBot"** with the password set above
3. Open browser: **`http://10.42.0.1:8765`**

### Switching between home WiFi and hotspot

The Jetson has a single WiFi adapter — it can only be on one network at a time.

```bash
# At home — disable hotspot, connect to router
sudo nmcli connection down LitterBot
sudo nmcli device wifi connect "YourHomeSSID" password "YourHomePassword"

# In the field — enable hotspot
sudo nmcli connection up LitterBot
```

### Optional: mDNS (access by name instead of IP)

If `avahi-daemon` is installed on the Jetson, you can use `litterbot.local` instead of remembering the IP:

```bash
# Home:  http://litterbot.local:8765
# Field: http://litterbot.local:8765  (same — works on both networks)
```

Requirements:
- Jetson: `avahi-daemon` installed and running
- macOS laptop: works out of the box
- Windows laptop: requires Bonjour (installed with iTunes, or standalone)
- Linux laptop: requires `avahi-daemon`

---

## Starting / Stopping the Stack

The ROS2 stack (including `ui_server`) auto-starts on Jetson boot via systemd.

```bash
# Check status
sudo systemctl status litterbot

# Start manually
sudo systemctl start litterbot

# Stop
sudo systemctl stop litterbot

# Follow logs live
sudo journalctl -u litterbot -f
```

Or check the launch log directly:
```bash
cat /tmp/litterbot.log
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Page won't load | Stack not running | `sudo systemctl start litterbot` |
| Page loads but no response to keys | STM32 not connected or not flashed | Check `/dev/ttyACM0` exists; reflash STM32 |
| Commands show in Jetson→STM32 panel but wheels don't move | STM32 firmware crashed | Check heartbeat blink on board; reflash |
| "LitterBot" WiFi not visible | Hotspot not enabled | `sudo nmcli connection up LitterBot` on Jetson |
| Port 8765 already in use | Previous process didn't exit cleanly | `sudo pkill -9 -f ui_server; sudo fuser -k 8765/tcp` |
