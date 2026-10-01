# Jetson Migration Guide

Moving LitterBot from WSL2 development to the NVIDIA Jetson as the primary compute platform.

---

## Overview

During development, everything runs on a Windows PC with WSL2 (Ubuntu 22.04). The Jetson migration means:

- ROS2 nodes run **on the Jetson** (not WSL2)
- The browser UI runs on a **laptop over WiFi** — point it at the Jetson's IP instead of localhost
- The STM32 is connected to the **Jetson's USB port** (not the PC)
- Nodes start automatically on boot — no SSH required to drive the robot

---

## Prerequisites

| Item | Status |
|------|--------|
| Jetson with JetPack 5.x (Ubuntu 20.04) or JetPack 6.x (Ubuntu 22.04) | TBD |
| ROS2 Humble (Ubuntu 22.04 JetPack) or Foxy (Ubuntu 20.04) | Install below |
| Same WiFi network as laptop | Required |
| USB-A cable for STM32 UART adapter | Required |

> **JetPack version note:** ROS2 Humble requires Ubuntu 22.04 (JetPack 6). If the Jetson is on JetPack 5 (Ubuntu 20.04), use ROS2 Foxy — but Humble is strongly preferred for nav2/SLAM compatibility.

---

## Step 1 — Install ROS2 on Jetson

```bash
# On the Jetson (SSH in or connect keyboard/monitor)

# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 apt repo
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

---

## Step 2 — Clone the Repo and Build

```bash
# On the Jetson
git clone https://github.com/<your-username>/LitterBot_STM32.git ~/LitterBot_STM32

# Create workspace and symlink package
mkdir -p ~/litterbot_ws/src
ln -sf ~/LitterBot_STM32/ros2/litterbot_bridge ~/litterbot_ws/src/litterbot_bridge

# Install Python deps
pip install -r ~/LitterBot_STM32/ros2/litterbot_bridge/requirements.txt

# Build
source /opt/ros/humble/setup.bash
cd ~/litterbot_ws && colcon build
```

---

## Step 3 — Serial Port Access

The STM32 connects via a USB-UART adapter. Give the Jetson user permission to access it:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for the group to take effect

# Find the port (usually /dev/ttyUSB0)
ls /dev/ttyUSB*
```

Test the port is accessible:
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 115200); print('OK'); s.close()"
```

---

## Step 4 — Run Manually (First Test)

Before setting up auto-start, verify everything works manually:

```bash
# Terminal 1 — SSH into Jetson
source /opt/ros/humble/setup.bash
source ~/litterbot_ws/install/setup.bash
ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyUSB0

# On laptop browser — find Jetson IP first:
#   ssh jetson "hostname -I"
# Then open:
#   http://<jetson-ip>:8765
```

Check the status bar in the UI:
- WebSocket dot goes green
- STM32 dot goes green when motors receive commands
- Bridge dot goes green

---

## Step 5 — Auto-Start on Boot (systemd)

Create a systemd service so nodes start automatically when the Jetson boots.

```bash
# On the Jetson — create the service file
sudo tee /etc/systemd/system/litterbot.service <<'EOF'
[Unit]
Description=LitterBot ROS2 Nodes
After=network.target

[Service]
Type=simple
User=<your-jetson-username>
Environment="HOME=/home/<your-jetson-username>"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /home/<your-jetson-username>/litterbot_ws/install/setup.bash && \
    ros2 launch litterbot_bridge litterbot.launch.py \
    mock_serial:=false serial_port:=/dev/ttyUSB0"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable litterbot.service
sudo systemctl start litterbot.service

# Check status
sudo systemctl status litterbot.service

# View live logs
journalctl -fu litterbot.service
```

To stop or restart:
```bash
sudo systemctl stop litterbot.service
sudo systemctl restart litterbot.service
```

---

## Step 6 — Network Access from Laptop

The UI server binds to `0.0.0.0:8765` so it's accessible from any device on the same network.

**Find the Jetson's IP:**
```bash
hostname -I   # on the Jetson
```

**On the laptop browser:**
```
http://192.168.x.x:8765
```

**Optional — use mDNS so you don't need the IP:**
```bash
# On the Jetson — install avahi
sudo apt install -y avahi-daemon
sudo systemctl enable avahi-daemon

# Set a hostname (e.g. litterbot)
sudo hostnamectl set-hostname litterbot

# Then access from laptop at:
#   http://litterbot.local:8765
```

---

## Step 7 — ROS2 Multi-Machine (Optional)

If you want `ros2 topic list` or rviz to work from the laptop, both machines must be on the same DDS domain:

```bash
# On both machines — add to ~/.bashrc:
export ROS_DOMAIN_ID=42           # any number 0-232, same on both
export ROS_LOCALHOST_ONLY=0       # allow cross-machine traffic
```

For complex networks (Docker, VPN, multiple interfaces), use Cyclone DDS:
```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## USB Device Persistence

USB ports on the Jetson can change (`ttyUSB0` → `ttyUSB1`) between reboots if multiple USB devices are connected. Create a udev rule to give the STM32 adapter a fixed name:

```bash
# Find the device attributes
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"

# Create rule (replace idVendor/idProduct with your adapter's values)
sudo tee /etc/udev/rules.d/99-litterbot-stm32.rules <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", \
    SYMLINK+="ttySTM32", MODE="0666"
EOF

sudo udevadm control --reload-rules

# Now use /dev/ttySTM32 in the launch file:
ros2 launch litterbot_bridge litterbot.launch.py \
    mock_serial:=false serial_port:=/dev/ttySTM32
```

---

## Step 8 — Field Deployment (No Router / Offline)

In the field there is no WiFi router. The Jetson creates its own WiFi hotspot and the
laptop connects directly to it. No internet is needed — the web UI only talks to the
Jetson, never the outside world.

### How it works

```
Normal (home/lab):
  Home router → Jetson gets 192.168.x.x → laptop connects to same router
  Browser: http://192.168.x.x:8765

Field (no router):
  Jetson IS the router → always at 10.42.0.1 → laptop connects to "LitterBot" WiFi
  Browser: http://10.42.0.1:8765
```

The laptop loses internet while connected to LitterBot WiFi. That is fine for
field operation — you are driving the robot, not browsing the web.

### One-time setup on Jetson

```bash
# Create the hotspot — SSID "LitterBot", password of your choice
sudo nmcli device wifi hotspot ifname wlan0 ssid "LitterBot" password "litterbot123"

# Give it a stable name so we can refer to it later
sudo nmcli connection modify Hotspot connection.id LitterBot

# Make it auto-start on boot
sudo nmcli connection modify LitterBot connection.autoconnect yes

# Verify it was created
nmcli connection show LitterBot
```

> **Note:** `ifname wlan0` assumes the Jetson's WiFi interface is `wlan0`. If it is
> different (e.g. `wlp2s0`), replace accordingly. Check with `ip link show`.

When the hotspot is active, the Jetson's IP on that network is always **10.42.0.1**.
This is fixed by NetworkManager for hotspot connections — it does not change between
reboots.

### Connecting from the laptop

1. On the laptop, open WiFi settings
2. Connect to **"LitterBot"** using the password set above
3. Open browser: **`http://10.42.0.1:8765`**
4. The command center loads — full teleop, status lights, all features work

### Switching between home WiFi and hotspot

The Jetson can only do one at a time (single WiFi adapter). When developing at home,
disable the hotspot and connect to the router normally. In the field, enable the hotspot.

```bash
# At home — disable hotspot, connect to router
sudo nmcli connection down LitterBot
sudo nmcli device wifi connect "YourHomeSSID" password "YourHomePassword"

# In the field — enable hotspot
sudo nmcli connection up LitterBot
```

If auto-start is enabled, the hotspot turns on at every boot. Disable auto-start if
you want to default to home WiFi:

```bash
sudo nmcli connection modify LitterBot connection.autoconnect no
```

### Optional — mDNS works on the hotspot too

If avahi-daemon is installed (Step 6), the laptop can reach the Jetson by name even
on the hotspot:

```bash
# Instead of remembering 10.42.0.1, use:
http://litterbot.local:8765
```

This requires avahi-daemon running on the Jetson and mDNS support on the laptop
(built into macOS; Windows needs Bonjour installed; Linux needs avahi-daemon).

### Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| "LitterBot" WiFi doesn't appear | Hotspot not running | `sudo nmcli connection up LitterBot` |
| Browser can't reach 10.42.0.1 | Not connected to LitterBot WiFi | Check laptop WiFi settings |
| Hotspot drops on reboot | autoconnect not set | `nmcli connection modify LitterBot connection.autoconnect yes` |
| `wlan0` not found | Different interface name | `ip link show` to find correct name |

---

## Migration Checklist

- [x] JetPack version confirmed (6.x preferred for Humble)
- [x] ROS2 Humble installed and sourced
- [ ] Repo cloned, workspace built
- [ ] `dialout` group added for serial access
- [ ] Manual launch tested — UI reachable from laptop
- [ ] STM32 LPUART1 word length fixed (9B → 8B) and reflashed
- [ ] All 19 firmware spec tests passing with `--serial-port /dev/ttySTM32`
- [ ] systemd service created and enabled
- [ ] udev rule for persistent STM32 device name
- [ ] mDNS hostname set (optional but convenient)
