# Launching the LitterBot UI

---

## Option A — WSL2 Dev Mode (no hardware, local browser)

Use this when testing UI changes on your laptop without the Jetson or STM32.

**Terminal: WSL2**

```bash
cd ~/litterbot_ws && colcon build && source install/setup.bash
ros2 launch litterbot_bridge litterbot.launch.py
```

**Browser:** `http://localhost:8765`

To kill it: `Ctrl+C` in the terminal. If nodes are orphaned:
```bash
pkill -f "ros2 launch"; pkill -f "ui_server"; pkill -f "stm32_bridge"
```

---

## Option B — Jetson (real hardware)

Use this when the Jetson and STM32 are connected.

**Step 1 — SSH into the Jetson**

Terminal: any (WSL2, PowerShell, macOS Terminal)
```bash
ssh ben@192.168.1.100
```

**Step 2 — Start the stack (runs in background, survives SSH disconnect)**

```bash
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash
nohup ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyACM0 > /tmp/litterbot.log 2>&1 &
```

**Step 3 — Open UI in laptop browser**

```
http://192.168.1.100:8765
```

**Check logs:**
```bash
cat /tmp/litterbot.log
```

**Kill everything:**
```bash
sudo pkill -9 -f ui_server; sudo pkill -9 -f stm32_bridge; sudo fuser -k 8765/tcp
```

---

## Auto-Start (Jetson boots with UI already running)

The Jetson is configured to start the ROS2 stack automatically on boot via systemd. You do not need to SSH in and launch manually — just power on the Jetson, wait ~30 seconds, and open the browser.

```bash
# Check if it's running
sudo systemctl status litterbot

# Start manually (if stopped)
sudo systemctl start litterbot

# Stop
sudo systemctl stop litterbot

# Follow live logs
sudo journalctl -u litterbot -f
```

---

## After Updating the Code

Any time you pull new changes or edit Python/HTML files, you must rebuild before launching:

```bash
cd ~/litterbot_ws && colcon build && source install/setup.bash
```

Then hard-refresh the browser (`Ctrl+Shift+R`) to clear cached HTML.

If using systemd auto-start, rebuild then restart the service:
```bash
cd ~/litterbot_ws && colcon build && source install/setup.bash
sudo systemctl restart litterbot
```

---

## Network Info

| Device | IP |
|--------|----|
| Jetson | `192.168.1.100` |
| Router/Gateway | `192.168.1.254` |
