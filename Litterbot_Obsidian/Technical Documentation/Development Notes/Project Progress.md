# LitterBot — Project Progress

> **Living document.** Update this after every session. For agent context files see `CLAUDE.md` (repo root). For deep technical detail see the other files in this folder.
>
> **Last updated: 2026-04-01 (session 5)**

---

## Overall Status

| Subsystem | Status |
|-----------|--------|
| STM32 — DC motors | **WORKING** — all 4 wheels respond to WASD teleop |
| STM32 — UART teleop | **WORKING** — confirmed 2026-03-29, updated to deploy-gated protocol 2026-04-01 |
| STM32 — Arm control | **Done** — limit switches, calibration mode, servo interpolation, Arm_Home() |
| STM32 — Deploy state | **Done** — STANDBY/DEPLOYING/ACTIVE state machine, dev mode, command gating |
| STM32 — Encoders | Driver written, **disabled in main loop** — not yet tested on hardware |
| STM32 — Stepper | Driver written, not yet tested on hardware |
| STM32 — Servo (HX-65HM) | Driver written, not yet tested on hardware |
| STM32 — Odometry | Module written, **disabled in main loop** — depends on encoders |
| STM32 — Scoop sequence | Written — **all constants are placeholders**, needs physical measurement |
| ROS2 — `stm32_bridge` | **Done** — deploy/dev mode, serial read thread, state query, `S SCOOP` protocol |
| ROS2 — `ui_server` | **Done** — deploy overlay, dev mode, state broadcasting, LiDAR pane placeholder |
| ROS2 — Launch file | **Done** — `ros2 launch litterbot_bridge litterbot.launch.py` |
| Jetson deployment | **Done** — ROS2 Humble, static IP `192.168.1.100`, **systemd auto-start enabled** |
| ROS2 — `odom_publisher` | Not started |
| ROS2 — `camera_node` (RealSense D455) | Not started |
| ROS2 — `rplidar_ros` | Not started |
| ROS2 — SLAM | Not started |
| ROS2 — Nav2 | Not started |
| ROS2 — `litter_picker` | Not started |

---

## What Is Working Right Now

- **All 4 wheels drive** via WASD in the web UI at `http://192.168.1.100:8765`
- **Web UI auto-starts** on Jetson boot via systemd (`litterbot.service`)
- **UART RX confirmed**: Jetson -> STM32 motor commands received and executed
- **Direction correct**: left side forward, right side forward when pressing W (right-side motors negated in firmware)
- **Heartbeat blink**: LD3 (red) blinks at 1 Hz — confirms firmware is alive
- **Flash workflow**: `sudo st-flash --reset write ...` — MCU auto-resets after programming
- **Deploy/standby system**: Full stack implemented (STM32 + ROS2 + UI) — boots in STANDBY, requires 5s deploy, dev mode bypass

---

## Immediate Action Items

- [ ] **Test deploy sequence** on hardware — arm homing + state transitions
- [ ] **Bring up odometry** — encoders connected but not yet enabled in firmware
- [ ] **Verify all motor directions** — confirm strafe (A/D) and rotation (Q/E) are correct, not just forward
- [ ] **Measure scoop mechanism** — fill in placeholder values in `scoop_sequence.h`
  - Servo angles: home, open, closed, dump positions
  - Stepper step count: home -> full extension
  - Step speeds: extend and retract
- [ ] **Test encoders** — spin wheels by hand, run `T\n` command, verify counts on STM32->Jetson log panel

---

## Next Steps — Prioritized

### Near Term (hardware bring-up)

- [ ] Confirm motor directions for all movement axes (strafe, rotate)
- [ ] Test encoder hardware: spin wheels by hand, confirm tick counts via `T` command in UI
- [ ] Test stepper motor: `stepper_test.c` firmware, B1 button → 3-phase speed sequence
- [ ] Test servo: `servo_test.c` firmware, B1 button → servo sweep
- [ ] Measure scoop mechanism values, fill in `scoop_sequence.h`
- [ ] Test full scoop sequence: press S in UI, verify arm extends, scoops, retracts

### Medium Term (ROS2)

- [ ] `odom_publisher` node — subscribes to `/encoder_raw`, publishes `/odom`
- [ ] Re-enable `Encoder_Update()` and `Odometry_Update()` in `main.c` once encoder hardware is confirmed working
- [ ] Test encoder telemetry: STM32 → Jetson encoder tick stream

### Long Term (autonomy stack)

- [ ] `camera_node` — RealSense RGB-D, publish `/color`, `/depth`
- [ ] `rplidar_ros` — 2D lidar scan
- [ ] SLAM — sensor fusion of RealSense + RPLidar (SLAM Toolbox or RTAB-Map, TBD)
- [ ] Nav2 — autonomous path planning and obstacle avoidance
- [ ] `litter_picker` — behavior controller: subscribe to `/detections`, send goals to Nav2, trigger scoop

### Housekeeping

- [ ] Set up udev rule for persistent `/dev/ttySTM32` device name (currently `/dev/ttyACM0` — can shift if other USB devices plugged in)
- [ ] Give `ben` user passwordless sudo for `st-flash` (currently requires password each time)
- [ ] Set up Jetson WiFi hotspot for field use (see `Command Center UI.md` for commands)

---

## STM32 Firmware

### Done

- [x] CubeMX configured: TIM2 PWM (4ch), TIM1/3/4/8 encoder mode, USART3 half-duplex, LPUART1 VCP
- [x] LPUART1 word length fixed: 9 Bits → **8 Bits** (8N1) — required for correct UART communication
- [x] `LPUART1_IRQHandler` added to `stm32l4xx_it.c` — missing handler was locking up MCU on first received byte
- [x] DC motor driver (`cytron_md10c.c/h`) — sign-magnitude PWM via TIM2, all 4 channels
- [x] Right-side motor direction fix — FR and RR speeds negated in `uart_cmd.c` (motors physically mounted in opposite orientation)
- [x] Stepper driver (`cl57t_stepper.c/h`) — step/dir GPIO bit-bang with DWT timing
- [x] Servo driver (`hx65_servo.c/h`) — HX-65HM half-duplex UART protocol
- [x] Encoder driver (`encoder.c/h`) — quadrature decode via TIM1/3/4/8
- [x] Odometry module (`odometry.c/h`) — mecanum dead-reckoning
- [x] UART command parser (`uart_cmd.c/h`) — `M` (motors), `X` (e-stop), `S` (scoop), `T` (encoder test)
- [x] Scoop sequence (`scoop_sequence.c/h`) — 6-phase coordinated stepper+servo motion
- [x] Test modules: `servo_test`, `dc_motor_test`, `stepper_test`, `combined_test`, `encoder_test`
- [x] LD3 heartbeat blink in main loop — 1 Hz confirms firmware is alive

### Disabled / Pending Hardware Validation

- [ ] `Encoder_Update()` — commented out in main loop; encoder timers init but hardware not yet verified
- [ ] `Odometry_Update()` — commented out; depends on encoders

### Scoop Mechanism — Needs Physical Measurement

All values in `scoop_sequence.h` are placeholders. **Do not run `S` command on hardware until these are measured:**

| Constant | What to measure |
|----------|----------------|
| `SCOOP_EXTEND_STEPS` | Step count from home to full arm extension |
| `SCOOP_EXTEND_DIR` | Verify CW vs CCW on your stepper |
| `SCOOP_EXTEND_SPEED_US` / `SCOOP_RETRACT_SPEED_US` | Tune step period (microseconds) |
| `SCOOP_SERVO_ID` | Ping servo to confirm ID |
| `SCOOP_POS_HOME/OPEN/CLOSED/DUMP` | Measure servo angles at each position |
| `SCOOP_SERVO_MOVE_MS` | Tune move duration |

### Hardware Notes

- NUCLEO-144 #1 fried — LD5 overcurrent from servo wiring incident — using identical spare
- USART3 TX rewired from PB10 (damaged pad) to PC10
- PB7 (FL encoder CHB) conflicts with LD2 LED — monitor for interference during encoder testing

---

## ROS2 Software

### Done

- [x] `litterbot_bridge` package — builds cleanly with `colcon build`
- [x] `stm32_bridge` node — mock + real serial, mecanum kinematics, e-stop (`X\n`), pause (`M 0 0 0 0\n`), scoop (`S\n`)
- [x] `ui_server` node — FastAPI+WebSocket, full command center UI, ROS2 node
- [x] Launch file — `ros2 launch litterbot_bridge litterbot.launch.py`
- [x] 38 tests — 19 passing (ROS2), 19 skipped (firmware hardware tests)

**UI features:**

| Feature | Control |
|---------|---------|
| Drive (omni) | WASD + QE keys |
| Speed | Slider (10–100%, default 60%) |
| Emergency stop | SPACE — always active |
| Pause | P key or button — zeros motors, watchdog stays alive |
| Scoop trigger | S key |
| Autonomy toggle | Disables keyboard when autonomous nav is active |
| Status lights | WebSocket, E-stop, STM32 activity, Bridge, Odom, Camera |
| Log panels | Jetson→STM32 (blue), STM32→Jetson (green), Node↔Node (purple) |

**Key design notes:**

- Motor speed values sent to STM32: **-1000 to +1000** (firmware divides by 10 → %)
- Teleop base speed: `vx = 0.5 m/s`, `vy = 0.5 m/s`, `wz = 0.8 rad/s` — scaled by speed slider
- E-stop = `X\n` (hard stop), Pause = `M 0 0 0 0\n` (resumable, keeps watchdog alive)
- UI accessible at `http://192.168.1.100:8765` (home) or `http://10.42.0.1:8765` (Jetson hotspot/field)

### Not Started

- [ ] `odom_publisher` — subscribes to `/encoder_raw`, publishes `/odom` (nav_msgs/Odometry)
- [ ] `realsense2_camera` — RealSense RGB-D, publishes `/color`, `/depth`
- [ ] `rplidar_ros` — 2D lidar scan publisher
- [ ] SLAM node — sensor fusion of RealSense + RPLidar; TBD between SLAM Toolbox and RTAB-Map
- [ ] Nav2 stack — autonomous path planning and obstacle avoidance
- [ ] `litter_picker` — behavior controller, subscribes to `/detections`, sends goals to Nav2, triggers scoop

### Running the Stack

```bash
# Jetson — started automatically by systemd on boot
sudo systemctl status litterbot    # check
sudo systemctl start litterbot     # start manually
sudo systemctl stop litterbot      # stop
sudo journalctl -u litterbot -f    # live logs

# Manual launch (for dev/debugging)
source /opt/ros/humble/setup.bash && source ~/litterbot_ws/install/setup.bash
ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyACM0

# Run tests
cd ros2/litterbot_bridge && python3 -m pytest test/ -v
```

---

## Jetson Deployment

### Done

- [x] ROS2 Humble installed, workspace at `~/litterbot_ws/`
- [x] Static IP: `192.168.1.100`, gateway `192.168.1.254`
- [x] SSH key-based auth from WSL2 working
- [x] `st-flash` + `arm-none-eabi-objcopy` installed (remote flashing from WSL2)
- [x] systemd service `/etc/systemd/system/litterbot.service` — auto-starts on boot
- [x] Sleep/suspend disabled permanently

### Pending

- [ ] WiFi hotspot setup for field use (see `Command Center UI.md`)
- [ ] udev rule for stable STM32 device name (`/dev/ttySTM32`)
- [ ] Passwordless sudo for `st-flash`

### Flash Workflow (from WSL2)

```bash
# Build in STM32CubeIDE (Ctrl+B), then:
scp /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/firmware/stm32/LitterBot/Debug/LitterBot.elf ben@192.168.1.100:/tmp/LitterBot.elf && \
ssh ben@192.168.1.100 "arm-none-eabi-objcopy -O binary /tmp/LitterBot.elf /tmp/LitterBot.bin && sudo st-flash --reset write /tmp/LitterBot.bin 0x08000000"
```

> **Always use `--reset`** — without it, the MCU stays halted after programming and the new firmware never runs.

---

## Hardware / Mechanical

### Scooping Mechanism

- Two arms connected by a scoop bucket (skid-steer loader style)
- Front-mounted ramp leads from floor to collection bin at top of robot
- **Stepper:** linear extension/retraction of both arms
- **Servo (HX-65HM, dual shaft):** rotates bucket up/down; shaft spans both arm shoulders

**Planned scoop sequence (6 phases):**
1. Pre-position servo (bucket neutral)
2. Arm extends fully forward (stepper)
3. Bucket rotates down to floor
4. Bucket rotates inward/up — scoops litter
5. Arm retracts — litter rides up ramp into bin
6. Servo and arm return to home

**Open questions:**
- [ ] Servo angle range — measure down, scoop, dump, home positions on hardware
- [ ] Stepper step count: home → full extension — measure on hardware
- [ ] Limit switch type: NC or NO (affects fail-safe wiring)
- [ ] Arm storage position while driving (likely up, latched — TBD)

### Known Hardware Issues

- NUCLEO-144 #1 fried — LD5 overcurrent after servo wiring incident — on identical spare
- PB7 (FL encoder CHB) conflicts with LD2 LED — watch during encoder testing
- FL wheel slow at low speeds — normal PWM deadband behavior, not a defect

---

## Lessons Learned

- **Always use `st-flash --reset write`** — without `--reset`, MCU stays halted and firmware never runs
- **Always add peripheral IRQ handlers to `stm32l4xx_it.c`** — CubeMX leaves them out; missing handler = MCU lockup on first interrupt (infinite loop in `Default_Handler`)
- **CubeMX LPUART word length defaults to 9 bits** — always verify 8N1 manually in generated code
- **Enable subsystems incrementally** — `Encoder_Update()` caused HardFault before hardware was validated; comment out untested modules until hardware is confirmed
- **Right-side mecanum motors mount facing opposite direction** — negate FR and RR speeds to get correct forward rotation
