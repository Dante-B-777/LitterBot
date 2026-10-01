# LitterBot — Master Project Brief

> **For any AI agent opening this repo:** Read this file first, then read the subsystem CLAUDE.md files listed below. This file is kept up to date as the project evolves. Do not rely on git history for current state — trust this file.

---

## What Is LitterBot?

LitterBot is a senior robotics engineering project — an autonomous mobile robot that collects litter. It uses a **4-wheel mecanum drive** chassis for omnidirectional movement and a **stepper + servo scooping mechanism** to pick up trash.

**Weight:** 42 lbs
**Wheels:** 104mm mecanum wheels (52mm radius)
**Chassis:** Wheelbase L=0.168m, Track width W=0.2025m

---

## Current Project Status (update this section every session)

**Last updated: 2026-04-05 (session 10)**

| Subsystem | Status |
|-----------|--------|
| STM32 firmware — DC motors | **WORKING** — all 4 wheels respond to WASD teleop |
| STM32 firmware — Encoders | Driver written, not yet tested on hardware |
| STM32 firmware — Stepper | Driver written, not yet tested — **CL57T driver powered but STEP/DIR not yet wired to STM32** |
| STM32 firmware — Servo | Driver written, not yet tested |
| STM32 firmware — UART teleop | **WORKING** — re-confirmed 2026-04-05 (wheels moving via WASD) |
| STM32 firmware — Odometry | Module written, not yet tested |
| STM32 firmware — Scoop sequence | `scoop_sequence.c/h` written — uses CalStore values at runtime, falls back to defaults if uncalibrated |
| STM32 firmware — Arm control | **Done** — limit switches, calibration mode, servo interpolation, Arm_Home(); HX65_Init fix applied |
| STM32 firmware — Deploy state | **Done** — deploy now goes ACTIVE even if Arm_Home() fails (arm not yet wired); flashed 2026-04-03 |
| STM32 firmware — Calibration store | **Done** — `calibration_store.c/h` — flash-backed cal values (extend_steps + 4 servo positions); compiled and flashed 2026-04-03 |
| ROS2 — `stm32_bridge` node | **Done** — deploy/dev mode, serial read thread, state query, S SCOOP protocol, CAL commands via /cal_cmd topic |
| ROS2 — `ui_server` node | **Done** — pane-level deploy overlay, dev mode gating, 3s deploy hold, LiDAR status dot, calibration UI section |
| ROS2 — launch file | **Done** — `ros2 launch litterbot_bridge litterbot.launch.py` |
| Jetson deployment | **Done** — ROS2 Humble, static IP `192.168.1.100`, **systemd auto-start enabled**; all ROS2 files synced 2026-04-05 |
| ROS2 — `odom_publisher` | Not started |
| ROS2 — `camera_node` | Not started |
| ROS2 — `litter_picker` | Not started |

**Immediate action items (next session — do in this order):**
1. **Boot verification** — power on Jetson + STM32, open `http://192.168.1.100:8765`, confirm STATE shows STANDBY, enable DEV mode, press W — wheels should move. If not, check `sudo journalctl -u litterbot -n 30` on Jetson.
2. **Verify motor directions** — with DEV mode on, test WASD: W=forward, S=backward, A=strafe left, D=strafe right. If any wheel spins wrong way, flip DIR polarity in firmware for that wheel.
3. **Wire CL57T stepper driver** — connect STEP→PD14, DIR→PF15 from STM32 to CL57T. Also wire ALARM pin if available.
4. **Test stepper** — once wired, use DEV mode + deploy to trigger arm home sequence. Confirm stepper moves.
5. **Run full calibration** — `C CALIBRATE` (arm extend/retract, counts steps, auto-saves to flash) → servo setup via UI (4 position saves).

**Known operational notes:**
- Deploy always goes ACTIVE even if arm homing fails — teleop works without arm hardware
- DEV mode bypasses deploy gate — use for testing without full deploy sequence
- Jetson ROS2 files fully synced to local repo as of 2026-04-05 (rsync + colcon build done)
- If multiple stm32_bridge processes accumulate (commands send N×): `ps aux | grep stm32` to find PIDs, kill by PID, relaunch
- If port 8765 busy on relaunch: `sudo lsof -i :8765 -t | xargs -r sudo kill -9`
- Flash workflow: CubeIDE Ctrl+B on Windows → `scp firmware/.../Debug/LitterBot.elf ben@192.168.1.100:/tmp/` → `arm-none-eabi-objcopy -O binary ... && st-flash --reset write ...`
- B1 button (PC13) configured as plain polled input with pull-up — used as e-stop during arm calibration

**Jetson info:**
- IP: `192.168.1.100` (static), gateway `192.168.1.254`
- SSH: `ssh ben@192.168.1.100`
- UI: `http://192.168.1.100:8765`
- Launch: `nohup ros2 launch litterbot_bridge litterbot.launch.py mock_serial:=false serial_port:=/dev/ttyACM0 > /tmp/litterbot.log 2>&1 &`
- Flash from Jetson: see `Integration Notes.md` for full workflow

---

## Compute Architecture

```
┌─────────────────────────────────────┐      ┌──────────────────────────────┐
│         NVIDIA Jetson (ROS2)         │      │   Laptop (Web UI only)       │
│                                     │      │                              │
│  ui_server ◄──── WebSocket ─────────┼──────┼──► browser: <jetson-ip>:8765 │
│      │                              │      │   WASD+QE teleop, speed      │
│      ▼                              │      │   slider, pause/e-stop,      │
│  /cmd_vel  /emergency_stop          │      │   status lights, scoop key   │
│  /pause_cmd  /scoop_cmd             │      └──────────────────────────────┘
│      │                              │
│  stm32_bridge ──► UART 115200 ──────┼──► STM32L4A6ZG (bare-metal C)
│      │  /scoop_cmd /pause_cmd       │         │
│  /encoder_raw                       │    4x DC motors + encoders
│      │                              │    Stepper motor (arm extension)
│  odom_publisher ──► /odom           │    HX-65HM servo (bucket rotation)
│                                     │
│  realsense2_camera ──► /color /depth │
│  rplidar_ros ──► /scan              │
│      │                              │
│  SLAM (SLAM Toolbox or RTAB-Map)    │
│      │ /map  /odom                  │
│  Nav2 ──► /cmd_vel (autonomous)     │
│                                     │
│  camera_node ──► /detections        │
│      │                              │
│  litter_picker ──► /cmd_vel         │
└─────────────────────────────────────┘
```

---

## Repo Structure

```
LitterBot_STM32/
├── CLAUDE.md                          ← YOU ARE HERE — master project brief
├── ros2/                              ← ROS2 software stack
│   ├── CLAUDE.md                      ← ROS2 subsystem brief (read this next)
│   └── litterbot_bridge/              ← Main ROS2 package (stm32_bridge + ui_server)
├── firmware/
│   └── stm32/LitterBot/
│       ├── CLAUDE.md                  ← STM32 firmware brief
│       └── Core/Inc|Src/              ← All driver and test files
├── Litterbot_Obsidian/
│   └── Technical Documentation/
│       ├── ROS2 Development.md        ← Detailed ROS2 dev log (nodes, protocols, bugs)
│       └── STM32 System Testing.md    ← STM32 test procedures
├── CHANGELOG.md                       ← Session log — one entry per merge to main
├── LB_FIRM/                           ← Reference firmware (original wheel4 code)
├── docs/
│   ├── superpowers/specs/             ← Design specs from brainstorming sessions
│   └── superpowers/plans/             ← Implementation plans (executed by Claude Code)
└── Report/                            ← Course report and presentation files
```

---

## Documentation Structure

This project uses a **four-layer documentation system** designed so any AI agent or new developer can pick up the project at any point without needing onboarding:

| Layer | File | Purpose |
|-------|------|---------|
| **1 — Agent briefs** | `CLAUDE.md` files | Machine-readable context for AI agents. Auto-loaded by Claude Code. Kept at repo root and each subsystem root. Always current. |
| **2 — Changelog** | `CHANGELOG.md` | Session log — one entry per merge to main. Use `[Unreleased]` on feature branches. Add an entry every time you merge. |
| **3 — Dev logs** | `Litterbot_Obsidian/Technical Documentation/*.md` | Human+agent readable. Detailed technical docs: protocols, test procedures, node specs, known bugs, kinematics math. Updated as we build and test. |
| **4 — Reference** | `docs/`, `LB_FIRM/`, `Report/` | Static reference material: datasheets, logbooks, original firmware, course deliverables. Not updated regularly. |

**Rules:**
- After every working session, update the **Current Project Status** table above
- After building or testing any node/module, update the relevant Obsidian doc
- Keep `CLAUDE.md` files factual and current — they are the source of truth for any agent starting a new session
- Add a `CHANGELOG.md` entry when merging to `main`. Use `[Unreleased]` for in-progress work on feature branches.

---

## Key Hardware Facts (for any agent)

| Item | Value |
|------|-------|
| MCU | STM32L4A6ZG, NUCLEO-144 board |
| SYSCLK | ~47.333 MHz |
| DC Motor PWM | TIM2, CH1-4, PA0-PA3, ~20kHz |
| Encoders | TIM1/3/4/8 encoder mode, 537.7 ticks/rev |
| Servo UART | USART3 half-duplex, PC10, 1,000,000 baud |
| STM32↔Jetson UART | LPUART1, 115200 baud (word length fix needed: 9B→8B) |
| Motor command | `M <fl> <fr> <rl> <rr>\n`, values -1000 to 1000 [GATED] |
| E-stop | `X\n` — immediate hardware stop [ALWAYS] |
| Pause (keep watchdog alive) | `M 0 0 0 0\n` — motors zeroed, resumable [GATED] |
| Scoop sequence | `S SCOOP\n` — 6-phase arm motion with coordinated servo [GATED] |
| Arm calibration | `C CALIBRATE\n` — extend/retract slowly, print step counts (B1 = e-stop) [GATED] |
| Deploy | `D DEPLOY\n` — arm homing, STANDBY to ACTIVE transition [STANDBY/ACTIVE] |
| Dev mode | `DEV ON\n` / `DEV OFF\n` — bypass safety gate [ALWAYS] |
| State query | `Q\n` — responds `STATE <state> [DEV]\r\n` [ALWAYS] |
| Encoder test | `T\n` — debug only [ALWAYS] |

---

## Subsystem CLAUDE.md Files

- **ROS2:** `ros2/CLAUDE.md`
- **STM32 Firmware:** `firmware/stm32/LitterBot/CLAUDE.md`

## Deploying to the Jetson

See `docs/jetson-claude-prompt.md` — paste that prompt into Claude Code on the Jetson
to get it oriented and running the migration guide automatically.

**Repo URL:** `https://github.com/benjam1ntavares/LitterBot_STM32.git`

---

## Development Environment

| Task | Environment |
|------|-------------|
| STM32 firmware | Windows — STM32CubeIDE |
| ROS2 nodes | WSL2 Ubuntu 22.04 — ROS2 Humble |
| Both | Same git repo (`C:\Users\benji\Documents\GitHub\LitterBot_STM32\` = `/mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32/`) |

To start a ROS2 dev session in WSL:
```bash
cd /mnt/c/Users/benji/Documents/GitHub/LitterBot_STM32
claude
```

---

## Team & Collaboration

**This is a group senior design project.** Multiple team members use Claude Code independently. The documentation system is designed so any person or agent can pick up the project mid-stream without a handoff.

**Team members (all have repo access):**
- benji (benjam1ntavares) — lead developer, ROS2 + firmware integration

**When starting a session:**
1. `git pull` — get latest from main or your branch
2. Read this file — Current Project Status table tells you where things stand
3. Check `CHANGELOG.md` — `[Unreleased]` section shows what's in-flight on active branches
4. Read the relevant subsystem `CLAUDE.md` before touching that subsystem

**When ending a session:**
1. Update the **Current Project Status** table above with what changed
2. Update the relevant Obsidian doc if you built or tested something
3. Commit and push your branch
4. If merging to `main`: move the `[Unreleased]` CHANGELOG entry to a versioned heading

**Branch convention:**
- `main` — stable, tested code only
- `feature/<name>` — one logical feature per branch
- Keep branches short-lived (merge within a session or two)
- Do not merge to `main` without testing on hardware or mock_serial

**Claude Code on this project:**
- Each team member runs their own Claude Code session
- Claude reads this file automatically at session start — it is the single source of truth
- Do not rely on Claude's memory of previous sessions — trust the CLAUDE.md files and CHANGELOG
- If Claude gives advice that contradicts this file, this file wins
