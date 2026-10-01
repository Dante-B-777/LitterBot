# Changelog

Entries are added when merging to `main`. Use `[Unreleased]` for in-progress work on feature branches.

Format: one entry per session — date, author, bullet list of what changed.

---

## [Unreleased] — feature/arm-control

### 2026-04-05 session 10 (benji)
- Diagnosed DEV mode not working — Jetson had stale ui_server.py (missing dev_mode WebSocket handler)
- Fixed by rsync-ing all ROS2 files to Jetson + colcon rebuild — Jetson now fully in sync with local repo
- Confirmed DEV mode working: enable DEV in UI → WASD moves wheels without deploying
- Committed stray deploy_state.c fix (session 8 change that was flashed but never committed)
- Pushed all 16 commits to origin/feature/arm-control

### 2026-04-05 session 9 (benji)
- Fixed UART RX interrupt chain breaking on boot — added `HAL_UART_ErrorCallback` to re-arm `HAL_UART_Receive_IT` after errors
- Fixed UART interrupt storm — error callback now clears sticky ORE/FE/NE/PE flags before re-arming (without this, overrun flag caused infinite error loop, starving main loop)
- Fixed stm32_bridge dropping initial `Q\n` state query — added 500ms settle delay after serial open to let STM32 recover from USB connection glitch
- **RESULT: WASD teleop confirmed working 2026-04-05, wheels moving**
- Jetson repo updated via SCP (changes not yet pushed to remote — needs `git push` from Windows)

### 2026-04-03 session 8 (benji)
- Fixed deploy getting stuck on yellow DEPLOYING screen — `DeployState_StartDeploy()` now goes ACTIVE even if `Arm_Home()` fails (arm hardware not yet wired)
- Built and flashed full calibration store firmware (50,980 bytes) — `calibration_store.c` confirmed compiled in
- Diagnosed and fixed commands sending 6× — multiple stm32_bridge processes were accumulating from repeated launches; fixed with kill-by-PID + clean relaunch
- Diagnosed deploy block on teleop — deploy state machine gates M commands; workaround is DEV ON, permanent fix is the deploy_state.c change above

### 2026-04-03 session 7 (benji)
- Added `calibration_store` firmware module — extend_steps and 4 servo positions stored in STM32 flash (Bank 2, page 255)
- Arm calibration (`C CALIBRATE`) now auto-saves extend_steps to flash on completion
- Added `CAL SERVO START/STOP` and `CAL SAVE HOME/OPEN/CLOSED/DUMP` UART commands to firmware
- Fixed `HX65_Init()` never being called in `Arm_Init()` (servo driver was silently broken)
- `scoop_sequence.c` now reads calibrated values from CalStore at runtime (falls back to #define defaults if uncalibrated)
- ROS2: added `/cal_cmd` topic routing cal commands from browser → STM32
- UI: calibration section in teleop panel — 2s hold arm cal button + servo setup workflow with 4 save buttons
- LD3 firmware crash pending investigation (see CLAUDE.md action items)

### 2026-04-02 (benji)
- Replaced full-screen deploy overlay with pane-level overlay scoped to teleop panel
- Deploy hold time reduced from 5s to 3s
- Added LiDAR status dot placeholder to status bar
- Created CHANGELOG.md and added doc conventions to CLAUDE.md
- Updated ros2/CLAUDE.md to reflect ui_server changes
