# LitterBot -- Deploy and Standby System

> Safety gate system that keeps the robot completely inert on boot. The operator must deliberately deploy the robot through the web UI before any motors, arm, or scoop commands are accepted.
>
> **Last updated: 2026-04-01**

---

## Purpose

LitterBot weighs 42 lbs and has powerful DC motors, a stepper-driven arm, and a servo-actuated scoop. If the robot powered on in an active state, accidental commands (stale topics, reconnecting bridges, operator error) could cause unexpected movement.

The Deploy and Standby system solves this by making **STANDBY the boot default**. In STANDBY, all motion commands are rejected at the firmware level. The operator must perform a deliberate 5-second press-and-hold in the web UI to transition to ACTIVE. This guarantees that the robot never moves without explicit human intent.

---

## Prerequisites

| Requirement | Details |
|-------------|---------|
| STM32 firmware | Must include `deploy_state` module and updated `uart_cmd` with command gating |
| ROS2 stack | `stm32_bridge` and `ui_server` nodes with deploy/state topic support |
| Web UI | Updated `index.html` with standby overlay and deploy button |
| Arm hardware | Limit switch wired for homing (used during deploy sequence) |

---

## Source Files

| File | Path | Purpose |
|------|------|---------|
| `deploy_state.h` | `firmware/stm32/LitterBot/Core/Inc/` | State enum, gate check function, dev mode API |
| `deploy_state.c` | `firmware/stm32/LitterBot/Core/Src/` | State machine implementation |
| `arm_control.h` | `firmware/stm32/LitterBot/Core/Inc/` | Arm_Home() declaration |
| `arm_control.c` | `firmware/stm32/LitterBot/Core/Src/` | Arm_Home() implementation for deploy sequence |
| `uart_cmd.c` | `firmware/stm32/LitterBot/Core/Src/` | Command gating logic and `D DEPLOY` handler |
| `stm32_bridge.py` | `ros2/litterbot_bridge/litterbot_bridge/` | Serial read thread, deploy/dev topic handling |
| `ui_server.py` | `ros2/litterbot_bridge/litterbot_bridge/` | WebSocket deploy and state message handlers |
| `index.html` | `ros2/litterbot_bridge/litterbot_bridge/ui/` | Standby overlay, deploy button, dev mode toggle |

---

## State Machine

The state machine is owned by the STM32 firmware. The Jetson and web UI mirror the state but do not control it directly -- they send commands, and the STM32 decides the transitions.

```
STANDBY --(D DEPLOY)--> DEPLOYING --(arm homing OK)--> ACTIVE
   ^                        |                             |
   |                   (homing fail)                      |
   +------------------------+          (return to standby)
   +------------------------------------------------------+
```

### States

| State | Description | Boot default |
|-------|-------------|:------------:|
| STANDBY | All gated commands rejected with `ERR:STANDBY` response. Robot is completely inert. | Yes |
| DEPLOYING | Arm homing in progress (blocking). Transitional state. | -- |
| ACTIVE | Normal operation. All commands accepted. | -- |

---

## Command Gating

Commands are divided into two categories at the firmware level:

### Always Allowed (any state)

| Command | Purpose |
|---------|---------|
| `X` | Emergency stop |
| `Q` | State query |
| `T` | Encoder test |
| `DEV ON` / `DEV OFF` | Toggle dev mode |

### Gated (blocked in STANDBY)

| Command | Purpose |
|---------|---------|
| `M <fl> <fr> <rl> <rr>` | Motor drive |
| `S SCOOP` | Scoop sequence |
| `C CALIBRATE` | Arm calibration |

When a gated command is received in STANDBY (without dev mode), the firmware responds with `ERR:STANDBY` and takes no action.

---

## Deploy Sequence (step by step)

1. **UI:** Operator presses and holds the DEPLOY button for 5 seconds. A progress bar fills during the hold to confirm intent.
2. **UI --> WebSocket:** Browser sends `{"type":"deploy"}` message.
3. **ui_server:** Receives WebSocket message, publishes to `/deploy_cmd` ROS2 topic.
4. **stm32_bridge:** Receives `/deploy_cmd`, sends `D DEPLOY\n` over UART to STM32.
5. **STM32:** Sets state to DEPLOYING, calls `Arm_Home()`.
6. **Arm_Home():** Retracts arm to the HOME limit switch position. Returns immediately if the arm is already home.
7. **On success:** State transitions to ACTIVE. STM32 responds with `DEPLOY OK\r\n`.
8. **On failure:** State transitions back to STANDBY. STM32 responds with `DEPLOY FAIL <code>\r\n`.
9. **stm32_bridge:** Reads the response, then queries state with `Q\n` to confirm.
10. **State propagation:** `/stm32_state` topic --> `ui_server` --> WebSocket --> browser overlay is removed (or re-shown on failure).

Re-deploying from ACTIVE is supported -- sending `D DEPLOY` while already active will re-run the homing sequence.

---

## Dev Mode

Dev mode is a bypass for development and testing. When enabled, all commands pass through regardless of the current state.

### Toggling

| Command | Effect |
|---------|--------|
| `DEV ON` | Enable dev mode -- all commands accepted regardless of state |
| `DEV OFF` | Disable dev mode -- normal gating resumes |

### UI Indicators

- **Yellow banner** across the top of the UI: "DEV MODE ACTIVE -- SAFETY GATE BYPASSED"
- **Toggle button** in the topbar to enable/disable from the browser

Dev mode is intended for bench testing and debugging only. It should not be used during normal operation or demos.

---

## ROS2 Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/deploy_cmd` | `std_msgs/String` | ui_server --> stm32_bridge | Trigger deploy sequence |
| `/dev_mode_cmd` | `std_msgs/Bool` | ui_server --> stm32_bridge | Toggle dev mode |
| `/stm32_state` | `std_msgs/String` | stm32_bridge --> ui_server | Current state (STANDBY/DEPLOYING/ACTIVE) |
| `/stm32_rx` | `std_msgs/String` | stm32_bridge --> ui_server | Raw UART responses from STM32 |

---

## UI Elements

| Element | Location | Behavior |
|---------|----------|----------|
| Standby overlay | Full-screen overlay | Blocks entire UI with "ROBOT IN STANDBY" message. Cannot interact with controls underneath. |
| Deploy button | Center of standby overlay | 5-second press-and-hold with animated progress bar. Releases early = cancels. |
| State indicator dot | Status bar | Red = STANDBY, Orange = DEPLOYING, Green = ACTIVE |
| Dev mode toggle | Topbar | Button to send `DEV ON` / `DEV OFF` |
| Dev mode banner | Top of UI (below topbar) | Yellow warning banner when dev mode is active |

---

## Configuration

### Firmware (`deploy_state.h`)

The state machine has no configurable parameters beyond the state enum itself. Command gating is hardcoded in `uart_cmd.c` -- any new commands that control hardware should be added to the gated list.

### UI (`index.html`)

The deploy hold duration is set in the frontend JavaScript:

```javascript
// Deploy button hold time in milliseconds
const DEPLOY_HOLD_MS = 5000;
```

---

## Known Limitations

- **Arm_Home() is blocking.** While the arm is homing during the DEPLOYING state, the STM32 cannot process UART commands. This means an emergency stop (`X`) sent during homing will not be processed until homing completes or fails.
- **Future improvement:** Replace the blocking `Arm_Home()` call with a non-blocking state machine inside `UartCmd_Task()` so UART remains responsive during homing.

---

## Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| Deploy button does nothing | WebSocket disconnected -- check browser console and connection indicator |
| `ERR:STANDBY` in logs | A gated command was sent while in STANDBY without dev mode enabled |
| Deploy fails immediately | Arm limit switches not connected or wired incorrectly |
| Overlay won't dismiss after deploy | State message not reaching UI -- check `/stm32_state` topic with `ros2 topic echo /stm32_state` |
| Dev mode won't toggle | stm32_bridge not forwarding DEV commands -- check serial connection and bridge logs |
| Robot moves unexpectedly on boot | Firmware not updated -- ensure `deploy_state` module is compiled in and state defaults to STANDBY |

---

## Related Documentation

- [[Command Center UI]] -- Web UI architecture and access
- [[Arm Calibration and Control]] -- Arm homing and calibration details
- [[STM32 System Testing]] -- Firmware test procedures
- [[ROS2 Development]] -- Bridge node and topic documentation
