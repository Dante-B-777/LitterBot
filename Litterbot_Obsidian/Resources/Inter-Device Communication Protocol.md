# Inter-Device Communication Protocol

This document is the single source of truth for all communication between the Jetson (ROS2) and the STM32. It covers physical layer settings, every command in both directions, response formats, error codes, and timing rules.

---

## Physical Layer — Jetson ↔ STM32

| Parameter | Value |
|-----------|-------|
| Peripheral (STM32 side) | LPUART1 |
| Baud rate | 115,200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| Full spec | **8N1** |
| STM32 TX pin | PG7 (NUCLEO ST-LINK VCP) |
| STM32 RX pin | PG8 (NUCLEO ST-LINK VCP) |
| Physical connector | USB Type-B (NUCLEO CN1) |
| Jetson side | `/dev/ttyACM0` (USB-UART via ST-LINK) |
| Line terminator — Jetson → STM32 | `\n` (LF only) |
| Line terminator — STM32 → Jetson | `\r\n` (CR+LF) |

> **IMPORTANT — CubeMX action required before hardware testing:**
> LPUART1 word length is currently set to **9 Bits** in CubeMX. This must be changed to **8 Bits** to match 8N1. Until fixed, all UART communication will be corrupted.

---

## Commands — Jetson → STM32

All commands are plain ASCII, terminated with `\n`. The STM32 ignores `\r` and strips it if present. Commands longer than 80 characters are truncated.

### Command Gating (Safety)

Commands are divided into access levels. The STM32 enforces these regardless of what the Jetson sends.

| Level | When allowed |
|-------|-------------|
| `[ALWAYS]` | Any state, including STANDBY |
| `[GATED]` | Only in ACTIVE state, OR when dev mode is ON |
| `[STANDBY/ACTIVE]` | Only in STANDBY or ACTIVE (not mid-deploy) |

If a `[GATED]` command is sent while in STANDBY with dev mode OFF, the STM32 replies `ERR:STANDBY\r\n` and takes no action.

---

### Motor Command

```
M <fl> <fr> <rl> <rr>\n
```

| Field | Description |
|-------|-------------|
| `fl` | Front-left wheel speed, integer -1000 to +1000 |
| `fr` | Front-right wheel speed, integer -1000 to +1000 |
| `rl` | Rear-left wheel speed, integer -1000 to +1000 |
| `rr` | Rear-right wheel speed, integer -1000 to +1000 |
| Access | `[GATED]` |
| STM32 response | None (silent accept) |

Speed mapping: the STM32 divides each value by 10 to get a percentage (-100.0% to +100.0%) for `MD10C_SetSpeed()`. The right-side motors (FR, RR) have their sign inverted internally because they are physically mounted facing the opposite direction.

Values outside -1000 to +1000 are clamped at the STM32.

Examples:
```
M 500 500 500 500\n      → all wheels forward at 50% PWM
M 0 0 0 0\n              → all wheels stopped (watchdog keep-alive)
M -300 -300 -300 -300\n  → all wheels reverse at 30%
M 500 -500 500 -500\n    → strafe right (mecanum)
```

**Watchdog:** The STM32 auto-stops all motors if no `M` command arrives within **250 ms**. The ROS2 `stm32_bridge` sends `M 0 0 0 0\n` at ~4 Hz when paused to keep the watchdog satisfied without moving.

---

### Emergency Stop

```
X\n
```

| Field | Description |
|-------|-------------|
| Access | `[ALWAYS]` — never gated |
| Action | Calls `MD10C_StopAll()` immediately. Also resets the watchdog timer. |
| STM32 response | None (silent) |

This is a hardware-level stop. Use for faults, collisions, and any safety-critical situation. Not resumable via `M 0 0 0 0` — you must send a new motor command after the situation is resolved.

Distinguish from pause (`M 0 0 0 0\n`): E-stop is irreversible-until-commanded; pause is a soft zero that the watchdog accepts.

---

### Scoop Sequence

```
S SCOOP\n
```

| Field | Description |
|-------|-------------|
| Access | `[GATED]` |
| Action | Stops drive motors, executes the 6-phase coordinated stepper + servo scoop motion |
| STM32 response | None (blocking — returns to main loop when complete) |

The full keyword `S SCOOP` is required (not just `S`). This prevents noise or partial packets from accidentally triggering the scoop. All numeric constants in `scoop_sequence.h` are currently placeholders — they must be measured on the physical hardware before use.

---

### Arm Calibration

```
C CALIBRATE\n
```

| Field | Description |
|-------|-------------|
| Access | `[GATED]` |
| Action | Stops drive motors, extends arm slowly to physical limit, retracts to home, prints step counts over LPUART1 |
| Emergency stop during calibration | Press **B1** (PC13) — latches an e-stop that halts the stepper |
| STM32 response | Prints step counts as plain text over LPUART1 during execution |

The full keyword `C CALIBRATE` is required (not just `C`).

---

### Deploy

```
D DEPLOY\n
```

| Field | Description |
|-------|-------------|
| Access | `[STANDBY/ACTIVE]` — blocked mid-deploy (DEPLOYING state) |
| Action | Runs arm homing sequence, then transitions state machine: STANDBY → DEPLOYING → ACTIVE |
| If called from ACTIVE | Returns to STANDBY first, then re-deploys |
| STM32 response on success | `DEPLOY OK\r\n` |
| STM32 response on failure | `DEPLOY FAIL <code>\r\n` |

After a successful deploy, `[GATED]` commands (M, S SCOOP, C CALIBRATE) become active.

---

### Dev Mode Toggle

```
DEV ON\n
DEV OFF\n
```

| Field | Description |
|-------|-------------|
| Access | `[ALWAYS]` |
| Action | Enables or disables dev mode. Dev mode bypasses the deploy state gate for all `[GATED]` commands. |
| STM32 response | `DEV ON\r\n` or `DEV OFF\r\n` (echo confirmation) |

Dev mode is intended for bench testing without going through the full deploy sequence. It does **not** affect `[ALWAYS]` or `[STANDBY/ACTIVE]` commands — only `[GATED]` ones.

---

### State Query

```
Q\n
```

| Field | Description |
|-------|-------------|
| Access | `[ALWAYS]` |
| Action | Requests the current deploy state |
| STM32 response | `STATE <STANDBY\|DEPLOYING\|ACTIVE>\r\n` (normal) |
| With dev mode on | `STATE <state> DEV\r\n` |

The ROS2 `stm32_bridge` sends `Q\n` on serial connect to immediately sync state.

---

### Encoder Test (Debug)

```
T\n
```

| Field | Description |
|-------|-------------|
| Access | `[ALWAYS]` |
| Action | Runs ~5-second encoder diagnostic. Prints raw tick counts for all 4 wheels over LPUART1. |
| STM32 response | Encoder count data printed as plain text during the ~5 sec window |

Debug use only. Blocking during the test window.

---

## Responses — STM32 → Jetson

All responses are ASCII terminated with `\r\n`.

| Response | Trigger | Meaning |
|----------|---------|---------|
| `STATE STANDBY\r\n` | `Q\n` query | Robot is in standby — drive commands gated |
| `STATE DEPLOYING\r\n` | `Q\n` query | Deploy sequence in progress |
| `STATE ACTIVE\r\n` | `Q\n` query | Robot is deployed and operational |
| `STATE STANDBY DEV\r\n` | `Q\n` with dev mode | Standby + dev mode on |
| `STATE ACTIVE DEV\r\n` | `Q\n` with dev mode | Active + dev mode on |
| `DEPLOY OK\r\n` | `D DEPLOY\n` success | Deploy sequence completed |
| `DEPLOY FAIL <n>\r\n` | `D DEPLOY\n` failure | Deploy failed; `<n>` is error code integer |
| `DEV ON\r\n` | `DEV ON\n` | Dev mode enabled (echo confirmation) |
| `DEV OFF\r\n` | `DEV OFF\n` | Dev mode disabled (echo confirmation) |
| `ERR:STANDBY\r\n` | Any `[GATED]` command while in STANDBY without dev mode | Command rejected — robot not deployed |

**Telemetry (not yet implemented):** Planned encoder tick counts at ~50 Hz as `E <fl> <fr> <rl> <rr>\r\n`. Not active in current firmware.

---

## Timing and Watchdog

| Parameter | Value |
|-----------|-------|
| Motor command watchdog timeout | 250 ms |
| Watchdog action on timeout | `MD10C_StopAll()` — all 4 motors set to 0% PWM |
| Watchdog reset trigger | Any valid `M` command (including `M 0 0 0 0\n`) or `X\n` |
| ROS2 keep-alive rate (when paused) | ~4 Hz via `M 0 0 0 0\n` |
| UART HAL transmit timeout | 100 ms per frame (blocking HAL call on STM32) |

The watchdog means any loss of UART connectivity (Jetson crash, cable unplug) will automatically stop the drive motors within 250 ms. It does **not** stop the arm/stepper — those are only stopped by `X\n` or command completion.

---

## Deploy State Machine

```
                   Power-on
                      │
                      ▼
                  STANDBY ◄──────────────────────────────────────────┐
                      │                                              │
                 D DEPLOY\n                            DeployState_ReturnToStandby()
                      │                                              │
                      ▼                                              │
                 DEPLOYING                                      (re-deploy)
               (arm homing in progress)                             │
                      │                                              │
               homing complete                                       │
                      │                                              │
                      ▼                                              │
                   ACTIVE ──────── D DEPLOY\n (from ACTIVE) ────────┘
```

- **STANDBY:** Boot state. `[GATED]` commands rejected unless dev mode is on.
- **DEPLOYING:** Transient. Arm is homing. No commands accepted except `X\n`, `Q\n`, `DEV ON/OFF`.
- **ACTIVE:** Normal operating state. All commands active.

---

## Internal Bus — STM32 Servo (HX-65HM)

This is a separate internal UART bus for the servo motor only. It is **not** accessible from the Jetson directly — the STM32 handles it internally as part of `S SCOOP` and `D DEPLOY`.

| Parameter | Value |
|-----------|-------|
| Peripheral | USART3 |
| Mode | Half-duplex (single wire TX only) |
| Pin | PC10 (rewired from PB10 which was damaged) |
| Baud rate | 1,000,000 (1 Mbaud) |
| Protocol | Hiwonder LX-series packet: `0x55 0x55 ID LEN CMD [params] CHKSUM` |
| GPIO requirement | Push-pull (override CubeMX open-drain default in `USART3_MspInit`) |

The servo is addressed by ID. Position range is 0–1000 (mapped to 0°–240° for this model).

---

## Quick Reference Card

```
--- JETSON → STM32 (all end with \n) ---

M <fl> <fr> <rl> <rr>    Drive wheels, -1000 to +1000     [GATED]
X                         Emergency stop                    [ALWAYS]
S SCOOP                   Run scoop sequence                [GATED]
C CALIBRATE               Run arm calibration               [GATED]
D DEPLOY                  Deploy robot                      [STANDBY/ACTIVE]
DEV ON                    Enable dev mode (bypass gate)     [ALWAYS]
DEV OFF                   Disable dev mode                  [ALWAYS]
Q                         Query state                       [ALWAYS]
T                         Encoder test (debug)              [ALWAYS]

--- STM32 → JETSON (all end with \r\n) ---

STATE STANDBY             Robot in standby
STATE DEPLOYING           Deploy in progress
STATE ACTIVE              Robot operational
STATE <state> DEV         Any state + dev mode on
DEPLOY OK                 Deploy succeeded
DEPLOY FAIL <n>           Deploy failed
DEV ON                    Dev mode enabled (echo)
DEV OFF                   Dev mode disabled (echo)
ERR:STANDBY               Gated command rejected

--- PHYSICAL ---

LPUART1  |  115200 8N1  |  PG7 TX / PG8 RX  |  /dev/ttyACM0
```
