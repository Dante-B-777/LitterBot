# Calibration Store Design

**Date:** 2026-04-03
**Branch:** feature/arm-control (or new branch feature/calibration-store)
**Author:** benji

---

## Summary

Replace all hardcoded placeholder constants in `scoop_sequence.h` with values stored in STM32 flash and managed at runtime. Add firmware commands and a UI workflow to measure and save calibration values without rebuilding firmware. Values persist across power cycles and take effect immediately on save — no reboot required.

---

## Values Stored

| Value | Source | Default (fallback) |
|-------|--------|--------------------|
| `extend_steps` | Arm calibration (auto-measured) | `SCOOP_EXTEND_STEPS` (500) |
| `pos_home` | Servo setup (manual position) | `SCOOP_POS_HOME` (512) |
| `pos_open` | Servo setup (manual position) | `SCOOP_POS_OPEN` (256) |
| `pos_closed` | Servo setup (manual position) | `SCOOP_POS_CLOSED` (768) |
| `pos_dump` | Servo setup (manual position) | `SCOOP_POS_DUMP` (1024) |

Speed constants (`SCOOP_EXTEND_SPEED_US`, `SCOOP_RETRACT_SPEED_US`, `SCOOP_SERVO_MOVE_MS`) remain hardcoded — tuned by rebuilding after physical testing.

---

## Flash Storage

STM32L4A6ZG: 1MB dual-bank flash, 2KB pages.
- Reserved page: Bank 2, page 255 → address `0x080FF800`
- Layout: 2 x 64-bit doublewords (16 bytes total)

```
Word 0 (bytes 0-7):  [magic: 0xCAFEBABE (32b)] [extend_steps (32b)]
Word 1 (bytes 8-15): [pos_home (16b)] [pos_open (16b)] [pos_closed (16b)] [pos_dump (16b)]
```

Magic number `0xCAFEBABE` in word 0 indicates valid calibration data. If absent (erased flash = all 0xFF), all values fall back to `#define` defaults.

---

## New Firmware Module: `calibration_store.c/h`

```c
void     CalStore_Init(void);                         // Load from flash into RAM on boot
uint32_t CalStore_GetExtendSteps(void);               // Returns stored or default
uint16_t CalStore_GetServoPos(CalServoPos_t which);   // Returns stored or default
void     CalStore_SaveExtendSteps(uint32_t steps);    // Write to flash + update RAM
void     CalStore_SaveServoPos(CalServoPos_t which, uint16_t pos); // Write + update RAM

typedef enum { CAL_POS_HOME, CAL_POS_OPEN, CAL_POS_CLOSED, CAL_POS_DUMP } CalServoPos_t;
```

---

## Firmware Changes

### `scoop_sequence.c`
Replace all `SCOOP_*` constant references with `CalStore_Get*()` calls. The `#define` values in `scoop_sequence.h` become fallback defaults only.

### `arm_control.c`
- Add `HX65_Init(&huart3)` to `Arm_Init()` — **currently missing, servo driver UART handle is NULL**
- At end of `Arm_Calibrate()`: call `CalStore_SaveExtendSteps(larger_count)` and print `CAL SAVED: extend_steps=%lu\r\n`

### `uart_cmd.c`
New UART commands (all gated — require ACTIVE or dev mode):

| Command | Action |
|---------|--------|
| `CAL SERVO START\n` | `HX65_DisableTorque()`, set `servo_cal_active=true` |
| `CAL SERVO STOP\n` | `HX65_EnableTorque()`, set `servo_cal_active=false` |
| `CAL SAVE HOME\n` | `HX65_ReadPosition()` → `CalStore_SaveServoPos(CAL_POS_HOME, pos)` → print `CAL SAVED: pos_home=%u\r\n` |
| `CAL SAVE OPEN\n` | Same for OPEN |
| `CAL SAVE CLOSED\n` | Same for CLOSED |
| `CAL SAVE DUMP\n` | Same for DUMP |

`UartCmd_Task()`: when `servo_cal_active`, read and print `POS: XXXX\r\n` every 500ms.

### `main.c`
Add `CalStore_Init()` call in USER CODE BEGIN 2, before `UartCmd_Init`.

---

## New UART Commands Summary

| Command | Gate | Response |
|---------|------|----------|
| `C CALIBRATE\n` | GATED | Step counts printed, `CAL SAVED: extend_steps=XXXX\r\n` at end |
| `CAL SERVO START\n` | GATED | Torque off, periodic `POS: XXXX\r\n` every 500ms |
| `CAL SERVO STOP\n` | GATED | Torque on, position printing stops |
| `CAL SAVE HOME\n` | GATED | `CAL SAVED: pos_home=XXXX\r\n` |
| `CAL SAVE OPEN\n` | GATED | `CAL SAVED: pos_open=XXXX\r\n` |
| `CAL SAVE CLOSED\n` | GATED | `CAL SAVED: pos_closed=XXXX\r\n` |
| `CAL SAVE DUMP\n` | GATED | `CAL SAVED: pos_dump=XXXX\r\n` |

---

## ROS2 Changes

### `stm32_bridge.py`
New WebSocket message types mapped to UART commands:

| WS `type` | UART command sent |
|-----------|------------------|
| `cal_arm` | `C CALIBRATE\n` |
| `cal_servo_start` | `CAL SERVO START\n` |
| `cal_servo_stop` | `CAL SERVO STOP\n` |
| `cal_save_home` | `CAL SAVE HOME\n` |
| `cal_save_open` | `CAL SAVE OPEN\n` |
| `cal_save_closed` | `CAL SAVE CLOSED\n` |
| `cal_save_dump` | `CAL SAVE DUMP\n` |

---

## UI Changes (`index.html`)

New "Calibration" section inside `#teleop-panel`, below existing controls. Visible in all states; buttons disabled (grey) when not ACTIVE and not in dev mode.

### Arm Calibration
- 2-second hold button "CAL ARM" — same press-and-hold mechanic as Deploy button
- On complete: sends `cal_arm` WebSocket message
- Result visible in STM32→Jetson log (green panel)

### Servo Setup
- "Start Servo Setup" button → `cal_servo_start`
  - When active: 4 Save buttons appear (Save Home / Save Open / Save Closed / Save Dump)
  - "Done" button → `cal_servo_stop`
- Each Save button sends the corresponding `cal_save_*` message
- Confirmations visible in STM32→Jetson log

---

## What Does NOT Change

- `scoop_sequence.h` `#define` values — kept as fallback defaults
- `SCOOP_EXTEND_DIR`, speed constants — remain hardcoded (require physical verification/tuning)
- `SCOOP_SERVO_ID` — remains hardcoded (verify ID=1 separately)
- Deploy sequence (`D DEPLOY\n` / `Arm_Home()`) — unchanged
- WebSocket protocol for motor commands, e-stop, scoop — unchanged
