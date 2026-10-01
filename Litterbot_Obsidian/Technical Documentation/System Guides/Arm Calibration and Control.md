# Arm Calibration and Control

## Purpose

Calibrate the scoop arm travel distance (step count) so the firmware can run the scoop sequence with a hardcoded, safe step count. The limit switches provide fail-safe backup but are NOT load-bearing during normal operation.

## Prerequisites

- STM32 flashed with firmware from `feature/arm-control` branch
- Stepper motor wired: PUL+ = PD14, DIR+ = PF15
- Servo wired: USART3 TX on PC10
- Limit switches wired (NC to GND, COM to STM32):
  - HOME (fully retracted) = PD15
  - EXTEND (fully extended) = PE10
- Serial terminal connected to LPUART1 (115200 baud, 8N1)
- Power supplies on for stepper (24-48V) and servo (9-12.6V)

## Limit Switch Wiring

| Terminal | Connection | Notes |
|----------|-----------|-------|
| COM | STM32 GPIO pin (PD15 or PE10) | Signal wire |
| NC | GND | Fail-safe: wire break = triggered |
| NO | Not connected | Leave floating |

**Fail-safe behavior:** Internal pull-up on STM32. When switch is closed (normal), pin reads LOW. When switch opens (triggered) or wire breaks, pin reads HIGH = stop.

## File Locations

| File | Path | Purpose |
|------|------|---------|
| `limit_switch.h/.c` | `Core/Inc/` and `Core/Src/` | GPIO driver for both limit switches |
| `arm_control.h/.c` | `Core/Inc/` and `Core/Src/` | Calibration mode + normal extend/retract |
| `scoop_sequence.h/.c` | `Core/Inc/` and `Core/Src/` | 6-phase scoop using arm_control |
| `uart_cmd.c` | `Core/Src/` | Parses `C\n` command to trigger calibration |

## How to Run Calibration

1. Connect serial terminal to LPUART1 at 115200 baud
2. Have your hand on B1 (blue button on Nucleo) — this is your emergency stop
3. Send `C\n` (or type C and press Enter)
4. Watch the arm:
   - **Phase 1:** Arm extends slowly toward the EXTEND limit switch
   - When EXTEND switch triggers, it prints: `EXTEND limit hit at XXX steps.`
   - **Phase 2:** After 1 second pause, arm retracts toward HOME limit switch
   - When HOME switch triggers, it prints: `HOME limit hit at XXX steps.`
5. Final output shows both step counts
6. Take the **larger** of the two values and put it in `scoop_sequence.h` as `SCOOP_EXTEND_STEPS`

## Safety During Calibration

| Event | Result |
|-------|--------|
| B1 button pressed | **Latched stop** — motor halts immediately. Requires MCU reset to clear. |
| Wrong-direction limit switch (HOME triggers while extending) | **Immediate stop** + error message. Wiring may be reversed. |
| Wrong-direction limit switch (EXTEND triggers while retracting) | **Immediate stop** + error message. Wiring may be reversed. |
| Max steps reached (10,000) without switch trigger | **Stop** + error. Check switch wiring. |
| Expected limit switch triggers | **Normal stop** — step count recorded and printed. |

## Normal Scoop Operation

After calibration, the scoop uses the hardcoded `SCOOP_EXTEND_STEPS` value. Limit switches act as passive emergency backup only:

- If EXTEND switch triggers during a normal extend, the arm stops (something overshot)
- If HOME switch triggers during a normal retract, the arm stops
- If neither switch triggers, the arm runs its full step count and stops normally

## Servo Interpolation During Retract

During Phase 4 (retract), the servo angle changes as a function of stepper position:

```
servo_pos = CLOSED_POS + ((DUMP_POS - CLOSED_POS) * steps_done) / total_steps
```

Updated every 30 steps to avoid flooding the servo UART. This keeps the bucket angle changing smoothly as the arm pulls back, preventing litter from spilling.

## Pin Assignments

| Signal | Pin | Port | Config |
|--------|-----|------|--------|
| Stepper PUL+ | PD14 | GPIOD | Push-pull output |
| Stepper DIR+ | PF15 | GPIOF | Push-pull output |
| Limit HOME | PD15 | GPIOD | Input, pull-up, NC to GND |
| Limit EXTEND | PE10 | GPIOE | Input, pull-up, NC to GND |
| Servo SIG | PC10 | GPIOC | USART3 TX half-duplex |

## Configuration

Key constants in `scoop_sequence.h` (all are PLACEHOLDERS until measured):

```c
#define SCOOP_EXTEND_STEPS      500U    // Measure with calibration command
#define SCOOP_EXTEND_SPEED_US   300U    // Tune: smaller = faster
#define SCOOP_RETRACT_SPEED_US  200U    // Can be faster than extend
#define SCOOP_EXTEND_DIR        CL57T_DIR_CW  // Verify which direction extends
#define SCOOP_SERVO_ID          1U      // Confirm with servo ping
#define SCOOP_POS_HOME          512U    // Measure stowed angle
#define SCOOP_POS_OPEN          256U    // Measure open/flat angle
#define SCOOP_POS_CLOSED        768U    // Measure grip angle
#define SCOOP_POS_DUMP          1024U   // Measure dump angle
#define SCOOP_SERVO_MOVE_MS     500U    // Tune servo speed
```

Key constants in `arm_control.h`:

```c
#define ARM_CAL_SPEED_US            500U    // Calibration step delay (slow)
#define ARM_CAL_MAX_STEPS           10000U  // Safety limit for calibration
#define ARM_SERVO_UPDATE_INTERVAL   30U     // Steps between servo updates during retract
```

## Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| Calibration prints "HOME switch triggered while extending" | Limit switch wires are swapped (PD15 ↔ PE10) |
| Arm doesn't move during calibration | Stepper power not on, or PUL+/DIR+ wiring wrong |
| Max steps reached, no switch trigger | Switch not wired, wrong NC/NO terminal used, or wire broken |
| B1 doesn't stop the arm | B1 (PC13) not configured as input — check CubeMX |
| Servo doesn't move during scoop | Check USART3 baud (should be 1,000,000), check servo ID |
