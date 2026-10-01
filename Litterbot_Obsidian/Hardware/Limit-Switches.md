# Limit Switches

**Last updated:** 2026-04-01

## Overview
Two NC (normally closed) micro limit switches define the arm travel limits. They are wired fail-safe: NC contacts connect the switch pin to GND. When the switch is triggered (arm reaches limit), the circuit opens and the pin reads HIGH via internal pull-up.

## Specifications
- Type: Micro limit switch, NC (normally closed) contacts
- Wiring: NC terminal to GND, COM terminal to STM32 GPIO
- Logic: HIGH = triggered (switch open), LOW = normal (switch closed to GND)
- STM32 config: GPIO input mode with internal pull-up resistor

## Pin Assignments

| Switch | Function | Pin | Port | GPIO Config |
|--------|----------|-----|------|-------------|
| HOME | Arm fully retracted | PD15 | GPIOD | Input, pull-up |
| EXTEND | Arm fully extended | PE10 | GPIOE | Input, pull-up |

## Why NC (Normally Closed) Wiring?

This is a fail-safe design. If a wire breaks or disconnects:
- Broken wire = circuit open = pin reads HIGH = "triggered"
- The system treats a broken wire the same as hitting the limit
- This prevents the arm from continuing to move into a hard stop

With NO (normally open) wiring, a broken wire would look like "not triggered" -- the arm would keep moving past the limit and potentially damage something.

## Software Implementation

| File | Path | Purpose |
|------|------|---------|
| limit_switch.h | Core/Inc/limit_switch.h | API: Init, HomeTriggered, ExtendTriggered |
| limit_switch.c | Core/Src/limit_switch.c | GPIO init with pull-up, read functions |

### Key Functions
- `LimitSwitch_Init()` -- Configures GPIO clocks and pins as input with pull-up
- `LimitSwitch_HomeTriggered()` -- Returns true if HOME switch is triggered (HIGH)
- `LimitSwitch_ExtendTriggered()` -- Returns true if EXTEND switch is triggered (HIGH)

## Usage in Arm Control

The limit switches serve two roles:

1. **Calibration mode** (`C CALIBRATE` command): Active safety -- stops arm immediately if wrong-direction switch triggers, counts steps between HOME and EXTEND
2. **Normal operation**: Passive emergency backup -- checked during extend/retract but primary motion uses hardcoded step counts from calibration

During deploy sequence (`D DEPLOY`), `Arm_Home()` retracts until the HOME switch triggers.

## Wiring Diagram
```
STM32 Pin (PD15 or PE10)
    |
    +---[NC contact]---[switch]---GND
    |
    +---[internal pull-up to 3.3V]
    
Normal: NC closed -> pin pulled to GND -> LOW
Triggered: NC opens -> pull-up pulls to 3.3V -> HIGH
Wire break: same as triggered -> HIGH (fail-safe)
```

## Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| Switch always reads triggered (HIGH) | Wire disconnected, NC terminal not wired to GND |
| Switch never triggers | Wired to NO terminal instead of NC |
| Arm calibration reports wrong-direction error | HOME and EXTEND switches swapped |
| Arm homes but overshoots | Switch mechanical play, adjust mounting position |

## See Also
- [[Stepper-Motor]] -- stepper drives the arm that these switches limit
- [[Arm Calibration and Control]] -- calibration procedure using these switches
- [[Master Pin Assignments]] -- complete pin reference
