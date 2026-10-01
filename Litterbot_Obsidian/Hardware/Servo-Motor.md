# Bus Servo

**Model:** Hiwonder HX-65HM
**Application:** Scoop bucket rotation (up/down)

## Role in Scooping Mechanism

The servo uses a **dual shaft** — one end connects to each arm at the shoulder joint — so a single servo controls the bucket angle on both arms simultaneously.

Intended motion: rotate down to open/lower the bucket at full extension, then rotate inward/upward to scoop litter and carry it back toward the ramp.

**Implemented:** Servo angle is a function of stepper position during the return stroke. `Arm_RetractWithServo()` linearly interpolates the servo from CLOSED to DUMP position as the arm retracts, updated every 30 steps. Exact angle range TBD (requires physical measurement on hardware).

See [[Stepper-Motor]] for the full scooping sequence.

## Specifications

- Voltage: 9-12.6V
- Torque: 50-65 KG·cm
- Range: 0-360° (0-4095 steps)
- Precision: 0.3°
- Current: 80mA no-load, 5A stall

## Communication

- Protocol: UART half-duplex
- Baud: 1,000,000 (1 Mbaud)
- Packet: Header (0xFF 0xFF) + ID + Length + Instruction + Parameters + Checksum
- ID Range: 0-253

## Operating Modes

1. Position Mode
2. Velocity Mode (closed-loop)
3. PWM Mode (open-loop)

## Implementation

**Files:** `hx65_servo.c/h`

**Peripherals:**
- USART3 half-duplex on PC10 (was PB10, pin damaged and rewired)

See [[Master Pin Assignments]] for full pin details.

## Common Commands

- PING (0x01): Query status
- READ_DATA (0x02): Read control table
- WRITE_DATA (0x03): Write control table
- SYNC_WRITE (0x83): Control multiple servos

## Daisy-Chain

- Up to 253 servos on single bus
- Assign unique IDs individually before chaining

## Datasheet

`docs/datasheets/Servo Motor/HX-65HM Bus Servo User Manual.pdf`
