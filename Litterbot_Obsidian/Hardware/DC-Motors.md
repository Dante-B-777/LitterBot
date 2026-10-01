# DC Drive Motors

**Driver:** Cytron MD10C
**Application:** 4-wheel drive

## Specifications

- Type: Brushed DC with quadrature encoders
- Current: ~13A continuous per driver
- Control: PWM + DIR (sign-magnitude)
- Logic: 3.3V/5V tolerant

## Control Method

- PWM duty cycle = speed
- DIR pin = direction (HIGH/LOW)

## Implementation

**Files:** `cytron_md10c.c/h`

**Peripherals:**
- TIM2: PWM generation (4 channels, 20 kHz)
- GPIO: 4 direction pins
- TIM1, TIM3, TIM4, TIM8: Encoder inputs

## Pin Assignments

See [[Master Pin Assignments]]

## Datasheet

`docs/datasheets/Planetary Gear Motor Driver Datasheet.pdf`
