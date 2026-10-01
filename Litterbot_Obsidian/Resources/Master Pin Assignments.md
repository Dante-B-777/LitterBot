# Master Pin Assignments -- STM32L4A6ZG

> **The single definitive reference for every pin used in the LitterBot project.**
> MCU: STM32L4A6ZG on NUCLEO-144 board | SYSCLK: ~47.333 MHz
>
> **Last updated:** 2026-04-01

---

## Complete Pin Map

Sorted alphabetically by pin name. Every STM32 pin used in this project appears exactly once here.

| Pin   | Port   | Peripheral | Function                  | Subsystem              | Notes                                      |
|-------|--------|------------|---------------------------|------------------------|--------------------------------------------|
| PA0   | GPIOA  | TIM2_CH1   | FL motor PWM              | [[DC-Motors]]          | ~20 kHz, duty 0-2366                       |
| PA1   | GPIOA  | TIM2_CH2   | FR motor PWM              | [[DC-Motors]]          | ~20 kHz, duty 0-2366                       |
| PA2   | GPIOA  | TIM2_CH3   | RL motor PWM              | [[DC-Motors]]          | ~20 kHz, duty 0-2366                       |
| PA3   | GPIOA  | TIM2_CH4   | RR motor PWM              | [[DC-Motors]]          | ~20 kHz, duty 0-2366                       |
| PD12  | GPIOD  | TIM4_CH1   | FL encoder CHA            | [[Encoders]]           | White wire, per current CubeMX `.ioc`      |
| PD13  | GPIOD  | TIM4_CH2   | FL encoder CHB            | [[Encoders]]           | Blue wire, per current CubeMX `.ioc`       |
| PE3   | GPIOE  | TIM3_CH1   | FR encoder CHA            | [[Encoders]]           | White wire, per current CubeMX `.ioc`      |
| PE4   | GPIOE  | TIM3_CH2   | FR encoder CHB            | [[Encoders]]           | Blue wire, per current CubeMX `.ioc`       |
| PA11  | GPIOA  | USB_OTG_FS | USB DM                    | System                 | USB OTG Full Speed                         |
| PA12  | GPIOA  | USB_OTG_FS | USB DP                    | System                 | USB OTG Full Speed                         |
| PB0   | GPIOB  | GPIO OUT   | FL motor DIR              | [[DC-Motors]]          |                                            |
| PB1   | GPIOB  | GPIO OUT   | FR motor DIR              | [[DC-Motors]]          |                                            |
| PB2   | GPIOB  | GPIO OUT   | RL motor DIR              | [[DC-Motors]]          |                                            |
| PB14  | GPIOB  | GPIO OUT   | LD3 LED (red)             | System                 | Used by encoder test as alternate indicator |
| PC6   | GPIOC  | TIM8_CH1   | RL encoder CHA            | [[Encoders]]           | White wire                                 |
| PC7   | GPIOC  | TIM8_CH2   | RL encoder CHB            | [[Encoders]]           | Blue wire                                  |
| PC10  | GPIOC  | USART3 TX  | Bus servo signal          | [[Servo]]              | Half-duplex, 1 Mbaud, push-pull override   |
| PC13  | GPIOC  | GPIO IN    | B1 user button            | System                 | Active LOW, internal pull-up               |
| PD14  | GPIOD  | GPIO OUT   | Stepper PUL+ (step pulse) | [[Stepper-Motor]]      | DWT bit-bang timing                        |
| PD15  | GPIOD  | GPIO IN    | Limit switch HOME         | [[Stepper-Motor]]      | NC wiring, pull-up, HIGH = triggered       |
| PE9   | GPIOE  | TIM1_CH1   | RR encoder CHA            | [[Encoders]]           | White wire                                 |
| PE10  | GPIOE  | GPIO IN    | Limit switch EXTEND       | [[Stepper-Motor]]      | NC wiring, pull-up, HIGH = triggered       |
| PE11  | GPIOE  | TIM1_CH2   | RR encoder CHB            | [[Encoders]]           | Blue wire                                  |
| PF14  | GPIOF  | GPIO OUT   | RR motor DIR              | [[DC-Motors]]          | Was PB10 originally, moved for servo       |
| PF15  | GPIOF  | GPIO OUT   | Stepper DIR+ (direction)  | [[Stepper-Motor]]      |                                            |
| PG7   | GPIOG  | LPUART1 TX | Serial TX to Jetson/PC    | [[UART-Communications]]| ST-LINK VCP, 115200 baud                   |
| PG8   | GPIOG  | LPUART1 RX | Serial RX from Jetson/PC  | [[UART-Communications]]| ST-LINK VCP, 115200 baud                   |

---

## Timer Allocation

| Timer | Function                   | Pins              | Mode              | Config                        |
|-------|----------------------------|--------------------|-------------------|-------------------------------|
| TIM2  | DC motor PWM (4 channels)  | PA0, PA1, PA2, PA3 | PWM output        | PSC=0, ARR=2366, ~20 kHz     |
| TIM3  | FR encoder                 | PE3, PE4           | Encoder (TI1+TI2) | Period=65535, 4x counting     |
| TIM4  | FL encoder                 | PD12, PD13         | Encoder (TI1+TI2) | Period=65535, 4x counting     |
| TIM8  | RL encoder                 | PC6, PC7           | Encoder (TI1+TI2) | Period=65535, 4x counting     |
| TIM1  | RR encoder                 | PE9, PE11          | Encoder (TI1+TI2) | Period=65535, 4x counting     |
| DWT   | Stepper microsecond delays | (internal)         | Cycle counter      | SYSCLK = 47.333 MHz          |

---

## Subsystem Quick Reference

### DC Motors (4x Cytron MD10C)

See [[DC-Motors]] for driver details. Motor wire colors: FL=red, FR=blue, RL=green, RR=yellow.

| Motor | PWM Pin | Timer Channel | DIR Pin | Values       |
|-------|---------|---------------|---------|--------------|
| FL    | PA0     | TIM2_CH1      | PB0     | -1000 to 1000 |
| FR    | PA1     | TIM2_CH2      | PB1     | -1000 to 1000 |
| RL    | PA2     | TIM2_CH3      | PB2     | -1000 to 1000 |
| RR    | PA3     | TIM2_CH4      | PF14    | -1000 to 1000 |

Command protocol: `M <fl> <fr> <rl> <rr>\n` over LPUART1. E-stop: `X\n`. Watchdog timeout: 250 ms.

### Encoders (4x goBILDA Yellow Jacket 5203-2402-0019)

See [[Encoders]] for driver details. Hall-effect quadrature, 28 CPR at motor shaft, gear ratio 19.2:1, yielding 537.7 ticks/rev in 4x counting mode. Wire colors: White=CHA, Blue=CHB.

| Wheel | CHA Pin | CHB Pin | Timer |
|-------|---------|---------|-------|
| FL    | PD12    | PD13    | TIM4  |
| FR    | PE3     | PE4     | TIM3  |
| RL    | PC6     | PC7     | TIM8  |
| RR    | PE9     | PE11    | TIM1  |

### Stepper Motor (CL57T driver, NEMA 23)

See [[Stepper-Motor]] for driver details. 5V logic driver, can accept 3.3V (tested). Encoder internally powered by driver.

| Signal | Pin  | Wiring             |
|--------|------|--------------------|
| PUL+   | PD14 | Green wire to GPIO |
| DIR+   | PF15 | Blue wire to GPIO  |
| PUL-   | GND  | Black wire         |
| DIR-   | GND  | Black wire         |

Originally assigned to PA0/PB0 -- reassigned to PD14/PF15 to resolve conflict with DC motor FL.

### Limit Switches (Micro switches, NC wiring)

| Switch | Pin  | Trigger State | Location Rationale                           |
|--------|------|---------------|----------------------------------------------|
| HOME   | PD15 | HIGH          | Adjacent to stepper PUL+ (PD14) on morpho    |
| EXTEND | PE10 | HIGH          | Near encoder pins PE9/PE11 on morpho          |

Wiring: Switch COM to STM32 pin, Switch NC terminal to GND. Normal state: pin reads LOW. Triggered (or wire break): pin reads HIGH (fail-safe).

### Bus Servo (HX-65HM)

See [[Servo]] for driver details.

| Signal | Pin  | Peripheral | Config                                  |
|--------|------|------------|-----------------------------------------|
| SIG    | PC10 | USART3 TX  | Half-duplex, 1,000,000 baud, push-pull  |

- VIN: 9-12.6V external supply (NOT from STM32)
- GND: Common ground with STM32
- USART3 is configured as open-drain by CubeMX for half-duplex; **must override to push-pull** in `USART3_MspInit` USER CODE section
- Ribbon cable orientation: On servo housing (curve up) L to R: G, P, BUS. On STM side (curve up) L to R: BUS, P, G
- Pin history: Originally PC4, moved to PB10 (Week 4 logbook), PB10 physically damaged, rewired to PC10

### UART Communications

| Peripheral | Function              | TX Pin | RX Pin | Baud      | Mode         |
|------------|-----------------------|--------|--------|-----------|--------------|
| LPUART1    | Jetson/PC serial link | PG7    | PG8    | 115200    | Full duplex  |
| USART3     | Bus servo (HX-65HM)  | PC10   | --     | 1,000,000 | Half-duplex  |

**LPUART1 known issue:** CubeMX currently sets word length to `UART_WORDLENGTH_9B`. Must be changed to **8 Bits (including Parity)** for standard 8N1 serial. PG7/PG8 are the NUCLEO ST-LINK VCP pins.

### Board/System Pins

| Pin  | Function         | Config                      | Notes                                     |
|------|------------------|-----------------------------|-------------------------------------------|
| PC13 | B1 user button   | GPIO input, active LOW      | Internal pull-up                           |
| PB7  | LD2 LED (green)  | GPIO output                 | Currently still configured as LED in CubeMX |
| PB14 | LD3 LED (red)    | GPIO output                 | Alternate status indicator                 |
| PA11 | USB OTG FS DM    | USB                         |                                            |
| PA12 | USB OTG FS DP    | USB                         |                                            |

---

## Known Conflicts

| Pin | Conflict                                      | Resolution                                                                 |
|-----|-----------------------------------------------|----------------------------------------------------------------------------|
| PB7 | Historical conflict with FL encoder CHB docs   | Current CubeMX uses `PD12/PD13` for FL encoder, so PB7 remains LD2 unless `.ioc` is changed. |
| PB10| (Historical) Was RR DIR, then servo USART3 TX | RR DIR moved to PF14. PB10 later physically damaged. Servo rewired to PC10. Pin is no longer used. |

---

## Resolved Pin History

These conflicts have been fully resolved and are documented here for reference only.

| Original Assignment          | Conflict With              | Resolution                                    |
|------------------------------|----------------------------|-----------------------------------------------|
| PA0/PB0 for stepper          | DC motor FL (PA0 PWM, PB0 DIR) | Stepper moved to PD14 (PUL+) / PF15 (DIR+) |
| PB10 for RR motor DIR        | Servo USART3               | RR DIR moved to PF14                         |
| PB10 for servo USART3 TX     | Pin physically damaged     | Servo rewired to PC10                        |
