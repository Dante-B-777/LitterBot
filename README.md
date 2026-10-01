# LitterBot — Autonomous Vision-Based Litter Collection Rover

A 42 lb mecanum-drive rover that detects, approaches, and collects ground-level
litter without human input. Senior project (EE 460/463/464) at Cal Poly San Luis
Obispo, September 2025 – June 2026.

Project write-up, photos, and recorded autonomy runs:
**[dantebenedetti.com/projects/litterbot.html](https://dantebenedetti.com/projects/litterbot.html)**

## Results

| Metric | Value |
|---|---|
| Mean cross-track error, nominal path following | 0.033 m |
| Mean cross-track error while avoiding obstacles | 0.059 m |
| Collection success rate, plastic bottles | 100 % |
| Training images behind the YOLO litter detector | ~12,700 |

## Architecture

Two compute domains, split so the hard real-time loop stays deterministic while
the perception stack runs at its own cadence.

```
NVIDIA Jetson Orin Nano (ROS 2 Humble)        STM32L4A6ZG (bare-metal HAL)
  perception  — YOLO litter detection           4× mecanum DC motors (Cytron MD10C)
  localization — EKF: wheel odom + LiDAR        quadrature encoders + odometry
  planning    — Theta* + pure pursuit          CL57T stepper (scoop extend)
  web UI      — live command center             HX65 servos (2-DOF arm)
        └───────── USB serial, ASCII protocol ─────────┘
```

## Layout

```
firmware/stm32/LitterBot/   STM32CubeIDE project — motor, encoder, stepper,
                            servo, odometry, arm control, flash-backed
                            calibration store, UART command parser
ros2/litterbot_bridge/      serial bridge, EKF localization, odom publisher,
                            mecanum kinematics, web UI server, launch file, tests
ros2/litterbot_vision/      RGB-D camera node, bag → video export
tools/                      motor characterization script and measured data
docs/                       build guides, logbooks, design specs, measurements
Litterbot_Obsidian/         engineering notebook
CLAUDE.md                   full subsystem brief: pin map, protocol, state machine
```

## Running it

Firmware builds in STM32CubeIDE (Ctrl+B) and flashes over ST-Link.

```bash
# on the Jetson
ros2 launch litterbot_bridge litterbot.launch.py \
  mock_serial:=false serial_port:=/dev/ttyACM0
# command center UI at http://<jetson-ip>:8765
```

Host-side tests, no hardware needed:

```bash
cd ros2/litterbot_bridge && python -m pytest test/
```

## Attribution

Three-engineer team. My scope: autonomy, motor control, embedded integration,
and CAD / 3D-printed parts. This repository is the portion of the work I wrote,
extracted from the team repository at
[benjam1ntavares/LitterBot_STM32](https://github.com/benjam1ntavares/LitterBot_STM32),
which holds the full shared history. Report media and vendor datasheets are
omitted here; the final report PDF is linked from the project page above.
