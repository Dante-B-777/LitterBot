# LitterBot Project

## Hardware
- [[DC-Motors]]
- [[Stepper-Motor]]
- [[Servo-Motor]]
- [[Limit-Switches]]
- [[STM32-Nucleo]]

## Technical Documentation

### Development Notes
- [[Project Progress]] -- current status and next steps
- [[ROS2 Development]] -- node architecture, protocols, kinematics
- [[Integration Notes]] -- hardware bring-up sessions, bugs, fixes
- [[Arm Design Considerations]] -- design decisions with rationale

### System Guides
- [[Command Center UI]] -- web UI features, network setup, hotspot deployment
- [[Deploy and Standby System]] -- safety gate, state machine, dev mode
- [[Arm Calibration and Control]] -- calibration procedure, servo interpolation
- [[STM32 System Testing]] -- test procedures for each subsystem
- [[Jetson Migration Guide]] -- WSL2 to Jetson deployment steps

### Future
- [[Computer Vision Pipeline Options]] -- YOLO, RealSense, SLAM planning

## Resources
- [[Master Pin Assignments]] -- THE definitive STM32 pinout reference
- [[Inter-Device Communication Protocol]] -- all Jetson↔STM32 commands, UART settings, responses, timing
- [[Datasheets]]

## Project Paths

Repository:
```
cd 'C:\Users\benji\Documents\GitHub\LitterBot_STM32'
```

Firmware:
```
cd 'C:\Users\benji\Documents\GitHub\LitterBot_STM32\firmware\stm32\LitterBot'
```
