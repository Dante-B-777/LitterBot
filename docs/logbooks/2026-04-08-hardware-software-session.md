# 2026-04-08 Hardware + Software Session

## Summary

This session restored the Jetson/STM32/webapp workflow, added recording playback and odometry plumbing, and isolated two hardware-side issues:

- wheel drive failures were caused by insufficient battery output under load
- the front-right encoder still does not report ticks after multiple firmware pin-map tests and now appears to be a hardware-path issue

## What Was Verified

- WSL was installed and configured for SSH access to the Jetson
- passwordless SSH from WSL to the Jetson was set up
- STM32 firmware can be built headlessly from the repo
- STM32 firmware can be flashed through the Jetson using `st-flash`
- persistent STM32 serial alias `/dev/ttySTM32` was created on the Jetson
- `litterbot.service` now survives ACM port renumbering by using `/dev/ttySTM32`
- webapp teleop path was verified end to end:
  - webapp -> Jetson bridge -> STM32 -> state replies
- camera feed was integrated into the webapp
- LiDAR feed and live scan visualization were integrated into the webapp
- recordings page was added
- ROS bag recordings now support MP4 export for camera playback
- odom UI was added to the webapp
- STM32 encoder telemetry was implemented as `ENC <fl> <fr> <rl> <rr>`
- Jetson bridge now parses encoder telemetry and publishes `/encoder_raw`
- `odom_publisher` was added and wired into the ROS2 stack

## Battery / Drive Finding

The robot repeatedly failed to move even though:

- STM32 firmware was flashed successfully
- webapp teleop was sending repeated `M ...` commands
- STM32 state replies were normal

After reconnecting a freshly charged battery, wheel motion returned. This strongly indicates the original wheel non-motion was a battery or motor-power-delivery issue rather than a teleop or firmware-path issue.

## Encoder Status

Current observed encoder behavior:

- rear-left: working
- rear-right: working
- front-left: working after remap
- front-right: still not working

Multiple front-right firmware mappings were tested:

- `PA6/PA7`
- `PE3/PE4`

The front-right encoder remained dead after these mapping changes. Because the rest of the encoder pipeline is working, the remaining likely cause is a hardware fault on the front-right encoder path:

- bad encoder in the motor
- bad signal wire or crimp
- missing encoder power or ground
- damaged board input path

## Current Firmware Notes

The latest pushed STM32 state includes:

- raw stepper test command:
  - `TEST STEP <CW|CCW> <steps> [delay_us]`
- parser fix so `TEST ...` commands are not shadowed by the bare `T` encoder test

Latest tested front encoder mapping state in firmware:

- front-left: `PB6/PB7`
- front-right: `PE3/PE4`

This reflects the last cross-test configuration pushed to GitHub, not a final validated encoder map.

## Stepper Result

A dedicated raw stepper command was added and tested directly over the STM32 serial link. The command returned clean acknowledgements:

- `TEST STEP START ...`
- `TEST STEP OK ...`

Physical observation confirmed that this raw stepper test moved the stepper correctly.

## Webapp / Recordings State

The webapp now has:

- live camera panel
- live LiDAR panel and point plot
- odom panel
- recordings page at `/recordings`
- named test runs via prompt on `REC DATA`
- MP4 playback for exported camera recordings

## Branch / GitHub State

Work was pushed to:

- branch: `ben/full-scoop`

Latest session commit pushed:

- `0fdbd1c` - `Add raw stepper test command and latest encoder pin map`

## Recommended Next Steps

- fix or replace the front-right encoder hardware path
- settle the final front encoder pin map and clean up stale docs/comments
- decide whether odom should remain usable via `/cmd_vel` fallback when encoder data is incomplete
- optionally add a dedicated in-UI stepper/arm diagnostics panel
