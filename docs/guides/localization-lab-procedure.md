# LitterBot Localization Lab Procedure

This guide is the lab procedure for bringing up the localization stack on the Jetson, calibrating the sensors, and running the commanded-trajectory validation test.

## Purpose

The localization stack under `ros2/litterbot_bridge` fuses:

- mecanum encoder odometry from the STM32
- RealSense D455f IMU data
- RPLidar data and external LiDAR pose correction

The validation test commands a known motion profile and compares:

- ideal commanded trajectory vs time
- EKF localization trajectory vs time
- XY ideal path vs XY localized path

## Code Location

Primary modules:

- `ros2/litterbot_bridge/litterbot_bridge/ekf_localization_node.py`
- `ros2/litterbot_bridge/litterbot_bridge/ekf_core.py`
- `ros2/litterbot_bridge/litterbot_bridge/mecanum_motion_model.py`
- `ros2/litterbot_bridge/litterbot_bridge/encoder_model.py`
- `ros2/litterbot_bridge/litterbot_bridge/imu_model.py`
- `ros2/litterbot_bridge/litterbot_bridge/lidar_model.py`
- `ros2/litterbot_bridge/litterbot_bridge/localization_calibrator.py`
- `ros2/litterbot_bridge/litterbot_bridge/localization_test_runner.py`
- `ros2/litterbot_bridge/launch/litterbot.launch.py`

## Bot-Specific Values To Verify Before Testing

These values are used by the software and must match the physical robot:

- `wheel_radius_m`
  Current default: `0.052`
- `encoder_ticks_per_rev`
  Current default: `537.7`
- `wheelbase_m`
  Current default: `0.168`
- `track_width_m`
  Current default: `0.2025`
- IMU mounting orientation relative to `base_link`
- LiDAR frame alignment and `laser` TF

If `wheelbase_m` or `track_width_m` are wrong, strafing and rotation estimates will be wrong even if the EKF is running correctly.

## Required Software On The Jetson

Assumptions:

- Ubuntu with ROS 2 Humble installed
- `~/litterbot_ws` is the ROS workspace
- this repository is checked out into `~/litterbot_ws/src/LitterBot`

Install required ROS packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rplidar-ros \
  ros-humble-realsense2-camera \
  ros-humble-tf2-ros \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs
```

Install Python dependencies if they are not already present:

```bash
cd ~/litterbot_ws/src/LitterBot/ros2/litterbot_bridge
python3 -m pip install -r requirements.txt
```

## Build Procedure

```bash
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select litterbot_bridge litterbot_vision
source install/setup.bash
```

If `colcon build` fails, do not proceed to hardware testing. Fix the build first.

## Hardware Setup

Before power-on:

1. Place the robot in an open indoor area with a flat floor.
2. Make sure the battery is charged.
3. Connect:
   - STM32 USB
   - RPLidar USB
   - RealSense D455f USB
4. Confirm the LiDAR has line of sight to walls or other features.
5. Clear the test area of obstacles and people.

The launch file auto-detects serial devices, so the STM32 and RPLidar do not need fixed USB port positions.

## Pre-Test Expectations

When localization is healthy:

- `/encoder_raw` should update continuously
- `/imu/data_raw` should update continuously
- `/scan` should update continuously
- `/odometry/ekf` should update continuously
- the UI `LOCALIZATION` tab should show changing position and velocity

If any of those streams are missing, the test is not valid.

## Bring-Up Procedure

Open an SSH session to the Jetson.

In terminal 1, start the bridge and sensor stack:

```bash
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch litterbot_bridge litterbot.launch.py \
  mock_serial:=false \
  enable_camera:=false \
  enable_localization:=false \
  enable_realsense_imu:=true \
  enable_lidar:=true
```

Notes:

- `enable_camera:=false` is intentional for localization testing.
- The D455f RGB and IMU are from the same device. For this test, do not start the legacy OpenCV camera node.
- Localization is started separately because `ekf_localization` prompts for calibration in the terminal.

In terminal 2, start localization:

```bash
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run litterbot_bridge ekf_localization
```

When prompted:

- enter `y` if you want a fresh stationary calibration before the test
- enter `n` if you want to use the last saved calibration

If calibrating:

1. Keep the robot completely still.
2. Wait for calibration to finish.
3. Confirm the file was written to:
   - `~/.litterbot/localization_calibration.json`

## Pre-Run Validation Checks

Run these checks in a third terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/litterbot_ws/install/setup.bash
ros2 topic list
```

Expected important topics:

- `/encoder_raw`
- `/imu/data_raw`
- `/scan`
- `/odometry/ekf`
- `/localization/status`
- `/cmd_vel`

Optional sanity checks:

```bash
ros2 topic hz /encoder_raw
ros2 topic hz /imu/data_raw
ros2 topic hz /odometry/ekf
ros2 topic echo /localization/status
```

Do not start the trajectory test if:

- `/odometry/ekf` is missing
- the IMU topic is dead
- the robot drifts badly while standing still
- LiDAR is not publishing `/scan`

## Localization Test Procedure

In terminal 3, run:

```bash
cd ~/litterbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run litterbot_bridge localization_test
```

The script will prompt you to append motion segments.

Supported segment types:

- `circle`
- `line`
- `rotate`
- `done`

### Recommended First Test

Use one circle segment:

- radius: `0.5`
- speed: `0.20`
- laps: `1.0`
- direction: `ccw`

This produces a constant-speed circular trajectory.

### Example Session

```text
Segment type [circle/line/rotate/done]: circle
Circle radius in meters [0.5]: 0.5
Linear speed in m/s [0.2]: 0.2
Number of laps [1.0]: 1
Direction [ccw/cw] (default ccw): ccw
Segment type [circle/line/rotate/done]: done
```

Then:

1. place the robot at the chosen start point
2. align the robot so its starting heading is known
3. clear the path
4. let the countdown finish
5. do not touch the robot during the run

## Output Files

Each run writes a timestamped folder to:

```text
~/.litterbot/localization_tests/<timestamp>/
```

Expected files:

- `trajectory_comparison.csv`
- `summary.json`
- `xy_path.png`
- `coordinates_vs_time.png`

## What To Expect From The Output

### `xy_path.png`

This should show:

- one ideal circular path
- one actual EKF path

Good run:

- actual path roughly overlaps the ideal circle
- start and end points are close
- no large kinks or jumps

Bad run:

- circle radius is clearly wrong
- path spirals outward or inward
- sharp discontinuities appear
- loop does not close at all

### `coordinates_vs_time.png`

This should show:

- ideal `x` and actual `x`
- ideal `y` and actual `y`
- ideal `yaw` and actual `yaw`
- position error magnitude vs time

Good run:

- actual `x` and `y` track the ideal curves with small lag
- yaw tracks smoothly
- position error remains bounded

Bad run:

- position error grows monotonically through the entire run
- yaw estimate oscillates or jumps
- actual coordinates have obvious delay or stair-stepping

### `summary.json`

This contains:

- sample count
- position RMSE
- x RMSE
- y RMSE
- yaw RMSE
- max position error

## Acceptance Guidance

Initial practical targets for an indoor first-pass test:

- stationary drift near zero over 10 to 20 seconds
- circle path visually recognizable as a circle
- end-point error after one `0.5 m` radius lap less than about `0.20 m`
- no sudden jumps in pose estimate

Those are not final production specs. They are reasonable first-pass bring-up thresholds.

## Common Failure Modes

### Encoder sign or wheel geometry error

Symptoms:

- commanded circle becomes a lopsided arc
- commanded rotation causes translation
- strafing estimate is obviously wrong

Actions:

- verify encoder ordering
- verify mecanum wheel sign conventions
- verify `wheelbase_m` and `track_width_m`

### IMU frame misalignment

Symptoms:

- yaw rotates the wrong direction
- large heading bias appears immediately
- estimate changes while the robot is stationary

Actions:

- verify D455f mounting
- verify IMU axes relative to `base_link`

### LiDAR correction unavailable

Symptoms:

- `/scan` exists but no absolute pose correction is entering the filter
- encoder and IMU run, but long-term drift is not bounded

Actions:

- confirm the LiDAR pose topic matches the EKF input
- confirm your SLAM or localization node is actually running

### RealSense device contention

Symptoms:

- IMU topics do not appear
- the camera opens in one node but not another

Actions:

- keep `enable_camera:=false` during localization testing
- use the RealSense driver as the IMU source

## Official Test Run Checklist

1. Build succeeds.
2. All three sensors are connected.
3. `litterbot.launch.py` is running.
4. `ekf_localization` is running.
5. `/odometry/ekf` is live.
6. Stationary calibration was either completed or a known-good saved calibration exists.
7. Test area is clear.
8. Circle test completes and outputs plots.
9. Results are archived from `~/.litterbot/localization_tests/`.

## Recommended Follow-On Tests

After the `0.5 m` circle test passes, run:

1. in-place `360 deg` rotation
2. `1.0 m` forward line
3. `1.0 m` strafe line
4. two consecutive circle laps

This sequence isolates:

- yaw behavior
- forward odometry
- mecanum lateral odometry
- long-run drift
