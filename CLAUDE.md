# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)

ROS 2 (Humble) workspace for the Exia Ground robot — an Ackermann-steered ATV platform.

Dont write any comments in the code.

## Architecture

Three-motor Ackermann: cmd_vel -> ackermann_drive_node -> steering servo + throttle motor + brake actuator -> ros2_control (sim) or Arduino serial (hardware). Simulator: Gazebo Fortress (not Classic).

## Robot Specs

| Property | Value |
|----------|-------|
| Dimensions | 2.11m x 1.2m x 0.6m |
| Wheel radius / track / wheelbase | 0.3m / 1.1m / 1.3m |
| Mass | 200 kg |
| Max steering | 0.6 rad (~34 deg) |
| Max speed / accel / decel | 5.0 m/s / 2.0 m/s^2 / 4.0 m/s^2 |

## Project Structure

```
exia_ws/
├── scripts/              # udev rules, startup, systemd, radio key gen
├── arduino/
│   ├── exia_servo_controller/   # Jetson-controlled servo bridge
│   └── rc_control_pwm/          # RC receiver + gear shifting + autonomous mode
├── src/
│   ├── exia_bringup/     # Launch files, config, URDF, worlds (ament_cmake)
│   ├── exia_control/     # Nav/control: ackermann_drive, dynamic_navigator, nav_to CLI (ament_python)
│   ├── exia_driver/      # HW drivers: rc_driver, gps_transform, radio_bridge (ament_python)
│   └── exia_msgs/        # NavigationGoal.msg (ament_cmake)
```

## Build

```bash
colcon build                    # exia_msgs builds first for message gen
source install/setup.bash       # required after building exia_msgs
```

## Quick Start

```bash
# Sim
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
ros2 launch exia_bringup sim.launch.py
ros2 launch exia_bringup autonomous.launch.py
ros2 run exia_control nav_to xy 22.0 24.0

# Hardware
ros2 launch exia_bringup autonomous_hw.launch.py
ros2 run exia_control nav_to xy 22.0 24.0
```

## nav_to CLI

Send goals at runtime via `/navigation/goal` (NavigationGoal msg). All coords GPS-referenced when available, falls back to SLAM map-frame.

```bash
ros2 run exia_control nav_to xy 30.0 15.0
ros2 run exia_control nav_to latlon 49.666667 11.841389
ros2 run exia_control nav_to dms "49 40 0 N" "11 50 29 E"
ros2 run exia_control nav_to cancel

# Direct mode: straight-line, stops on obstacle, reverses 1.5m, CLI blocks
ros2 run exia_control nav_to xy 30.0 15.0 --direct

# Movement commands (CLI blocks until complete)
ros2 run exia_control nav_to forward 3s 2.0     # time + speed
ros2 run exia_control nav_to forward 50m 1.5    # distance + speed
ros2 run exia_control nav_to turn 90            # heading degrees (positive=left)
ros2 run exia_control nav_to turn 3s 0.5        # time + angular vel
```

## Key Packages

- **exia_bringup**: sim.launch.py, autonomous.launch.py, autonomous_hw.launch.py, rc_control.launch.py, gps_navigation.launch.py, radio.launch.py
- **exia_control**: ackermann_drive_node (cmd_vel->actuators, odom, TF), dynamic_navigator_node (goal nav + replanning), nav_to (CLI)
- **exia_driver**: rc_driver_node (RC + autonomous bridge), gps_transform_node (GPS->local XY, RTK-gated), radio_bridge_node (RFD900x encrypted link)
- **exia_msgs**: NavigationGoal.msg (xy/latlon/dms/forward/turn + direct mode)

## Sensors

- **Lidar**: Velodyne VLP-32C, `/scan` (LaserScan), IP 192.168.1.201
- **Depth Camera**: Orbbec Gemini 435Le, `/depth/points` (costmap only, not SLAM)
- **IMU**: Yahboom 10-axis, `/imu/data`, 200Hz
- **GPS**: ARK MOSAIC-X5 RTK, `/gnss/fix` (hw) or `/gps/fix` (sim), septentrio_gnss_driver

## Hardware Deployment

- **autonomous_hw.launch.py**: rc_driver_node bridges cmd_vel to ODrive steering + Arduino throttle/brake/gear
- **Arduino protocol**: `AUTO`/`MANUAL`, `J<throttle>,<brake>`, `K<gear>` (0=reverse, 1=neutral, 2=high)
- **Radio**: `ros2 launch exia_bringup radio.launch.py role:=robot|base` (RSA+AES encrypted, 10Hz heartbeat, e-stop on timeout)
- **Systemd**: `sudo ~/exia_ws/scripts/exia_enable_autostart.sh`

## GPS Navigation

- gps_transform_node always launches with autonomous.launch.py
- Dynamic navigator computes live GPS->map offset for goal placement
- EKF fusion: `ros2 launch exia_bringup autonomous.launch.py use_ekf:=true`
- When moving locations, update `gps_params.yaml` + `exia_world.sdf` (sim) or just use `nav_to latlon` (hw)

## Key Design Decisions

- Ackermann steering: cannot rotate in place, must have forward velocity to turn
- Foxglove Studio for visualization (not RViz). Set 3D panel frame to `map`
- Gazebo multicast warnings: `export IGN_IP=127.0.0.1`
- path_validator uses uint8 costmap (not int8) for values 128-255
- dynamic_navigator uses threading.Lock with ReentrantCallbackGroup
- Distant goals (>50m) handled via costmap clamping + progressive replanning
- After building exia_msgs: must re-source install/setup.bash
