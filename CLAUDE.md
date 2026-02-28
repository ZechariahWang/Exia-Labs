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
- **IMU**: SYD Dynamics TransducerM TM171, `/imu/data`, 200Hz, USB Type-C (`/dev/exia_imu`)
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

## Coordinate Input System

Goals are sent via `/navigation/goal` using `NavigationGoal.msg`. The `coord_type` field determines interpretation:

| coord_type | Fields Used | Description |
|------------|-------------|-------------|
| `xy` | `x`, `y` | Local map-frame coordinates (meters) |
| `latlon` | `lat`, `lon`, optional `origin_lat`/`origin_lon` | Decimal GPS, converted to local XY via equirectangular projection |
| `dms` | `lat_dms`, `lon_dms`, optional `origin_lat`/`origin_lon` | Degrees-Minutes-Seconds strings (e.g. `49D40'00"N`), parsed then converted like latlon |
| `forward` | `move_type` (`time`/`distance`), `move_value`, `move_speed` | Open-loop forward by duration (`3s`) or distance (`50m`) |
| `turn` | `move_type` (`time`/`heading`), `move_value`, `move_speed` | Turn by duration or heading degrees (positive=left) |

- GPS->local: `x = (lon - origin_lon) * cos(origin_lat) * 111320`, `y = (lat - origin_lat) * 111320`
- DMS formats: `49°40'00"N`, `49D40'00"N`, `49°40.5'N`
- If `origin_lat`/`origin_lon` are 0 in msg, falls back to node's `ORIGIN_GPS` params
- `--direct` flag (xy/latlon/dms): straight-line nav, stops on obstacle, reverses 1.5m, CLI blocks
- `forward`/`turn` always block CLI until terminal status on `/navigation/status`
- Terminal statuses: `GOAL_REACHED`, `OBSTACLE_STOPPED`, `PATH_ENDED`, `PATH_BLOCKED`, `MOVE_COMPLETE`, `TURN_COMPLETE`
- Cancel: `nav_to cancel` or `/navigation/cancel` service (std_srvs/Trigger)

## Radio Bridge

Encrypted wireless link between a base station laptop and the robot via RFD900x radios + FTDI USB serial (`/dev/exia_radio`, 57600 baud). The `radio_bridge_node` runs on both sides with different roles.

```bash
# Robot side
ros2 launch exia_bringup radio.launch.py role:=robot
# Base station
ros2 launch exia_bringup radio.launch.py role:=base
```

**Encryption**: RSA-2048 handshake + AES-256-GCM session. Base generates a random AES key each connection, encrypts it with the robot's RSA public key, sends in two fragments (`$K1`/`$K2`). Robot decrypts with private key, ACKs with SHA-256 hash. All subsequent frames are AES-GCM encrypted with monotonic nonce counters (even=base, odd=robot).

**Key setup**: `scripts/generate_radio_keys.sh` creates RSA key pair in `~/.exia/`. Copy both `radio_private.pem` and `radio_public.pem` to `~/.exia/` on both machines.

**Message types** (encrypted frame payload):

| Type | Direction | Purpose |
|------|-----------|---------|
| `H` | base->robot | Heartbeat (10Hz), carries sequence number |
| `A` | robot->base | Heartbeat ACK |
| `N` | base->robot | Navigation goal (all coord_types supported) |
| `C` | base->robot | Cancel navigation |
| `E` | base->robot | E-stop activate |
| `X` | base->robot | E-stop clear |
| `V` | base->robot | cmd_vel relay (linear.x, angular.z) |
| `S` | robot->base | Telemetry (state, x, y, heading, speed) at 5Hz |
| `R` | robot->base | Relay `/navigation/status` messages |
| `EA` | robot->base | E-stop acknowledgment |

**Safety**: Robot triggers e-stop if no heartbeat ACK for 0.5s. Base resets handshake after 15 missed ACKs. E-stop publishes zero cmd_vel at 20Hz and cancels active navigation. Heartbeat-loss e-stop auto-clears when heartbeats resume.

**Base station topics/services**:
- Sub: `/navigation/goal` -> sends over radio
- Sub: `/radio/cmd_vel` -> relays to robot
- Pub: `/navigation/status` (relayed from robot)
- Srv: `/radio/cancel`, `/radio/estop`, `/radio/estop_clear`

**Config**: `src/exia_bringup/config/radio_params.yaml`
**Udev rule**: `scripts/99-exia-radio.rules` (FTDI 0403:6001 -> `/dev/exia_radio`)

## Key Design Decisions

- Ackermann steering: cannot rotate in place, must have forward velocity to turn
- Foxglove Studio for visualization (not RViz). Set 3D panel frame to `map`
- Gazebo multicast warnings: `export IGN_IP=127.0.0.1`
- path_validator uses uint8 costmap (not int8) for values 128-255
- dynamic_navigator uses threading.Lock with ReentrantCallbackGroup
- Distant goals (>50m) handled via costmap clamping + progressive replanning
- After building exia_msgs: must re-source install/setup.bash
