# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)
# Updated: February 2026 - Safety Hardening + Runtime Navigation Commands

ROS 2 workspace for the Exia Ground robot platform.

## Three-Motor Ackermann Architecture

The Exia Ground robot uses a three-motor control architecture designed for
easy transition between simulation and real ATV hardware:

```
                    +------------------+
                    |    cmd_vel       |
                    | (Twist message)  |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    | Ackermann Drive  |
                    |      Node        |
                    | (Kinematics +    |
                    |  Safety Logic)   |
                    | - Accel limiter  |
                    | - cmd_vel wdog   |
                    | - HW serial mode |
                    +--------+---------+
                             |
              +--------------+--------------+
              |              |              |
              v              v              v
        +---------+    +---------+    +---------+
        |Steering |    |Throttle |    | Brake   |
        | (servo) |    | (motor) |    |(actuator|
        +---------+    +---------+    +---------+
              |              |              |
              v              v              v
        +----------------------------------------+
        |         ros2_control / HAL             |
        |  - gz_ros2_control (Gazebo Fortress)   |
        |  - Serial/PWM (Arduino Hardware)       |
        +----------------------------------------+
```

## Project Structure

```
exia_ws/
├── scripts/                        # System-level scripts
│   ├── 99-exia-odrive.rules        # udev rules for ODrive
│   ├── exia_startup.sh             # Robot startup script
│   ├── exia_enable_autostart.sh    # Enable systemd autostart
│   ├── exia_disable_autostart.sh   # Disable systemd autostart
│   ├── exia_status.sh              # Check robot status
│   └── exia-robot.service          # Systemd service unit file
├── arduino/
│   ├── exia_servo_controller/
│   │   └── exia_servo_controller.ino  # Jetson-controlled servo bridge
│   └── rc_control_pwm/
│       └── rc_control_pwm.ino         # RC receiver + gear shifting
├── src/
│   ├── exia_bringup/               # Launch files, config, URDF (ament_cmake)
│   │   ├── launch/
│   │   │   ├── sim.launch.py       # Gazebo simulation
│   │   │   ├── autonomous.launch.py # SLAM + Nav2 + Dynamic navigator
│   │   │   ├── rc_control.launch.py # Hardware RC driver mode
│   │   │   ├── gps_navigation.launch.py # GPS + EKF sensor fusion
│   │   │   ├── drive_test.launch.py # Straight-line drive test
│   │   │   └── turn_test.launch.py  # Heading turn test
│   │   ├── config/
│   │   │   ├── ackermann_controllers.yaml
│   │   │   ├── nav2_params.yaml
│   │   │   ├── slam_toolbox_params.yaml
│   │   │   ├── pointcloud_to_laserscan.yaml
│   │   │   ├── gps_params.yaml
│   │   │   ├── ekf_params.yaml
│   │   │   └── septentrio_rover.yaml
│   │   ├── urdf/
│   │   │   └── exia_ground.urdf.xacro
│   │   └── worlds/
│   │       └── exia_world.sdf
│   │
│   ├── exia_control/               # Navigation/control logic (ament_python)
│   │   └── exia_control/
│   │       ├── ackermann_drive_node.py    # cmd_vel -> actuators, odom, TF
│   │       ├── dynamic_navigator_node.py  # Goal navigation with replanning
│   │       ├── nav_to_cmd.py              # CLI tool for runtime goals
│   │       ├── drive_forward_test_node.py # Straight-line drive test
│   │       ├── turn_to_heading_test_node.py # Heading turn test
│   │       ├── planning/
│   │       │   └── pure_pursuit.py
│   │       └── navigation/
│   │           ├── path_validator.py
│   │           └── planner_interface.py
│   │
│   ├── exia_driver/                # Hardware drivers (ament_python)
│   │   └── exia_driver/
│   │       ├── rc_driver_node.py
│   │       └── gps_transform_node.py
│   │
│   └── exia_msgs/                  # Custom messages (ament_cmake)
│       └── msg/
│           └── NavigationGoal.msg  # Multi-format navigation goal
│
├── build/
├── install/
└── log/
```

## Robot Specifications

| Property | Value |
|----------|-------|
| Chassis dimensions | 2.11m x 1.2m x 0.6m (L x W x H) |
| Wheel radius | 0.3m |
| Track width | 1.1m |
| Wheel base | 1.3m |
| Chassis mass | 200.0 kg |
| Max steering angle | 0.6 rad (~34 degrees) |
| Minimum turning radius | 1.9m |
| Max speed | 5.0 m/s |
| Max acceleration | 2.0 m/s^2 |
| Max deceleration | 4.0 m/s^2 |

## Quick Start

```bash
# Terminal 1: Start simulation
source ~/exia_ws/install/setup.bash
ros2 launch exia_bringup sim.launch.py

# Terminal 2: Start autonomous navigation (after Gazebo loads)
source ~/exia_ws/install/setup.bash
ros2 launch exia_bringup autonomous.launch.py

# Terminal 3: Send navigation goals at runtime (no restart needed)
source ~/exia_ws/install/setup.bash
ros2 run exia_control nav_to xy 22.0 24.0

# With EKF sensor fusion (GPS + wheel odom + IMU)
ros2 launch exia_bringup autonomous.launch.py use_ekf:=true
```

## Runtime Navigation Commands (nav_to CLI)

Send navigation goals at runtime without restarting the simulation. Uses the `NavigationGoal` custom message on `/navigation/goal`.

```bash
# XY coordinates (map frame, meters)
ros2 run exia_control nav_to xy 30.0 15.0

# Lat/lon coordinates (uses default origin from node)
ros2 run exia_control nav_to latlon 49.666667 11.841389

# Lat/lon with custom origin
ros2 run exia_control nav_to latlon 49.666667 11.841389 --origin 49.6664 11.8411

# DMS coordinates
ros2 run exia_control nav_to dms "49 40 0 N" "11 50 29 E"

# Cancel current navigation
ros2 run exia_control nav_to cancel
```

### NavigationGoal Message (exia_msgs/msg/NavigationGoal)

```
string coord_type        # "xy", "latlon", or "dms"
float64 x                # X coordinate (xy mode)
float64 y                # Y coordinate (xy mode)
float64 lat              # Latitude (latlon mode)
float64 lon              # Longitude (latlon mode)
string lat_dms           # Latitude DMS string (dms mode)
string lon_dms           # Longitude DMS string (dms mode)
float64 origin_lat       # Optional origin override
float64 origin_lon       # Optional origin override
```

## GPS Waypoint Navigation

The robot supports GPS waypoint navigation for outdoor operation. GPS coordinates are converted to local XY using equirectangular projection.

### GPS Configuration

Goals can be sent at runtime using the `nav_to` CLI (preferred), or configured as defaults in `dynamic_navigator_node.py`:
```python
USE_GPS_MODE = True
TARGET_GPS = [49.666667, 11.841389]  # [lat, lon] destination
ORIGIN_GPS = [49.666400, 11.841100]  # [lat, lon] reference origin
```

### When Moving to a New Location

**For hardware deployment**, either:
- Use `ros2 run exia_control nav_to latlon <lat> <lon>` at runtime
- Or update defaults in `dynamic_navigator_node.py`: `TARGET_GPS`, `ORIGIN_GPS`

**For simulation**, also update:
- `src/exia_bringup/worlds/exia_world.sdf` - spherical_coordinates
- `src/exia_bringup/config/gps_params.yaml` - origin_lat/origin_lon

### GPS Architecture

```
┌─────────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│  ARK MOSAIC-X5  │     │ septentrio_gnss_    │     │ gps_transform_   │
│  RTK GPS        │────>│     driver          │────>│     node         │
│  /dev/ttyACM0   │     │  /gnss/fix (NavSat) │     │  GPS -> Local XY │
└─────────────────┘     └─────────────────────┘     │  /gps/odom       │
                                                     │  (waits for RTK  │
        OR (Simulation)                              │   fix before     │
                                                     │   setting origin)│
┌─────────────────┐     ┌─────────────────────┐     └────────┬─────────┘
│ Gazebo NavSat   │────>│   ros_gz_bridge     │────>         │
│ Sensor Plugin   │     │  /gps/fix           │              v
└─────────────────┘     └─────────────────────┘     ┌──────────────────┐
                                                     │ dynamic_navigator│
                        ┌─────────────────────┐     │  /navigation/goal│
                        │  nav_to CLI tool    │────>│  (xy/latlon/dms) │
                        └─────────────────────┘     └──────────────────┘
```

### EKF Sensor Fusion

When `use_ekf:=true`, robot_localization fuses:
- `/gps/odom` - GPS position (X, Y)
- `/odom` - Wheel odometry (velocity)
- `/imu/data` - IMU (orientation, angular velocity)

Output: `/odometry/filtered` (fused pose estimate)

## Packages

### exia_bringup (ament_cmake)
Launch files, configuration, URDF, and world files.

| Launch File | Purpose |
|-------------|---------|
| `sim.launch.py` | Gazebo Fortress simulation with controllers |
| `autonomous.launch.py` | SLAM + Nav2 planner + Dynamic navigator |
| `rc_control.launch.py` | Hardware RC driver for physical robot |
| `gps_navigation.launch.py` | GPS driver + EKF sensor fusion |
| `drive_test.launch.py` | Straight-line drive validation test |
| `turn_test.launch.py` | Heading turn validation test |

### exia_control (ament_python)
Navigation and control logic.

| Executable | Purpose |
|------------|---------|
| `ackermann_drive_node` | cmd_vel -> controller commands, publishes odom + TF |
| `dynamic_navigator_node` | Goal-based navigation with replanning |
| `nav_to` | CLI tool for sending navigation goals at runtime |
| `drive_forward_test_node` | Straight-line drive test |
| `turn_to_heading_test_node` | Heading turn test |

| Module | Purpose |
|--------|---------|
| `planning/pure_pursuit.py` | Pure Pursuit path following |
| `navigation/path_validator.py` | Collision detection (uint8 costmap) |
| `navigation/planner_interface.py` | Nav2 planner interface |

### exia_driver (ament_python)
Hardware drivers for physical robot.

| Executable | Purpose |
|------------|---------|
| `rc_driver_node` | RC radio receiver + Arduino serial control |
| `gps_transform_node` | GPS lat/lon to local XY conversion (RTK-gated origin) |

### exia_msgs (ament_cmake)
Custom message definitions.

| Message | Purpose |
|---------|---------|
| `NavigationGoal.msg` | Multi-format navigation goal (xy/latlon/dms) |

## Safety Systems

### Ackermann Drive Node Safety

| Feature | Description |
|---------|-------------|
| Acceleration limiter | Ramps speed at MAX_ACCEL=2.0 m/s^2, MAX_DECEL=4.0 m/s^2 |
| cmd_vel watchdog | Stops robot if no cmd_vel received within 0.2s |
| Hardware serial mode | `hardware_mode:=true` sends S,T,B commands to Arduino |
| Graceful shutdown | Sends DISARM to Arduino on node destroy |
| Midpoint odom integration | Uses midpoint theta for reduced odometry drift |

### Dynamic Navigator Safety

| Feature | Description |
|---------|-------------|
| Sensor health watchdog | Stops robot if /scan or /odom stale >2s |
| Thread-safe callbacks | threading.Lock protects concurrent callback access |
| One-shot timers | Prevents timer storms from repeated create_timer calls |

### Arduino Safety (exia_servo_controller.ino)

| Feature | Description |
|---------|-------------|
| Throttle governor | Capped at 120 (out of 180) to limit max speed |
| Command timeout | Emergency stop if no serial command for 2s |
| Emergency stop | Centers steering, neutrals throttle, full brake |

### Arduino Safety (rc_control_pwm.ino)

| Feature | Description |
|---------|-------------|
| Jetson timeout watchdog | Safe state if Jetson serial lost for 300ms |
| RC signal loss detection | Brake + neutral on RC timeout (500ms) |
| Median filter | 3-sample median on all RC channels |
| Rate-limited actuators | Smooth servo transitions with configurable rates |
| Gear shift interlock | Requires brake engaged before shifting |

### GPS Transform Node Safety

| Feature | Description |
|---------|-------------|
| RTK fix gate | Waits for STATUS_GBAS_FIX before auto-setting origin |
| Covariance propagation | GPS covariance passed through to odom output |

### RC Driver Node Safety

| Feature | Description |
|---------|-------------|
| Startup grace period | 15s grace before requiring heartbeat |

## Navigation Architecture

```
     /navigation/goal          Goal Pose (x, y)
     (NavigationGoal)               |
          |                         |
          v                         v
+------------------------------------------+
|        Dynamic Navigator Node            |
|  State: IDLE -> PLANNING -> EXECUTING    |
|                    ^           |         |
|                    +-- RECOVERING        |
|                                          |
|  - /navigation/goal subscriber           |
|    (accepts xy, latlon, dms coords)      |
|  - 500ms continuous replanning           |
|  - Obstacle-triggered immediate replan   |
|  - Pure Pursuit path execution           |
|  - Sensor health watchdog                |
|  - Thread-safe with nav_lock             |
+------------------------------------------+
         |                    ^
         v                    |
     /cmd_vel          /global_costmap
         |                    |
         v                    |
+----------------+    +----------------+
| Ackermann      |    | slam_toolbox   |<-- /scan
| Drive Node     |    | (map->odom TF) |<-- /imu
| - Accel ramp   |    +----------------+
| - cmd_vel wdog |
| - HW serial    |
+----------------+
         |
         v
+----------------+
| Controllers    |
| (ros2_control) |
| OR             |
| Arduino Serial |
+----------------+
```

**Parameters (dynamic_navigator_node):**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_speed` | `2.0` | Target speed (m/s) |
| `lookahead_distance` | `2.5` | Pure Pursuit lookahead (m) |
| `replan_period` | `0.5` | Replanning interval (s) |
| `goal_tolerance` | `1.0` | Goal reached threshold (m) |
| `obstacle_lookahead` | `8.0` | Obstacle detection range (m) |

**Parameters (ackermann_drive_node):**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_tf` | `true` | Publish odom->base_footprint TF |
| `hardware_mode` | `false` | Enable Arduino serial bridge |
| `serial_port` | `/dev/arduino_control` | Arduino serial port |
| `serial_baud` | `115200` | Serial baud rate |

## Control Commands

```bash
# Send navigation goal (preferred method)
ros2 run exia_control nav_to xy 22.0 24.0

# Drive forward at 1.0 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}" -1

# Turn left while driving
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}" -1

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

## Build Commands

```bash
cd ~/exia_ws

# Build all packages (exia_msgs must build first for message generation)
colcon build

# Build specific packages
colcon build --packages-select exia_msgs exia_control exia_driver exia_bringup

# Source after build (required after building exia_msgs for the first time)
source install/setup.bash
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands |
| `/odom` | Odometry | Robot odometry |
| `/scan` | LaserScan | Lidar data |
| `/imu/data` | Imu | IMU data |
| `/planned_path` | Path | Current path |
| `/global_costmap/costmap` | OccupancyGrid | Obstacle map |
| `/steering_controller/commands` | Float64MultiArray | Steering joint commands |
| `/throttle_controller/commands` | Float64MultiArray | Wheel velocity commands |
| `/gps/fix` | NavSatFix | GPS coordinates (lat/lon) |
| `/gps/odom` | Odometry | GPS as local XY odometry |
| `/odometry/filtered` | Odometry | EKF fused odometry |
| `/navigation/goal` | NavigationGoal | Runtime navigation goal input |

## Sensors

### Lidar (Slamtec RPlidar S3 compatible)
- Topic: `/scan`
- Range: 0.2m - 20m
- FOV: 360 degrees
- Rate: 10Hz

### IMU (Yahboom 10-axis compatible)
- Topic: `/imu/data`
- Rate: 200Hz

### GPS (ARK MOSAIC-X5 RTK)
- Hardware: Septentrio mosaic-X5 chip
- Topic: `/gnss/fix` (hardware), `/gps/fix` (simulation)
- Driver: `septentrio_gnss_driver`
- Rate: 10Hz
- Accuracy: RTK cm-level (with base station)
- Origin auto-set waits for RTK fix quality

## TF Tree

```
map (from SLAM)
  +-- odom (from ackermann_drive_node)
        +-- base_footprint
              +-- base_link
                    +-- front_left_steer_link -> front_left_wheel
                    +-- front_right_steer_link -> front_right_wheel
                    +-- rear_left_wheel
                    +-- rear_right_wheel
                    +-- lidar_link
                    +-- imu_link
                    +-- gps_link
```

## Debugging

```bash
# Check nodes
ros2 node list

# Check TF tree
ros2 run tf2_ros tf2_echo map odom

# Check if ackermann_drive is running
ros2 node list | grep ackermann

# Check odom is being published
ros2 topic echo /odom --once

# Check costmap
ros2 topic hz /global_costmap/costmap

# Check planner
ros2 lifecycle get /planner_server

# View path
ros2 topic echo /planned_path --once

# GPS debugging
ros2 topic echo /gps/fix --once
ros2 topic echo /gps/odom --once
ros2 topic echo /odometry/filtered --once

# Navigation goal debugging
ros2 topic echo /navigation/goal --once

# Sensor health (check if scan/odom are publishing)
ros2 topic hz /scan
ros2 topic hz /odom
```

## Hardware Deployment

### Ackermann Drive in Hardware Mode

The `ackermann_drive_node` supports a `hardware_mode` parameter that sends serial commands directly to the Arduino instead of publishing to ros2_control topics:

```bash
ros2 run exia_control ackermann_drive_node --ros-args -p hardware_mode:=true -p serial_port:=/dev/arduino_control
```

In hardware mode:
- Sends `S<steer>,T<throttle>,B<brake>` commands over serial
- Sends `ARM` on startup, `DISARM` on shutdown
- Throttle capped at 120 (matching Arduino governor)
- Acceleration limiter active (2.0 m/s^2 accel, 4.0 m/s^2 decel)

### RC Driver Mode (Jetson + Arduino)

The `rc_driver_node` handles hardware control via serial:
- Connects to Arduino on `/dev/arduino_control`
- Communicates with ODrive for motor control
- Protocol: `S<steer>,T<throttle>,B<brake>\n`
- 15s startup grace period before requiring heartbeat

```bash
ros2 run exia_driver rc_driver_node
```

### Systemd Autostart

```bash
# Enable autostart on boot
sudo ~/exia_ws/scripts/exia_enable_autostart.sh

# Check status
~/exia_ws/scripts/exia_status.sh

# Manual start/stop
sudo systemctl start exia-robot
sudo systemctl stop exia-robot

# View logs
journalctl -u exia-robot -f

# Disable autostart
sudo ~/exia_ws/scripts/exia_disable_autostart.sh
```

## Arduino Servo Controller (exia_servo_controller.ino)

The Arduino Mega runs `exia_servo_controller.ino` as the Jetson-controlled servo bridge.

**Pin Assignments:**
| Pin | Function |
|-----|----------|
| 9 | Steering servo |
| 10 | Throttle ESC |
| 11 | Brake servo |
| 13 | Status LED |

**Serial Commands:**
| Command | Description |
|---------|-------------|
| `ARM` | Enable servo control, release brake |
| `DISARM` | Emergency stop, full brake |
| `S90,T90,B0` | Set steering=90, throttle=90, brake=0 |

**Safety Limits:**
| Actuator | Range | Notes |
|----------|-------|-------|
| Steering | 0-180 | Centered at 90 |
| Throttle | 0-120 | Governed (hardware max is 180) |
| Brake | 0-180 | 180 = full brake, 0 = released |

## Arduino RC Controller (rc_control_pwm.ino)

Handles RC receiver input with gear shifting for manual driving.

**Pin Assignments:**
| Pin | Function |
|-----|----------|
| 2 | CH1 (Steering RC input) |
| 3 | CH2 (Throttle/Brake RC input) |
| 4 | CH3 (Gear shift RC input) |
| 44 | Throttle servo output |
| 45 | Brake servo output |
| 46 | Gear servo output |
| 21 | Status LED |

**Gear Positions:**
| Gear | Servo Angle | Description |
|------|-------------|-------------|
| Reverse | 57 | Requires brake + CH3 down |
| Neutral | 80 | Default on startup/safe state |
| High | 110 | Requires brake + CH3 up |

## Dependencies

```bash
# GPS and sensor fusion packages
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-septentrio-gnss-driver
sudo apt install ros-humble-nmea-msgs ros-humble-gps-msgs
```

## Development Notes

- **Simulator**: Gazebo Fortress (not Classic)
- **Ackermann steering**: Cannot rotate in place, must have forward velocity to turn
- **RViz Fixed Frame**: Use `odom` or `map` depending on navigation mode
- **Gazebo multicast warnings**: Set `export IGN_IP=127.0.0.1` to suppress
- **GPS Mode**: Set `USE_GPS_MODE = True` in dynamic_navigator_node.py for GPS waypoints
- **After building exia_msgs**: Must re-source `install/setup.bash` before launching nodes that depend on it
- **Costmap data type**: path_validator uses uint8 (not int8) to correctly handle obstacle values 128-255
- **Thread safety**: dynamic_navigator_node uses threading.Lock for concurrent callback protection with ReentrantCallbackGroup

Dont write any comments in the code.
