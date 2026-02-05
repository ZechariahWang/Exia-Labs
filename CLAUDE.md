# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)
# Updated: February 2026 - Multi-Package Architecture + GPS Integration

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
│   │   └── exia_servo_controller.ino
│   └── rc_control_pwm/
│       └── rc_control_pwm.ino
├── src/
│   ├── exia_bringup/               # Launch files, config, URDF (ament_cmake)
│   │   ├── launch/
│   │   │   ├── sim.launch.py       # Gazebo simulation
│   │   │   ├── autonomous.launch.py # SLAM + Nav2 + Dynamic navigator
│   │   │   ├── rc_control.launch.py # Hardware RC driver mode
│   │   │   └── gps_navigation.launch.py # GPS + EKF sensor fusion
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
│   │       ├── ackermann_drive_node.py
│   │       ├── dynamic_navigator_node.py
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

## Quick Start

```bash
# Terminal 1: Start simulation
source ~/exia_ws/install/setup.bash
ros2 launch exia_bringup sim.launch.py

# Terminal 2: Start autonomous navigation (after Gazebo loads)
source ~/exia_ws/install/setup.bash
ros2 launch exia_bringup autonomous.launch.py

# With EKF sensor fusion (GPS + wheel odom + IMU)
ros2 launch exia_bringup autonomous.launch.py use_ekf:=true
```

## GPS Waypoint Navigation

The robot supports GPS waypoint navigation for outdoor operation. GPS coordinates are converted to local XY using equirectangular projection.

### GPS Configuration

Edit `src/exia_control/exia_control/dynamic_navigator_node.py`:
```python
USE_GPS_MODE = True
TARGET_GPS = [49.666667, 11.841389]  # [lat, lon] destination
ORIGIN_GPS = [49.666400, 11.841100]  # [lat, lon] reference origin
```

### When Moving to a New Location

**For hardware deployment**, update these values in `dynamic_navigator_node.py`:
- `TARGET_GPS` - Your destination coordinates
- `ORIGIN_GPS` - A reference point near your operating area

**For simulation**, also update:
- `src/exia_bringup/worlds/exia_world.sdf` - spherical_coordinates
- `src/exia_bringup/config/gps_params.yaml` - origin_lat/origin_lon

### DMS Coordinate Format

The navigator supports DMS (degrees-minutes-seconds) input:
```
49°40'00"N 11°50'29"E  →  [49.666667, 11.841389]
```

### GPS Architecture

```
┌─────────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│  ARK MOSAIC-X5  │     │ septentrio_gnss_    │     │ gps_transform_   │
│  RTK GPS        │────▶│     driver          │────▶│     node         │
│  /dev/ttyACM0   │     │  /gnss/fix (NavSat) │     │  GPS → Local XY  │
└─────────────────┘     └─────────────────────┘     │  /gps/odom       │
                                                     └────────┬─────────┘
        OR (Simulation)                                       │
                                                              ▼
┌─────────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│ Gazebo NavSat   │────▶│   ros_gz_bridge     │────▶│ dynamic_navigator│
│ Sensor Plugin   │     │  /gps/fix           │     │  GPS waypoints   │
└─────────────────┘     └─────────────────────┘     └──────────────────┘
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

### exia_control (ament_python)
Navigation and control logic.

| Executable | Purpose |
|------------|---------|
| `ackermann_drive_node` | cmd_vel -> controller commands, publishes odom + TF |
| `dynamic_navigator_node` | Goal-based navigation with replanning |

| Module | Purpose |
|--------|---------|
| `planning/pure_pursuit.py` | Pure Pursuit path following |
| `navigation/path_validator.py` | Collision detection |
| `navigation/planner_interface.py` | Nav2 planner interface |

### exia_driver (ament_python)
Hardware drivers for physical robot.

| Executable | Purpose |
|------------|---------|
| `rc_driver_node` | RC radio receiver + Arduino serial control |
| `gps_transform_node` | GPS lat/lon to local XY conversion |

### exia_msgs (ament_cmake)
Custom message definitions (placeholder).

## Navigation Architecture

```
         Goal Pose (x, y)
              |
              v
+------------------------------------------+
|        Dynamic Navigator Node            |
|  State: IDLE -> PLANNING -> EXECUTING    |
|                    ^           |         |
|                    +-- RECOVERING        |
|                                          |
|  - 500ms continuous replanning           |
|  - Obstacle-triggered immediate replan   |
|  - Pure Pursuit path execution           |
+------------------------------------------+
         |                    ^
         v                    |
     /cmd_vel          /global_costmap
         |                    |
         v                    |
+----------------+    +----------------+
| Ackermann      |    | slam_toolbox   |<-- /scan
| Drive Node     |    | (map->odom TF) |<-- /imu
+----------------+    +----------------+
         |
         v
+----------------+
| Controllers    |
| (ros2_control) |
+----------------+
```

**Configuration** (`dynamic_navigator_node.py`):
```python
TARGET_POINT = [22, 24]  # [x, y] in meters (map frame)
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_speed` | `2.0` | Target speed (m/s) |
| `lookahead_distance` | `2.5` | Pure Pursuit lookahead (m) |
| `replan_period` | `0.5` | Replanning interval (s) |
| `goal_tolerance` | `1.0` | Goal reached threshold (m) |
| `obstacle_lookahead` | `8.0` | Obstacle detection range (m) |

## Control Commands

```bash
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

# Build all packages
colcon build

# Build specific packages
colcon build --packages-select exia_bringup exia_control exia_driver

# Source after build
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

## TF Tree

```
map (from SLAM)
  └── odom (from ackermann_drive_node)
        └── base_footprint
              └── base_link
                    ├── front_left_steer_link -> front_left_wheel
                    ├── front_right_steer_link -> front_right_wheel
                    ├── rear_left_wheel
                    ├── rear_right_wheel
                    ├── lidar_link
                    ├── imu_link
                    └── gps_link
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
```

## Hardware Deployment

### RC Driver Mode (Jetson + Arduino)

The `rc_driver_node` handles hardware control via serial:
- Connects to Arduino on `/dev/arduino_control`
- Communicates with ODrive for motor control
- Protocol: `S<steer>,T<throttle>,B<brake>\n`

```bash
# Manual test
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

## Arduino Servo Controller

The Arduino Mega runs `exia_servo_controller.ino` to control physical servos.

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
| `ARM` | Enable servo control |
| `DISARM` | Disable and safe servos |
| `S90,T90,B0` | Set steering=90, throttle=90, brake=0 |

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

Dont write any comments in the code.
