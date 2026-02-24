# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)
# Updated: February 2026 - Safety Hardening + Runtime Navigation Commands + Radio Bridge (fragmented key exchange, e-stop distinction, status relay) + GPS-Referenced Navigation + Foxglove Visualization

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
│   ├── exia-robot.service          # Systemd service unit file
│   ├── generate_radio_keys.sh     # Generate RSA keys for radio encryption
│   └── 99-exia-radio.rules        # udev rules for RFD900x radio (/dev/exia_radio)
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
│   │   │   ├── radio.launch.py      # Radio bridge (RFD900x)
│   │   │   └── autonomous_hw.launch.py # Hardware autonomous driving
│   │   ├── config/
│   │   │   ├── ackermann_controllers.yaml
│   │   │   ├── nav2_params.yaml
│   │   │   ├── slam_toolbox_params.yaml
│   │   │   ├── pointcloud_to_laserscan.yaml
│   │   │   ├── gps_params.yaml
│   │   │   ├── ekf_params.yaml
│   │   │   ├── septentrio_rover.yaml
│   │   │   └── radio_params.yaml
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
│   │       ├── planning/
│   │       │   └── pure_pursuit.py
│   │       └── navigation/
│   │           ├── path_validator.py
│   │           └── planner_interface.py
│   │
│   ├── exia_driver/                # Hardware drivers (ament_python)
│   │   └── exia_driver/
│   │       ├── rc_driver_node.py
│   │       ├── gps_transform_node.py
│   │       └── radio_bridge_node.py
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
# Terminal 1: Foxglove Bridge (default visualization)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# Terminal 2: Start simulation
source ~/exia_ws/install/setup.bash
ros2 launch exia_bringup sim.launch.py

# Terminal 3: Start autonomous stack (after Gazebo loads, waits for nav_to command)
source ~/exia_ws/install/setup.bash
ros2 launch exia_bringup autonomous.launch.py

# Terminal 4: Send navigation goals at runtime (no restart needed)
source ~/exia_ws/install/setup.bash
ros2 run exia_control nav_to xy 22.0 24.0

# Direct mode: straight-line navigation, stops on obstacle, CLI stays open
ros2 run exia_control nav_to xy 30.0 15.0 --direct

# With EKF sensor fusion (GPS + wheel odom + IMU)
ros2 launch exia_bringup autonomous.launch.py use_ekf:=true

# Hardware autonomous driving (real ATV, no simulation)
source ~/exia_ws/install/setup.bash
ros2 launch exia_bringup autonomous_hw.launch.py

# Send goals to hardware (same CLI as simulation)
ros2 run exia_control nav_to xy 22.0 24.0
```

## Runtime Navigation Commands (nav_to CLI)

Send navigation goals at runtime without restarting the simulation. Uses the `NavigationGoal` custom message on `/navigation/goal`.

All coordinates are GPS-referenced when GPS is available. The `gps_transform_node` converts GPS lat/lon to local XY, and the dynamic navigator computes a live GPS-to-map offset so all goal types use GPS as the coordinate reference. When GPS is unavailable, falls back to SLAM map-frame coordinates.

```bash
# XY coordinates (GPS-referenced local meters, relative to GPS origin)
ros2 run exia_control nav_to xy 30.0 15.0

# Negative coordinates work (navigates via costmap clamping for distant goals)
ros2 run exia_control nav_to xy -50.0 -50.0

# Lat/lon coordinates (converted to local XY via equirectangular projection)
ros2 run exia_control nav_to latlon 49.666667 11.841389

# Lat/lon with custom origin
ros2 run exia_control nav_to latlon 49.666667 11.841389 --origin 49.6664 11.8411

# DMS coordinates
ros2 run exia_control nav_to dms "49 40 0 N" "11 50 29 E"

# Cancel current navigation
ros2 run exia_control nav_to cancel
```

### Direct Navigation Mode (`--direct`)

Append `--direct` to any goal command for straight-line navigation without path replanning. The robot drives directly toward the target. If an obstacle is detected within 3m (via raw lidar scan in a +/-30 degree forward cone), the robot stops, straightens its wheels, reverses 1.5m to create clearance, and then the CLI exits with the robot's current position so the user can manually issue the next waypoint.

```bash
# Direct mode with XY
ros2 run exia_control nav_to xy 30.0 15.0 --direct

# Direct mode with lat/lon
ros2 run exia_control nav_to latlon 49.666667 11.841389 --direct

# Direct mode with DMS
ros2 run exia_control nav_to dms "49D40'00\"N" "11D50'29\"E" --direct
```

In direct mode the CLI blocks until one of:
- **Goal reached**: robot arrives within goal tolerance, prints final position
- **Obstacle detected**: robot stops, reverses, prints position for manual waypoint update
- **Path ended**: path exhausted before reaching goal, prints position
- **Ctrl+C**: cancels navigation via `/navigation/cancel` service

Without `--direct`, behavior is 100% unchanged (full A* planning + replanning + recovery).

### Movement Commands (`forward` / `turn`)

Low-level movement commands that send cmd_vel directly without path planning. The CLI blocks until the movement completes or an obstacle is detected. Ackermann constraint: the robot cannot rotate in place, so turn commands use a forward speed while steering.

```bash
# Move forward for 3 seconds at 2.0 m/s
ros2 run exia_control nav_to forward 3s 2.0

# Move forward 50 meters at 1.5 m/s
ros2 run exia_control nav_to forward 50m 1.5

# Move forward for 5 seconds at default speed (1.0 m/s)
ros2 run exia_control nav_to forward 5s

# Turn for 3 seconds at 0.5 rad/s angular velocity (forward at 1.0 m/s)
ros2 run exia_control nav_to turn 3s 0.5

# Turn left 90 degrees (positive = counterclockwise)
ros2 run exia_control nav_to turn 90

# Turn right 90 degrees (negative = clockwise)
ros2 run exia_control nav_to turn -90

# Turn 180 degrees
ros2 run exia_control nav_to turn 180
```

**Forward movement** checks lidar in a +/-30 degree forward cone at 3m. If an obstacle is detected, the robot stops immediately and the CLI reports `OBSTACLE_STOPPED` with position. Speed is capped at 5.0 m/s (robot max).

**Turn commands** drive forward at 0.5 m/s while steering (Ackermann constraint). For timed turns, the angular velocity is applied directly. For heading turns, the robot steers at the specified rate toward the target heading and stops within 3-degree tolerance.

The CLI blocks until one of:
- **Move/turn complete**: prints final position and heading
- **Obstacle detected** (forward only): stops, prints position
- **Ctrl+C**: cancels via `/navigation/cancel` service

### NavigationGoal Message (exia_msgs/msg/NavigationGoal)

```
string coord_type        # "xy", "latlon", "dms", "forward", or "turn"
float64 x                # X coordinate (xy mode)
float64 y                # Y coordinate (xy mode)
float64 lat              # Latitude (latlon mode)
float64 lon              # Longitude (latlon mode)
string lat_dms           # Latitude DMS string (dms mode)
string lon_dms           # Longitude DMS string (dms mode)
float64 origin_lat       # Optional origin override
float64 origin_lon       # Optional origin override
bool direct              # Direct mode (no replanning, stop on obstacle)
string move_type         # "time", "distance", or "heading" (forward/turn modes)
float64 move_value       # Duration(s), distance(m), or heading(deg)
float64 move_speed       # Speed(m/s) or angular velocity(rad/s)
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
                        └─────────────────────┘     │                  │
                                                     │ GPS->map offset: │
                                                     │  goal_map = goal │
                                                     │  _gps + offset   │
                                                     │ offset = map_pos │
                                                     │  - gps_pos       │
                                                     └──────────────────┘
```

`gps_transform_node` always launches with `autonomous.launch.py` (no longer gated behind `use_ekf`). The dynamic navigator subscribes to `/gps/odom` and computes a live GPS-to-map offset to convert GPS-local goals into the SLAM map frame for the A* planner. Goal-reached checks use GPS-local distance. When GPS is unavailable, all behavior falls back to SLAM map-frame coordinates.

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
| `autonomous.launch.py` | SLAM + Nav2 planner + GPS transform + Dynamic navigator |
| `rc_control.launch.py` | Hardware RC driver for physical robot |
| `gps_navigation.launch.py` | GPS driver + EKF sensor fusion |
| `radio.launch.py` | RFD900x radio bridge (base or robot role) |
| `autonomous_hw.launch.py` | Hardware autonomous driving (real ATV) |

### exia_control (ament_python)
Navigation and control logic.

| Executable | Purpose |
|------------|---------|
| `ackermann_drive_node` | cmd_vel -> controller commands, publishes odom + TF |
| `dynamic_navigator_node` | Goal-based navigation with replanning |
| `nav_to` | CLI tool for sending navigation goals at runtime |

| Module | Purpose |
|--------|---------|
| `planning/pure_pursuit.py` | Pure Pursuit path following |
| `navigation/path_validator.py` | Collision detection (uint8 costmap) |
| `navigation/planner_interface.py` | Nav2 planner interface |

### exia_driver (ament_python)
Hardware drivers for physical robot.

| Executable | Purpose |
|------------|---------|
| `rc_driver_node` | RC radio receiver + Arduino serial control + autonomous cmd_vel bridge |
| `gps_transform_node` | GPS lat/lon to local XY conversion (RTK-gated origin) |
| `radio_bridge_node` | RFD900x encrypted radio bridge (base/robot roles) |

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
| Direct mode lidar scan | Raw LaserScan obstacle check at 50Hz in +/-30deg forward cone |
| Direct mode reverse | Straightens wheels + reverses 1.5m after obstacle stop for clearance |
| Forward move obstacle check | Lidar obstacle detection during forward movement commands (3m, +/-30deg) |
| Turn heading tolerance | 3-degree tolerance for heading turn completion |
| Move speed cap | Forward speed validated at CLI level (max 5.0 m/s) |
| GPS-referenced coordinates | All nav_to goals use GPS as coordinate reference when available |
| Costmap clamping | Goals beyond costmap range use intermediate waypoints with progressive replanning |
| GPS fallback | Falls back to SLAM map-frame when GPS unavailable |

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
| RC signal loss detection | Brake + neutral on RC timeout (500ms), skipped in autonomous mode |
| Median filter | 3-sample median on all RC channels |
| Rate-limited actuators | Smooth servo transitions with configurable rates (applies to autonomous targets too) |
| Gear shift interlock | Requires brake engaged before shifting (RC mode only) |
| Autonomous mode | `AUTO`/`MANUAL` serial commands, `J<throttle>,<brake>` and `K<gear>` from Jetson |
| Autonomous command timeout | 500ms without J/K command → safe state + exit autonomous mode |

### GPS Transform Node Safety

| Feature | Description |
|---------|-------------|
| RTK fix gate | Waits for STATUS_GBAS_FIX before auto-setting origin |
| Covariance propagation | GPS covariance passed through to odom output |

### Radio Bridge Safety

| Feature | Description |
|---------|-------------|
| RSA + AES-256-GCM encryption | All frames encrypted after handshake |
| Fragmented key exchange | RSA-encrypted AES key split into $K1/$K2 frames (serial buffer limit), 0.5s fragment timeout |
| Heartbeat watchdog | 10Hz heartbeat, 0.5s timeout triggers e-stop |
| Heartbeat ACK miss limit | 15 missed ACKs resets handshake and generates new AES key |
| E-stop distinction | Heartbeat-loss e-stop auto-clears on next heartbeat; remote e-stop requires explicit clear via `/radio/estop_clear` |
| E-stop acknowledgment | Robot sends EA frame on e-stop activate/clear/auto-clear |
| Continuous e-stop Twist | 20Hz zero cmd_vel while e-stop active |
| Nonce replay protection | Sequential counters (even=base, odd=robot), increment by 2 per send |
| Status relay | Robot relays `/navigation/status` to base via R frame type |
| Serial reconnect | Auto-reconnects every 1s on USB disconnect |
| Buffer overflow protection | Serial buffer capped at 4096 bytes |
| Startup grace | Watchdog only activates after first heartbeat received |
| Goal dedup | Hash + 2s dedup window prevents feedback loops on single-machine testing |
| Telemetry throttling | Logs telemetry at info level every 5s, debug otherwise |

### RC Driver Node Safety

| Feature | Description |
|---------|-------------|
| Startup grace period | 15s grace before requiring heartbeat |
| Autonomous cmd_vel watchdog | Stops throttle/brake if no cmd_vel for 0.5s |
| Autonomous mode toggle | `/driver/set_mode` service to switch between RC and autonomous |
| Auto gear management | Shifts HIGH when driving forward, NEUTRAL when stopped |
| RC steering ignored in autonomous | cmd_vel controls ODrive steering, RC CH1 relay still ACKed |

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
|                    +-- DIRECT_REVERSING  |
|         IDLE -> MOVE_FORWARD (cmd_vel)   |
|         IDLE -> MOVE_TURN    (cmd_vel)   |
|                                          |
|  - /navigation/goal subscriber           |
|    (xy, latlon, dms, forward, turn)      |
|  - GPS-referenced coordinate system      |
|    (subscribes /gps/odom, computes       |
|     GPS->map offset for goal placement)  |
|  - Costmap clamping for distant goals    |
|  - 500ms continuous replanning           |
|  - Obstacle-triggered immediate replan   |
|  - Pure Pursuit path execution           |
|  - Movement commands: forward/turn with  |
|    lidar obstacle detection              |
|  - Sensor health watchdog                |
|  - Thread-safe with nav_lock             |
|  - Direct mode: straight-line path,      |
|    lidar obstacle stop, reverse maneuver |
|  - /navigation/status feedback to CLI    |
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
| `auto_start` | `false` | Auto-navigate to default target on launch |

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

# Direct mode (straight-line, stop on obstacle, CLI blocks)
ros2 run exia_control nav_to xy 30.0 15.0 --direct

# Movement commands (CLI blocks until complete)
ros2 run exia_control nav_to forward 3s 2.0    # 3 seconds at 2 m/s
ros2 run exia_control nav_to forward 50m 1.5   # 50 meters at 1.5 m/s
ros2 run exia_control nav_to turn 90           # turn left 90 degrees
ros2 run exia_control nav_to turn -90          # turn right 90 degrees
ros2 run exia_control nav_to turn 3s 0.5       # turn 3s at 0.5 rad/s

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
| `/navigation/status` | String | Navigation status feedback (direct mode) |
| `/navigation/cancel` | Trigger (service) | Cancel current navigation |
| `/radio/cancel` | Trigger (service) | Cancel navigation over radio (laptop) |
| `/radio/estop` | Trigger (service) | Remote emergency stop over radio (laptop) |
| `/radio/estop_clear` | Trigger (service) | Clear remote e-stop over radio (laptop) |
| `/depth/points` | PointCloud2 | Depth camera point cloud (costmap only) |

## Sensors

### Lidar (Velodyne VLP-32C)
- Hardware: 32-channel 3D lidar
- Topic: `/velodyne_points` (PointCloud2), `/scan` (LaserScan via velodyne_laserscan)
- Range: 1.0m - 200m
- Horizontal FOV: 360 degrees
- Vertical FOV: +/-20 degrees (32 channels)
- Rate: ~10Hz (600 RPM default)
- Driver: `velodyne_driver` / `velodyne_pointcloud`
- Default sensor IP: 192.168.1.201 (Ethernet, UDP port 2368)
- Web interface: http://192.168.1.201

### Depth Camera (Orbbec Gemini 435Le)
- Hardware: Structured-light stereo depth camera
- Topic: `/depth/points` (PointCloud2)
- Range: 0.2m - 20m (reliable outdoor range ~8m)
- Horizontal FOV: 101 degrees
- Resolution: 640x480
- Rate: 15Hz
- Driver: `orbbec_camera` (hardware), Gazebo depth_camera sensor (simulation)
- Usage: Costmap obstacle marking only (not used for SLAM)

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
                    +-- camera_link -> camera_depth_optical_frame
                    +-- imu_link
                    +-- gps_link
```

## Visualization (Foxglove Studio)

Foxglove Studio is the default visualization tool for this project. It connects to ROS 2 topics via the Foxglove Bridge WebSocket server.

### Setup

```bash
# Install (one-time)
sudo apt install ros-$ROS_DISTRO-foxglove-bridge

# Launch bridge (run before or after simulation)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Open Foxglove Studio (desktop app or web) and connect: **Open connection > Foxglove WebSocket > `ws://localhost:8765`**

For remote access (e.g. Jetson), connect to `ws://<jetson-ip>:8765`.

### 3D Panel Configuration

Set the **Frame** dropdown to `map` (globally correct positioning). Do not use `base_link` as the frame — the camera follows the robot and it will appear stationary.

Enable these topics in the 3D panel sidebar:

| Topic | What it shows |
|-------|---------------|
| `/robot_description` | Robot URDF model |
| `/tf` | Live transform tree (robot motion) |
| `/tf_static` | Static transforms (sensor frames) |
| `/scan` | Lidar point cloud (environment outline, obstacles, trees) |
| `/map` | SLAM occupancy grid |
| `/global_costmap/costmap` | Obstacle costmap overlay |
| `/planned_path` | Navigation path line (only while navigating) |
| `/nav_markers` | Navigation visualization markers |
| `/depth/points` | Depth camera point cloud (forward-facing obstacles) |

### Additional Useful Panels

| Panel | Topic | What it shows |
|-------|-------|---------------|
| Plot | `/cmd_vel` | Linear/angular velocity over time |
| Raw Messages | `/navigation/status` | Nav status feedback |
| Raw Messages | `/navigation/goal` | Goal commands |
| GPS | `/gps/fix` | GPS position on world map |

### Notes

- Foxglove does NOT render the Gazebo world (ground, trees, buildings). Use `/scan` for lidar outlines and `/map` for the SLAM-built map instead.
- `/planned_path` only publishes while the robot is actively navigating to a goal.
- `odom` frame is locally smooth but drifts over time. `map` frame is globally correct but can have small jumps when SLAM corrects drift. Use `map` for navigation visualization.

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
ros2 topic echo /navigation/status --once

# Sensor health (check if scan/odom are publishing)
ros2 topic hz /scan
ros2 topic hz /odom

# Radio bridge debugging
ros2 node list | grep radio_bridge
ros2 service list | grep radio
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

### Hardware Autonomous Mode (Jetson + Arduino)

The `autonomous_hw.launch.py` launch file runs the full autonomous stack on the real ATV. The `rc_driver_node` bridges `/cmd_vel` to the physical actuators:
- ODrive steering via cmd_vel → Ackermann angle → position command
- Arduino throttle/brake via `J<throttle>,<brake>` serial commands
- Arduino gear via `K<gear>` serial commands (0=reverse, 1=neutral, 2=high)

```bash
ros2 launch exia_bringup autonomous_hw.launch.py

ros2 run exia_control nav_to xy 30.0 15.0
```

**Architecture:**
- `rc_driver_node` (autonomous_mode=true): subscribes `/cmd_vel`, controls ODrive + Arduino
- `ackermann_drive_node` (hardware_mode=false): publishes odom + TF only (no actuator control)
- `dynamic_navigator_node`: publishes `/cmd_vel` goals (unchanged from simulation)
- SLAM, Nav2, GPS transform: same as simulation autonomous stack

**Arduino serial protocol (autonomous mode):**
| Command | Direction | Description |
|---------|-----------|-------------|
| `AUTO\n` | Jetson → Arduino | Enter autonomous mode, ack with `AUTONOMOUS\n` |
| `MANUAL\n` | Jetson → Arduino | Exit autonomous mode, ack with `MANUAL\n` |
| `J<throttle>,<brake>\n` | Jetson → Arduino | Set throttle (0-75) and brake (80-180) |
| `K<gear>\n` | Jetson → Arduino | Set gear: 0=reverse, 1=neutral, 2=high |

**Parameters (rc_driver_node, autonomous additions):**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `autonomous_mode` | `false` | Enable autonomous cmd_vel control |

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/driver/set_mode` | Trigger | Toggle between autonomous and manual mode |

### Radio Bridge (RFD900x)

Encrypted radio link between home base laptop and Jetson using RFD900x modems. Single node with two roles selected by parameter. Uses udev rule (`99-exia-radio.rules`) to create stable `/dev/exia_radio` symlink.

**Setup (one-time):**
```bash
bash ~/exia_ws/scripts/generate_radio_keys.sh
# Copy ~/.exia/radio_private.pem and ~/.exia/radio_public.pem to both machines
# Install udev rule for stable device name:
sudo cp ~/exia_ws/scripts/99-exia-radio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

**Robot (Jetson):**
```bash
ros2 launch exia_bringup radio.launch.py role:=robot
```

**Base (Laptop):**
```bash
ros2 launch exia_bringup radio.launch.py role:=base
```

**Send commands from laptop (existing CLI, unchanged):**
```bash
ros2 run exia_control nav_to xy 30.0 15.0
ros2 service call /radio/cancel std_srvs/srv/Trigger
ros2 service call /radio/estop std_srvs/srv/Trigger
ros2 service call /radio/estop_clear std_srvs/srv/Trigger
```

**Protocol:** RSA-2048 key exchange (fragmented $K1/$K2 frames for serial buffer limits) + AES-256-GCM frame encryption. 10Hz heartbeat with ACK, 0.5s e-stop timeout. 5Hz telemetry. DMS payloads use pipe `|` separator. Nonce counters use even (base) / odd (robot) starting values, increment by 2.

**Frame types:**
| Frame | Direction | Description |
|-------|-----------|-------------|
| `$K1,` / `$K2,` | base -> robot | Fragmented RSA-encrypted AES key |
| `$A,` | robot -> base | Key exchange ACK (plaintext hash) |
| `H` | base -> robot | Heartbeat with sequence number |
| `A` | robot -> base | Heartbeat ACK |
| `N` | base -> robot | Navigation goal |
| `C` | base -> robot | Cancel navigation |
| `E` | base -> robot | Remote e-stop |
| `X` | base -> robot | Clear remote e-stop |
| `EA` | robot -> base | E-stop acknowledgment (activated/cleared/auto_cleared) |
| `S` | robot -> base | Telemetry (state, position, heading, speed) |
| `R` | robot -> base | Status relay (/navigation/status forwarded to base) |

**E-stop behavior:** Two distinct e-stop types:
- **Heartbeat loss**: Triggered after 0.5s without heartbeat. Auto-clears on next heartbeat received (sends `EA,auto_cleared`).
- **Remote e-stop**: Triggered via `/radio/estop` service. Requires explicit clear via `/radio/estop_clear` service. Sends `EA,activated` on trigger, `EA,cleared` on clear.
Both types: zero cmd_vel at 20Hz, cancel active navigation. Nodes stay alive. After 15 missed heartbeat ACKs, base resets handshake and generates new AES key. Navigation does NOT auto-resume after reconnection.

**Local testing with socat:**
```bash
socat -d -d pty,raw,echo=0,link=/tmp/radio0 pty,raw,echo=0,link=/tmp/radio1
ros2 launch exia_bringup radio.launch.py role:=robot serial_port:=/tmp/radio0
ros2 launch exia_bringup radio.launch.py role:=base serial_port:=/tmp/radio1
```

**Parameters (radio_bridge_node):**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `role` | `robot` | Bridge role: `base` or `robot` |
| `serial_port` | `/dev/exia_radio` | Serial port for RFD900x radio (udev symlink) |
| `serial_baud` | `57600` | Serial baud rate |
| `heartbeat_rate` | `10.0` | Heartbeat frequency (Hz) |
| `heartbeat_timeout` | `0.5` | Seconds before e-stop on heartbeat loss |
| `status_rate` | `5.0` | Telemetry frequency (Hz) |
| `key_dir` | `~/.exia` | Directory containing RSA key pair |

### Systemd Autostart

The startup script launches rc_driver_node, autonomous stack, and radio bridge as background processes. Requires RSA keys in `~/.exia/`. Skips radio bridge if radio not plugged in.

```bash
# Enable autostart on boot
sudo ~/exia_ws/scripts/exia_enable_autostart.sh

# Check status (includes radio key check)
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

Handles RC receiver input with gear shifting for manual driving. Supports autonomous mode where Jetson controls throttle/brake/gear via serial commands while RC steering relay continues.

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

## Standalone Lidar Testing (VLP-32C)

Test the Velodyne VLP-32C lidar independently (laptop or Jetson).

### Network Setup

The VLP-32C connects via Ethernet. The host must be on the same subnet as the sensor (default 192.168.1.201).

```bash
# Find ethernet interface name
ip link show

# Laptop: typically enp108s0 or similar
# Jetson: typically eth0
sudo ip addr add 192.168.1.100/24 dev <ethernet-interface>
sudo ip link set <ethernet-interface> up

# Verify connectivity
ping 192.168.1.201
```

### Launch Driver

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-velodyne  # one-time

# Terminal 1: Velodyne driver
ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py

# Terminal 2: Verify data
ros2 topic hz /velodyne_points

# Terminal 3: Foxglove bridge for visualization
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

### Foxglove Visualization

Connect Foxglove to `ws://localhost:8765`. Add a 3D panel, enable `/velodyne_points`, set frame to `velodyne`. Set Color by to `intensity` or `z` with Turbo colormap.

### Jetson Deployment

Same commands as laptop. The only difference is the Ethernet interface name (typically `eth0` on Jetson). If the Jetson ethernet is already used for another device, use a USB-to-Ethernet adapter and configure the new interface. For remote visualization, run Foxglove bridge on the Jetson and connect from your laptop at `ws://<jetson-ip>:8765`.

```bash
# Jetson network setup (eth0 is typical)
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up
ping 192.168.1.201
```

To make the network config persist across reboots on the Jetson, add a netplan config:
```bash
sudo tee /etc/netplan/99-velodyne.yaml <<EOF
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.1.100/24
EOF
sudo netplan apply
```

### Shutdown

Ctrl+C each terminal (Foxglove bridge, then driver). The sensor has no software shutdown; disconnect power/ethernet in any order.

## Dependencies

```bash
# Visualization
sudo apt install ros-$ROS_DISTRO-foxglove-bridge

# GPS and sensor fusion packages
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-septentrio-gnss-driver
sudo apt install ros-humble-nmea-msgs ros-humble-gps-msgs

# Lidar driver
sudo apt install ros-humble-velodyne

# Depth camera driver (hardware only)
sudo apt install ros-humble-orbbec-camera

# Radio bridge encryption
pip install cryptography
```

## Development Notes

- **Simulator**: Gazebo Fortress (not Classic)
- **Ackermann steering**: Cannot rotate in place, must have forward velocity to turn
- **Foxglove Studio**: Default visualization tool. Run `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765` and connect at `ws://localhost:8765`. Set 3D panel frame to `map`
- **Gazebo multicast warnings**: Set `export IGN_IP=127.0.0.1` to suppress
- **GPS Mode**: Set `USE_GPS_MODE = True` in dynamic_navigator_node.py for GPS waypoints. GPS coordinate reference is automatic when `gps_transform_node` is running (always launched by `autonomous.launch.py`)
- **Coordinate reference**: All `nav_to` goals are GPS-referenced when GPS is available. The navigator computes a live GPS->map offset. When GPS is unavailable, coordinates are treated as SLAM map-frame
- **Distant goals**: Goals beyond the costmap range (50m from robot) are handled via intermediate waypoint clamping with progressive replanning
- **After building exia_msgs**: Must re-source `install/setup.bash` before launching nodes that depend on it
- **Costmap data type**: path_validator uses uint8 (not int8) to correctly handle obstacle values 128-255
- **Thread safety**: dynamic_navigator_node uses threading.Lock for concurrent callback protection with ReentrantCallbackGroup

Dont write any comments in the code.
