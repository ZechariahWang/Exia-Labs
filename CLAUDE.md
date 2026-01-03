# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)
# Updated: December 2025 - ATV-Scale Vehicle (2.11m x 1.2m x 0.6m)

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
        |    Hardware Abstraction Layer (HAL)    |
        |  - SimulationHAL (Gazebo Fortress)     |
        |  - HardwareHAL (PWM/CAN/Serial)        |
        +----------------------------------------+
```

## Project Structure

```
exia_ws/
├── arduino/
│   └── exia_servo_controller/
│       └── exia_servo_controller.ino  # Arduino Mega servo controller
├── src/
│   └── exia_ground_description/
│       ├── urdf/
│       │   └── exia_ground.urdf.xacro    # Robot model
│       ├── launch/
│       │   ├── exia_ground_sim.launch.py # Gazebo simulation
│       │   ├── exia_ground_state.launch.py # RViz only
│       │   ├── dynamic_nav.launch.py     # Dynamic navigation (RECOMMENDED)
│       │   ├── mission_nav.launch.py     # Predefined path following
│       │   └── slam.launch.py            # SLAM only
│       ├── config/
│       │   ├── ackermann_controllers.yaml
│       │   ├── nav2_params.yaml
│       │   ├── slam_toolbox_params.yaml
│       │   └── hardware_config.yaml
│       ├── rviz/
│       │   ├── exia_ground.rviz
│       │   └── exia_slam_nav.rviz
│       ├── worlds/
│       │   └── exia_world.sdf
│       ├── scripts/active/              # ROS2 node entry points
│       │   ├── ackermann_drive_node.py  # Drive controller
│       │   ├── ackermann_odometry.py    # Fallback odometry
│       │   ├── xbox_teleop_node.py      # Xbox controller teleop
│       │   ├── ps4_teleop_node.py       # PS4 DualShock teleop
│       │   ├── path_follower_node.py    # Pure Pursuit demo
│       │   ├── mission_navigator_node.py # Predefined path nav
│       │   └── dynamic_navigator_node.py # Dynamic goal nav
│       └── src/exia_control/            # Modular Python package
│           ├── hal/                     # Hardware Abstraction Layer
│           ├── control/                 # Control algorithms
│           ├── planning/                # Path planning
│           └── navigation/              # Obstacle detection
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
ros2 launch exia_ground_description exia_ground_sim.launch.py

# Terminal 2: Start dynamic navigation (navigates to TARGET_POINT)
ros2 launch exia_ground_description dynamic_nav.launch.py

# Terminal 3 (optional): RViz visualization
rviz2 -d ~/exia_ws/src/exia_ground_description/rviz/exia_slam_nav.rviz
```

## Navigation Modes

### 1. Dynamic Navigation (RECOMMENDED)

Goal-based navigation with continuous A* replanning. Robot navigates to a single
target coordinate using SLAM for localization and 500ms replanning for dynamic
obstacle avoidance.

**Architecture:**
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
```

**Configuration** (`dynamic_navigator_node.py`):
```python
TARGET_POINT = [0, 20]  # [x, y] in meters (map frame)
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_speed` | `2.0` | Target speed (m/s) |
| `lookahead_distance` | `2.5` | Pure Pursuit lookahead (m) |
| `replan_period` | `0.5` | Replanning interval (s) |
| `goal_tolerance` | `1.0` | Goal reached threshold (m) |
| `obstacle_lookahead` | `8.0` | Obstacle detection range (m) |

**Launch:**
```bash
ros2 launch exia_ground_description dynamic_nav.launch.py
```

### 2. Mission Navigation

Follows predefined paths with obstacle avoidance. Uses A* for detour planning.

**Launch:**
```bash
ros2 launch exia_ground_description mission_nav.launch.py
ros2 launch exia_ground_description mission_nav.launch.py path_type:=square
```

### 3. Path Follower (Demo)

Simple Pure Pursuit demo following geometric paths. No obstacle avoidance.

```bash
ros2 run exia_ground_description path_follower_node.py
ros2 run exia_ground_description path_follower_node.py --ros-args -p path_type:=circle
```

## Control Commands

```bash
# Drive forward at 1.0 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}" -1

# Turn left while driving
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}" -1

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1

# Emergency stop
ros2 service call /ackermann/emergency_stop std_srvs/srv/Trigger
```

## Teleop Control

Manual control using gamepad controllers. Two versions available:

### Xbox 360/One Controller (`xbox_teleop_node.py`)

| Input | Action |
|-------|--------|
| Left Stick X | Steering |
| RT (Right Trigger) | Throttle |
| LT (Left Trigger) | Brake |
| A Button | Toggle enable/disable |
| B Button | Emergency stop |
| Start Button | Clear E-stop |
| Back Button | Reset steering trim |

```bash
ros2 run joy joy_node &
ros2 run exia_ground_description xbox_teleop_node.py
```

### PS4 DualShock 4 Controller (`ps4_teleop_node.py`)

| Input | Action |
|-------|--------|
| Left Stick X | Steering |
| R2 (Right Trigger) | Throttle |
| L2 (Left Trigger) | Brake |
| X (Cross) Button | Toggle enable/disable |
| Circle Button | Emergency stop |
| Options Button | Clear E-stop |
| Share Button | Reset steering trim |

```bash
ros2 run joy joy_node &
ros2 run exia_ground_description ps4_teleop_node.py
```

**Serial Communication:**
- Port: `/dev/ttyACM0` (Arduino Mega)
- Baud: 115200
- Protocol: `S<steer>,T<throttle>,B<brake>\n`

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

**Safety Features:**
- 2-second command timeout triggers emergency stop
- Must send ARM before servo commands are accepted
- LED indicates armed state

## Motor Smoothing & Soft-Start

Both Python and Arduino implement smoothing and rate limiting to prevent jittery movements and protect hardware.

**Arduino (exponential filter + rate limiting):**
```cpp
// Smoothing
const float SMOOTH_ALPHA = 0.25;  // 0.1=smooth, 0.5=responsive

// Rate limiting (degrees per 10ms update cycle)
const int MAX_STEERING_RATE = 5;   // Can increase after testing
const int MAX_THROTTLE_RATE = 3;   // Slower for motor protection
const int MAX_BRAKE_RATE = 10;     // Faster for safety
```

**Python (hysteresis + ramping + warmup):**
- Servo values only change if difference > 1 degree (hysteresis)
- Throttle/brake use configurable ramping
- Trigger deadzone: 8%
- Serial rate: 50Hz
- **Soft-start**: Throttle limited to 30% for first 3 seconds

**Configurable ROS Parameters:**
```bash
ros2 run exia_ground_description xbox_teleop_node.py --ros-args \
  -p throttle_ramp_rate:=5.0 \
  -p brake_ramp_rate:=8.0 \
  -p max_initial_throttle:=0.3 \
  -p warmup_duration:=3.0
```

**Tuning Parameters:**
| Parameter | Location | Default | Effect |
|-----------|----------|---------|--------|
| `SMOOTH_ALPHA` | Arduino | 0.25 | Lower = smoother, slower response |
| `MAX_THROTTLE_RATE` | Arduino | 3 | Max degrees per 10ms cycle |
| `throttle_ramp_rate` | Python | 5.0 | Lower = gentler acceleration |
| `max_initial_throttle` | Python | 0.3 | Throttle limit during warmup (30%) |
| `warmup_duration` | Python | 3.0 | Seconds before full throttle allowed |

## Build Commands

```bash
cd /home/zech/exia_ws
colcon build --packages-select exia_ground_description
source install/setup.bash
```

## Key Files

### Launch Files
| File | Purpose |
|------|---------|
| `exia_ground_sim.launch.py` | Gazebo Fortress simulation |
| `dynamic_nav.launch.py` | SLAM + A* + Dynamic navigator |
| `mission_nav.launch.py` | Predefined path + obstacle avoidance |
| `slam.launch.py` | SLAM only (mapping) |

### Configuration
| File | Purpose |
|------|---------|
| `nav2_params.yaml` | Planner + costmap config |
| `slam_toolbox_params.yaml` | SLAM configuration |
| `ackermann_controllers.yaml` | ros2_control config |

### ROS2 Nodes (`scripts/active/`)
| Node | Purpose |
|------|---------|
| `ackermann_drive_node.py` | cmd_vel -> motor commands, publishes odom |
| `xbox_teleop_node.py` | Xbox controller -> servo commands via serial |
| `ps4_teleop_node.py` | PS4 DualShock 4 -> servo commands via serial |
| `dynamic_navigator_node.py` | Goal-based nav with 500ms replanning |
| `mission_navigator_node.py` | Predefined path following |
| `path_follower_node.py` | Pure Pursuit demo |

### Python Package (`src/exia_control/`)
| Module | Purpose |
|--------|---------|
| `hal/` | Hardware Abstraction Layer (simulation/hardware) |
| `control/` | Ackermann kinematics |
| `planning/` | Pure Pursuit, predefined paths |
| `navigation/` | Path validation, planner interface |

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands |
| `/odom` | Odometry | Robot odometry |
| `/scan` | LaserScan | Lidar data |
| `/imu/data` | Imu | IMU data |
| `/goal_pose` | PoseStamped | Navigation goal |
| `/planned_path` | Path | Current path |
| `/global_costmap/costmap` | OccupancyGrid | Obstacle map |

## Sensors

### Lidar (Slamtec RPlidar S3 compatible)
- Topic: `/scan`
- Range: 0.2m - 20m
- FOV: 360 degrees
- Rate: 10Hz

### IMU (Yahboom 10-axis compatible)
- Topic: `/imu/data`
- Rate: 200Hz

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
                    └── imu_link
```

## Debugging

```bash
# Check nodes
ros2 node list

# Check TF tree
ros2 run tf2_ros tf2_echo map odom

# Check costmap
ros2 topic hz /global_costmap/costmap

# Check planner
ros2 lifecycle get /planner_server

# View path
ros2 topic echo /planned_path --once
```

## Development Notes

- **Simulator**: Gazebo Fortress (not Classic)
- **Ackermann steering**: Cannot rotate in place, must have forward velocity to turn
- **RViz Fixed Frame**: Use `odom` or `map` depending on navigation mode
- **Sensors require custom world**: Use `exia_world.sdf` (includes Sensors plugin)

## Hardware Deployment

The HAL architecture enables easy hardware transition:

1. Change `hal_type: 'hardware'` in parameters
2. Implement `HardwareHAL` class in `hal/hardware.py`
3. Configure PWM/CAN/Serial for physical actuators

Hardware checklist:
- Steering: PWM servo
- Throttle: ESC/Motor controller
- Brake: Servo or linear actuator
- Lidar: RPlidar S3 via rplidar_ros
- IMU: Yahboom 10-axis driver
