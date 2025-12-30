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
├── src/
│   └── exia_ground_description/    # Robot description package
│       ├── urdf/                   # URDF/Xacro robot models
│       ├── launch/                 # Launch files
│       │   ├── exia_ground_sim.launch.py    # Gazebo simulation
│       │   ├── mission_nav.launch.py        # Mission navigation (RECOMMENDED)
│       │   ├── slam.launch.py               # SLAM only (legacy)
│       │   └── slam_nav_simple.launch.py    # SLAM + Nav (legacy)
│       ├── config/                 # Controller configurations
│       │   ├── ackermann_controllers.yaml   # ros2_control config
│       │   └── nav2_params.yaml             # Costmap + planner config
│       ├── rviz/                   # RViz configuration
│       │   ├── exia_ground.rviz             # Basic visualization
│       │   └── exia_slam_nav.rviz           # SLAM + Navigation
│       ├── worlds/                 # Gazebo world files
│       │   └── exia_world.sdf      # Custom world with obstacles & sensors
│       ├── scripts/
│       │   ├── active/             # Current ROS2 node entry points
│       │   │   ├── ackermann_drive_node.py   # Main drive controller
│       │   │   ├── path_follower_node.py     # Pure Pursuit demo paths
│       │   │   ├── mission_navigator_node.py # Mission nav with obstacle avoidance
│       │   │   └── ackermann_odometry.py     # Fallback odometry node
│       │   └── archive/            # Legacy scripts (reference only)
│       └── src/
│           └── exia_control/       # Modular Python package
│               ├── hal/            # Hardware Abstraction Layer
│               │   ├── base.py     # HAL interface & data types
│               │   └── simulation.py  # Gazebo Fortress HAL
│               ├── control/        # Control algorithms
│               │   └── ackermann_drive.py  # Ackermann kinematics
│               ├── planning/       # Path planning & following
│               │   ├── pure_pursuit.py  # Pure Pursuit controller
│               │   └── paths.py         # Predefined paths
│               └── navigation/     # Obstacle detection & replanning
│                   ├── path_validator.py     # Costmap collision check
│                   ├── planner_interface.py  # Nav2 A* planner API
│                   └── replan_manager.py     # Replan decision logic
├── build/                          # Build artifacts (generated)
├── install/                        # Install space (generated)
└── log/                            # Build logs (generated)
```

## Packages

### exia_ground_description
Robot description package containing URDF model and launch files for visualization and simulation.

- **URDF**: 4-wheel Ackermann steering ground robot (car-like)
- **Drive**: Three-motor Ackermann (steering, throttle, brake)
- **Simulator**: Gazebo Fortress with gz_ros2_control
- **HAL**: Hardware Abstraction Layer for sim/hardware switching
- **Maintainer**: zech (zechariahwang@gmail.com)

## Robot Specifications

| Property | Value |
|----------|-------|
| Chassis dimensions | 2.11m x 1.2m x 0.6m (L x W x H) |
| Wheel radius | 0.3m |
| Wheel width | 0.15m |
| Track width | 1.1m (wheel center to center) |
| Wheel base | 1.3m (front to rear axle) |
| Chassis mass | 200.0 kg |
| Wheel mass | 10.0 kg each |
| Max steering angle | 0.6 rad (~34 degrees) |
| Max speed | 5.0 m/s |

## Drive System: Three-Motor Ackermann

The robot uses a **three-motor Ackermann steering** architecture designed for
ATV-style vehicles:

### Motor Architecture
| Motor | Function | Interface | Control Type |
|-------|----------|-----------|--------------|
| Steering | Front wheel angle | Servo/Linear Actuator | Position (radians) |
| Throttle | Rear wheel drive | ESC/Motor Controller | Velocity (rad/s) |
| Brake | Brake actuator | Servo/Caliper | Position (0-1) |

### Control Flow
1. **cmd_vel** (Twist) -> Ackermann Drive Node
2. Ackermann kinematics compute steering angle from angular velocity
3. HAL translates to motor commands (simulation or hardware)
4. Three separate controllers execute commands

### Ackermann Geometry
- Inner/outer wheel angles calculated for proper geometry
- Speed-dependent steering limits prevent rollover
- Proper turning radius: `R = wheelbase / tan(steering_angle)`

### TF Tree Structure
```
odom
  └── base_footprint
        └── base_link (chassis)
              ├── front_left_steer_link (revolute - steering)
              │     └── front_left_wheel (continuous - rotation)
              ├── front_right_steer_link (revolute - steering)
              │     └── front_right_wheel (continuous - rotation)
              ├── rear_left_wheel (continuous - driven)
              ├── rear_right_wheel (continuous - driven)
              ├── brake_actuator_link (revolute - brake)
              ├── imu_link (fixed)
              └── lidar_link (fixed)
```

## Build Commands

```bash
# Build the workspace
cd /home/zech/exia_ws
colcon build

# Build specific package
colcon build --packages-select exia_ground_description

# Source the workspace
source install/setup.bash
```

## Launch Commands

```bash
# Launch robot state publisher with joint state GUI (for RViz visualization only)
ros2 launch exia_ground_description exia_ground_state.launch.py

# Launch Gazebo Fortress simulation
ros2 launch exia_ground_description exia_ground_sim.launch.py

# Visualize in RViz2 (use when Gazebo GUI has issues)
rviz2 -d ~/exia_ws/src/exia_ground_description/rviz/exia_ground.rviz

# Run path follower (after simulation is running)
ros2 run exia_ground_description path_follower_node.py

# Run path follower with specific path type
ros2 run exia_ground_description path_follower_node.py --ros-args -p path_type:=square
ros2 run exia_ground_description path_follower_node.py --ros-args -p path_type:=circle
ros2 run exia_ground_description path_follower_node.py --ros-args -p path_type:=figure_eight
```

## Control Commands

### High-Level Control (Recommended)
Use cmd_vel for velocity commands - the Ackermann drive node handles translation:

```bash
# Drive forward at 1.0 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}" -1

# Turn left while driving (0.3 rad/s angular velocity)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}" -1

# Turn right while driving
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: -0.3}}" -1

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1

# Emergency stop (via service)
ros2 service call /ackermann/emergency_stop std_srvs/srv/Trigger

# Clear emergency stop
ros2 service call /ackermann/clear_estop std_srvs/srv/Trigger
```

### Low-Level Control (Direct Controller Access)
For debugging or direct control:

```bash
# Direct steering control (position in radians)
ros2 topic pub /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.3]}" -1

# Direct throttle control (velocity in rad/s)
ros2 topic pub /throttle_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0, 5.0]}" -1

# Direct brake control (0.0 = no brake, 1.0 = full brake)
ros2 topic pub /brake_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}" -1

# Stop wheels
ros2 topic pub /throttle_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" -1

# Center steering
ros2 topic pub /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" -1
```

## Path Following

The robot includes a Pure Pursuit path following system for autonomous navigation.

### Path Types
| Path | Description |
|------|-------------|
| `line` | Straight line along X-axis |
| `square` | Square path (returns to start) |
| `circle` | Circular path |
| `figure_eight` | Figure-8 pattern |
| `slalom` | Weaving slalom pattern |

### Path Follower Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `path_type` | `figure_eight` | Type of path to follow |
| `path_scale` | `2.0` | Scale factor for path size |
| `speed` | `0.5` | Target speed (m/s) |
| `lookahead_distance` | `2.0` | Pure Pursuit lookahead (m) |
| `goal_tolerance` | `0.5` | Waypoint reached threshold (m) |
| `auto_start` | `true` | Start following immediately |

### Path Follower Services
```bash
# Start path following
ros2 service call /path_follower/start std_srvs/srv/Trigger

# Stop path following
ros2 service call /path_follower/stop std_srvs/srv/Trigger

# Reset to start of path
ros2 service call /path_follower/reset std_srvs/srv/Trigger
```

### Visualization Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | nav_msgs/Path | Current path being followed |
| `/path_markers` | visualization_msgs/MarkerArray | Waypoint markers |

## Mission Navigation (Simplified)

The robot follows **predefined paths** with **obstacle avoidance**. No SLAM required.

### Architecture

```
Predefined Path (odom frame)
         |
         v
+---------------------------------------------+
|          Mission Navigator Node             |
|  +-------------+  +-----------+  +--------+ |
|  |  Waypoint   |  | Obstacle  |  | Detour | |
|  |   Tracker   |--| Monitor   |--| Planner| |
|  +-------------+  +-----------+  +--------+ |
|         |               |             |     |
|         v               v             v     |
|  +--------------------------------------------+
|  |         Pure Pursuit Controller            |
|  +--------------------------------------------+
+---------------------------------------------+
         |                             ^
         v                             |
     /cmd_vel                   /global_costmap
         |                             |
         v                             |
+----------------+         +----------------------+
| Ackermann      |         | Nav2 Planner Server  |
| Drive Node     |         | (30m rolling window) |
+----------------+         +----------------------+
         |                             ^
         v                             |
    Physical Robot  <--------------  /scan (Lidar)
```

### Key Components

| Component | Purpose |
|-----------|---------|
| Mission Navigator | Follows predefined path, detects obstacles, plans detours |
| Planner Server | Smac Hybrid-A* for computing detour paths |
| Costmap | 30m rolling window in odom frame (no SLAM needed) |
| Path Validator | Checks lookahead path for obstacles |
| Pure Pursuit | Executes path with Ackermann kinematics |

### State Machine

```
IDLE --> FOLLOWING --> BLOCKED --> PLANNING_DETOUR --> DETOURING
                 ^                                          |
                 |                                          |
                 +------------- (rejoin path) <-------------+
```

### Ackermann-Specific Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Minimum turning radius | 1.9m | wheelbase / tan(max_steering) |
| Motion model | REEDS_SHEPP | Allows forward + reverse |
| Robot footprint | 2.1m x 1.2m | Actual robot size |
| Rotate in place | Disabled | Ackermann cannot rotate in place |

### Launch Commands

```bash
# Start simulation first (Terminal 1)
ros2 launch exia_ground_description exia_ground_sim.launch.py

# Terminal 2: Mission navigation (RECOMMENDED)
ros2 launch exia_ground_description mission_nav.launch.py

# With specific path type
ros2 launch exia_ground_description mission_nav.launch.py path_type:=square

# Terminal 3: RViz visualization
rviz2 -d ~/exia_ws/src/exia_ground_description/rviz/exia_ground.rviz
```

### Mission Navigator Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `path_type` | `figure_eight` | Predefined path (line, square, circle, figure_eight, slalom) |
| `path_scale` | `3.0` | Scale factor for path size |
| `target_speed` | `1.0` | Target speed (m/s) |
| `lookahead_distance` | `2.0` | Pure Pursuit lookahead (m) |
| `obstacle_lookahead` | `5.0` | Distance to check for obstacles (m) |
| `detour_clearance` | `3.0` | Distance past obstacle to rejoin (m) |
| `auto_start` | `true` | Start mission automatically |

### Mission Services

```bash
# Start mission
ros2 service call /mission/start std_srvs/srv/Trigger

# Stop mission
ros2 service call /mission/stop std_srvs/srv/Trigger

# Reset mission (start from beginning)
ros2 service call /mission/reset std_srvs/srv/Trigger
```

### Navigation Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | nav_msgs/Path | Current mission path |
| `/path_markers` | visualization_msgs/MarkerArray | Waypoint visualization |
| `/planner_server/global_costmap/costmap` | nav_msgs/OccupancyGrid | Obstacle costmap |

### Legacy SLAM Navigation

For advanced use cases requiring SLAM:

```bash
# SLAM only (for mapping)
ros2 launch exia_ground_description slam.launch.py

# SLAM + Navigation (if needed)
ros2 launch exia_ground_description slam_nav_simple.launch.py
```

## ROS Topics

### Command Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | High-level velocity commands |
| `/emergency_stop` | std_msgs/Bool | Emergency stop trigger |

### Controller Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/steering_controller/commands` | std_msgs/Float64MultiArray | Steering angle [left, right] (rad) |
| `/throttle_controller/commands` | std_msgs/Float64MultiArray | Wheel velocity [left, right] (rad/s) |
| `/brake_controller/commands` | std_msgs/Float64MultiArray | Brake engagement [0-1] |

### Feedback Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Joint positions and velocities |
| `/odom` | nav_msgs/Odometry | Odometry estimate |
| `/scan` | sensor_msgs/LaserScan | 2D lidar scan data (15Hz) |
| `/imu/data` | sensor_msgs/Imu | IMU sensor data (200Hz) |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

### Debug Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/ackermann/steering` | std_msgs/Float64 | Current steering command |
| `/ackermann/throttle` | std_msgs/Float64 | Current throttle command |
| `/ackermann/brake` | std_msgs/Float64 | Current brake command |
| `/ackermann/estop_status` | std_msgs/Bool | E-stop status |

### System Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/robot_description` | std_msgs/String | URDF robot description |
| `/clock` | rosgraph_msgs/Clock | Simulation clock |

## Controllers (ros2_control)

| Controller | Type | Joints |
|------------|------|--------|
| joint_state_broadcaster | JointStateBroadcaster | All joints |
| steering_controller | ForwardCommandController | front_left_steer_joint, front_right_steer_joint |
| throttle_controller | ForwardCommandController | rear_left_wheel_joint, rear_right_wheel_joint |
| brake_controller | ForwardCommandController | brake_joint |

## Sensors

### Lidar (Slamtec RPlidar S3 compatible)
- **Topic**: `/scan`
- **Frame**: `lidar_link`
- **Message Type**: `sensor_msgs/LaserScan`
- **Update rate**: 10Hz
- **Range**: 0.2m - 20m
- **FOV**: 360 degrees
- **Samples**: 360 per scan (1 degree resolution)
- **Position**: Forward of center, lowered for obstacle detection (X=0.3m, Z=0.15m from base_link)
- **Tilt**: 5 degrees down (0.087 rad) - hits ground at ~1.7m to detect low obstacles
- **Simulation**: Gazebo Fortress GPU lidar sensor (requires Sensors system plugin)
- **Hardware**: Slamtec RPlidar S3 via rplidar_ros package

### IMU (Yahboom 10-axis compatible)
- **Topic**: `/imu/data`
- **Frame**: `imu_link`
- **Update rate**: 200Hz
- **Position**: Centered on top of chassis
- **Noise**: Gaussian (gyro: 0.0002 rad/s, accel: 0.017 m/s²)

## Gazebo World

The simulation uses a custom world (`worlds/exia_world.sdf`) with:
- **Sensors system plugin** - Required for IMU and Lidar to function
- **Obstacle course** - Various boxes, cylinders, and walls for lidar testing
- **Top-down camera** - Default view looking down from 25m

### World Obstacles
| Obstacle | Position | Color |
|----------|----------|-------|
| North box | (8, 0) | Red |
| South box | (-8, 0) | Blue |
| East box | (0, 8) | Green |
| West box | (0, -8) | Yellow |
| 4 corner cylinders | (±6, ±6) | Various |
| 3 angled walls | Various | Brown/Green/Blue |
| Small obstacles | Near center | Various |

## RViz Visualization

### Launch RViz with pre-configured display:
```bash
rviz2 -d ~/exia_ws/src/exia_ground_description/rviz/exia_ground.rviz
```

### Manual RViz Setup:
1. Set **Fixed Frame** to `odom`
2. Add **RobotModel** (Description Topic: `/robot_description`)
3. Add **TF** to see coordinate frames
4. Add **LaserScan** (Topic: `/scan`, Size: 0.1m)
5. For top-down view: View → Type → TopDownOrtho

### Visualize Path Following:
- Add **Path** display (Topic: `/planned_path`)
- Add **MarkerArray** display (Topic: `/path_markers`)

## Dependencies

- ROS 2 Humble
- Gazebo Fortress (`gz-fortress`)
- ros_gz (`ros-humble-ros-gz`)
- gz_ros2_control (`ros-humble-gz-ros2-control`)
- ros2_control, ros2_controllers
- robot_state_publisher
- xacro
- joint_state_publisher_gui (for state launch only)
- slam_toolbox (`ros-humble-slam-toolbox`) - SLAM
- nav2_bringup (`ros-humble-nav2-bringup`) - Navigation stack
- nav2_smac_planner (`ros-humble-nav2-smac-planner`) - Ackermann-aware A* planner

## Key Files

### Robot Model & Configuration
- `urdf/exia_ground.urdf.xacro` - Robot model (Xacro format)
- `config/ackermann_controllers.yaml` - Three-motor controller configuration
- `config/hardware_config.yaml` - Minimal config for gz_ros2_control

### Launch Files
- `launch/exia_ground_sim.launch.py` - Gazebo Fortress simulation
- `launch/exia_ground_state.launch.py` - State publisher (RViz only)

### World Files
- `worlds/exia_world.sdf` - Custom Gazebo world with Sensors plugin and obstacles

### ROS2 Node Entry Points (`scripts/active/`)
- `ackermann_drive_node.py` - Unified drive controller (cmd_vel -> motors)
- `path_follower_node.py` - Pure Pursuit path following demo (predefined paths)
- `navigation_node.py` - SLAM + A* navigation with obstacle avoidance
- `ackermann_odometry.py` - Fallback odometry node

### Legacy Scripts (`scripts/archive/`)
- Archived reference scripts (not installed, kept for reference)

### Modular Python Package (`src/exia_control/`)

**HAL - Hardware Abstraction Layer** (`hal/`)
- `base.py` - AckermannConfig, AckermannCommand, AckermannState, AckermannHAL interface
- `simulation.py` - SimulationHAL for Gazebo Fortress (ros2_control)

**Control Algorithms** (`control/`)
- `ackermann_drive.py` - DriveControllerConfig, AckermannDriveController (cmd_vel -> Ackermann)

**Path Planning** (`planning/`)
- `pure_pursuit.py` - PurePursuitConfig, PurePursuitController (geometric path tracking)
- `paths.py` - Predefined paths (line, square, circle, figure-eight, slalom)

**Navigation** (`navigation/`)
- `path_validator.py` - PathValidator (checks path against costmap for collisions)
- `planner_interface.py` - PlannerInterface (async interface to Nav2 A* planner)
- `replan_manager.py` - ReplanManager (decides when to trigger replanning)
- `costmap_monitor.py` - CostmapMonitor (subscribes to costmap updates)

### Visualization
- `rviz/exia_ground.rviz` - RViz config (Fixed Frame: odom)

## Debugging Commands

```bash
# List all active nodes
ros2 node list

# List all topics
ros2 topic list

# Check controllers
ros2 control list_controllers

# Check joint states
ros2 topic echo /joint_states --once

# Check odometry
ros2 topic echo /odom --once

# Check IMU data
ros2 topic echo /imu/data --once

# Check TF tree
ros2 run tf2_tools view_frames

# View TF between frames
ros2 run tf2_ros tf2_echo odom base_footprint

# Check lidar scan data
ros2 topic echo /scan --once

# Check lidar scan rate
ros2 topic hz /scan

# Check Gazebo topics (use 'ign' for Fortress)
ign topic -l

# Check if sensors are publishing in Gazebo
ign topic -l | grep -E "(lidar|imu|scan)"

# ===== Navigation Debugging =====

# Check map is being published
ros2 topic echo /map --once

# Check navigation status
ros2 topic echo /navigation/active --once

# View planned path
ros2 topic echo /planned_path --once

# Check planner server status
ros2 lifecycle get /planner_server

# View TF map->odom transform (from SLAM)
ros2 run tf2_ros tf2_echo map odom

# Check costmap updates
ros2 topic hz /planner_server/global_costmap/costmap
```

## Development Notes

- **Simulator**: Gazebo Fortress with gz_ros2_control (not Gazebo Classic)
- **Ackermann steering**: Unlike differential drive, robot cannot rotate in place. Must have forward velocity to turn.
- Gazebo Fortress uses `ros_gz_bridge` to bridge topics between Gazebo and ROS 2
- RViz Fixed Frame must be set to `odom` to see robot movement
- `use_sim_time: True` is set for proper Gazebo time synchronization
- The robot uses `forward_command_controller` for direct joint control

## Known Issues

- Gazebo Fortress GUI may show libEGL warnings on some systems - simulation still works
- If controllers fail to load, ensure controller_manager service is running
- Ackermann steering has minimum turning radius - cannot turn sharply at high speeds
- **Sensors require custom world**: The default `empty.sdf` doesn't include the Sensors system plugin. Use `exia_world.sdf` for IMU/Lidar to work
- **Use `ign` not `gz`**: For Gazebo Fortress commands, use `ign topic -l` (not `gz topic -l`)
- **Nav2 bt_navigator library issue**: The full Nav2 stack may have library compatibility issues with `bt_navigator`. Use `slam_nav_simple.launch.py` which bypasses this by only using the planner server
- **Path planning in unknown space**: If planner fails with "no valid path found", ensure `allow_unknown: true` in nav2_params.yaml

## Hardware Deployment Notes

### Switching from Simulation to Real Hardware

The HAL architecture makes hardware deployment straightforward:

1. **Change HAL Type**: In launch file or parameters:
   ```yaml
   hal_type: 'hardware'  # Instead of 'simulation'
   ```

2. **Create Hardware HAL** (`src/exia_control/hal/hardware.py`):
   - Implement `HardwareHAL` class extending `AckermannHAL`
   - Set driver type: `GPIO_PWM`, `PCA9685`, `SERIAL`, or `CAN`
   - Configure PWM channels and pulse widths for your servos/ESCs
   - Implement encoder feedback for closed-loop control

### Hardware Setup Checklist

| Component | Simulation | Real Hardware |
|-----------|------------|---------------|
| Steering | gz_ros2_control | PWM servo (50Hz, 1000-2000us) |
| Throttle | gz_ros2_control | ESC/Motor controller |
| Brake | gz_ros2_control | Servo or linear actuator |
| Lidar | Gazebo GPU sensor | RPlidar S3 via rplidar_ros |
| IMU | Gazebo sensor | Yahboom 10-axis driver |
| Encoders | Gazebo physics | Real wheel encoders |

### RPlidar S3 Hardware Deployment

To switch from simulated lidar to real RPlidar S3:

1. **Install rplidar_ros package**:
   ```bash
   sudo apt install ros-humble-rplidar-ros
   ```

2. **Configure USB permissions**:
   ```bash
   # Create udev rule for persistent device naming
   echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

3. **Launch RPlidar node** (instead of Gazebo sensor):
   ```bash
   ros2 launch rplidar_ros rplidar_s3_launch.py serial_port:=/dev/rplidar frame_id:=lidar_link
   ```

4. **Create hardware launch file** (`exia_ground_hardware.launch.py`):
   ```python
   # Add RPlidar node instead of Gazebo bridge for /scan
   rplidar_node = Node(
       package='rplidar_ros',
       executable='rplidar_node',
       name='rplidar_node',
       parameters=[{
           'serial_port': '/dev/rplidar',
           'serial_baudrate': 256000,  # RPlidar S3 baudrate
           'frame_id': 'lidar_link',
           'angle_compensate': True,
           'scan_mode': 'Standard',
       }],
       output='screen'
   )
   ```

**RPlidar S3 Specifications**:
| Parameter | Value |
|-----------|-------|
| Range | 0.1m - 40m |
| Sample Rate | 32,000 samples/sec |
| Scan Frequency | 10-20 Hz |
| Angular Resolution | 0.225 deg |
| Interface | USB (CP2102 UART) |
| Baudrate | 256000 |
| Weight | 190g |

### Example Hardware HAL Implementation

```python
from exia_control.hal.base import AckermannHAL, AckermannConfig, AckermannCommand

class HardwareHAL(AckermannHAL):
    def __init__(self, node, config: AckermannConfig):
        super().__init__(config)
        self.node = node
        # Initialize your hardware drivers here

    def initialize(self) -> bool:
        # Setup PWM, CAN, Serial, etc.
        return True

    def set_command(self, command: AckermannCommand) -> bool:
        # Convert command to hardware signals
        # command.steering_angle (radians)
        # command.throttle (0-1)
        # command.brake (0-1)
        return True
```

### Safety Considerations

1. **Emergency Stop**: Hardware E-stop should be wired independently
2. **Watchdog**: HAL includes 500ms timeout - stops motors if no commands
3. **Soft Limits**: Configure max steering angle and speed in parameters
4. **Brake Failsafe**: On shutdown or E-stop, brake is automatically engaged
