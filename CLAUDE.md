# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)

ROS 2 workspace for the Exia Ground robot platform.

## Project Structure

```
exia_ws/
├── src/
│   └── exia_ground_description/    # Robot description package
│       ├── urdf/                   # URDF/Xacro robot models
│       ├── launch/                 # Launch files
│       ├── config/                 # Controller configurations
│       ├── rviz/                   # RViz configuration
│       ├── scripts/                # Python control scripts
│       ├── include/                # C++ headers (empty)
│       └── src/                    # C++ source (empty)
├── build/                          # Build artifacts (generated)
├── install/                        # Install space (generated)
└── log/                            # Build logs (generated)
```

## Packages

### exia_ground_description
Robot description package containing URDF model and launch files for visualization and simulation.

- **URDF**: 4-wheel Ackermann steering ground robot (car-like)
- **Drive**: Ackermann steering - front wheels steer, rear wheels drive
- **Simulator**: Gazebo Fortress with gz_ros2_control
- **Maintainer**: zech (zechariahwang@gmail.com)

## Robot Specifications

| Property | Value |
|----------|-------|
| Chassis dimensions | 0.6m x 0.4m x 0.2m (L x W x H) |
| Wheel radius | 0.1m |
| Wheel width | 0.05m |
| Wheel separation | 0.45m (center to center) |
| Wheel base | 0.4m (front to rear axle) |
| Chassis mass | 20.0 kg |
| Wheel mass | 2.0 kg each |
| Max steering angle | 0.6 rad (~34 degrees) |
| Max speed | 5.0 m/s |

## Drive System: Ackermann Steering

The robot uses **Ackermann steering** (car-like kinematics):
- **Front wheels**: Steerable via revolute joints (position control)
- **Rear wheels**: Drive wheels (velocity control)
- **Control**: ros2_control with forward_command_controller

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
              └── imu_link (fixed)
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
```

## Control Commands

```bash
# Direct steering control (position in radians)
ros2 topic pub /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.3]}" -1

# Direct throttle control (velocity in rad/s)
ros2 topic pub /throttle_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0, 5.0]}" -1

# Stop wheels
ros2 topic pub /throttle_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" -1

# Center steering
ros2 topic pub /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" -1
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/steering_controller/commands` | std_msgs/Float64MultiArray | Steering angle commands [left, right] |
| `/throttle_controller/commands` | std_msgs/Float64MultiArray | Wheel velocity commands [left, right] |
| `/joint_states` | sensor_msgs/JointState | Joint positions and velocities |
| `/odom` | nav_msgs/Odometry | Odometry from ackermann_odometry node |
| `/imu/data` | sensor_msgs/Imu | IMU sensor data (200Hz) |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/robot_description` | std_msgs/String | URDF robot description |
| `/clock` | rosgraph_msgs/Clock | Simulation clock |

## Controllers (ros2_control)

| Controller | Type | Joints |
|------------|------|--------|
| joint_state_broadcaster | JointStateBroadcaster | All joints |
| steering_controller | ForwardCommandController | front_left_steer_joint, front_right_steer_joint |
| throttle_controller | ForwardCommandController | rear_left_wheel_joint, rear_right_wheel_joint |

## Sensors

### IMU (Yahboom 10-axis compatible)
- **Topic**: `/imu/data`
- **Frame**: `imu_link`
- **Update rate**: 200Hz
- **Position**: Centered on top of chassis
- **Noise**: Gaussian (gyro: 0.0002 rad/s, accel: 0.017 m/s²)

## Dependencies

- ROS 2 Humble
- Gazebo Fortress (`gz-fortress`)
- ros_gz (`ros-humble-ros-gz`)
- gz_ros2_control (`ros-humble-gz-ros2-control`)
- ros2_control, ros2_controllers
- robot_state_publisher
- xacro
- joint_state_publisher_gui (for state launch only)

## Key Files

- `src/exia_ground_description/urdf/exia_ground.urdf.xacro` - Robot model (Xacro format)
- `src/exia_ground_description/config/ackermann_controllers.yaml` - Controller configuration
- `src/exia_ground_description/launch/exia_ground_sim.launch.py` - Gazebo Fortress simulation launch
- `src/exia_ground_description/launch/exia_ground_state.launch.py` - State publisher launch (RViz only)
- `src/exia_ground_description/rviz/exia_ground.rviz` - RViz config (Fixed Frame: odom)
- `src/exia_ground_description/scripts/ackermann_odometry.py` - Odometry calculation node

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

## Hardware Deployment Notes

When deploying to real hardware:
1. **IMU**: Replace Gazebo sensor with Yahboom IMU driver (publish to `/imu/data` with `frame_id: imu_link`)
2. **Motor Control**: Replace gz_ros2_control with real hardware interface for ros2_control
3. **Steering**: Implement servo control for front wheel steering via ros2_control
4. **Encoders**: Add wheel encoders for real odometry feedback
