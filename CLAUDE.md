# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)
# Updated: December 2025 - Three-Motor HAL Architecture

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
│       ├── config/                 # Controller configurations
│       ├── rviz/                   # RViz configuration
│       ├── scripts/                # Python control scripts
│       │   ├── ackermann_hal_base.py        # HAL base interface
│       │   ├── ackermann_hal_simulation.py  # Gazebo HAL
│       │   ├── ackermann_hal_hardware.py    # Real hardware HAL
│       │   └── ackermann_drive_node.py      # Main drive controller
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
- **Drive**: Three-motor Ackermann (steering, throttle, brake)
- **Simulator**: Gazebo Fortress with gz_ros2_control
- **HAL**: Hardware Abstraction Layer for sim/hardware switching
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

### Robot Model & Configuration
- `src/exia_ground_description/urdf/exia_ground.urdf.xacro` - Robot model (Xacro format)
- `src/exia_ground_description/config/ackermann_controllers.yaml` - Three-motor controller configuration

### Launch Files
- `src/exia_ground_description/launch/exia_ground_sim.launch.py` - Gazebo Fortress simulation
- `src/exia_ground_description/launch/exia_ground_state.launch.py` - State publisher (RViz only)

### HAL System (Hardware Abstraction Layer)
- `src/exia_ground_description/scripts/ackermann_hal_base.py` - Base HAL interface and data types
- `src/exia_ground_description/scripts/ackermann_hal_simulation.py` - Gazebo Fortress HAL implementation
- `src/exia_ground_description/scripts/ackermann_hal_hardware.py` - Real hardware HAL (PWM/CAN/Serial)

### Control Nodes
- `src/exia_ground_description/scripts/ackermann_drive_node.py` - Unified drive controller (cmd_vel -> motors)
- `src/exia_ground_description/scripts/ackermann_odometry.py` - Fallback odometry node

### Visualization
- `src/exia_ground_description/rviz/exia_ground.rviz` - RViz config (Fixed Frame: odom)

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

### Switching from Simulation to Real Hardware

The HAL architecture makes hardware deployment straightforward:

1. **Change HAL Type**: In launch file or parameters:
   ```yaml
   hal_type: 'hardware'  # Instead of 'simulation'
   ```

2. **Configure Hardware HAL** (`ackermann_hal_hardware.py`):
   - Set driver type: `GPIO_PWM`, `PCA9685`, `SERIAL`, or `CAN`
   - Configure PWM channels and pulse widths for your servos/ESCs
   - Implement encoder feedback for closed-loop control

### Hardware Setup Checklist

| Component | Simulation | Real Hardware |
|-----------|------------|---------------|
| Steering | gz_ros2_control | PWM servo (50Hz, 1000-2000us) |
| Throttle | gz_ros2_control | ESC/Motor controller |
| Brake | gz_ros2_control | Servo or linear actuator |
| IMU | Gazebo sensor | Yahboom 10-axis driver |
| Encoders | Gazebo physics | Real wheel encoders |

### Example Hardware Configurations

**1. RC-Style (Raspberry Pi + PCA9685)**
```python
from ackermann_hal_hardware import HardwareHAL, HardwareDriver, PWMConfig

hal = HardwareHAL(
    node=self,
    driver_type=HardwareDriver.PCA9685,
    steering_pwm=PWMConfig(channel=0, min_pulse_us=1000, max_pulse_us=2000),
    throttle_pwm=PWMConfig(channel=1, min_pulse_us=1000, max_pulse_us=2000),
    brake_pwm=PWMConfig(channel=2, min_pulse_us=1000, max_pulse_us=2000),
)
```

**2. Industrial (CAN Bus Motor Controllers)**
- Modify `_init_can()` in `ackermann_hal_hardware.py`
- Implement CAN message encoding for your motor controller protocol

### Safety Considerations

1. **Emergency Stop**: Hardware E-stop should be wired independently
2. **Watchdog**: HAL includes 500ms timeout - stops motors if no commands
3. **Soft Limits**: Configure max steering angle and speed in parameters
4. **Brake Failsafe**: On shutdown or E-stop, brake is automatically engaged
