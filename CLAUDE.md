# Exia Ground Robot Workspace
# Author: Zechariah Wang (Dec 25, 2025)

ROS 2 workspace for the Exia Ground robot platform.

## Project Structure

```
exia_ws/
├── src/
│   └── exia_ground_description/    # Robot description package
│       ├── urdf/                   # URDF robot models
│       ├── launch/                 # Launch files
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

- **URDF**: 4-wheel skid-steer ground robot (0.6m x 0.4m x 0.2m chassis)
- **Drive**: Differential drive with 4 wheels (front and rear pairs)
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
| Max wheel torque | 100 Nm |

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

# Launch Gazebo simulation (with GUI)
ros2 launch exia_ground_description exia_ground_sim.launch.py

# Visualize in RViz2 (use when Gazebo GUI has issues)
rviz2 -d ~/exia_ws/src/exia_ground_description/rviz/exia_ground.rviz
```

## Control Commands

```bash
# Move robot forward using the provided script
python3 ~/exia_ws/src/exia_ground_description/scripts/move_forward.py

# Manual velocity control via command line
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -1

# Rotate robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -1

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" -1
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (subscribed by diff_drive) |
| `/odom` | nav_msgs/Odometry | Odometry from diff_drive plugin |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/robot_description` | std_msgs/String | URDF robot description |

## Dependencies

- ROS 2 Humble
- gazebo_ros (Gazebo Classic 11)
- robot_state_publisher
- joint_state_publisher (for state launch only)
- joint_state_publisher_gui (for state launch only)

## Key Files

- `src/exia_ground_description/urdf/exia_ground.urdf` - Robot model with diff_drive plugin
- `src/exia_ground_description/launch/exia_ground_sim.launch.py` - Gazebo simulation launch
- `src/exia_ground_description/launch/exia_ground_state.launch.py` - State publisher launch (RViz only)
- `src/exia_ground_description/rviz/exia_ground.rviz` - RViz config (Fixed Frame: odom)
- `src/exia_ground_description/scripts/move_forward.py` - Example velocity publisher

## Debugging Commands

```bash
# List all active nodes
ros2 node list

# List all topics
ros2 topic list

# Check odometry
ros2 topic echo /odom --once

# Check TF tree
ros2 run tf2_tools view_frames

# View TF between frames
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Development Notes

- Robot uses 4-wheel skid-steer drive (all wheels driven)
- Gazebo launches with GUI by default (gzserver + gzclient)
- If Gazebo GUI crashes (common on Wayland), use RViz2 for visualization - gzserver still runs
- RViz Fixed Frame must be set to `odom` to see robot movement
- The diff_drive plugin handles wheel TF publishing (`publish_wheel_tf: true`)
- `use_sim_time: True` is set for proper Gazebo time synchronization

## Known Issues

- Gazebo GUI may crash on Wayland systems (shader/rendering errors) - simulation still works headless
- If robot doesn't move, verify `/cmd_vel` topic exists: `ros2 topic list | grep cmd_vel`
