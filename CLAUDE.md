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
│       ├── include/                # C++ headers (empty)
│       └── src/                    # C++ source (empty)
├── build/                          # Build artifacts (generated)
├── install/                        # Install space (generated)
└── log/                            # Build logs (generated)
```

## Packages

### exia_ground_description
Robot description package containing URDF model and launch files for visualization and simulation.

- **URDF**: Simple box-shaped ground robot (0.6m x 0.4m x 0.2m) fixed to world frame
- **Maintainer**: zech (zechariahwang@gmail.com)

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
# Launch robot state publisher with joint state GUI (for RViz visualization)
ros2 launch exia_ground_description exia_ground_state.launch.py

# Launch Gazebo simulation (headless gzserver - no GUI)
ros2 launch exia_ground_description exia_ground_sim.launch.py

# Visualize the simulation in RViz2 (required since Gazebo runs headless)
rviz2 -d ~/exia_ws/src/exia_ground_description/rviz/exia_ground.rviz
```

## Dependencies

- ROS 2 (ament_cmake build system)
- joint_state_publisher_gui
- robot_state_publisher
- gazebo_ros

## Key Files

- `src/exia_ground_description/urdf/exia_ground.urdf` - Robot model definition
- `src/exia_ground_description/launch/exia_ground_state.launch.py` - State publisher launch
- `src/exia_ground_description/launch/exia_ground_sim.launch.py` - Gazebo simulation launch
- `src/exia_ground_description/rviz/exia_ground.rviz` - RViz visualization config

## Debugging Commands

```bash
# List all active nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic (e.g., joint states)
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames
```

## Development Notes

- The robot is currently a simple fixed box attached to the world frame
- No movable joints defined yet (fixed joint only)
- Gazebo runs headless (gzserver only, no gzclient GUI) - use RViz2 to visualize the simulation
