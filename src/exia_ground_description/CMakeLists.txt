cmake_minimum_required(VERSION 3.8)
project(exia_ground_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ==================== Install Directories ====================

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# ==================== Install Python Package ====================
# The exia_control package contains reusable modules:
#   - hal/       Hardware Abstraction Layer
#   - control/   Control algorithms
#   - planning/  Path planning algorithms

install(DIRECTORY src/exia_control
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# ==================== Install Entry-Point Scripts ====================
# Active scripts - ROS2 node executables that use the exia_control package

install(
  PROGRAMS
    scripts/active/ackermann_drive_node.py
    scripts/active/path_follower_node.py
    scripts/active/ackermann_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

# Archive scripts - Legacy/reference scripts (not actively used)
# These are kept for reference but not installed by default
# To install archive scripts, uncomment the block below:
# install(
#   PROGRAMS
#     scripts/archive/ackermann_hal_base.py
#     scripts/archive/ackermann_hal_simulation.py
#     scripts/archive/ackermann_hal_hardware.py
#     scripts/archive/ackermann_controller.py
#     scripts/archive/atv_controller.py
#     scripts/archive/atv_hardware_interface.py
#     scripts/archive/steering_controller.py
#     scripts/archive/move_forward.py
#     scripts/archive/waypoint_follower.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
