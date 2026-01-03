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
# ROS2 node executables that use the exia_control package

install(
  PROGRAMS
    scripts/active/ackermann_drive_node.py
    scripts/active/ackermann_odometry.py
    scripts/active/path_follower_node.py
    scripts/active/dynamic_navigator_node.py
    scripts/active/xbox_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
