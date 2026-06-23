cmake_minimum_required(VERSION 3.8)
project(pose_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Include our custom Python setup script helper
include(ament_package_python_installation.cmake)

# 3. Compile the custom service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToPose.srv"
  DEPENDENCIES geometry_msgs
)

# 4. Install the Python package modules and your executable scripts
ament_package_python_installation()

install(PROGRAMS
  scripts/pose_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Finalize the package setup
ament_package()
