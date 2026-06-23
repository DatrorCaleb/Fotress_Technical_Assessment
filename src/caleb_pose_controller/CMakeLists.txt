cmake_minimum_required(VERSION 3.8)
project(caleb_pose_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include custom Python setup script helper
include(framework_python_install_helper.cmake OPTIONAL)
if(NOT COMMAND ament_package_python_installation)
  macro(ament_package_python_installation)
    find_package(ament_cmake_python REQUIRED)
    ament_python_install_package(${PROJECT_NAME})
  endmacro()
endif()

# Generate the custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToPose.srv"
  DEPENDENCIES geometry_msgs
)

# Install Python scripts explicitly to the correct project library path
ament_package_python_installation()

install(PROGRAMS
  scripts/pose_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

colors_define()
record_install()
print_install_status()

ament_package()
