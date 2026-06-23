cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# This block compiles your GoToPose.srv file so ROS 2 can use it!
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToPose.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

# This installs your Python script correctly into the workspace
install(PROGRAMS
  scripts/pose_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
