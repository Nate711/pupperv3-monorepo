cmake_minimum_required(VERSION 3.5)
project(joy_utils)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Add the executable
add_executable(estop_controller src/estop_controller.cpp)

# Link against ROS 2 libraries
ament_target_dependencies(estop_controller
  rclcpp
  sensor_msgs
  std_msgs
  controller_manager_msgs
)

# Install the executable
install(TARGETS estop_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
