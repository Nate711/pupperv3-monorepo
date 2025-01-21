cmake_minimum_required(VERSION 3.8)
project(control_board_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)

# add compile directive for shared library
add_library(control_board_hardware_interface SHARED
  src/control_board_hardware_interface.cpp
  src/rt/rt_spi.cpp
  src/rt/rt_bno055.cpp
  src/imu_manager.cpp
)

# add include directory
target_include_directories(control_board_hardware_interface
  PRIVATE
    include
)

ament_target_dependencies(
  control_board_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
)

pluginlib_export_plugin_description_file(hardware_interface control_board_hardware_interface.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY test/
  DESTINATION test
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  transmission_interface
)
ament_package()