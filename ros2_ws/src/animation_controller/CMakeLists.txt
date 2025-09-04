cmake_minimum_required(VERSION 3.8)
project(animation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(animation_controller SHARED 
  src/animation_controller.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(
  animation_controller_parameters # cmake target name for the parameter library
  src/animation_controller_parameters.yaml # path to input yaml file
)

target_link_libraries(animation_controller 
  animation_controller_parameters
)

target_include_directories(animation_controller PRIVATE include)

ament_target_dependencies(
  animation_controller
  ament_index_cpp
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
)

pluginlib_export_plugin_description_file(controller_interface animation_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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

ament_package()