cmake_minimum_required(VERSION 3.8)
project(pupper_mujoco_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/lib)

# Determine architecture-specific library folder
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
  set(MUJOCO_LIB_DIR third_party/mujoco-2.3.1/lib_arm)
else()
  set(MUJOCO_LIB_DIR third_party/mujoco-2.3.1/lib_x86)
endif()

set(MUJOCO_INCLUDE_DIR third_party/mujoco-2.3.1/include)

# Add the library path
link_directories(${MUJOCO_LIB_DIR})

# Add the include directory
include_directories(${MUJOCO_INCLUDE_DIR})

add_executable(simulate src/simulate.cc src/uitools.c)
target_link_libraries(simulate mujoco glfw)

add_executable(basic src/basic.cc)
target_link_libraries(basic mujoco glfw)

add_executable(compile src/compile.cc)
target_link_libraries(compile mujoco glfw)

add_library(mujoco_hardware_interface SHARED lib/mujoco_hardware_interface.cpp
                                             lib/uitools.c)
target_link_libraries(mujoco_hardware_interface mujoco glfw)
ament_target_dependencies(
  mujoco_hardware_interface
  rclcpp
  pluginlib
  hardware_interface
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  builtin_interfaces
  rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface
                                         mujoco_hardware_interface.xml)

install(TARGETS mujoco_hardware_interface DESTINATION lib)
install(DIRECTORY lib/ DESTINATION lib)

ament_export_include_directories(lib)
ament_export_libraries(mujoco_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle
                          transmission_interface)

install(TARGETS basic DESTINATION lib/${PROJECT_NAME})
install(TARGETS simulate DESTINATION lib/${PROJECT_NAME})
install(TARGETS compile DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ros_testing REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
