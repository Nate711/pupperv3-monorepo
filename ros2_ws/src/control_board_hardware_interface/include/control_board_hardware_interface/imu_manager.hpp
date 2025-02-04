#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <atomic>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <utility>

#include "rt/rt_bno055.h"

#define IMU_I2C_DEVICE_NUMBER 1

class IMUManager {
 public:
  // whether IMU is enabled on robot
  bool enabled;

  // Latest copy of imu data (thread safe)
  BNO055::Output imu_data;
  uint32_t imu_sampling_time_us;

  // IMU offset
  tf2::Matrix3x3 offset_rotation_matrix;
  tf2::Quaternion offset_quaternion;

  // Set rotation matrix based on roll, pitch, yaw
  void set_imu_offset(double roll, double pitch, double yaw);

  // Begin threads and start reading IMU data
  void begin(uint32_t micros_between_reports);

  // Stop reading threads
  void stop();

  // Thread-safe get imu data
  BNO055::Output get_imu_data();

 private:
  bool contains_nan(const Eigen::Quaternionf &q);

  std::optional<std::pair<BNO055::Output, uint32_t>> sample_imu_multiple_attempts(int max_samples,
                                                                                  int delay_ms);

  void poll_imu();


  void imu_read_thread_fn();

  const char *RED_ANSI = "\033[1;31m";
  const char *YELLOW_ANSI = "\033[1;33m";
  const char *RESET_ANSI = "\033[0m";

  // mutex for imu
  std::mutex imu_mutex_;
  // ptr to thread for imu reading
  std::unique_ptr<std::thread> imu_read_thread_ = nullptr;
  // signal to stop read thread
  std::atomic<bool> stop_imu_read_thread_;

  // Imu reader
  std::unique_ptr<BNO055> imu_ = nullptr;
};