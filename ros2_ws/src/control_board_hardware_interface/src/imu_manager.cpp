#include "control_board_hardware_interface/imu_manager.hpp"

// set rotation matrix based on roll, pitch, yaw
void IMUManager::set_imu_offset(double roll, double pitch, double yaw) {
  offset_quaternion.setRPY(roll, pitch, yaw);
  offset_rotation_matrix = tf2::Matrix3x3(offset_quaternion);
}

bool IMUManager::contains_nan(const Eigen::Quaternionf &q) {
  return std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w());
}

std::optional<std::pair<BNO055::Output, uint32_t>> IMUManager::sample_imu_multiple_attempts(
    int max_samples, int delay_ms) {
  /*
   * Sample the IMU up to max_samples times until a good sample is received.
   * A good sample is defined as a quaternion that is not near zero and not NaN.
   */

  // Record start time
  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < max_samples; i++) {
    BNO055::Output output = imu_->sample();
    if (contains_nan(output.quat) || output.quat.isApprox(Eigen::Quaternionf(0, 0, 0, 0), 1e-6)) {
      RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "%sBad IMU sample [%f, %f, %f, %f]. Retrying...%s", YELLOW_ANSI, output.quat.x(),
                  output.quat.y(), output.quat.z(), output.quat.w(), RESET_ANSI);
    } else {
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      return std::make_pair(output, duration.count());
    }
    if (delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
  }
  return std::nullopt;
}

void IMUManager::poll_imu() {
  auto maybe_output = sample_imu_multiple_attempts(/*max_samples=*/5, /*delay_ms=*/1);
  if (maybe_output) {
    // lock and copy imu data
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data = maybe_output->first;
    imu_sampling_time_us = maybe_output->second;
  }
}

BNO055::Output IMUManager::get_imu_data() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_data;
}

void IMUManager::imu_read_thread_fn() {
  while (!stop_imu_read_thread_) {
    poll_imu();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void IMUManager::begin(uint32_t micros_between_reports) {
  imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER, micros_between_reports);

  stop_imu_read_thread_ = false;
  imu_read_thread_ = std::make_unique<std::thread>(&IMUManager::imu_read_thread_fn, this);
}

void IMUManager::stop() {
  stop_imu_read_thread_ = true;
  if (imu_read_thread_ && imu_read_thread_->joinable()) {
    imu_read_thread_->join();
  }
}
