#include "animation_controller/animation_controller.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace animation_controller {

AnimationController::AnimationController()
    : controller_interface::ControllerInterface() {}

// Check parameter vectors have the correct size
bool AnimationController::check_param_vector_size() {
  const std::vector<std::pair<std::string, size_t>> param_sizes = {
      {"kps", params_.kps.size()},
      {"kds", params_.kds.size()},
      {"init_kps", params_.init_kps.size()},
      {"init_kds", params_.init_kds.size()},
      {"joint_names", params_.joint_names.size()}};

  for (const auto &[name, size] : param_sizes) {
    if (size != kActionSize) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s size is %ld, expected %d", name.c_str(), size,
                   kActionSize);
      return false;
    }
  }
  return true;
}

bool AnimationController::load_animation_csv() {
  animation_keyframes_.clear();

  if (params_.csv_file_path.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "CSV file path is empty");
    return false;
  }

  std::ifstream file(params_.csv_file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to open CSV file: %s", params_.csv_file_path.c_str());
    return false;
  }

  std::string line;
  size_t line_number = 0;
  
  while (std::getline(file, line)) {
    line_number++;
    
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;
    
    std::stringstream ss(line);
    std::string cell;
    std::array<double, kActionSize> keyframe;
    size_t col_index = 0;
    
    while (std::getline(ss, cell, ',') && col_index < kActionSize) {
      try {
        keyframe[col_index] = std::stod(cell);
        col_index++;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), 
                     "Failed to parse CSV value '%s' at line %zu, column %zu: %s", 
                     cell.c_str(), line_number, col_index + 1, e.what());
        return false;
      }
    }
    
    if (col_index != kActionSize) {
      RCLCPP_ERROR(get_node()->get_logger(), 
                   "Expected %d values per row, got %zu at line %zu", 
                   kActionSize, col_index, line_number);
      return false;
    }
    
    animation_keyframes_.push_back(keyframe);
  }

  if (animation_keyframes_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No keyframes loaded from CSV file");
    return false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Loaded %zu keyframes from %s", 
              animation_keyframes_.size(), params_.csv_file_path.c_str());
  return true;
}

void AnimationController::interpolate_keyframes(double alpha, size_t frame_a, size_t frame_b, 
                                               std::array<double, kActionSize>& result) {
  // Clamp alpha to [0, 1]
  alpha = std::clamp(alpha, 0.0, 1.0);
  
  // Clamp frame indices
  frame_a = std::min(frame_a, animation_keyframes_.size() - 1);
  frame_b = std::min(frame_b, animation_keyframes_.size() - 1);
  
  for (size_t i = 0; i < kActionSize; i++) {
    result[i] = animation_keyframes_[frame_a][i] * (1.0 - alpha) + 
                animation_keyframes_[frame_b][i] * alpha;
  }
}

controller_interface::CallbackReturn AnimationController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    if (params_.frame_rate <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Frame rate must be > 0.0. Got %f", params_.frame_rate);
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!load_animation_csv()) {
      return controller_interface::CallbackReturn::ERROR;
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!check_param_vector_size()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AnimationController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "Animation controller configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AnimationController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::InterfaceConfiguration AnimationController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn AnimationController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  
  // Populate the command interfaces map
  RCLCPP_INFO(get_node()->get_logger(), "Populating command interfaces map");
  command_interfaces_map_.clear();
  for (auto &command_interface : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "Prefix %s. Adding command interface %s",
                command_interface.get_prefix_name().c_str(),
                command_interface.get_interface_name().c_str());
    command_interfaces_map_[command_interface.get_prefix_name()].insert_or_assign(
        command_interface.get_interface_name(), std::ref(command_interface));
  }

  // Populate the state interfaces map
  state_interfaces_map_.clear();
  for (auto &state_interface : state_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "Prefix %s. Adding state interface %s",
                state_interface.get_prefix_name().c_str(),
                state_interface.get_interface_name().c_str());
    state_interfaces_map_[state_interface.get_prefix_name()].insert_or_assign(
        state_interface.get_interface_name(), std::ref(state_interface));
  }

  // Store the initial joint positions
  for (int i = 0; i < kActionSize; i++) {
    init_joint_pos_[i] =
        state_interfaces_map_.at(params_.joint_names[i]).at("position").get().get_value();
  }

  // Initialize target positions to first frame if available
  if (!animation_keyframes_.empty()) {
    target_positions_ = animation_keyframes_[0];
  }

  // Reset animation state
  init_time_ = get_node()->now();
  animation_start_time_ = get_node()->now();
  current_animation_time_ = 0.0;
  current_frame_index_ = 0;
  estop_active_ = false;
  animation_active_ = params_.auto_start;

  // Create services for animation control
  play_animation_service_ = get_node()->create_service<std_srvs::srv::Empty>(
      "~/play_animation",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
              std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
        animation_active_ = true;
        animation_start_time_ = get_node()->now();
        current_animation_time_ = 0.0;
        current_frame_index_ = 0;
        RCLCPP_INFO(get_node()->get_logger(), "Animation playback started");
      });

  stop_animation_service_ = get_node()->create_service<std_srvs::srv::Empty>(
      "~/stop_animation",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
              std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
        animation_active_ = false;
        RCLCPP_INFO(get_node()->get_logger(), "Animation playback stopped");
      });

  reset_animation_service_ = get_node()->create_service<std_srvs::srv::Empty>(
      "~/reset_animation",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
              std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
        animation_active_ = false;
        current_animation_time_ = 0.0;
        current_frame_index_ = 0;
        if (!animation_keyframes_.empty()) {
          target_positions_ = animation_keyframes_[0];
        }
        RCLCPP_INFO(get_node()->get_logger(), "Animation reset to beginning");
      });

  // Initialize the publisher
  animation_state_publisher_ =
      get_node()->create_publisher<AnimationStateMsg>("~/animation_state", rclcpp::SystemDefaultsQoS());
  rt_animation_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<AnimationStateMsg>>(animation_state_publisher_);

  RCLCPP_INFO(get_node()->get_logger(), "Animation controller activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AnimationController::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::FAILURE;
}

controller_interface::CallbackReturn AnimationController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  for (auto &command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  for (int i = 0; i < kActionSize; i++) {
    command_interfaces_map_.at(params_.joint_names[i])
        .at("kd")
        .get()
        .set_value(params_.estop_kd);
  }

  // Clear command and state interfaces maps
  command_interfaces_map_.clear();
  state_interfaces_map_.clear();

  // Release underlying command and state interfaces
  command_interfaces_.clear();
  state_interfaces_.clear();

  // Release command and state interfaces from superclass
  release_interfaces();

  RCLCPP_INFO(get_node()->get_logger(), "Animation controller deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AnimationController::update(const rclcpp::Time &time,
                                                             const rclcpp::Duration &period) {
  if (animation_keyframes_.empty()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, 
                         "No animation keyframes loaded");
    return controller_interface::return_type::OK;
  }

  // During initialization, smoothly move to the first animation frame
  double time_since_init = (time - init_time_).seconds();
  if (time_since_init < params_.init_duration) {
    for (int i = 0; i < kActionSize; i++) {
      // Interpolate between the initial joint positions and the first animation frame
      double interpolated_joint_pos =
          init_joint_pos_[i] * (1 - time_since_init / params_.init_duration) +
          animation_keyframes_[0][i] * (time_since_init / params_.init_duration);
      
      command_interfaces_map_.at(params_.joint_names[i])
          .at("position")
          .get()
          .set_value(interpolated_joint_pos);
      command_interfaces_map_.at(params_.joint_names[i])
          .at("kp")
          .get()
          .set_value(params_.init_kps[i]);
      command_interfaces_map_.at(params_.joint_names[i])
          .at("kd")
          .get()
          .set_value(params_.init_kds[i]);
    }
    return controller_interface::return_type::OK;
  }

  // If an emergency stop is active, set all commands to 0 and high damping
  if (estop_active_) {
    for (auto &command_interface : command_interfaces_) {
      command_interface.set_value(0.0);
    }
    for (int i = 0; i < kActionSize; i++) {
      command_interfaces_map_.at(params_.joint_names[i])
          .at("kd")
          .get()
          .set_value(params_.estop_kd);
    }
    return controller_interface::return_type::OK;
  }

  // Update animation time if playing
  if (animation_active_) {
    current_animation_time_ = (time - animation_start_time_).seconds();
    
    // Calculate current frame based on frame rate
    double frame_time = 1.0 / params_.frame_rate;
    double exact_frame = current_animation_time_ / frame_time;
    
    // Check if animation has finished
    if (exact_frame >= animation_keyframes_.size()) {
      if (params_.loop_animation) {
        // Loop back to beginning
        animation_start_time_ = time;
        current_animation_time_ = 0.0;
        exact_frame = 0.0;
      } else {
        // Stop at last frame
        animation_active_ = false;
        exact_frame = animation_keyframes_.size() - 1;
      }
    }
    
    current_frame_index_ = static_cast<size_t>(exact_frame);
    
    // Calculate target positions
    if (params_.interpolation_enabled && animation_keyframes_.size() > 1) {
      // Interpolate between current and next frame
      size_t next_frame = std::min(current_frame_index_ + 1, animation_keyframes_.size() - 1);
      double alpha = exact_frame - current_frame_index_;
      interpolate_keyframes(alpha, current_frame_index_, next_frame, target_positions_);
    } else {
      // Use exact frame positions (no interpolation)
      target_positions_ = animation_keyframes_[current_frame_index_];
    }
  }

  // Send commands to hardware interface
  for (int i = 0; i < kActionSize; i++) {
    command_interfaces_map_.at(params_.joint_names[i])
        .at("position")
        .get()
        .set_value(target_positions_[i]);
    command_interfaces_map_.at(params_.joint_names[i])
        .at("kp")
        .get()
        .set_value(params_.kps[i]);
    command_interfaces_map_.at(params_.joint_names[i])
        .at("kd")
        .get()
        .set_value(params_.kds[i]);
  }

  // Publish animation state
  if (rt_animation_state_publisher_->trylock()) {
    rt_animation_state_publisher_->msg_.data.resize(kActionSize + 3);  // positions + metadata
    for (int i = 0; i < kActionSize; i++) {
      rt_animation_state_publisher_->msg_.data[i] = target_positions_[i];
    }
    // Add metadata: current_frame, total_frames, animation_active
    rt_animation_state_publisher_->msg_.data[kActionSize] = static_cast<float>(current_frame_index_);
    rt_animation_state_publisher_->msg_.data[kActionSize + 1] = static_cast<float>(animation_keyframes_.size());
    rt_animation_state_publisher_->msg_.data[kActionSize + 2] = animation_active_ ? 1.0f : 0.0f;
    rt_animation_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace animation_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(animation_controller::AnimationController,
                       controller_interface::ControllerInterface)