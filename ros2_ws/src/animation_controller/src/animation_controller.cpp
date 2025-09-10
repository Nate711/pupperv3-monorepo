#include "animation_controller/animation_controller.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <map>
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
    : controller_interface::ControllerInterface(),
      rt_animation_select_ptr_(nullptr) {}

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
      RCLCPP_ERROR(get_node()->get_logger(), "%s size is %ld, expected %d",
                   name.c_str(), size, kActionSize);
      return false;
    }
  }
  return true;
}

bool AnimationController::load_all_animations() {
  animations_.clear();

  // Get launch directory path
  std::string launch_dir;
  try {
    std::string package_path =
        ament_index_cpp::get_package_share_directory("animation_controller");
    launch_dir = package_path + "/launch";
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to find animation_controller package: %s", e.what());
    return false;
  }

  // Auto-discover all CSV files in launch directory
  std::vector<std::string> csv_files;
  try {
    for (const auto &entry : std::filesystem::directory_iterator(launch_dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".csv") {
        std::string filename = entry.path().filename().string();
        // Remove .csv extension to get animation name
        std::string animation_name = filename.substr(0, filename.length() - 4);
        csv_files.push_back(filename);

        if (!load_animation_csv(animation_name, filename)) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Failed to load animation '%s' from file '%s'",
                       animation_name.c_str(), filename.c_str());
          return false;
        }
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error scanning directory '%s': %s",
                 launch_dir.c_str(), e.what());
    return false;
  }

  if (animations_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "No CSV animation files found in '%s'", launch_dir.c_str());
    return false;
  }

  // Set default animation
  if (!params_.default_animation.empty()) {
    if (animations_.find(params_.default_animation) == animations_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Default animation '%s' not found in discovered animations",
                   params_.default_animation.c_str());
      return false;
    }
    current_animation_name_ = params_.default_animation;
  } else if (!animations_.empty()) {
    // Use first animation as default if none specified
    current_animation_name_ = animations_.begin()->first;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Auto-discovered %zu animations from CSV files. Current: %s",
              animations_.size(), current_animation_name_.c_str());
  return true;
}

bool AnimationController::load_animation_csv(const std::string &name,
                                             const std::string &csv_path) {
  if (csv_path.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "CSV file path is empty for animation '%s'", name.c_str());
    return false;
  }

  // Always resolve the CSV file path to the animation_controller launch folder
  std::string resolved_path;
  try {
    std::string package_path =
        ament_index_cpp::get_package_share_directory("animation_controller");
    resolved_path = package_path + "/launch/" + csv_path;
    RCLCPP_INFO(get_node()->get_logger(), "Loading animation '%s' from: %s",
                name.c_str(), resolved_path.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to find animation_controller package: %s", e.what());
    return false;
  }

  std::ifstream file(resolved_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to open CSV file: %s",
                 resolved_path.c_str());
    return false;
  }

  std::string line;
  size_t line_number = 0;
  std::map<std::string, int>
      column_mapping; // Maps joint names to column indices
  bool header_parsed = false;

  while (std::getline(file, line)) {
    line_number++;

    // Skip empty lines and comments
    if (line.empty() || line[0] == '#')
      continue;

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> cells;

    // Parse all cells in the line
    while (std::getline(ss, cell, ',')) {
      // Trim whitespace including newlines
      cell.erase(0, cell.find_first_not_of(" \t\r\n"));
      cell.erase(cell.find_last_not_of(" \t\r\n") + 1);
      cells.push_back(cell);
    }
    // Get the last cell after the final comma (handles no trailing comma)
    if (!ss.eof()) {
      std::getline(ss, cell);
      // Trim whitespace including newlines from last cell
      cell.erase(0, cell.find_first_not_of(" \t\r\n"));
      cell.erase(cell.find_last_not_of(" \t\r\n") + 1);
      cells.push_back(cell);
    }

    // Parse header if not yet parsed
    if (!header_parsed) {
      // Build column mapping from header
      for (size_t i = 0; i < cells.size(); i++) {
        const std::string &column_name = cells[i];
        RCLCPP_INFO(get_node()->get_logger(), "Found column '%s' at index %zu",
                    column_name.c_str(), i);

        // Skip timestamp columns
        if (column_name == "timestamp_ns" || column_name == "timestamp_sec") {
          continue;
        }

        // Check if this column corresponds to a joint we care about
        for (size_t j = 0; j < params_.joint_names.size(); j++) {
          if (column_name == params_.joint_names[j]) {
            column_mapping[params_.joint_names[j]] = static_cast<int>(i);
            RCLCPP_INFO(get_node()->get_logger(),
                        "Mapped joint '%s' to column %zu",
                        params_.joint_names[j].c_str(), i);
            break;
          }
        }
      }

      // Verify all joints are found
      for (const auto &joint_name : params_.joint_names) {
        if (column_mapping.find(joint_name) == column_mapping.end()) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Joint '%s' not found in CSV header",
                       joint_name.c_str());
          return false;
        }
      }

      RCLCPP_INFO(get_node()->get_logger(),
                  "Successfully mapped %zu joints from CSV header",
                  column_mapping.size());
      header_parsed = true;
      continue;
    }

    // Parse data row
    std::array<double, kActionSize> keyframe;

    for (size_t i = 0; i < params_.joint_names.size(); i++) {
      const std::string &joint_name = params_.joint_names[i];
      int col_idx = column_mapping[joint_name];

      if (col_idx >= static_cast<int>(cells.size())) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Column index %d for joint '%s' out of range at line %zu",
                     col_idx, joint_name.c_str(), line_number);
        return false;
      }

      try {
        keyframe[i] = std::stod(cells[col_idx]);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to parse value '%s' for joint '%s' at line %zu: %s",
            cells[col_idx].c_str(), joint_name.c_str(), line_number, e.what());
        return false;
      }
    }

    animations_[name].push_back(keyframe);
  }

  if (animations_[name].empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "No keyframes loaded from CSV file for animation '%s'",
                 name.c_str());
    return false;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Loaded %zu keyframes for animation '%s' from %s",
              animations_[name].size(), name.c_str(), resolved_path.c_str());
  return true;
}

void AnimationController::switch_animation(const std::string &animation_name) {
  if (animations_.find(animation_name) == animations_.end()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Animation '%s' not found, ignoring switch request",
                animation_name.c_str());
    return;
  }

  if (current_animation_name_ == animation_name) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Restarting animation '%s'",
                 animation_name.c_str());
    // Don't return - fall through to restart the animation
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
                "Switching from animation '%s' to '%s'",
                current_animation_name_.c_str(), animation_name.c_str());
    current_animation_name_ = animation_name;
  }

  // Reset animation state for new animation
  current_animation_time_ = 0.0;
  current_frame_index_ = 0;

  // If not currently playing, start playing the new animation
  if (!animation_active_) {
    animation_active_ = true;
    init_time_ = get_node()->now();
    animation_start_time_.reset(); // Will be set after interpolation
  }
}

void AnimationController::interpolate_keyframes(
    double alpha, size_t frame_a, size_t frame_b,
    std::array<double, kActionSize> &result) {
  auto it = animations_.find(current_animation_name_);
  if (it == animations_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Animation '%s' not found in loaded animations",
                 current_animation_name_.c_str());
    return;
  }

  if (it->second.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Animation '%s' has no keyframes",
                 current_animation_name_.c_str());
    return;
  }

  // Clamp alpha to [0, 1]
  alpha = std::clamp(alpha, 0.0, 1.0);

  const auto &keyframes = it->second;
  // Clamp frame indices
  frame_a = std::min(frame_a, keyframes.size() - 1);
  frame_b = std::min(frame_b, keyframes.size() - 1);

  for (size_t i = 0; i < kActionSize; i++) {
    result[i] =
        keyframes[frame_a][i] * (1.0 - alpha) + keyframes[frame_b][i] * alpha;
  }
}

controller_interface::CallbackReturn AnimationController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    if (params_.frame_rate <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Frame rate must be > 0.0. Got %f",
                   params_.frame_rate);
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!load_all_animations()) {
      return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during init stage: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!check_param_vector_size()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AnimationController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(),
              "Animation controller configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AnimationController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::InterfaceConfiguration
AnimationController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn AnimationController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  // Populate the command interfaces map
  RCLCPP_INFO(get_node()->get_logger(), "Populating command interfaces map");
  command_interfaces_map_.clear();
  for (auto &command_interface : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Prefix %s. Adding command interface %s",
                command_interface.get_prefix_name().c_str(),
                command_interface.get_interface_name().c_str());
    command_interfaces_map_[command_interface.get_prefix_name()]
        .insert_or_assign(command_interface.get_interface_name(),
                          std::ref(command_interface));
  }

  // Populate the state interfaces map
  state_interfaces_map_.clear();
  for (auto &state_interface : state_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Prefix %s. Adding state interface %s",
                state_interface.get_prefix_name().c_str(),
                state_interface.get_interface_name().c_str());
    state_interfaces_map_[state_interface.get_prefix_name()].insert_or_assign(
        state_interface.get_interface_name(), std::ref(state_interface));
  }

  // Store the initial joint positions
  for (int i = 0; i < kActionSize; i++) {
    init_joint_pos_[i] = state_interfaces_map_.at(params_.joint_names[i])
                             .at("position")
                             .get()
                             .get_value();
  }

  // Initialize target positions to first frame if available
  auto it = animations_.find(current_animation_name_);
  if (it == animations_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot initialize: animation '%s' not found",
                 current_animation_name_.c_str());
  } else if (it->second.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot initialize: animation '%s' has no keyframes",
                 current_animation_name_.c_str());
  } else {
    target_positions_ = it->second[0];
  }

  // Reset animation state
  init_time_ = get_node()->now();
  animation_start_time_.reset(); // Will be set after interpolation completes
  current_animation_time_ = 0.0;
  current_frame_index_ = 0;
  animation_active_ = false; // Only start when explicitly requested via topic

  // Create animation selection subscriber
  animation_select_subscriber_ =
      get_node()->create_subscription<std_msgs::msg::String>(
          "~/animation_select", rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::String::SharedPtr msg) {
            rt_animation_select_ptr_.writeFromNonRT(msg);
          });

  // Initialize RT buffer
  rt_animation_select_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::String>>(
          nullptr);

  // Initialize the publisher
  animation_state_publisher_ = get_node()->create_publisher<AnimationStateMsg>(
      "~/animation_state", rclcpp::SystemDefaultsQoS());
  rt_animation_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<AnimationStateMsg>>(
          animation_state_publisher_);

  RCLCPP_INFO(get_node()->get_logger(),
              "Animation controller activate successful");
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
        .set_value(0.1); // High damping for safe deactivation
  }

  // Clear RT buffer
  rt_animation_select_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::String>>(
          nullptr);

  // Clear command and state interfaces maps
  command_interfaces_map_.clear();
  state_interfaces_map_.clear();

  // Release underlying command and state interfaces
  command_interfaces_.clear();
  state_interfaces_.clear();

  // Release command and state interfaces from superclass
  release_interfaces();

  RCLCPP_INFO(get_node()->get_logger(),
              "Animation controller deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
AnimationController::update(const rclcpp::Time &time,
                            const rclcpp::Duration &period) {

  // Check for animation switch requests
  auto animation_select = rt_animation_select_ptr_.readFromRT();
  if (animation_select && animation_select->get()) {
    switch_animation(animation_select->get()->data);
  }

  auto it = animations_.find(current_animation_name_);
  if (it == animations_.end()) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          5000, "Animation '%s' not found in loaded animations",
                          current_animation_name_.c_str());
    return controller_interface::return_type::OK;
  }

  if (it->second.empty()) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          5000, "Animation '%s' has no keyframes",
                          current_animation_name_.c_str());
    return controller_interface::return_type::OK;
  }
  const auto &keyframes = it->second;

  // During initialization, smoothly move to the first animation frame
  double time_since_init = (time - init_time_).seconds();
  if (time_since_init < params_.init_duration) {
    for (int i = 0; i < kActionSize; i++) {
      // Interpolate between the initial joint positions and the first animation
      // frame
      const auto &target_first_frame = keyframes[0];

      double interpolated_joint_pos =
          init_joint_pos_[i] * (1 - time_since_init / params_.init_duration) +
          target_first_frame[i] * (time_since_init / params_.init_duration);

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

  // Set animation start time after interpolation completes
  if (time_since_init >= params_.init_duration &&
      !animation_start_time_.has_value()) {
    animation_start_time_ = time;
  }

  // Emergency stop logic removed - handled by controller manager

  // Update animation time if playing
  if (animation_active_ && animation_start_time_.has_value()) {
    current_animation_time_ = (time - animation_start_time_.value()).seconds();

    // Calculate current frame based on frame rate
    double frame_time = 1.0 / params_.frame_rate;
    double exact_frame = current_animation_time_ / frame_time;

    // Check if animation has finished
    if (exact_frame >= keyframes.size()) {
      if (params_.loop_animation) {
        // Loop back to beginning
        animation_start_time_ = time;
        current_animation_time_ = 0.0;
        exact_frame = 0.0;
      } else {
        // Stop at last frame
        animation_active_ = false;
        exact_frame = keyframes.size() - 1;
      }
    }

    current_frame_index_ = static_cast<size_t>(exact_frame);

    // Calculate target positions with interpolation for smooth animation
    if (keyframes.size() > 1) {
      // Interpolate between current and next frame
      size_t next_frame =
          std::min(current_frame_index_ + 1, keyframes.size() - 1);
      double alpha = exact_frame - current_frame_index_;
      interpolate_keyframes(alpha, current_frame_index_, next_frame,
                            target_positions_);
    } else {
      // Use exact frame positions (single frame animation)
      target_positions_ = keyframes[current_frame_index_];
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
    rt_animation_state_publisher_->msg_.data.resize(kActionSize +
                                                    3); // positions + metadata
    for (int i = 0; i < kActionSize; i++) {
      rt_animation_state_publisher_->msg_.data[i] = target_positions_[i];
    }
    // Add metadata: current_frame, total_frames, animation_active
    rt_animation_state_publisher_->msg_.data[kActionSize] =
        static_cast<float>(current_frame_index_);
    rt_animation_state_publisher_->msg_.data[kActionSize + 1] =
        static_cast<float>(keyframes.size());
    rt_animation_state_publisher_->msg_.data[kActionSize + 2] =
        animation_active_ ? 1.0f : 0.0f;
    rt_animation_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

} // namespace animation_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(animation_controller::AnimationController,
                       controller_interface::ControllerInterface)