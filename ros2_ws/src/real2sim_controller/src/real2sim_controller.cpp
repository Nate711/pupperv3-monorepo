#include "real2sim_controller/real2sim_controller.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace real2sim_controller {

// Returns 1 for values >- 0 and -1 otherwise
template <typename T>
int binary_sign(T val) {
  return static_cast<int>(val >= T(0)) * 2 - 1;
}

Real2SimController::Real2SimController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn Real2SimController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string sweep_name = params_.do_sweep ? "freq_sweep" : "constant_freq";
  std::string sinusoid_type = params_.square_wave ? "_square" : "";
  std::string freq_type = params_.do_sweep
                              ? fmt::format("freq_min_{}Hz_max_{}Hz_", params_.sweep_min_frequency,
                                            params_.sweep_max_frequency)
                              : fmt::format("freq_{}Hz_", params_.test_frequency);
  std::string filename = fmt::format(
      "{}{}_motor_{}_{}amp_{}rad_kp_{}_kd_{}.csv", sweep_name, sinusoid_type,
      params_.test_motor_index, freq_type, params_.test_amplitude,
      params_.kps.at(params_.test_motor_index), params_.kds.at(params_.test_motor_index));

  file_.open(filename);
  if (!file_.is_open()) {
    std::cerr << "Failed to open file!" << std::endl;
    return controller_interface::CallbackReturn::ERROR;
  }
  file_ << "time_since_fade_in (s)"
        << ","
        << "phase_"
        << ","
        << "amp (rad)"
        << ","
        << "freq (Hz)"
        << ","
        << "motor_idx"
        << ","
        << "policy_output"
        << ","
        << "command (rad)"
        << ","
        << "motor_position (rad)"
        << ","
        << "motor_velocity (rad/s)"
        << "\n";

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Real2SimController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Real2SimController::command_interface_configuration()
    const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::InterfaceConfiguration Real2SimController::state_interface_configuration()
    const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn Real2SimController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Populate the command interfaces map
  for (auto &command_interface : command_interfaces_) {
    command_interfaces_map_[command_interface.get_prefix_name()].emplace(
        command_interface.get_interface_name(), std::ref(command_interface));
  }

  // Populate the state interfaces map
  for (auto &state_interface : state_interfaces_) {
    state_interfaces_map_[state_interface.get_prefix_name()].emplace(
        state_interface.get_interface_name(), std::ref(state_interface));
  }

  // Store the initial joint positions
  for (int i = 0; i < ACTION_SIZE; i++) {
    init_joint_pos_[i] =
        state_interfaces_map_.at(params_.joint_names[i]).at("position").get().get_value();
  }

  init_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Real2SimController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto &command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Real2SimController::update(const rclcpp::Time &time,
                                                             const rclcpp::Duration &period) {
  // When started, return to the default joint positions
  double time_since_init = (time - init_time_).seconds();
  if (time_since_init < params_.init_duration) {
    for (int i = 0; i < ACTION_SIZE; i++) {
      // Interpolate between the initial joint positions and the default joint
      // positions
      double interpolated_joint_pos =
          init_joint_pos_[i] * (1 - time_since_init / params_.init_duration) +
          params_.default_joint_pos[i] * (time_since_init / params_.init_duration);
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

  // After the init_duration has passed, fade in the policy actions
  double time_since_fade_in = (time - init_time_).seconds() - params_.init_duration;
  float fade_in_multiplier = std::min(time_since_fade_in / params_.fade_in_duration, 1.0);

  // If an emergency stop has been triggered, set all commands to 0 and return
  if (estop_active_) {
    for (auto &command_interface : command_interfaces_) {
      command_interface.set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  params_ = param_listener_->get_params();
  int motor_idx = params_.test_motor_index;

  // Process the actions
  std::array<float, ACTION_SIZE> policy_output = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // --------------------------- CUSTOM CODE     ----------------------------------------------- //

  double actual_period = time_since_fade_in - last_time_since_fade_in_;
  last_time_since_fade_in_ = time_since_fade_in;

  double motor_position =
      state_interfaces_map_.at(params_.joint_names.at(motor_idx)).at("position").get().get_value() -
      params_.default_joint_pos.at(motor_idx);
  double motor_velocity =
      state_interfaces_map_.at(params_.joint_names.at(motor_idx)).at("velocity").get().get_value();

  if (params_.do_sweep) {
    // Frequency sweep mode

    // Override fade in multiplier
    fade_in_multiplier = 1.0;

    // Calculate frequency
    int freq_idx = (int)(time_since_fade_in / params_.sweep_secs_per_frequency);
    double freq = params_.sweep_min_frequency + freq_idx * params_.sweep_frequency_step;

    // Stop the sweep once we go past the max frequency
    if (freq > params_.sweep_max_frequency + 1e-6) {
      freq = 0.0;
      file_.close();
    }

    // phase_ += freq * 2 * M_PI * actual_period;

    // If all frequencies are integer than this will give smooth motion!
    phase_ = time_since_fade_in * 2 * M_PI * freq;
    double output = std::sin(phase_);
    if (params_.square_wave) {
      output = binary_sign(output);
    }
    double position_command = output * params_.test_amplitude;
    policy_output.at(motor_idx) = position_command;

    // Save data to file
    if (file_.is_open()) {
      file_ << time_since_fade_in << "," << phase_ << "," << params_.test_amplitude << "," << freq
            << "," << motor_idx << "," << position_command << ","
            << position_command * params_.action_scales.at(motor_idx) << "," << motor_position
            << "," << motor_velocity << "\n";
    }

    std::cout << time_since_fade_in << " " << period.seconds() << " " << actual_period << " "
              << freq << " " << phase_ << " " << position_command << "\n";
  } else {
    // Override fade in multiplier
    fade_in_multiplier = 1.0;

    // Constant frequency mode
    phase_ += params_.test_frequency * 2 * M_PI * actual_period;

    double output = std::sin(phase_);
    if (params_.square_wave) {
      output = binary_sign(output);
    }
    double position_command = output * params_.test_amplitude;
    policy_output.at(motor_idx) = position_command;

    // Save data to file
    file_ << time_since_fade_in << ", " << phase_ << ", " << params_.test_amplitude << ", "
          << params_.test_frequency << ", " << motor_idx << ", " << policy_output.at(motor_idx)
          << ", " << policy_output.at(motor_idx) * params_.action_scales.at(motor_idx) << ", "
          << motor_position << ", " << motor_velocity << "\n";

    std::cout << time_since_fade_in << " " << period.seconds() << " " << actual_period << " "
              << params_.test_frequency << " " << phase_ << " " << position_command << "\n";
  }

  // ------------------------------------------------------------------------------------------- //

  // std::cout << time_since_fade_in << " " << period.seconds() << " " << actual_period << "\n";

  for (int i = 0; i < ACTION_SIZE; i++) {
    // Clip the action
    float action_clipped = std::max(std::min(policy_output[i], (float)params_.action_limit),
                                    (float)-params_.action_limit);
    // Scale and de-normalize to get the action vector
    if (params_.action_types[i] == "position") {
      action_.at(i) = fade_in_multiplier * action_clipped * params_.action_scales[i] +
                      params_.default_joint_pos[i];
    } else {
      action_.at(i) = fade_in_multiplier * action_clipped * params_.action_scales[i];
    }

    // Send the action to the hardware interface
    command_interfaces_map_.at(params_.joint_names[i])
        .at(params_.action_types[i])
        .get()
        .set_value((double)action_.at(i));
    command_interfaces_map_.at(params_.joint_names[i]).at("kp").get().set_value(params_.kps[i]);
    command_interfaces_map_.at(params_.joint_names[i]).at("kd").get().set_value(params_.kds[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace real2sim_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(real2sim_controller::Real2SimController,
                       controller_interface::ControllerInterface)