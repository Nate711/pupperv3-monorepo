#include "neural_controller/neural_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace neural_controller {
NeuralController::NeuralController()
    : controller_interface::ControllerInterface(),
      rt_cmd_vel_ptr_(nullptr),
      rt_cmd_pose_ptr_(nullptr) {}

// Check parameter vectors have the correct size
bool NeuralController::check_param_vector_size() {
  const std::vector<std::pair<std::string, size_t>> param_sizes = {
      {"action_scales", params_.action_scales.size()},
      {"action_types", params_.action_types.size()},
      {"kps", params_.kps.size()},
      {"kds", params_.kds.size()},
      {"init_kps", params_.init_kps.size()},
      {"init_kds", params_.init_kds.size()},
      {"default_joint_pos", params_.default_joint_pos.size()},
      {"joint_lower_limits", params_.joint_lower_limits.size()},
      {"joint_upper_limits", params_.joint_upper_limits.size()},
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

controller_interface::CallbackReturn NeuralController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    if (params_.gain_multiplier < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Gain_multiplier must be >= 0.0. Stopping");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (params_.gain_multiplier != 1.0) {
      RCLCPP_WARN(get_node()->get_logger(), "Gain_multiplier is set to %f",
                  params_.gain_multiplier);
    }

    std::ifstream json_stream(params_.model_path, std::ifstream::binary);
    model_ = RTNeural::json_parser::parseJson<float>(json_stream, true);

    // Read params json file using nholsojson to extract metadata
    nlohmann::json j;
    std::ifstream json_file(params_.model_path);
    json_file >> j;

    auto set_param_from_json_vector = [&](const std::string &key, auto &param) {
      if (j.find(key) != j.end()) {
        RCLCPP_INFO(get_node()->get_logger(), "From JSON, setting %s vector element-by-element",
                    key.c_str());
        if (j[key].size() != kActionSize) {
          std::string error_msg = "Invalid size for " + key + " (" + std::to_string(j[key].size()) +
                                  ") != " + std::to_string(kActionSize);
          RCLCPP_ERROR(get_node()->get_logger(), "%s", error_msg.c_str());
          throw std::runtime_error(error_msg);
        }
        param.resize(j[key].size(), 0.0);
        for (int i = 0; i < param.size(); i++) {
          param.at(i) = j[key].at(i);
        }
      }
    };

    auto set_param_from_json_scalar = [&](const std::string &key, auto &param, int size) {
      if (j.find(key) != j.end()) {
        RCLCPP_INFO(get_node()->get_logger(), "From JSON, setting %s[:]=%f", key.c_str(),
                    static_cast<double>(j[key]));
        param.resize(size, 0.0);
        for (auto &p : param) {
          p = j[key];
        }
      }
    };

    auto set_param_from_json_mixed = [&](const std::string &key, auto &param, int size) {
      if (j.find(key) != j.end()) {
        if (j[key].is_array()) {
          set_param_from_json_vector(key, param);
        } else {
          set_param_from_json_scalar(key, param, size);
        }
      }
    };

    set_param_from_json_scalar("kp", params_.kps, kActionSize);
    set_param_from_json_scalar("kd", params_.kds, kActionSize);
    set_param_from_json_mixed("action_scale", params_.action_scales, kActionSize);
    set_param_from_json_vector("default_joint_pos", params_.default_joint_pos);
    set_param_from_json_vector("joint_lower_limits", params_.joint_lower_limits);
    set_param_from_json_vector("joint_upper_limits", params_.joint_upper_limits);

    // Warn user that use_imu should be set in the robot description
    if (j.find("use_imu") != j.end()) {
      params_.use_imu = j["use_imu"];
      RCLCPP_WARN(get_node()->get_logger(),
                  "From JSON, setting params_use_imu=%d. Verify robot description has proper value "
                  "of use_imu too.",
                  params_.use_imu);
    }

    if (j.find("observation_history") != j.end()) {
      params_.observation_history = j["observation_history"];
      RCLCPP_INFO(get_node()->get_logger(), "From JSON, setting params_.observation_history=%ld",
                  params_.observation_history);
    }

    // Check that the observation history is consistent with the model input shape
    if (j["in_shape"].at(1) != params_.observation_history * kSingleObservationSize) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "observation_history (%ld) * kSingleObservationSize (%d) != in_shape (%d)",
                   params_.observation_history, kSingleObservationSize,
                   static_cast<int>(j["in_shape"].at(1)));
      return controller_interface::CallbackReturn::ERROR;
    }

  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!check_param_vector_size()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NeuralController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration NeuralController::command_interface_configuration()
    const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::InterfaceConfiguration NeuralController::state_interface_configuration()
    const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn NeuralController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Clear command buffers to ignore pre-activation commands
  rt_cmd_vel_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>>(nullptr);
  rt_cmd_pose_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Pose>>(nullptr);

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
    init_joint_pos_.at(i) =
        state_interfaces_map_.at(params_.joint_names.at(i)).at("position").get().get_value();
  }

  // Reset estop caused by falling over
  estop_active_ = false;

  init_time_ = get_node()->now();
  repeat_action_counter_ = -1;

  cmd_x_vel_ = 0.0;
  cmd_y_vel_ = 0.0;
  cmd_yaw_vel_ = 0.0;

  // Initialize the observation vector
  observation_.resize(params_.observation_history * kSingleObservationSize, 0.0);

  // Set the gravity z-component in the initial observation vector
  for (int i = 0; i < params_.observation_history; i++) {
    observation_.at(i * kSingleObservationSize + kGravityZIndx) = -1.0;
  }

  // Initialize the command subscriber
  cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        rt_cmd_vel_ptr_.writeFromNonRT(msg);
      });

  cmd_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
      "/cmd_pose", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        rt_cmd_pose_ptr_.writeFromNonRT(msg);
      });

  emergency_stop_subscriber_ = get_node()->create_subscription<std_msgs::msg::Empty>(
      "/emergency_stop", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        estop_active_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "Emergency stop triggered");
      });

  // emergency_stop_reset_subscriber_ = get_node()->create_subscription<std_msgs::msg::Empty>(
  //     "/emergency_stop_reset", rclcpp::SystemDefaultsQoS(),
  //     [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  //       if (estop_active_) {
  //         estop_active_ = false;
  //         on_activate(rclcpp_lifecycle::State());
  //         RCLCPP_INFO(get_node()->get_logger(), "Emergency stop released");
  //       }
  //     });

  // Initialize the publishers
  policy_output_publisher_ =
      get_node()->create_publisher<ActionMsg>("~/policy_output", rclcpp::SystemDefaultsQoS());
  rt_policy_output_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<ActionMsg>>(policy_output_publisher_);

  position_command_publisher_ =
      get_node()->create_publisher<ActionMsg>("~/position_command", rclcpp::SystemDefaultsQoS());
  rt_position_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<ActionMsg>>(position_command_publisher_);

  observation_publisher_ =
      get_node()->create_publisher<ObservationMsg>("~/observation", rclcpp::SystemDefaultsQoS());
  rt_observation_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<ObservationMsg>>(observation_publisher_);

  // Create IMU latency publishers
  imu_latency_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(
      "~/imu_latency_seconds", rclcpp::SystemDefaultsQoS());
  rt_imu_latency_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float32>>(
          imu_latency_publisher_);

  // Create policy inference latency publishers
  policy_inference_latency_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(
      "~/policy_inference_latency_seconds", rclcpp::SystemDefaultsQoS());
  rt_policy_inference_latency_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float32>>(
          policy_inference_latency_publisher_);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn NeuralController::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::FAILURE;
}

controller_interface::CallbackReturn NeuralController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  rt_cmd_vel_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>>(nullptr);
  rt_cmd_pose_ptr_ =
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Pose>>(nullptr);

  for (auto &command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  for (int i = 0; i < kActionSize; i++) {
    command_interfaces_map_.at(params_.joint_names.at(i))
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

  RCLCPP_INFO(get_node()->get_logger(), "Deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type NeuralController::update(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period) {
  // When started, return to the default joint positions
  double time_since_init = (time - init_time_).seconds();
  if (time_since_init < params_.init_duration) {
    for (int i = 0; i < kActionSize; i++) {
      // Interpolate between the initial joint positions and the default joint
      // positions
      double interpolated_joint_pos =
          init_joint_pos_.at(i) * (1 - time_since_init / params_.init_duration) +
          params_.default_joint_pos.at(i) * (time_since_init / params_.init_duration);
      command_interfaces_map_.at(params_.joint_names.at(i))
          .at("position")
          .get()
          .set_value(interpolated_joint_pos);
      command_interfaces_map_.at(params_.joint_names.at(i))
          .at("kp")
          .get()
          .set_value(params_.init_kps.at(i));
      command_interfaces_map_.at(params_.joint_names.at(i))
          .at("kd")
          .get()
          .set_value(params_.init_kds.at(i));
    }
    return controller_interface::return_type::OK;
  }

  // After the init_duration has passed, fade in the policy actions
  double time_since_fade_in = (time - init_time_).seconds() - params_.init_duration;
  float fade_in_multiplier = std::min(time_since_fade_in / params_.fade_in_duration, 1.0);

  // Only get a new action from the policy when repeat_action_counter_ is 0
  repeat_action_counter_ += 1;
  repeat_action_counter_ %= params_.repeat_action;
  if (repeat_action_counter_ != 0) {
    return controller_interface::return_type::OK;
  }

  // Get the latest commanded velocities
  auto cmd_vel = rt_cmd_vel_ptr_.readFromRT();
  if (cmd_vel && cmd_vel->get()) {
    cmd_x_vel_ = cmd_vel->get()->linear.x;
    cmd_y_vel_ = cmd_vel->get()->linear.y;
    cmd_yaw_vel_ = cmd_vel->get()->angular.z;
  }

  // Get the latest commanded pose
  auto cmd_pose = rt_cmd_pose_ptr_.readFromRT();
  if (cmd_pose && cmd_pose->get()) {
    const auto &pose_msg = *cmd_pose->get();
    tf2::Quaternion q(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z,
                      pose_msg.orientation.w);
    desired_world_z_in_body_frame_ = tf2::Vector3(0, 0, 1);
    desired_world_z_in_body_frame_ = tf2::quatRotate(q.inverse(), desired_world_z_in_body_frame_);
  }

  // If an emergency stop has been triggered, set all commands to 0, set damping, and return
  // TODO: use deactivate instead?
  if (estop_active_) {
    for (auto &command_interface : command_interfaces_) {
      command_interface.set_value(0.0);
    }
    for (int i = 0; i < kActionSize; i++) {
      command_interfaces_map_.at(params_.joint_names.at(i))
          .at("kd")
          .get()
          .set_value(params_.estop_kd);
    }
    return controller_interface::return_type::OK;
  }

  // Get the latest observation
  double ang_vel_x = 0;
  double ang_vel_y = 0;
  double ang_vel_z = 0;
  double orientation_w = 0;
  double orientation_x = 0;
  double orientation_y = 0;
  double orientation_z = 0;
  double time_since_measurement_seconds = 0;
  try {
    // read IMU states from hardware interface
    RCLCPP_DEBUG(get_node()->get_logger(), "Attempting to read IMU angular_velocity.x from %s", params_.imu_sensor_name.c_str());
    ang_vel_x = state_interfaces_map_.at(params_.imu_sensor_name)
                    .at("angular_velocity.x")
                    .get()
                    .get_value();
    ang_vel_y = state_interfaces_map_.at(params_.imu_sensor_name)
                    .at("angular_velocity.y")
                    .get()
                    .get_value();
    ang_vel_z = state_interfaces_map_.at(params_.imu_sensor_name)
                    .at("angular_velocity.z")
                    .get()
                    .get_value();
    orientation_w =
        state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.w").get().get_value();
    orientation_x =
        state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.x").get().get_value();
    orientation_y =
        state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.y").get().get_value();
    orientation_z =
        state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.z").get().get_value();

    // Try to read time_since_measurement_seconds if available (optional for simulation)
    auto imu_interfaces = state_interfaces_map_.at(params_.imu_sensor_name);
    if (imu_interfaces.find("time_since_measurement_seconds") != imu_interfaces.end()) {
      time_since_measurement_seconds = imu_interfaces.at("time_since_measurement_seconds").get().get_value();
    } else {
      // Default to 0 if not available (simulation case)
      time_since_measurement_seconds = 0.0;
      RCLCPP_DEBUG_ONCE(get_node()->get_logger(), "time_since_measurement_seconds interface not available, using default value 0.0");
    }

    // Check that the orientation is identity if we are not using the IMU. Use approximate checks
    // to avoid floating point errors
    if (!params_.use_imu) {
      if (std::abs(orientation_w - 1.0) > 1e-3 || std::abs(orientation_x) > 1e-3 ||
          std::abs(orientation_y) > 1e-3 || std::abs(orientation_z) > 1e-3) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "use_imu is false but IMU orientation is not identity");
        return controller_interface::return_type::ERROR;
      }
    } else {
      // Check that the orientation is not identity if we are using the IMU
      if (std::abs(orientation_w - 1.0) < 1e-6 && std::abs(orientation_x) < 1e-6 &&
          std::abs(orientation_y) < 1e-6 && std::abs(orientation_z) < 1e-6) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "use_imu is true but IMU orientation is near identity");
      }
    }

    // Calculate the projected gravity vector
    tf2::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
    tf2::Matrix3x3 m(q);
    tf2::Vector3 world_gravity_vector(0, 0, -1);
    tf2::Vector3 projected_gravity_vector = m.inverse() * world_gravity_vector;

    // If the maximum body angle is exceeded, trigger an emergency stop
    if (-projected_gravity_vector[2] < cos(params_.max_body_angle)) {
      estop_active_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Emergency stop triggered");
      return controller_interface::return_type::OK;
    }

    // Fill the observation vector
    // Angular velocity
    observation_.at(0) = (float)ang_vel_x;
    observation_.at(1) = (float)ang_vel_y;
    observation_.at(2) = (float)ang_vel_z;
    // Projected gravity vector
    observation_.at(3) = (float)projected_gravity_vector[0];
    observation_.at(4) = (float)projected_gravity_vector[1];
    observation_.at(5) = (float)projected_gravity_vector[2];
    // Velocity commands
    observation_.at(6) = (float)cmd_x_vel_;
    observation_.at(7) = (float)cmd_y_vel_;
    observation_.at(8) = (float)cmd_yaw_vel_;
    // Orientation commands
    observation_.at(9) = (float)desired_world_z_in_body_frame_.getX();
    observation_.at(10) = (float)desired_world_z_in_body_frame_.getY();
    observation_.at(11) = (float)desired_world_z_in_body_frame_.getZ();

    // Joint positions
    for (int i = 0; i < kActionSize; i++) {
      // Only include the joint position in the observation if the action type
      // is position
      if (params_.action_types.at(i) == "position") {
        RCLCPP_DEBUG(get_node()->get_logger(), "Attempting to read joint position for %s (index %d)", params_.joint_names.at(i).c_str(), i);
        float joint_pos =
            state_interfaces_map_.at(params_.joint_names.at(i)).at("position").get().get_value();
        observation_.at(kJointPositionIdx + i) = joint_pos - params_.default_joint_pos.at(i);
      }
    }
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read states from hardware interface - std::out_of_range exception: %s", e.what());
    
    // Check which interfaces are missing
    RCLCPP_ERROR(get_node()->get_logger(), "=== Debug Information ===");
    
    // Check IMU interface
    if (state_interfaces_map_.find(params_.imu_sensor_name) == state_interfaces_map_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing IMU sensor interface: %s", params_.imu_sensor_name.c_str());
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "IMU sensor interface '%s' found", params_.imu_sensor_name.c_str());
      auto &imu_interfaces = state_interfaces_map_.at(params_.imu_sensor_name);
      std::vector<std::string> required_imu = {"angular_velocity.x", "angular_velocity.y", "angular_velocity.z", 
                                               "orientation.x", "orientation.y", "orientation.z", "orientation.w"};
      for (const auto &iface : required_imu) {
        if (imu_interfaces.find(iface) == imu_interfaces.end()) {
          RCLCPP_ERROR(get_node()->get_logger(), "Missing IMU interface: %s.%s", params_.imu_sensor_name.c_str(), iface.c_str());
        }
      }
    }
    
    // Check joint interfaces
    for (size_t i = 0; i < params_.joint_names.size(); i++) {
      const auto &joint_name = params_.joint_names.at(i);
      if (state_interfaces_map_.find(joint_name) == state_interfaces_map_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing joint interface: %s", joint_name.c_str());
      } else if (params_.action_types.at(i) == "position") {
        auto &joint_interfaces = state_interfaces_map_.at(joint_name);
        if (joint_interfaces.find("position") == joint_interfaces.end()) {
          RCLCPP_ERROR(get_node()->get_logger(), "Missing position interface for joint: %s", joint_name.c_str());
        }
      }
    }
    
    // List all available interfaces for debugging
    RCLCPP_ERROR(get_node()->get_logger(), "Available state interfaces:");
    for (const auto &[name, interfaces] : state_interfaces_map_) {
      std::string interface_list;
      for (const auto &[iface_name, iface_ref] : interfaces) {
        if (!interface_list.empty()) interface_list += ", ";
        interface_list += iface_name;
      }
      RCLCPP_ERROR(get_node()->get_logger(), "  %s: [%s]", name.c_str(), interface_list.c_str());
    }
    RCLCPP_ERROR(get_node()->get_logger(), "========================");
    
    return controller_interface::return_type::ERROR;
  }

  // Clip the observation vector
  for (auto &obs : observation_) {
    obs = std::clamp(obs, static_cast<float>(-params_.observation_limit),
                     static_cast<float>(params_.observation_limit));
  }

  // Check observation for NaNs
  if (contains_nan(observation_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "observation_ contains NaN");
    return controller_interface::return_type::ERROR;
  }

  // Publish the observation
  if (rt_observation_publisher_->trylock()) {
    // TODO make a custom msg type with header
    // rt_observation_publisher_->msg_.header.stamp = time;
    rt_observation_publisher_->msg_.data = observation_;
    rt_observation_publisher_->unlockAndPublish();
  }

  // Measure the time before policy inference
  auto start_time = std::chrono::high_resolution_clock::now();

  // Perform policy inference
  model_->forward(observation_.data());

  // Measure the time after policy inference
  auto end_time = std::chrono::high_resolution_clock::now();
  auto inference_duration_us =
      std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  RCLCPP_INFO(get_node()->get_logger(),
              "Policy inference took %.2f ms\tIMU measurement age: %.3f ms",
              inference_duration_us / 1000.0, time_since_measurement_seconds * 1000.0);

  // Shift the observation history to the right by kSingleObservationSize for the next control
  // step https://en.cppreference.com/w/cpp/algorithm/rotate
  std::rotate(observation_.rbegin(), observation_.rbegin() + kSingleObservationSize,
              observation_.rend());

  // Process the actions
  const float *policy_output = model_->getOutputs();

  // Publish the policy output
  if (rt_policy_output_publisher_->trylock()) {
    rt_policy_output_publisher_->msg_.data.resize(kActionSize, 0.0);
    for (int i = 0; i < kActionSize; i++) {
      rt_policy_output_publisher_->msg_.data.at(i) = policy_output[i];
    }
    // rt_policy_output_publisher_->msg_.header.stamp = time;
    rt_policy_output_publisher_->unlockAndPublish();
  }

  for (int i = 0; i < kActionSize; i++) {
    float action = policy_output[i];
    float action_scale = params_.action_scales.at(i);
    float default_joint_pos = params_.default_joint_pos.at(i);
    float lower_limit = params_.joint_lower_limits.at(i);
    float upper_limit = params_.joint_upper_limits.at(i);

    // Copy policy_output to the observation vector
    observation_.at(kLastActionIdx + i) = fade_in_multiplier * action;
    // Scale and de-normalize to get the action vector
    if (params_.action_types.at(i) == "position") {
      float unclipped = fade_in_multiplier * action * action_scale + default_joint_pos;
      action_.at(i) = std::clamp(unclipped, lower_limit, upper_limit);
    } else {
      action_.at(i) = fade_in_multiplier * action * action_scale;
    }

    if (std::isnan(action_.at(i))) {
      RCLCPP_ERROR(get_node()->get_logger(), "action_[%d] is NaN", i);
      return controller_interface::return_type::ERROR;
    }

    // Send the action to the hardware interface
    // Multiply by the gain multiplier to scale the gains to account for real2sim gap
    command_interfaces_map_.at(params_.joint_names.at(i))
        .at(params_.action_types.at(i))
        .get()
        .set_value((double)action_.at(i));
    command_interfaces_map_.at(params_.joint_names.at(i))
        .at("kp")
        .get()
        .set_value(params_.kps.at(i) * params_.gain_multiplier);
    command_interfaces_map_.at(params_.joint_names.at(i))
        .at("kd")
        .get()
        .set_value(params_.kds.at(i) * params_.gain_multiplier);
  }

  // Publish the scaled and final position command
  if (rt_position_command_publisher_->trylock()) {
    rt_position_command_publisher_->msg_.data.resize(kActionSize, 0.0);
    for (int i = 0; i < kActionSize; i++) {
      rt_position_command_publisher_->msg_.data.at(i) = action_.at(i);
    }
    // rt_position_command_publisher_->msg_.header.stamp = time;
    rt_position_command_publisher_->unlockAndPublish();
  }

  // Publish imu latency
  if (rt_imu_latency_publisher_->trylock()) {
    rt_imu_latency_publisher_->msg_.data = time_since_measurement_seconds;
    rt_imu_latency_publisher_->unlockAndPublish();
  }

  if (rt_policy_inference_latency_publisher_->trylock()) {
    rt_policy_inference_latency_publisher_->msg_.data = inference_duration_us / 1000000.0;
    rt_policy_inference_latency_publisher_->unlockAndPublish();
  }

  // Get the policy inference time
  // double policy_inference_time = (get_node()->now() - time).seconds();
  // RCLCPP_INFO(get_node()->get_logger(), "policy inference time: %f",
  // policy_inference_time);

  return controller_interface::return_type::OK;
}

}  // namespace neural_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(neural_controller::NeuralController,
                       controller_interface::ControllerInterface)