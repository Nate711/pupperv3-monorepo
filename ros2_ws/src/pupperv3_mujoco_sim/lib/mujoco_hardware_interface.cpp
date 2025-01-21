#include "mujoco_hardware_interface.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "actuator_model.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mujoco_hardware_interface {

MujocoHardwareInterface::~MujocoHardwareInterface() {
  // Deactivate everything when ctrl-c is pressed
  on_deactivate(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_state_positions_.resize(info_.joints.size(), 0.0);
  hw_state_velocities_.resize(info_.joints.size(), 0.0);

  size_t command_latency_timesteps =
      info_.hardware_parameters.count("command_latency_timesteps")
          ? std::stoi(info_.hardware_parameters.at("command_latency_timesteps"))
          : 0;
  command_buffer_.fill(command_latency_timesteps,
                       mujoco_interactive::zero_command(info_.joints.size()));
  hw_command_positions_.resize(info_.joints.size(), 0.0);
  hw_command_velocities_.resize(info_.joints.size(), 0.0);
  hw_command_efforts_.resize(info_.joints.size(), 0.0);
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  hw_command_kds_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // Set limits for each joint
    hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
    hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
    hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
    hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
    hw_actuator_kp_maxs_.push_back(std::stod(joint.parameters.at("kp_max")));
    hw_actuator_kd_maxs_.push_back(std::stod(joint.parameters.at("kd_max")));

    // Homing parameters
    hw_actuator_homing_stages_.push_back(std::stoi(joint.parameters.at("homing_stage")));
    hw_actuator_homing_velocities_.push_back(std::stod(joint.parameters.at("homing_velocity")));
    hw_actuator_homing_kps_.push_back(std::stod(joint.parameters.at("homing_kp")));
    hw_actuator_homing_kds_.push_back(std::stod(joint.parameters.at("homing_kd")));
    hw_actuator_homed_positions_.push_back(std::stod(joint.parameters.at("homed_position")));
    hw_actuator_zero_positions_.push_back(0.0);
    hw_actuator_homing_torque_thresholds_.push_back(
        std::stod(joint.parameters.at("homing_torque_threshold")));
    hw_actuator_is_homed_.push_back(false);
  }

  if (info_.hardware_parameters.count("backlash") &&
      (info_.hardware_parameters.at("backlash") == "true" ||
       info_.hardware_parameters.at("backlash") == "True")) {
    mujoco_interactive::backlash_ = true;
  }

  if (info_.hardware_parameters.count("use_imu") &&
      (info_.hardware_parameters.at("use_imu") == "false" ||
       info_.hardware_parameters.at("use_imu") == "False")) {
    use_imu_ = false;
  }
  imu_roll_ = std::stod(info_.sensors[0].parameters.at("roll"));
  imu_pitch_ = std::stod(info_.sensors[0].parameters.at("pitch"));
  imu_yaw_ = std::stod(info_.sensors[0].parameters.at("yaw"));

  // Initialize imu_buffer_
  size_t latency_timesteps = info_.hardware_parameters.count("imu_latency_timesteps")
                                 ? std::stoi(info_.hardware_parameters.at("imu_latency_timesteps"))
                                 : 0;
  imu_buffer_.fill(latency_timesteps, {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Set up mujoco simulation
  mujoco_interactive::init();

  // Get model xml file based on parameters
  std::string share_dir = ament_index_cpp::get_package_share_directory("pupper_v3_description");
  bool use_fixed_base = std::stoi(info_.hardware_parameters.at("fixed_base"));
  std::string backlash_str = info_.hardware_parameters.count("backlash")
                                 ? info_.hardware_parameters.at("backlash")
                                 : "false";
  bool backlash = backlash_str == "true" || backlash_str == "True";
  std::string pupper_xml;
  if (backlash) {
    pupper_xml = "pupper_v3_complete.backlash.xml";
  } else if (use_fixed_base) {
    pupper_xml = "pupper_v3_complete.fixed_base.xml";
  } else {
    pupper_xml = "pupper_v3_complete.xml";
  }
  std::string model_xml = share_dir + "/description/mujoco_xml/" + pupper_xml;

  float timestep = std::stod(info_.hardware_parameters.at("timestep"));

  // Construct actuator models based on parameters
  // TODO get rid of kp and kd from actuator params since they are not fixed
  ActuatorParams params(
      /*kp=*/0.0,
      /*kd=*/0.0,
      /*bus_voltage =*/std::stod(info_.hardware_parameters.at("bus_voltage")),
      /*kt=*/std::stod(info_.hardware_parameters.at("kt")),
      /*phase_resistance=*/
      std::stod(info_.hardware_parameters.at("phase_resistance")),
      /*saturation_torque=*/
      std::stod(info_.hardware_parameters.at("saturation_torque")),
      /*software_torque_limit =*/
      std::stod(info_.hardware_parameters.at("software_torque_limit")));
  PIDActuatorModel actuator_model(params);
  std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models(
      info_.joints.size(), std::make_shared<PIDActuatorModel>(params));

  mju::strcpy_arr(mujoco_interactive::filename, model_xml.c_str());
  mujoco_interactive::settings.loadrequest = 1;
  mujoco_interactive::loadmodel();
  mujoco_interactive::set_timestep(timestep);
  mujoco_interactive::set_actuator_models(actuator_models);

  // Necessary if you want to run rendering in a different thread than
  // the one this Node was created in
  mujoco_interactive::detach_opengl_context_from_thread();

  // Start physics simulation
  mujoco_interactive::start_simulation();

  // Start GUI thread
  mujoco_interactive::run_gui_detached();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
    //     &hw_state_[i]));
  }

  // Add IMU state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x",
                                                                   &hw_state_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y",
                                                                   &hw_state_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z",
                                                                   &hw_state_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w",
                                                                   &hw_state_imu_orientation_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("mujoco_sensor", "body_pos.x",
                                                                   &hw_state_mujoco_body_pos_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("mujoco_sensor", "body_pos.y",
                                                                   &hw_state_mujoco_body_pos_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("mujoco_sensor", "body_pos.z",
                                                                   &hw_state_mujoco_body_pos_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("mujoco_sensor", "body_vel.x",
                                                                   &hw_state_mujoco_body_vel_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("mujoco_sensor", "body_vel.y",
                                                                   &hw_state_mujoco_body_vel_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("mujoco_sensor", "body_vel.z",
                                                                   &hw_state_mujoco_body_vel_[2]));
  return state_interfaces;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_state_positions_.size(); i++) {
    hw_state_positions_[i] = 0.0;
    hw_state_velocities_[i] = 0.0;
    hw_command_positions_[i] = 0.0;
    hw_command_velocities_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    hw_command_kps_[i] = 7.5;  // 0.0;
    hw_command_kds_[i] = 0.5;  // 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
MujocoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kp", &hw_command_kps_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kd", &hw_command_kds_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Start calibration thread
  // mujoco_interactive::calibrate_motors_detached();
  // You can comment out calibration and override calibration to be true if you
  // want
  mujoco_interactive::is_robot_calibrated_ = true;
  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // TODO disable actuators somehow
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoHardwareInterface::read(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration &period) {
  for (int i = 0; i < info_.joints.size(); i++) {
    hw_state_positions_[i] = mujoco_interactive::actuator_position(i);
    hw_state_velocities_[i] = mujoco_interactive::actuator_velocity(i);
  }

  // RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"),
  //             "READ. state_pos: %f %f %f | %f %f %f | %f %f %f | %f %f %f",
  //             hw_state_positions_.at(0), hw_state_positions_.at(1),
  //             hw_state_positions_.at(2), hw_state_positions_.at(3),
  //             hw_state_positions_.at(4), hw_state_positions_.at(5),
  //             hw_state_positions_.at(6), hw_state_positions_.at(7),
  //             hw_state_positions_.at(8), hw_state_positions_.at(9),
  //             hw_state_positions_.at(10), hw_state_positions_.at(11));

  // Read the IMU
  auto lagged_sensor_data = imu_buffer_.enqueue(mujoco_interactive::sensor_data());

  // RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"),
  //             "Quat: %f %f %f %f | Gyro: %f %f %f | Acc: %f %f %f",
  //             sensor_data.at(0), sensor_data.at(1), sensor_data.at(2),
  //             sensor_data.at(3), sensor_data.at(4), sensor_data.at(5),
  //             sensor_data.at(6), sensor_data.at(7), sensor_data.at(8),
  //             sensor_data.at(9));

  // Note: We do not correct for the orientation of the IMU because we define it
  // as aligned with body XYZ axes in the model xml

  // Note: Mujoco puts real component first, which matches this constructor
  if (use_imu_) {
    hw_state_imu_orientation_[0] = lagged_sensor_data.at(1);  // x
    hw_state_imu_orientation_[1] = lagged_sensor_data.at(2);  // y
    hw_state_imu_orientation_[2] = lagged_sensor_data.at(3);  // z
    hw_state_imu_orientation_[3] = lagged_sensor_data.at(0);  // w

    // Angular velocity of body in body-frame
    hw_state_imu_angular_velocity_[0] = lagged_sensor_data.at(4);  // wx
    hw_state_imu_angular_velocity_[1] = lagged_sensor_data.at(5);  // wy
    hw_state_imu_angular_velocity_[2] = lagged_sensor_data.at(6);  // wz

    // Linear acceleration of body in body-frame
    hw_state_imu_linear_acceleration_[0] = lagged_sensor_data.at(7);  // ax
    hw_state_imu_linear_acceleration_[1] = lagged_sensor_data.at(8);  // ay
    hw_state_imu_linear_acceleration_[2] = lagged_sensor_data.at(9);  // az
  } else {
    hw_state_imu_orientation_[0] = 0.0;  // x
    hw_state_imu_orientation_[1] = 0.0;  // y
    hw_state_imu_orientation_[2] = 0.0;  // z
    hw_state_imu_orientation_[3] = 1.0;  // w

    // Angular velocity of body in body-frame
    hw_state_imu_angular_velocity_[0] = 0.0;  // wx
    hw_state_imu_angular_velocity_[1] = 0.0;  // wy
    hw_state_imu_angular_velocity_[2] = 0.0;  // wz

    // Linear acceleration of body in body-frame
    hw_state_imu_linear_acceleration_[0] = 0.0;  // ax
    hw_state_imu_linear_acceleration_[1] = 0.0;  // ay
    hw_state_imu_linear_acceleration_[2] = 0.0;  // az
  }

  // Ground-truth body position and velocity in world-frame
  hw_state_mujoco_body_pos_ = mujoco_interactive::base_position();
  hw_state_mujoco_body_vel_ = mujoco_interactive::base_velocity();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!mujoco_interactive::is_robot_calibrated()) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoHardwareInterface"),
                 "Robot not calibrated. Skipping hw interface write");
    return hardware_interface::return_type::OK;
  }
  // RCLCPP_INFO(rclcpp::get_logger("MujocoHardwareInterface"),
  //             "WRITE. com_pos: %f %f %f | %f %f %f | %f %f %f | %f %f %f",
  //             hw_command_positions_.at(0), hw_command_positions_.at(1),
  //             hw_command_positions_.at(2), hw_command_positions_.at(3),
  //             hw_command_positions_.at(4), hw_command_positions_.at(5),
  //             hw_command_positions_.at(6), hw_command_positions_.at(7),
  //             hw_command_positions_.at(8), hw_command_positions_.at(9),
  //             hw_command_positions_.at(10), hw_command_positions_.at(11));

  mujoco_interactive::ActuatorCommand command =
      mujoco_interactive::zero_command(info_.joints.size());
  for (int i = 0; i < info_.joints.size(); i++) {
    command.kp[i] = hw_command_kps_[i];
    command.kd[i] = hw_command_kds_[i];

    command.position_target[i] = hw_command_positions_[i];
    command.velocity_target[i] = hw_command_velocities_[i];
    command.feedforward_torque[i] = hw_command_efforts_[i];
  }
  auto lagged_command = command_buffer_.enqueue(command);
  mujoco_interactive::set_actuator_command(lagged_command);
  return hardware_interface::return_type::OK;
}

}  // namespace mujoco_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_hardware_interface::MujocoHardwareInterface,
                       hardware_interface::SystemInterface)