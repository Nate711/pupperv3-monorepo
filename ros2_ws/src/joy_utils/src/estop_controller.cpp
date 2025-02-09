#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include <mutex>
#include <thread>

class EStopController : public rclcpp::Node {
public:
  EStopController()
      : Node("estop_controller"), prev_estop_state_(false),
        prev_estop_release_state_(false), service_call_in_progress_(false) {
    // Declare parameters for button indices
    this->declare_parameter<int>(
        "estop_index", 12); // Default index for pressing in right joystick
    this->declare_parameter<int>("estop_release_index",
                                 9); // Default index for start button
    this->declare_parameter<std::vector<long>>(
        "switch_button_indices",
        {0, 1}); // Default indices for 'x' and 'o' buttons

    // Declare parameters for controller names
    this->declare_parameter<std::vector<std::string>>(
        "controller_names",
        {"neural_controller", "neural_controller_three_legged"});

    // Get parameter values
    this->get_parameter("estop_index", estop_index_);
    this->get_parameter("estop_release_index", estop_release_index_);
    this->get_parameter("switch_button_indices", switch_button_indices_);
    this->get_parameter("controller_names", controller_names_);

    latest_active_controller_ = controller_names_.at(0);

    // Initialize previous switch states
    prev_switch_states_.resize(switch_button_indices_.size(), false);

    // Publishers
    pub_estop_ =
        this->create_publisher<std_msgs::msg::Empty>("/emergency_stop", 10);

    // Subscriber to /joy
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&EStopController::joy_callback, this, std::placeholders::_1));

    // Service client for switching controllers
    switch_controller_client_ =
        this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

    RCLCPP_INFO(this->get_logger(), "EStopController node has been started.");
  }

private:
  std::string latest_active_controller_ = "";

  void switch_to_controller(std::string controller_to_switch_to) {
    std::vector<std::string> deactivate_controllers;
    for (const auto &controller : controller_names_) {
      if (controller != controller_to_switch_to) {
        deactivate_controllers.push_back(controller);
      }
    }
    latest_active_controller_ = controller_to_switch_to;
    std::thread(&EStopController::switch_controllers_sync, this,
                std::vector<std::string>{controller_to_switch_to},
                deactivate_controllers,
                /*strict=*/false)
        .detach();
  }

  void deactivate_all_controllers_and_estop() {
    auto empty_msg = std_msgs::msg::Empty();
    pub_estop_->publish(empty_msg);
    RCLCPP_INFO(this->get_logger(),
                "Button %d pressed: Published to estop topic", estop_index_);
    std::thread(&EStopController::switch_controllers_sync, this,
                std::vector<std::string>{}, controller_names_,
                /*strict=*/false)
        .detach();
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check if estop button is pressed
    // TODO: Log warning if buttons is not the right size
    bool estop_pressed = msg->buttons.size() > estop_index_ &&
                         msg->buttons.at(estop_index_) == 1;
    if (estop_pressed && !prev_estop_state_) {
      deactivate_all_controllers_and_estop();
    }
    prev_estop_state_ = estop_pressed;

    // Check if estop release button is pressed
    bool estop_release_pressed = msg->buttons.size() > estop_release_index_ &&
                                 msg->buttons.at(estop_release_index_) == 1;
    if (estop_release_pressed && !prev_estop_release_state_) {
      RCLCPP_INFO(this->get_logger(),
                  "Button %d pressed: Published to estop release topic. "
                  "Activating latest controller: %s.",
                  estop_release_index_, latest_active_controller_.c_str());
      switch_to_controller(latest_active_controller_);
    }
    prev_estop_release_state_ = estop_release_pressed;

    // Check if any controller switch buttons are pressed
    for (size_t i = 0; i < switch_button_indices_.size(); ++i) {
      bool button_pressed =
          msg->buttons.size() > switch_button_indices_.at(i) &&
          msg->buttons.at(switch_button_indices_.at(i)) == 1;
      if (button_pressed && !prev_switch_states_.at(i)) {
        RCLCPP_INFO(this->get_logger(), "Button %ld pressed: Switching to %s",
                    switch_button_indices_.at(i),
                    controller_names_.at(i).c_str());
        switch_to_controller(controller_names_.at(i));
      }
      prev_switch_states_.at(i) = button_pressed;
    }
  }

  /**
   * @brief Synchronously switches the active and inactive controllers.
   *
   * This function locks a mutex to ensure that only one service call is in
   * progress at a time. It then creates a request to switch controllers, waits
   * for the switch controller service to be available, and sends the request.
   * The function logs the result of the service call.
   *
   * @param activate_controllers A vector of controllers to activate. If {} then
   * none activated.
   * @param deactivate_controllers A vector of controller names to be
   * deactivated.
   * @param strict A boolean indicating whether to use strict mode (true) or
   * best effort mode (false) for switching controllers.
   */
  void switch_controllers_sync(
      const std::vector<std::string> &activate_controllers,
      const std::vector<std::string> &deactivate_controllers, bool strict) {
    std::lock_guard<std::mutex> lock(service_call_mutex_);
    if (service_call_in_progress_) {
      RCLCPP_WARN(this->get_logger(), "Service call already in progress");
      return;
    }
    service_call_in_progress_ = true;

    auto request = std::make_shared<
        controller_manager_msgs::srv::SwitchController::Request>();

    request->activate_controllers = activate_controllers;
    request->deactivate_controllers = deactivate_controllers;
    request->strictness =
        strict ? controller_manager_msgs::srv::SwitchController::Request::STRICT
               : controller_manager_msgs::srv::SwitchController::Request::
                     BEST_EFFORT;

    if (!switch_controller_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(),
                  "Switch controller service is not available");
      service_call_in_progress_ = false;
      return;
    }

    auto result = switch_controller_client_->async_send_request(request).get();
    if (result->ok) {
      RCLCPP_INFO(this->get_logger(), "Switched controllers successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers");
    }
    service_call_in_progress_ = false;
  }

  // Parameters for button indices
  int estop_index_;
  int estop_release_index_;
  std::vector<long> switch_button_indices_;

  // Parameters for controller names
  std::vector<std::string> controller_names_;

  // Previous button states
  bool prev_estop_state_;
  bool prev_estop_release_state_;
  std::vector<bool> prev_switch_states_;

  // ROS 2 publishers
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_estop_;

  // ROS 2 subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // ROS 2 service client
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;

  // Mutex for service call
  std::mutex service_call_mutex_;
  bool service_call_in_progress_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EStopController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
