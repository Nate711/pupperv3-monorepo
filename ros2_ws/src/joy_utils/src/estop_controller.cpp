#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>

class EStopController : public rclcpp::Node {
public:
  EStopController()
      : Node("estop_controller"), prev_estop_state_(false),
        prev_estop_release_state_(false) {
    // Declare parameters for button indices
    this->declare_parameter<int>(
        "estop_index",
        12); // Default index for pressing in right joystick
    this->declare_parameter<int>("estop_release_index",
                                 9); // Default index for start button

    // Get parameter values
    this->get_parameter("estop_index", estop_index_);
    this->get_parameter("estop_release_index", estop_release_index_);

    // Publishers
    pub_estop_ =
        this->create_publisher<std_msgs::msg::Empty>("/emergency_stop", 10);
    pub_estop_release_ = this->create_publisher<std_msgs::msg::Empty>(
        "/emergency_stop_reset", 10);

    // Subscriber to /joy
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&EStopController::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "EStopController node has been started.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check if estop button is pressed
    bool estop_pressed =
        msg->buttons.size() > estop_index_ && msg->buttons[estop_index_] == 1;
    if (estop_pressed && !prev_estop_state_) {
      auto empty_msg = std_msgs::msg::Empty();
      pub_estop_->publish(empty_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Button %d pressed: Published to estop topic", estop_index_);
    }
    prev_estop_state_ = estop_pressed;

    // Check if estop release button is pressed
    bool estop_release_pressed = msg->buttons.size() > estop_release_index_ &&
                                 msg->buttons[estop_release_index_] == 1;
    if (estop_release_pressed && !prev_estop_release_state_) {
      auto empty_msg = std_msgs::msg::Empty();
      pub_estop_release_->publish(empty_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Button %d pressed: Published to estop release topic",
                  estop_release_index_);
    }
    prev_estop_release_state_ = estop_release_pressed;
  }

  // Parameters for button indices
  int estop_index_;
  int estop_release_index_;

  // Previous button states
  bool prev_estop_state_;
  bool prev_estop_release_state_;

  // ROS 2 publishers
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_estop_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_estop_release_;

  // ROS 2 subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EStopController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
