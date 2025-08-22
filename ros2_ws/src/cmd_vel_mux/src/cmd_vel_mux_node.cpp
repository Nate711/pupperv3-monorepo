#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

class CmdVelMux : public rclcpp::Node
{
public:
    CmdVelMux() : Node("cmd_vel_mux")
    {
        // Declare parameters
        this->declare_parameter("deadband", 0.05);
        this->declare_parameter("joystick_timeout_ms", 500);

        deadband_ = this->get_parameter("deadband").as_double();
        joystick_timeout_ = std::chrono::milliseconds(
            this->get_parameter("joystick_timeout_ms").as_int());

        // Subscribers
        joystick_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/teleop_cmd_vel", 10,
            std::bind(&CmdVelMux::joystickCmdVelCallback, this, std::placeholders::_1));

        llm_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/llm_cmd_vel", 10,
            std::bind(&CmdVelMux::llmCmdVelCallback, this, std::placeholders::_1));

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd_vel_mux/active_source", 10);

        // Initialize state
        joystick_active_ = false;
        last_joystick_activity_ = this->now();
        current_active_source_ = "none";

        RCLCPP_INFO(this->get_logger(), "CmdVelMux node initialized");
        RCLCPP_INFO(this->get_logger(), "Deadband: %f", deadband_);
        RCLCPP_INFO(this->get_logger(), "Joystick timeout: %ld ms", joystick_timeout_.count());

        // Timer to check joystick timeout
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CmdVelMux::timerCallback, this));
    }

private:
    void joystickCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_joystick_cmd_vel_ = *msg;
        joystick_cmd_vel_received_ = true;

        // Check if joystick command indicates activity (movement outside deadband)
        bool activity_detected = (std::abs(msg->linear.x) > deadband_ ||
                                  std::abs(msg->linear.y) > deadband_ ||
                                  std::abs(msg->angular.z) > deadband_);

        if (activity_detected)
        {
            last_joystick_activity_ = this->now();
            if (!joystick_active_)
            {
                joystick_active_ = true;
                RCLCPP_INFO(this->get_logger(), "Joystick activated");
            }
        }
    }

    void llmCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_llm_cmd_vel_ = *msg;
        llm_cmd_vel_received_ = true;
    }

    void timerCallback()
    {
        auto now = this->now();
        auto time_since_joystick = now - last_joystick_activity_;

        // Check if joystick has timed out
        if (joystick_active_ && time_since_joystick > rclcpp::Duration(joystick_timeout_))
        {
            joystick_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Joystick timed out, switching to LLM control");
        }

        // Publish appropriate cmd_vel and status
        geometry_msgs::msg::Twist output_cmd_vel;
        std::string active_source;

        if (joystick_active_ && joystick_cmd_vel_received_)
        {
            output_cmd_vel = latest_joystick_cmd_vel_;
            active_source = "joystick";
        }
        else if (llm_cmd_vel_received_)
        {
            output_cmd_vel = latest_llm_cmd_vel_;
            active_source = "llm";
        }
        else
        {
            // No commands received, publish zero velocity
            output_cmd_vel = geometry_msgs::msg::Twist();
            active_source = "none";
        }

        // Only publish if something changed or we have an active command
        if (active_source != current_active_source_ ||
            active_source == "joystick" || active_source == "llm")
        {

            cmd_vel_pub_->publish(output_cmd_vel);

            if (active_source != current_active_source_)
            {
                current_active_source_ = active_source;

                std_msgs::msg::String status_msg;
                status_msg.data = active_source;
                status_pub_->publish(status_msg);

                RCLCPP_INFO(this->get_logger(), "Active source: %s", active_source.c_str());
            }
        }
    }

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joystick_cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr llm_cmd_vel_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double deadband_;
    std::chrono::milliseconds joystick_timeout_;

    // State variables
    bool joystick_active_;
    rclcpp::Time last_joystick_activity_;
    std::string current_active_source_;

    // Latest received messages
    geometry_msgs::msg::Twist latest_joystick_cmd_vel_;
    geometry_msgs::msg::Twist latest_llm_cmd_vel_;
    bool joystick_cmd_vel_received_ = false;
    bool llm_cmd_vel_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelMux>());
    rclcpp::shutdown();
    return 0;
}