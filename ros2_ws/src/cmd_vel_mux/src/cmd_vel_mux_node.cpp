#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <vector>
#include <map>
#include <algorithm>

class CmdVelMux : public rclcpp::Node
{
public:
    CmdVelMux() : Node("cmd_vel_mux")
    {
        // Declare parameters
        this->declare_parameter("deadband", 0.05);
        this->declare_parameter("timeout_ms", 500);

        // Declare input sources as a list in priority order
        // First in list = highest priority
        std::vector<std::string> default_inputs = {
            "/teleop_cmd_vel",
            "/llm_cmd_vel",
            "/person_following_cmd_vel"
        };

        this->declare_parameter("inputs", default_inputs);

        deadband_ = this->get_parameter("deadband").as_double();
        timeout_ = std::chrono::milliseconds(this->get_parameter("timeout_ms").as_int());

        auto input_config = this->get_parameter("inputs").as_string_array();

        // Parse input configuration - order in list determines priority
        for (size_t i = 0; i < input_config.size(); i++)
        {
            std::string topic = input_config[i];
            int priority = static_cast<int>(i);  // Index is priority (0 = highest)

            // Create input source
            InputSource source;
            source.topic = topic;
            source.priority = priority;
            source.active = false;
            source.received = false;
            source.last_activity = this->now();
            source.cmd_vel = geometry_msgs::msg::Twist();

            // Create subscriber
            source.subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
                topic, 10,
                [this, topic](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    this->cmdVelCallback(topic, msg);
                });

            input_sources_[topic] = source;

            RCLCPP_INFO(this->get_logger(), "Added input: %s with priority %d",
                        topic.c_str(), priority);
        }

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd_vel_mux/active_source", 10);

        // Initialize state
        current_active_source_ = "none";

        RCLCPP_INFO(this->get_logger(), "CmdVelMux node initialized with %zu input sources",
                    input_sources_.size());
        RCLCPP_INFO(this->get_logger(), "Deadband: %f", deadband_);
        RCLCPP_INFO(this->get_logger(), "Timeout: %ld ms", timeout_.count());

        // Timer to check timeouts and publish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CmdVelMux::timerCallback, this));
    }

private:
    struct InputSource
    {
        std::string topic;
        int priority;
        bool active;
        bool received;
        rclcpp::Time last_activity;
        geometry_msgs::msg::Twist cmd_vel;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
    };

    void cmdVelCallback(const std::string& topic, const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto& source = input_sources_[topic];
        source.cmd_vel = *msg;
        source.received = true;

        // Special handling for joystick/teleop input - requires deadband
        bool is_joystick = (topic == "/teleop_cmd_vel" || topic == "/joystick_cmd_vel");

        if (is_joystick)
        {
            // Joystick requires movement outside deadband to be considered active
            bool activity_detected = (std::abs(msg->linear.x) > deadband_ ||
                                     std::abs(msg->linear.y) > deadband_ ||
                                     std::abs(msg->angular.z) > deadband_);

            if (activity_detected)
            {
                source.last_activity = this->now();
                if (!source.active)
                {
                    source.active = true;
                    RCLCPP_INFO(this->get_logger(), "Input %s activated (deadband exceeded)", topic.c_str());
                }
            }
        }
        else
        {
            // Other inputs are considered active on any message (even zeros)
            source.last_activity = this->now();
            if (!source.active)
            {
                source.active = true;
                RCLCPP_INFO(this->get_logger(), "Input %s activated", topic.c_str());
            }
        }
    }

    void timerCallback()
    {
        auto now = this->now();

        // Update active status for all sources based on timeout
        for (auto& [topic, source] : input_sources_)
        {
            if (source.active)
            {
                auto time_since_activity = now - source.last_activity;
                if (time_since_activity > rclcpp::Duration(timeout_))
                {
                    source.active = false;
                    RCLCPP_INFO(this->get_logger(), "Input %s timed out", topic.c_str());
                }
            }
        }

        // STEP 1: Find the highest priority source that is currently active
        // "best_source" is a pointer to the InputSource struct with the best priority
        InputSource* best_source = nullptr;
        int best_priority = std::numeric_limits<int>::max();  // Lower number = higher priority

        // First pass: Look for sources that are both active AND have received data
        for (auto& [topic, source] : input_sources_)
        {
            if (source.active && source.received)
            {
                if (source.priority < best_priority)  // Remember: lower number = higher priority
                {
                    best_priority = source.priority;
                    best_source = &source;
                }
            }
        }

        // STEP 2: Determine output velocity and active source name
        // "active_source" is just a string with the topic name (or "none")
        geometry_msgs::msg::Twist output_cmd_vel;
        std::string active_source;

        if (best_source)
        {
            // We found an active source to use
            output_cmd_vel = best_source->cmd_vel;
            active_source = best_source->topic;  // Just the topic name as a string
        }
        else
        {
            // No active sources - publish zero velocity for safety
            output_cmd_vel = geometry_msgs::msg::Twist();  // Zero velocity
            active_source = "none";  // Special string indicating no source
        }

        // Always publish velocity commands when we have an active source
        // Only skip publishing if source is "none" AND hasn't changed
        bool should_publish = (active_source != "none") || (active_source != current_active_source_);

        if (should_publish)
        {
            cmd_vel_pub_->publish(output_cmd_vel);
        }

        // Update status if the active source changed
        if (active_source != current_active_source_)
        {
            current_active_source_ = active_source;

            std_msgs::msg::String status_msg;
            status_msg.data = active_source;
            status_pub_->publish(status_msg);

            RCLCPP_INFO(this->get_logger(), "Active source: %s", active_source.c_str());
        }
    }

    // Input sources map
    std::map<std::string, InputSource> input_sources_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double deadband_;
    std::chrono::milliseconds timeout_;

    // State variables
    std::string current_active_source_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelMux>());
    rclcpp::shutdown();
    return 0;
}