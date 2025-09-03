#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ImuToTf : public rclcpp::Node
{
public:
  ImuToTf() : Node("imu_to_tf")
  {
    world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
    body_frame_ = this->declare_parameter<std::string>("body_frame", "base_link");
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_sensor_broadcaster/imu", 10, std::bind(&ImuToTf::cb, this, std::placeholders::_1));
  }

private:
  void cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = world_frame_;
    t.child_frame_id = body_frame_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation = msg->orientation;
    br_->sendTransform(t);
    RCLCPP_DEBUG(this->get_logger(), "Sent transform");
  }
  std::string world_frame_, body_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuToTf>());
  rclcpp::shutdown();
  return 0;
}