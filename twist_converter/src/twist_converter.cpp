#include "twist_converter/twist_converter.hpp"

namespace twist_converter
{

TwistConverter::TwistConverter() : Node("twist_converter")
{
  // Declare parameters
  this->declare_parameter("input_topic", "cmd_vel");
  this->declare_parameter("output_topic", "cmd_vel_stamped");
  // this->declare_parameter("frame_id", "base_link");

  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  // frame_id_ = this->get_parameter("frame_id").as_string();

  twist_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    output_topic_, 10);
  
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_topic_, 10, 
    std::bind(&TwistConverter::twist_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Twist Converter initialized");
  RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
  // RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
}

void TwistConverter::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
  
  twist_stamped_msg.header.stamp = this->now();
  // twist_stamped_msg.header.frame_id = frame_id_;
  
  twist_stamped_msg.twist = *twist_msg;
  
  twist_stamped_pub_->publish(twist_stamped_msg);
}

} 

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<twist_converter::TwistConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}