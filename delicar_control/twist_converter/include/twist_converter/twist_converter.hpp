#ifndef TWIST_CONVERTER_HPP
#define TWIST_CONVERTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace twist_converter
{

class TwistConverter : public rclcpp::Node
{
public:
  TwistConverter();

private:
  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  // std::string frame_id_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
};

} 

#endif // TWIST_CONVERTER_HPP