#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <memory>
#include <string>

class MapSaverNode : public rclcpp::Node
{
public:
  MapSaverNode() : Node("map_saver_node")
  {
    // Parameters
    this->declare_parameter("cloud_topic", "/laser_cloud_surround");
    this->declare_parameter("save_file_name", "lego_loam_map.pcd");
    this->declare_parameter("auto_save", false);
    this->declare_parameter("auto_save_timer", 60.0);  // In seconds
    
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    save_file_name_ = this->get_parameter("save_file_name").as_string();
    auto_save_ = this->get_parameter("auto_save").as_bool();
    auto_save_timer_ = this->get_parameter("auto_save_timer").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Map will be saved to: %s", save_file_name_.c_str());
    
    // Create service to save map
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_map", 
      std::bind(&MapSaverNode::save_map_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Subscribe to the point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, 10, 
      std::bind(&MapSaverNode::cloud_callback, this, std::placeholders::_1));
    
    if (auto_save_) {
      auto_save_timer_obj_ = this->create_wall_timer(
        std::chrono::duration<double>(auto_save_timer_),
        std::bind(&MapSaverNode::auto_save_callback, this));
      RCLCPP_INFO(this->get_logger(), "Auto save enabled. Map will be saved every %.1f seconds", auto_save_timer_);
    }
    
    RCLCPP_INFO(this->get_logger(), "Map saver node initialized");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_cloud_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received cloud with %u points", msg->width * msg->height);
  }
  
  void save_map()
  {
    if (!latest_cloud_) {
      RCLCPP_WARN(this->get_logger(), "No point cloud data received yet, nothing to save");
      return;
    }
    
    try {
      // Convert to PCL point cloud
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*latest_cloud_, *pcl_cloud);
      
      // Save to file
      RCLCPP_INFO(this->get_logger(), "Saving map with %lu points to %s", 
                pcl_cloud->points.size(), save_file_name_.c_str());
      
      if (pcl::io::savePCDFileBinary(save_file_name_, *pcl_cloud) == 0) {
        RCLCPP_INFO(this->get_logger(), "Map saved successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save map");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while saving map: %s", e.what());
    }
  }
  
  void auto_save_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Auto-saving map...");
    save_map();
  }
  
  void save_map_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    save_map();
    response->success = true;
    response->message = "Map saved to " + save_file_name_;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr auto_save_timer_obj_;
  
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::string cloud_topic_;
  std::string save_file_name_;
  bool auto_save_;
  double auto_save_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}