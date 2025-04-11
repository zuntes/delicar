#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <string>

class RoboSenseToVelodyneConverter : public rclcpp::Node {
public:
  RoboSenseToVelodyneConverter() : Node("robosense_to_velodyne_converter") {
    // Declare parameters
    this->declare_parameter("input_topic", "/rslidar_points");
    this->declare_parameter("output_topic", "/velodyne_points");
    this->declare_parameter("frame_id", "velodyne");
    this->declare_parameter("flat_organization", true);
    this->declare_parameter("max_ring_value", 16);
    this->declare_parameter("synthetic_scan_time", 0.1);

    // Get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    flat_organization_ = this->get_parameter("flat_organization").as_bool();
    max_ring_value_ = this->get_parameter("max_ring_value").as_int();
    synthetic_scan_time_ = this->get_parameter("synthetic_scan_time").as_double();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();

    // Create subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, qos,
      std::bind(&RoboSenseToVelodyneConverter::pointcloud_callback, this, std::placeholders::_1));

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

    RCLCPP_INFO(this->get_logger(), "RoboSense to Velodyne converter initialized");
    RCLCPP_INFO(this->get_logger(), "Converting from %s to %s", input_topic.c_str(), output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Using flat organization: %s", flat_organization_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr rslidar_cloud) {
    if (!rslidar_cloud || rslidar_cloud->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud, skipping conversion");
      return;
    }

    // Debug information about input cloud
    RCLCPP_DEBUG(this->get_logger(), "Input cloud: width=%d, height=%d, point_step=%d", 
                rslidar_cloud->width, rslidar_cloud->height, rslidar_cloud->point_step);
    
    // Print field information for debugging
    for (const auto& field : rslidar_cloud->fields) {
      RCLCPP_DEBUG(this->get_logger(), "Field: %s, offset: %d, datatype: %d", 
                  field.name.c_str(), field.offset, field.datatype);
    }
    
    // Create output pointcloud with Velodyne format
    auto velodyne_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    
    // Copy header but change frame_id if needed
    velodyne_cloud->header = rslidar_cloud->header;
    if (!frame_id_.empty()) {
      velodyne_cloud->header.frame_id = frame_id_;
    }
    
    // Calculate total points
    uint32_t total_points = rslidar_cloud->width * rslidar_cloud->height;
    
    // Setup height and width for typical Velodyne format (flat organization)
    if (flat_organization_) {
      velodyne_cloud->height = 1;
      velodyne_cloud->width = total_points;
    } else {
      velodyne_cloud->height = rslidar_cloud->height;
      velodyne_cloud->width = rslidar_cloud->width;
    }
    
    velodyne_cloud->is_bigendian = rslidar_cloud->is_bigendian;
    velodyne_cloud->is_dense = rslidar_cloud->is_dense;
    
    // Find field offsets and data types in input cloud
    int x_offset = -1, y_offset = -1, z_offset = -1;
    int intensity_offset = -1, ring_offset = -1, time_offset = -1;
    uint8_t intensity_datatype = 0, ring_datatype = 0, time_datatype = 0;
    
    for (const auto& field : rslidar_cloud->fields) {
      if (field.name == "x") {
        x_offset = field.offset;
      } else if (field.name == "y") {
        y_offset = field.offset;
      } else if (field.name == "z") {
        z_offset = field.offset;
      } else if (field.name == "intensity") {
        intensity_offset = field.offset;
        intensity_datatype = field.datatype;
      } else if (field.name == "ring") {
        ring_offset = field.offset;
        ring_datatype = field.datatype;
      } else if (field.name == "timestamp" || field.name == "time") {
        time_offset = field.offset;
        time_datatype = field.datatype;
      }
    }
    
    // Validate required fields
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      RCLCPP_ERROR(this->get_logger(), "Required fields (x, y, z) not found in input cloud");
      return;
    }
    
    // Set up Velodyne fields (x, y, z, intensity, ring, time)
    velodyne_cloud->fields.resize(6);
    
    // X field
    velodyne_cloud->fields[0].name = "x";
    velodyne_cloud->fields[0].offset = 0;
    velodyne_cloud->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    velodyne_cloud->fields[0].count = 1;
    
    // Y field
    velodyne_cloud->fields[1].name = "y";
    velodyne_cloud->fields[1].offset = 4;
    velodyne_cloud->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    velodyne_cloud->fields[1].count = 1;
    
    // Z field
    velodyne_cloud->fields[2].name = "z";
    velodyne_cloud->fields[2].offset = 8;
    velodyne_cloud->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    velodyne_cloud->fields[2].count = 1;
    
    // Intensity field
    velodyne_cloud->fields[3].name = "intensity";
    velodyne_cloud->fields[3].offset = 12;
    velodyne_cloud->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    velodyne_cloud->fields[3].count = 1;
    
    // Ring field
    velodyne_cloud->fields[4].name = "ring";
    velodyne_cloud->fields[4].offset = 16;
    velodyne_cloud->fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
    velodyne_cloud->fields[4].count = 1;
    
    // Time field
    velodyne_cloud->fields[5].name = "time";
    velodyne_cloud->fields[5].offset = 18;
    velodyne_cloud->fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    velodyne_cloud->fields[5].count = 1;
    
    // Point size is 22 bytes (4+4+4+4+2+4)
    velodyne_cloud->point_step = 22;
    velodyne_cloud->row_step = velodyne_cloud->point_step * velodyne_cloud->width;
    
    // Allocate memory for data
    velodyne_cloud->data.resize(velodyne_cloud->row_step * velodyne_cloud->height);
    
    // Convert and copy points directly
    const uint8_t* input_ptr = rslidar_cloud->data.data();
    uint8_t* output_ptr = velodyne_cloud->data.data();
    
    for (uint32_t i = 0; i < total_points; ++i) {
      // Copy x, y, z (always float32 in both formats)
      std::memcpy(output_ptr + 0, input_ptr + x_offset, sizeof(float));
      std::memcpy(output_ptr + 4, input_ptr + y_offset, sizeof(float));
      std::memcpy(output_ptr + 8, input_ptr + z_offset, sizeof(float));
      
      // Copy intensity with datatype conversion if needed
      float intensity = 0.0f;
      if (intensity_offset >= 0) {
        switch (intensity_datatype) {
          case sensor_msgs::msg::PointField::FLOAT32:
            intensity = *reinterpret_cast<const float*>(input_ptr + intensity_offset);
            break;
          case sensor_msgs::msg::PointField::UINT8:
            intensity = static_cast<float>(*reinterpret_cast<const uint8_t*>(input_ptr + intensity_offset));
            break;
          case sensor_msgs::msg::PointField::UINT16:
            intensity = static_cast<float>(*reinterpret_cast<const uint16_t*>(input_ptr + intensity_offset));
            break;
          default:
            // Default to 0 if datatype is not recognized
            break;
        }
      }
      std::memcpy(output_ptr + 12, &intensity, sizeof(float));
      
      // Copy ring with datatype conversion if needed
      uint16_t ring = 0;
      if (ring_offset >= 0) {
        switch (ring_datatype) {
          case sensor_msgs::msg::PointField::UINT16:
            ring = *reinterpret_cast<const uint16_t*>(input_ptr + ring_offset);
            break;
          case sensor_msgs::msg::PointField::UINT8:
            ring = static_cast<uint16_t>(*reinterpret_cast<const uint8_t*>(input_ptr + ring_offset));
            break;
          default:
            // Default to 0 if datatype is not recognized
            break;
        }
      }
      std::memcpy(output_ptr + 16, &ring, sizeof(uint16_t));
      
      // Copy time/timestamp with conversion if needed
      float time_value = 0.0f;
      if (time_offset >= 0) {
        switch (time_datatype) {
          case sensor_msgs::msg::PointField::FLOAT32:
            time_value = *reinterpret_cast<const float*>(input_ptr + time_offset);
            break;
          case sensor_msgs::msg::PointField::FLOAT64:
            // Convert double timestamp to float time
            time_value = static_cast<float>(*reinterpret_cast<const double*>(input_ptr + time_offset));
            break;
          default:
            // Default to synthetic time if datatype is not recognized
            if (ring_offset >= 0) {
              // Calculate a synthetic time based on ring number
              time_value = static_cast<float>(ring) / static_cast<float>(max_ring_value_) * synthetic_scan_time_;
            }
            break;
        }
      } else if (ring_offset >= 0) {
        // No time field available, calculate a synthetic time based on ring number
        time_value = static_cast<float>(ring) / static_cast<float>(max_ring_value_) * synthetic_scan_time_;
      }
      std::memcpy(output_ptr + 18, &time_value, sizeof(float));
      
      // Advance pointers
      input_ptr += rslidar_cloud->point_step;
      output_ptr += velodyne_cloud->point_step;
    }
    
    // Publish converted pointcloud
    publisher_->publish(std::move(velodyne_cloud));
    RCLCPP_DEBUG(this->get_logger(), "Published converted point cloud: %d points", total_points);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::string frame_id_;
  bool flat_organization_;
  int max_ring_value_;
  double synthetic_scan_time_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::install_signal_handlers();
  
  auto node = std::make_shared<RoboSenseToVelodyneConverter>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}