#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "./tictoc.hpp"

using namespace std::chrono_literals;

class PcdPublisher : public rclcpp::Node {
 public:
  explicit PcdPublisher(const rclcpp::NodeOptions &options) : rclcpp::Node("pcd_publisher", options) {
    // Declare parameters
    declare_parameter("pcd_file_path", "");
    declare_parameter("publish_rate", 1.0);  // Hz
    declare_parameter("frame_id", "map");
    // 不要声明 use_sim_time，因为它是 ROS 2 自动声明的
    declare_parameter("verbose", false);
    declare_parameter("publish_once", false);
    
    // Get parameters
    pcd_file_path_ = get_parameter("pcd_file_path").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    double publish_rate = get_parameter("publish_rate").as_double();
    verbose_ = get_parameter("verbose").as_bool();
    publish_once_ = get_parameter("publish_once").as_bool();

    if (pcd_file_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No PCD file path provided. Please set the 'pcd_file_path' parameter.");
      return;
    }

    // Create a publisher with QoS settings for reliable delivery
    rclcpp::QoS qos(1);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos);

    // Load PCD file
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path_, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Could not load PCD file: %s", pcd_file_path_.c_str());
      return;
    }

    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Loaded PCD file with %zu points", cloud->size());
    }

    // Convert to ROS message
    pcl::toROSMsg(*cloud, cloud_msg_);
    cloud_msg_.header.frame_id = frame_id_;

    // Create a timer for publishing at the specified rate
    if (!publish_once_) {
      publish_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / publish_rate),
          std::bind(&PcdPublisher::publish_cloud, this));
    } else {
      // If publish_once is true, publish immediately and then once more after a short delay
      // to ensure subscribers have time to connect
      publish_cloud();
      
      // Publish once more after 1 second to ensure subscribers are connected
      one_shot_timer_ = this->create_wall_timer(
          1s, 
          [this]() {
            publish_cloud();
            one_shot_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Published point cloud once and will now stop publishing");
          });
    }
  }

 private:
  void publish_cloud() {
    // Update timestamp
    cloud_msg_.header.stamp = this->now();
    
    // Publish the point cloud
    cloud_publisher_->publish(cloud_msg_);
    
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Published point cloud with frame_id: %s", frame_id_.c_str());
    }
  }

  std::string pcd_file_path_;
  std::string frame_id_;
  bool verbose_;
  bool publish_once_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<PcdPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
