#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudHeightFilter : public rclcpp::Node
{
public:
  PointCloudHeightFilter() : Node("point_cloud_height_filter")
  {
    // Declare parameters
    this->declare_parameter<double>("height_threshold", 6.0);
    this->declare_parameter<std::string>("input_topic", "/unilidar/cloud");
    this->declare_parameter<std::string>("output_topic", "filtered_cloud");
    this->declare_parameter<std::string>("filter_field", "z");
    this->declare_parameter<bool>("keep_organized", true);
    this->declare_parameter<bool>("filter_negative", false);
    this->declare_parameter<bool>("extract_removed_indices", true); // New parameter

    // Get parameters
    height_threshold_ = this->get_parameter("height_threshold").as_double();
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    filter_field_ = this->get_parameter("filter_field").as_string();
    keep_organized_ = this->get_parameter("keep_organized").as_bool();
    filter_negative_ = this->get_parameter("filter_negative").as_bool();
    extract_removed_indices_ = this->get_parameter("extract_removed_indices").as_bool();

    RCLCPP_INFO(this->get_logger(), "Height threshold set to %f", height_threshold_);
    RCLCPP_INFO(this->get_logger(), "Filter field set to %s", filter_field_.c_str());

    // Create publisher and subscriber
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&PointCloudHeightFilter::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Listening on topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing on topic: %s", output_topic.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
  {
    // Log that a cloud was received
    RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %d points",
                input_cloud->width * input_cloud->height);

    // Convert from ROS message to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Initialize the filter with the extract_removed_indices parameter
    pcl::PassThrough<pcl::PointXYZ> pass(extract_removed_indices_);

    // Configure the filter
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName(filter_field_);

    if (filter_negative_) {
      // Remove points ABOVE the threshold
      pass.setFilterLimits(height_threshold_, std::numeric_limits<float>::infinity());
    } else {
      // Remove points ABOVE the threshold by keeping only points below it
      pass.setFilterLimits(-std::numeric_limits<float>::infinity(), height_threshold_);
    }

    pass.setNegative(filter_negative_);
    pass.setKeepOrganized(keep_organized_);

    // Apply the filter
    pass.filter(*cloud_filtered);

    // Convert back to ROS message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud_filtered, output_cloud);
    output_cloud.header = input_cloud->header;

    // Publish filtered cloud
    publisher_->publish(output_cloud);

    RCLCPP_DEBUG(this->get_logger(), "Published filtered cloud with %d points",
                cloud_filtered->width * cloud_filtered->height);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  double height_threshold_;
  std::string filter_field_;
  bool keep_organized_;
  bool filter_negative_;
  bool extract_removed_indices_;  // Added this member variable
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudHeightFilter>());
  rclcpp::shutdown();
  return 0;
}