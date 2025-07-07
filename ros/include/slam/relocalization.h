#pragma once

#ifndef KISS_MATCHER_RELOCALIZATION_H
#define KISS_MATCHER_RELOCALIZATION_H

#include <memory>
#include <string>
#include <thread>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "slam/loop_closure.h"
#include "slam/utils.hpp"
#include "kiss_matcher_ros/srv/relocalize.hpp"

namespace kiss_matcher {

struct RelocalizationConfig {
  // Frame IDs
  std::string map_frame = "map";
  std::string base_frame = "base_link";
  std::string odom_frame = "odom";
  
  // Map configuration
  std::string map_file_path = "";
  
  // Timing parameters
  double publish_rate = 10.0;
  double relocalization_timeout = 30.0;
  double tf_tolerance = 0.1;
  
  // Point cloud processing
  std::string scan_topic = "/velodyne_points";
  double voxel_resolution = 0.3;
  double max_scan_range = 100.0;
  double min_scan_range = 0.5;
  
  // Relocalization parameters
  bool enable_auto_relocalization = false;
  double confidence_threshold = 0.7;
  
  // Coarse registration settings
  bool enable_global_search = true;
  int max_search_attempts = 5;
  double search_radius = 50.0;
  size_t min_inliers_threshold = 100;
  
  // Fine registration settings (reuse existing GICPConfig structure)
  GICPConfig gicp_config;
  
  // TF and visualization
  bool broadcast_tf = true;
  bool publish_aligned_cloud = true;
  std::string aligned_cloud_topic = "/aligned_scan";
  
  // Debugging
  bool verbose = true;
  bool save_debug_clouds = false;
};

class RelocalizationNode : public rclcpp::Node {
public:
  explicit RelocalizationNode();
  ~RelocalizationNode() = default;

private:
  // Configuration
  RelocalizationConfig config_;
  
  // Loop closure handler for coarse-to-fine alignment
  std::unique_ptr<LoopClosure> loop_closure_;
  LoopClosureConfig loop_closure_config_;
  
  // Map data
  pcl::PointCloud<PointType>::Ptr map_cloud_;
  bool map_loaded_ = false;
  
  // Current scan data
  pcl::PointCloud<PointType>::Ptr current_scan_;
  bool scan_received_ = false;
  
  // Relocalization state
  bool is_localized_ = false;
  Eigen::Matrix4d map_to_odom_;
  double last_confidence_ = 0.0;
  
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Service<kiss_matcher_ros::srv::Relocalize>::SharedPtr relocalize_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  
  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr auto_relocalize_timer_;
  
  // Methods
  void loadParameters();
  bool loadMap();
  void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void publishTransform();
  void autoRelocalizationCallback();
  
  // Service callbacks
  void relocalizationCallback(
    const std::shared_ptr<kiss_matcher_ros::srv::Relocalize::Request> request,
    std::shared_ptr<kiss_matcher_ros::srv::Relocalize::Response> response);
  
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  // Core relocalization functions
  bool performRelocalization(const Eigen::Matrix4d& initial_guess, RegOutput& result);
  bool performGlobalRelocalization(RegOutput& result);
  pcl::PointCloud<PointType>::Ptr preprocessScan(const pcl::PointCloud<PointType>& scan);
  double calculateConfidence(const RegOutput& result);
  void publishAlignedCloud(const pcl::PointCloud<PointType>& cloud);
  
  // Utility functions
  Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& pose);
  geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& matrix);
  void saveDebugClouds(const pcl::PointCloud<PointType>& scan, 
                       const pcl::PointCloud<PointType>& aligned,
                       const std::string& suffix = "");
};

} // namespace kiss_matcher

#endif // KISS_MATCHER_RELOCALIZATION_H