#include "slam/relocalization.h"

using namespace kiss_matcher;

RelocalizationNode::RelocalizationNode() : Node("relocalization_node") {
  // Load parameters
  loadParameters();
  
  // Initialize point clouds
  map_cloud_.reset(new pcl::PointCloud<PointType>());
  current_scan_.reset(new pcl::PointCloud<PointType>());
  
  // Initialize transform
  map_to_odom_ = Eigen::Matrix4d::Identity();
  
  // Initialize TF
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Setup loop closure with configuration
  loop_closure_config_.verbose_ = config_.verbose;
  loop_closure_config_.voxel_res_ = config_.voxel_resolution;
  loop_closure_config_.num_inliers_threshold_ = config_.min_inliers_threshold;
  loop_closure_config_.gicp_config_ = config_.gicp_config;
  loop_closure_config_.enable_global_registration_ = config_.enable_global_search;
  
  loop_closure_ = std::make_unique<LoopClosure>(loop_closure_config_, this->get_logger());
  
  // Load map
  if (!loadMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map. Node will not function properly.");
    return;
  }
  
  // Setup ROS interfaces
  scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.scan_topic, 10,
    std::bind(&RelocalizationNode::scanCallback, this, std::placeholders::_1));
  
  if (config_.publish_aligned_cloud) {
    aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.aligned_cloud_topic, 10);
  }
  
  // Setup services
  relocalize_service_ = this->create_service<kiss_matcher_ros::srv::Relocalize>(
    "relocalize",
    std::bind(&RelocalizationNode::relocalizationCallback, this, 
              std::placeholders::_1, std::placeholders::_2));
  
  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_localization",
    std::bind(&RelocalizationNode::resetCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // Setup timers
  if (config_.broadcast_tf) {
    tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / config_.publish_rate)),
      std::bind(&RelocalizationNode::publishTransform, this));
  }
  
  if (config_.enable_auto_relocalization) {
    auto_relocalize_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),  // Try every 5 seconds
      std::bind(&RelocalizationNode::autoRelocalizationCallback, this));
  }
  
  RCLCPP_INFO(this->get_logger(), "Relocalization node initialized successfully");
}

void RelocalizationNode::loadParameters() {
  // Frame IDs
  config_.map_frame = this->declare_parameter("map_frame", config_.map_frame);
  config_.base_frame = this->declare_parameter("base_frame", config_.base_frame);
  config_.odom_frame = this->declare_parameter("odom_frame", config_.odom_frame);
  
  // Map configuration
  config_.map_file_path = this->declare_parameter("map_file_path", config_.map_file_path);
  
  // Timing parameters
  config_.publish_rate = this->declare_parameter("publish_rate", config_.publish_rate);
  config_.relocalization_timeout = this->declare_parameter("relocalization_timeout", config_.relocalization_timeout);
  config_.tf_tolerance = this->declare_parameter("tf_tolerance", config_.tf_tolerance);
  
  // Point cloud processing
  config_.scan_topic = this->declare_parameter("scan_topic", config_.scan_topic);
  config_.voxel_resolution = this->declare_parameter("voxel_resolution", config_.voxel_resolution);
  config_.max_scan_range = this->declare_parameter("max_scan_range", config_.max_scan_range);
  config_.min_scan_range = this->declare_parameter("min_scan_range", config_.min_scan_range);
  
  // Relocalization parameters
  config_.enable_auto_relocalization = this->declare_parameter("enable_auto_relocalization", config_.enable_auto_relocalization);
  config_.confidence_threshold = this->declare_parameter("confidence_threshold", config_.confidence_threshold);
  
  // Coarse registration
  config_.enable_global_search = this->declare_parameter("coarse_reg.enable_global_search", config_.enable_global_search);
  config_.max_search_attempts = this->declare_parameter("coarse_reg.max_search_attempts", config_.max_search_attempts);
  config_.search_radius = this->declare_parameter("coarse_reg.search_radius", config_.search_radius);
  config_.min_inliers_threshold = this->declare_parameter("coarse_reg.min_inliers_threshold", static_cast<int64_t>(config_.min_inliers_threshold));
  
  // Fine registration
  config_.gicp_config_.num_threads_ = this->declare_parameter("fine_reg.num_threads", config_.gicp_config_.num_threads_);
  config_.gicp_config_.correspondence_randomness_ = this->declare_parameter("fine_reg.correspondences_number", config_.gicp_config_.correspondence_randomness_);
  config_.gicp_config_.max_num_iter_ = this->declare_parameter("fine_reg.max_num_iter", config_.gicp_config_.max_num_iter_);
  config_.gicp_config_.max_corr_dist_ = this->declare_parameter("fine_reg.max_corr_dist", config_.gicp_config_.max_corr_dist_);
  config_.gicp_config_.overlap_threshold_ = this->declare_parameter("fine_reg.overlap_threshold", config_.gicp_config_.overlap_threshold_);
  
  // TF and visualization
  config_.broadcast_tf = this->declare_parameter("broadcast_tf", config_.broadcast_tf);
  config_.publish_aligned_cloud = this->declare_parameter("publish_aligned_cloud", config_.publish_aligned_cloud);
  config_.aligned_cloud_topic = this->declare_parameter("aligned_cloud_topic", config_.aligned_cloud_topic);
  
  // Debugging
  config_.verbose = this->declare_parameter("verbose", config_.verbose);
  config_.save_debug_clouds = this->declare_parameter("save_debug_clouds", config_.save_debug_clouds);
}

bool RelocalizationNode::loadMap() {
  if (config_.map_file_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Map file path is empty. Please set 'map_file_path' parameter.");
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", config_.map_file_path.c_str());
  
  if (pcl::io::loadPCDFile<PointType>(config_.map_file_path, *map_cloud_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map file: %s", config_.map_file_path.c_str());
    return false;
  }
  
  if (map_cloud_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Loaded map is empty");
    return false;
  }
  
  // Downsample map for faster processing
  auto voxelized_map = voxelize(map_cloud_, config_.voxel_resolution);
  *map_cloud_ = *voxelized_map;
  
  map_loaded_ = true;
  RCLCPP_INFO(this->get_logger(), "Map loaded successfully. Points: %zu", map_cloud_->size());
  
  return true;
}

void RelocalizationNode::scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  // Convert ROS message to PCL
  pcl::fromROSMsg(*msg, *current_scan_);
  scan_received_ = true;
  
  if (config_.verbose && !is_localized_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                         "Received scan with %zu points. Waiting for relocalization.", 
                         current_scan_->size());
  }
}

void RelocalizationNode::publishTransform() {
  if (!is_localized_) {
    return;
  }
  
  try {
    // Get current odom->base_link transform
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base = tf_buffer_->lookupTransform(config_.odom_frame, config_.base_frame, 
                                               tf2::TimePointZero, 
                                               tf2::durationFromSec(config_.tf_tolerance));
    
    // Convert to Eigen
    Eigen::Matrix4d odom_to_base_matrix;
    auto translation = odom_to_base.transform.translation;
    auto rotation = odom_to_base.transform.rotation;
    
    Eigen::Vector3d t(translation.x, translation.y, translation.z);
    Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
    
    odom_to_base_matrix.setIdentity();
    odom_to_base_matrix.block<3,3>(0,0) = q.toRotationMatrix();
    odom_to_base_matrix.block<3,1>(0,3) = t;
    
    // Calculate map->base transform
    Eigen::Matrix4d map_to_base = map_to_odom_ * odom_to_base_matrix;
    
    // Publish map->odom transform
    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = this->get_clock()->now();
    map_to_odom_msg.header.frame_id = config_.map_frame;
    map_to_odom_msg.child_frame_id = config_.odom_frame;
    
    // Extract translation and rotation from map_to_odom_
    Eigen::Vector3d translation = map_to_odom_.block<3,1>(0,3);
    Eigen::Matrix3d rotation_matrix = map_to_odom_.block<3,3>(0,0);
    Eigen::Quaterniond quaternion(rotation_matrix);
    
    map_to_odom_msg.transform.translation.x = translation.x();
    map_to_odom_msg.transform.translation.y = translation.y();
    map_to_odom_msg.transform.translation.z = translation.z();
    map_to_odom_msg.transform.rotation.x = quaternion.x();
    map_to_odom_msg.transform.rotation.y = quaternion.y();
    map_to_odom_msg.transform.rotation.z = quaternion.z();
    map_to_odom_msg.transform.rotation.w = quaternion.w();
    
    tf_broadcaster_->sendTransform(map_to_odom_msg);
    
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Could not get odom->base transform: %s", ex.what());
  }
}

void RelocalizationNode::autoRelocalizationCallback() {
  if (is_localized_ || !scan_received_ || !map_loaded_) {
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Attempting automatic relocalization...");
  
  RegOutput result;
  if (performGlobalRelocalization(result)) {
    double confidence = calculateConfidence(result);
    if (confidence >= config_.confidence_threshold) {
      is_localized_ = true;
      last_confidence_ = confidence;
      RCLCPP_INFO(this->get_logger(), 
                  "Automatic relocalization successful! Confidence: %.2f", confidence);
    } else {
      RCLCPP_WARN(this->get_logger(), 
                  "Relocalization failed due to low confidence: %.2f < %.2f", 
                  confidence, config_.confidence_threshold);
    }
  }
}

void RelocalizationNode::relocalizationCallback(
    const std::shared_ptr<kiss_matcher_ros::srv::Relocalize::Request> request,
    std::shared_ptr<kiss_matcher_ros::srv::Relocalize::Response> response) {
  
  if (!map_loaded_) {
    response->success = false;
    response->message = "Map not loaded";
    return;
  }
  
  if (!scan_received_) {
    response->success = false;
    response->message = "No scan data received";
    return;
  }
  
  RegOutput result;
  bool success = false;
  
  // Check if pose hint is provided (non-zero)
  auto& pose_hint = request->pose_hint.pose.pose;
  bool has_hint = (pose_hint.position.x != 0.0 || pose_hint.position.y != 0.0 || 
                   pose_hint.position.z != 0.0 || pose_hint.orientation.w != 1.0);
  
  if (has_hint) {
    RCLCPP_INFO(this->get_logger(), "Performing relocalization with pose hint");
    Eigen::Matrix4d initial_guess = poseToMatrix(pose_hint);
    success = performRelocalization(initial_guess, result);
  } else {
    RCLCPP_INFO(this->get_logger(), "Performing global relocalization");
    success = performGlobalRelocalization(result);
  }
  
  if (success) {
    double confidence = calculateConfidence(result);
    if (confidence >= config_.confidence_threshold) {
      is_localized_ = true;
      last_confidence_ = confidence;
      
      response->success = true;
      response->message = "Relocalization successful";
      response->confidence = confidence;
      
      // Convert result to pose message
      response->pose.header.stamp = this->get_clock()->now();
      response->pose.header.frame_id = config_.map_frame;
      response->pose.pose.pose = matrixToPose(result.pose_);
      
      // Set covariance based on confidence
      double cov_scale = 1.0 - confidence;
      std::fill(response->pose.pose.covariance.begin(), 
                response->pose.pose.covariance.end(), 0.0);
      response->pose.pose.covariance[0] = cov_scale * 0.1;  // x
      response->pose.pose.covariance[7] = cov_scale * 0.1;  // y
      response->pose.pose.covariance[14] = cov_scale * 0.1; // z
      response->pose.pose.covariance[21] = cov_scale * 0.1; // roll
      response->pose.pose.covariance[28] = cov_scale * 0.1; // pitch
      response->pose.pose.covariance[35] = cov_scale * 0.1; // yaw
      
      RCLCPP_INFO(this->get_logger(), 
                  "Relocalization successful! Confidence: %.2f", confidence);
    } else {
      response->success = false;
      response->message = "Relocalization confidence too low: " + std::to_string(confidence);
      response->confidence = confidence;
    }
  } else {
    response->success = false;
    response->message = "Relocalization failed";
    response->confidence = 0.0;
  }
}

void RelocalizationNode::resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  
  (void)request; // Suppress unused parameter warning
  
  is_localized_ = false;
  map_to_odom_ = Eigen::Matrix4d::Identity();
  last_confidence_ = 0.0;
  
  response->success = true;
  response->message = "Localization reset successful";
  
  RCLCPP_INFO(this->get_logger(), "Localization reset");
}

bool RelocalizationNode::performRelocalization(const Eigen::Matrix4d& initial_guess, RegOutput& result) {
  if (!scan_received_ || !map_loaded_) {
    return false;
  }
  
  auto preprocessed_scan = preprocessScan(*current_scan_);
  if (preprocessed_scan->empty()) {
    RCLCPP_WARN(this->get_logger(), "Preprocessed scan is empty");
    return false;
  }
  
  // Transform scan with initial guess
  auto transformed_scan = transformPcd(*preprocessed_scan, initial_guess);
  
  // Use loop closure for coarse-to-fine alignment
  loop_closure_->setSrcAndTgtCloud(transformed_scan, *map_cloud_);
  result = loop_closure_->coarseToFineAlignment(transformed_scan, *map_cloud_);
  
  if (result.is_valid_) {
    // Combine initial guess with refinement
    map_to_odom_ = result.pose_ * initial_guess;
    
    if (config_.publish_aligned_cloud && aligned_cloud_pub_) {
      auto final_aligned = transformPcd(*preprocessed_scan, map_to_odom_);
      publishAlignedCloud(*final_aligned);
    }
    
    if (config_.save_debug_clouds) {
      auto final_aligned = transformPcd(*preprocessed_scan, map_to_odom_);
      saveDebugClouds(*preprocessed_scan, *final_aligned, "hint");
    }
    
    return true;
  }
  
  return false;
}

bool RelocalizationNode::performGlobalRelocalization(RegOutput& result) {
  if (!scan_received_ || !map_loaded_) {
    return false;
  }
  
  auto preprocessed_scan = preprocessScan(*current_scan_);
  if (preprocessed_scan->empty()) {
    RCLCPP_WARN(this->get_logger(), "Preprocessed scan is empty");
    return false;
  }
  
  // Try global registration directly
  loop_closure_->setSrcAndTgtCloud(*preprocessed_scan, *map_cloud_);
  result = loop_closure_->coarseToFineAlignment(*preprocessed_scan, *map_cloud_);
  
  if (result.is_valid_) {
    map_to_odom_ = result.pose_;
    
    if (config_.publish_aligned_cloud && aligned_cloud_pub_) {
      auto aligned = transformPcd(*preprocessed_scan, map_to_odom_);
      publishAlignedCloud(*aligned);
    }
    
    if (config_.save_debug_clouds) {
      auto aligned = transformPcd(*preprocessed_scan, map_to_odom_);
      saveDebugClouds(*preprocessed_scan, *aligned, "global");
    }
    
    return true;
  }
  
  return false;
}

pcl::PointCloud<PointType>::Ptr RelocalizationNode::preprocessScan(const pcl::PointCloud<PointType>& scan) {
  auto filtered_scan = std::make_shared<pcl::PointCloud<PointType>>();
  
  // Range filtering
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(scan.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-config_.max_scan_range, config_.max_scan_range);
  pass.filter(*filtered_scan);
  
  // Remove close points
  auto far_filtered = std::make_shared<pcl::PointCloud<PointType>>();
  for (const auto& point : filtered_scan->points) {
    double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range >= config_.min_scan_range && range <= config_.max_scan_range) {
      far_filtered->push_back(point);
    }
  }
  
  // Voxelize
  return voxelize(far_filtered, config_.voxel_resolution);
}

double RelocalizationNode::calculateConfidence(const RegOutput& result) {
  if (!result.is_valid_) {
    return 0.0;
  }
  
  // Calculate confidence based on overlap percentage and number of inliers
  double overlap_score = std::min(result.overlapness_ / 100.0, 1.0);
  double inlier_score = std::min(static_cast<double>(result.num_final_inliers_) / 1000.0, 1.0);
  
  // Weighted combination
  return 0.7 * overlap_score + 0.3 * inlier_score;
}

void RelocalizationNode::publishAlignedCloud(const pcl::PointCloud<PointType>& cloud) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = this->get_clock()->now();
  cloud_msg.header.frame_id = config_.map_frame;
  aligned_cloud_pub_->publish(cloud_msg);
}

Eigen::Matrix4d RelocalizationNode::poseToMatrix(const geometry_msgs::msg::Pose& pose) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  
  Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, 
                                pose.orientation.y, pose.orientation.z);
  
  matrix.block<3,3>(0,0) = quaternion.toRotationMatrix();
  matrix.block<3,1>(0,3) = translation;
  
  return matrix;
}

geometry_msgs::msg::Pose RelocalizationNode::matrixToPose(const Eigen::Matrix4d& matrix) {
  geometry_msgs::msg::Pose pose;
  
  Eigen::Vector3d translation = matrix.block<3,1>(0,3);
  Eigen::Matrix3d rotation_matrix = matrix.block<3,3>(0,0);
  Eigen::Quaterniond quaternion(rotation_matrix);
  
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  
  return pose;
}

void RelocalizationNode::saveDebugClouds(const pcl::PointCloud<PointType>& scan, 
                                        const pcl::PointCloud<PointType>& aligned,
                                        const std::string& suffix) {
  if (!config_.save_debug_clouds) {
    return;
  }
  
  auto now = this->get_clock()->now();
  std::string timestamp = std::to_string(now.seconds());
  
  std::string scan_filename = "/tmp/debug_scan_" + suffix + "_" + timestamp + ".pcd";
  std::string aligned_filename = "/tmp/debug_aligned_" + suffix + "_" + timestamp + ".pcd";
  std::string map_filename = "/tmp/debug_map_" + suffix + "_" + timestamp + ".pcd";
  
  pcl::io::savePCDFileASCII(scan_filename, scan);
  pcl::io::savePCDFileASCII(aligned_filename, aligned);
  pcl::io::savePCDFileASCII(map_filename, *map_cloud_);
  
  RCLCPP_INFO(this->get_logger(), "Saved debug clouds: %s, %s, %s", 
              scan_filename.c_str(), aligned_filename.c_str(), map_filename.c_str());
}