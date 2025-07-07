# KISS-Matcher ROS Relocalization

This package provides a ROS 2 relocalization node that can determine a robot's position in a prior PCD map using a two-stage approach:
1. **Coarse localization** using KISS-Matcher for global registration
2. **Fine localization** using small-gicp for precise alignment

The node provides map->odom transformation compatible with the nav2 navigation stack.

## Features

- **Global relocalization**: Determine position without initial pose hint
- **Pose-hinted relocalization**: Refine position from approximate initial guess  
- **Real-time TF broadcasting**: Publishes map->odom transform for navigation
- **Service interface**: Trigger relocalization on demand
- **Auto-relocalization**: Automatically attempt relocalization on startup
- **Confidence scoring**: Evaluates localization quality
- **Visualization**: Publishes aligned point clouds for debugging

## Quick Start

### 1. Prepare Your Map

Save your map as a PCD file:
```bash
# Example: save map from running SLAM
ros2 service call /save_map std_srvs/srv/Trigger
```

### 2. Configure Parameters

Edit `config/relocalization_config.yaml`:
```yaml
/**:
  ros__parameters:
    map_file_path: "/path/to/your/map.pcd"
    scan_topic: "/velodyne_points"
    # ... other parameters
```

### 3. Launch Relocalization Node

```bash
ros2 launch kiss_matcher_ros relocalization.launch.py \
  map_file:=/path/to/your/map.pcd \
  scan_topic:=/velodyne_points
```

## Usage

### Service Interface

**Relocalize with pose hint:**
```bash
ros2 service call /relocalize kiss_matcher_ros/srv/Relocalize \
  "{pose_hint: {
     header: {frame_id: 'map'}, 
     pose: {
       pose: {
         position: {x: 1.0, y: 2.0, z: 0.0},
         orientation: {w: 1.0}
       }
     }
   }}"
```

**Global relocalization (no hint):**
```bash
ros2 service call /relocalize kiss_matcher_ros/srv/Relocalize "{}"
```

**Reset localization:**
```bash
ros2 service call /reset_localization std_srvs/srv/Trigger
```

### Topics

- **Subscribed:**
  - `/velodyne_points` (sensor_msgs/PointCloud2): Input lidar scan
  
- **Published:**
  - `/aligned_scan` (sensor_msgs/PointCloud2): Aligned scan for visualization
  - `/tf` (tf2_msgs/TFMessage): map->odom transform

### Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `map_file_path` | Path to prior PCD map | "" |
| `scan_topic` | Input point cloud topic | "/velodyne_points" |
| `confidence_threshold` | Minimum confidence to accept result | 0.7 |
| `voxel_resolution` | Downsampling resolution | 0.3 |
| `enable_auto_relocalization` | Auto-relocalize on startup | false |

## Integration with Nav2

The relocalization node publishes the `map->odom` transform required by nav2. Ensure your robot's odometry publishes `odom->base_link`.

**Launch with nav2:**
```bash
# Terminal 1: Start relocalization
ros2 launch kiss_matcher_ros relocalization.launch.py map_file:=/path/to/map.pcd

# Terminal 2: Trigger relocalization  
ros2 service call /relocalize kiss_matcher_ros/srv/Relocalize "{}"

# Terminal 3: Start nav2
ros2 launch nav2_bringup navigation_launch.py map:=/path/to/nav2_map.yaml
```

## Algorithm Details

### Coarse Registration (KISS-Matcher)
- Global 6-DOF registration without initial guess
- Robust to large pose uncertainties
- Fast execution suitable for real-time use

### Fine Registration (small-gicp)
- Refines coarse alignment using ICP variants
- Higher precision alignment
- Overlap-based quality assessment

### Confidence Scoring
Combines multiple metrics:
- Overlap percentage from ICP alignment
- Number of inlier correspondences
- Registration convergence status

## Troubleshooting

**"Map not loaded"**
- Check `map_file_path` parameter
- Ensure PCD file exists and is readable
- Verify file format (PCL-compatible PCD)

**"No scan data received"**
- Check `scan_topic` parameter matches your lidar topic
- Verify lidar is publishing point clouds
- Check topic data type (should be sensor_msgs/PointCloud2)

**"Relocalization confidence too low"**
- Try providing a pose hint closer to actual position
- Adjust `confidence_threshold` parameter
- Check if scan and map represent similar environments
- Verify proper voxel resolution for your data

**Poor localization accuracy**
- Decrease `voxel_resolution` for finer details
- Increase `fine_reg.max_num_iter` for more ICP iterations
- Adjust `fine_reg.overlap_threshold` based on expected overlap

## Performance Tips

- Use appropriate voxel resolution (0.1-0.5m for outdoor, 0.05-0.2m for indoor)
- Ensure map and scan cover overlapping areas
- Consider map size - very large maps may require spatial indexing
- Monitor confidence scores to assess localization quality