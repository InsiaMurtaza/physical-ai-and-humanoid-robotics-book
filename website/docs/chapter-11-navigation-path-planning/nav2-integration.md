---
sidebar_position: 2
---

# Isaac Integration with Navigation2 (Nav2)

## Nav2 Architecture Overview

Navigation2 provides a comprehensive navigation framework for ROS 2 that Isaac enhances through several integration points:

**Global Planner**: Computes optimal paths from start to goal positions using map-based algorithms.

**Local Planner**: Generates safe, collision-free trajectories in real-time while avoiding dynamic obstacles.

**Controller**: Translates planned trajectories into low-level robot commands for motion execution.

**Recovery Behaviors**: Implements strategies for handling navigation failures and challenging situations.

### Nav2 System Architecture

Navigation2 follows a behavior tree architecture that allows for flexible and modular navigation behavior. Here's an example of how to configure and use Nav2 components:

```xml
<!-- Example Nav2 launch file with Isaac enhancements -->
<launch>
  <!-- Arguments -->
  <arg name="namespace" default=""/>
  <arg name="use_sim_time" default="false"/>
  <arg name="autostart" default="true"/>
  <arg name="params_file" default="$(find-pkg-share my_robot_bringup)/config/nav2_params_isaac.yaml"/>
  <arg name="bt_xml_file" default="$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_w_replanning_and_recovery.xml"/>

  <!-- Map Server -->
  <node pkg="nav2_map_server" exec="map_server" name="map_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="params_file" value="$(var params_file)"/>
  </node>

  <!-- Local Costmap -->
  <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="local_costmap" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="params_file" value="$(var params_file)"/>
  </node>

  <!-- Global Costmap -->
  <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="global_costmap" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="params_file" value="$(var params_file)"/>
  </node>

  <!-- Isaac-enhanced Planner Server -->
  <node pkg="isaac_ros_nav" exec="isaac_planner_server" name="planner_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="params_file" value="$(var params_file)"/>
    <param name="gpu_acceleration" value="true"/>
  </node>

  <!-- Isaac-enhanced Controller Server -->
  <node pkg="isaac_ros_nav" exec="isaac_controller_server" name="controller_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="params_file" value="$(var params_file)"/>
    <param name="gpu_acceleration" value="true"/>
  </node>

  <!-- Isaac-enhanced Behavior Tree Navigator -->
  <node pkg="isaac_ros_nav" exec="isaac_bt_navigator" name="bt_navigator" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="bt_xml_filename" value="$(var bt_xml_file)"/>
    <param name="gpu_acceleration" value="true"/>
  </node>

  <!-- Isaac-enhanced Waypoint Follower -->
  <node pkg="isaac_ros_nav" exec="isaac_waypoint_follower" name="waypoint_follower" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="gpu_acceleration" value="true"/>
  </node>

  <!-- Isaac-enhanced Recovery Server -->
  <node pkg="isaac_ros_nav" exec="isaac_recovery_server" name="recovery_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="params_file" value="$(var params_file)"/>
  </node>

  <!-- Isaac-enhanced Lifecycle Manager -->
  <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="autostart" value="$(var autostart)"/>
    <param name="node_names" value="[map_server, local_costmap, global_costmap, planner_server, controller_server, bt_navigator, waypoint_follower, recovery_server]"/>
  </node>
</launch>
```

Here's an example configuration file for Nav2 with Isaac enhancements:

```yaml
# Isaac-enhanced Nav2 configuration (nav2_params_isaac.yaml)
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.2
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: False

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: False

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: False

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Isaac-enhanced FollowPath controller
    FollowPath:
      plugin: "isaac_ros.NavFNPlanner"
      speed_limit_scale: 0.75
      max_velocity: 0.5
      min_velocity: 0.1

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

local_costmap_rclcpp_node:
  ros__parameters:
    use_sim_time: False

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap_rclcpp_node:
  ros__parameters:
    use_sim_time: False

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]

    # Isaac-enhanced GridBased planner
    GridBased:
      plugin: "isaac_ros.NavFNPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      visualize_potential: false

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: False
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      wait_time: 200
```

### Isaac ROS Navigation Integration Example

Here's a more detailed example of how to create a custom navigation node that integrates Isaac's GPU-accelerated capabilities with Nav2:

```python
# Example Isaac-ROS Navigation Integration
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import math
from typing import Optional, Tuple

class IsaacNav2Integrator(Node):
    """
    Node that demonstrates Isaac integration with Navigation2
    """
    def __init__(self):
        super().__init__('isaac_nav2_integrator')

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Create subscribers for sensor data (for Isaac processing)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile)

        # Isaac-specific parameters
        self.declare_parameter('use_gpu_acceleration', True)
        self.declare_parameter('isaac_costmap_resolution', 0.05)
        self.declare_parameter('isaac_planning_algorithm', 'navfn')
        self.declare_parameter('isaac_collision_threshold', 100)

        # Internal state
        self.current_pose = None
        self.latest_scan = None
        self.navigation_goal = None
        self.isaac_costmap = None

        # Isaac processing timer
        self.isaac_timer = self.create_timer(0.1, self.process_isaac_data)

        self.get_logger().info('Isaac-Nav2 Integrator initialized')

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan data"""
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        self.current_pose = msg.pose.pose

    def process_isaac_data(self):
        """Process sensor data using Isaac's GPU-accelerated algorithms"""
        if self.latest_scan is not None and self.current_pose is not None:
            # Process scan data through Isaac's GPU-accelerated pipeline
            self.isaac_costmap = self.generate_isaac_costmap(self.latest_scan)

            # Perform Isaac-enhanced obstacle detection
            obstacles = self.detect_obstacles_with_isaac(self.latest_scan)

            # Update Nav2 costmaps with Isaac-enhanced data
            self.update_nav2_costmaps(obstacles)

    def generate_isaac_costmap(self, scan_data: LaserScan) -> Optional[np.ndarray]:
        """Generate costmap using Isaac's GPU-accelerated algorithms"""
        # This would use Isaac's CUDA-based costmap generation
        # For this example, we'll simulate the process

        # Get parameters
        resolution = self.get_parameter('isaac_costmap_resolution').value

        # Create a costmap based on laser scan data
        # In a real implementation, this would use Isaac's GPU acceleration
        width = int(10.0 / resolution)  # 10m x 10m area
        height = width
        costmap = np.zeros((height, width), dtype=np.uint8)

        # Process laser scan into costmap
        robot_x, robot_y = 0, 0  # Robot at center of costmap
        center_x, center_y = width // 2, height // 2

        for i, range_val in enumerate(scan_data.ranges):
            if not (scan_data.range_min <= range_val <= scan_data.range_max):
                continue

            angle = scan_data.angle_min + i * scan_data.angle_increment
            x = robot_x + range_val * math.cos(angle)
            y = robot_y + range_val * math.sin(angle)

            # Convert to costmap coordinates
            map_x = int(center_x + x / resolution)
            map_y = int(center_y + y / resolution)

            if 0 <= map_x < width and 0 <= map_y < height:
                # Mark obstacle in costmap
                costmap[map_y, map_x] = 254  # Obstacle

        return costmap

    def detect_obstacles_with_isaac(self, scan_data: LaserScan) -> list:
        """Detect obstacles using Isaac's perception algorithms"""
        # This would use Isaac's GPU-accelerated perception
        # For this example, we'll return detected obstacle positions
        obstacles = []

        for i, range_val in enumerate(scan_data.ranges):
            if scan_data.range_min <= range_val <= scan_data.range_max:
                angle = scan_data.angle_min + i * scan_data.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                if range_val < 1.0:  # Obstacle within 1m
                    obstacles.append((x, y, range_val))

        return obstacles

    def update_nav2_costmaps(self, obstacles: list):
        """Update Nav2 costmaps with Isaac-enhanced obstacle information"""
        # In a real implementation, this would publish to Nav2's costmap topics
        # For this example, we'll just log the information
        if obstacles:
            self.get_logger().info(f'Updated costmap with {len(obstacles)} obstacles from Isaac processing')

    def send_navigation_goal(self, x: float, y: float, theta: float = 0.0):
        """Send navigation goal to Nav2 with Isaac enhancements"""
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert theta to quaternion
        goal_pose.pose.orientation = self.euler_to_quaternion(0, 0, theta)

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send goal
        self.navigation_goal = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        self.navigation_goal.add_done_callback(self.navigation_result_callback)

        self.get_logger().info(f'Sent navigation goal to ({x}, {y}, {theta})')
        return True

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}')

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

def main(args=None):
    rclpy.init(args=args)

    nav_integrator = IsaacNav2Integrator()

    # Example: Send a navigation goal after a delay
    def send_example_goal():
        nav_integrator.send_navigation_goal(5.0, 5.0, 0.0)

    # Schedule example goal after 2 seconds
    timer = nav_integrator.create_timer(2.0, send_example_goal)

    try:
        rclpy.spin(nav_integrator)
    except KeyboardInterrupt:
        pass
    finally:
        nav_integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Isaac Enhancements to Nav2

### GPU Acceleration
Isaac provides hardware-accelerated implementations of computationally intensive navigation algorithms, including:

- Costmap generation and updates
- Path planning algorithms (A*, Dijkstra, etc.)
- Local trajectory optimization
- Dynamic obstacle detection and avoidance

Here's an example of how Isaac implements GPU-accelerated costmap generation:

```cpp
// Example of Isaac GPU-accelerated costmap implementation
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <vector>

class IsaacCostmapGenerator {
public:
    IsaacCostmapGenerator(int width, int height, float resolution)
        : width_(width), height_(height), resolution_(resolution) {

        // Allocate GPU memory for costmap
        cudaMalloc(&d_costmap_, width_ * height_ * sizeof(unsigned char));
        cudaMalloc(&d_scan_data_, 360 * sizeof(float)); // Assuming 360 laser beams

        // Create CUDA streams for asynchronous processing
        cudaStreamCreate(&processing_stream_);
    }

    ~IsaacCostmapGenerator() {
        cudaFree(d_costmap_);
        cudaFree(d_scan_data_);
        cudaStreamDestroy(processing_stream_);
    }

    void generateCostmap(const std::vector<float>& laser_ranges,
                        float robot_x, float robot_y, float robot_yaw) {

        // Copy laser scan data to GPU
        cudaMemcpyAsync(d_scan_data_, laser_ranges.data(),
                       laser_ranges.size() * sizeof(float),
                       cudaMemcpyHostToDevice, processing_stream_);

        // Launch GPU kernel for costmap generation
        dim3 blockSize(16, 16);
        dim3 gridSize((width_ + blockSize.x - 1) / blockSize.x,
                     (height_ + blockSize.y - 1) / blockSize.y);

        generateCostmapKernel<<<gridSize, blockSize, 0, processing_stream_>>>(
            d_costmap_, d_scan_data_, width_, height_, resolution_,
            robot_x, robot_y, robot_yaw);

        // Synchronize stream to ensure completion
        cudaStreamSynchronize(processing_stream_);
    }

    void getCostmap(std::vector<unsigned char>& output) {
        output.resize(width_ * height_);
        cudaMemcpy(output.data(), d_costmap_,
                  width_ * height_ * sizeof(unsigned char),
                  cudaMemcpyDeviceToHost);
    }

private:
    int width_, height_;
    float resolution_;
    unsigned char* d_costmap_;
    float* d_scan_data_;
    cudaStream_t processing_stream_;

    __global__ void generateCostmapKernel(unsigned char* costmap,
                                         const float* scan_data,
                                         int width, int height, float resolution,
                                         float robot_x, float robot_y, float robot_yaw) {

        int x = blockIdx.x * blockDim.x + threadIdx.x;
        int y = blockIdx.y * blockDim.y + threadIdx.y;

        if (x >= width || y >= height) return;

        // Convert pixel coordinates to world coordinates
        float world_x = (x - width/2) * resolution;
        float world_y = (y - height/2) * resolution;

        // Initialize costmap cell
        unsigned char cost = 0;

        // Check for obstacles based on laser scan data
        for (int i = 0; i < 360; ++i) {
            float angle = i * M_PI / 180.0 + robot_yaw;
            float range = scan_data[i];

            if (range > 0 && range < 10.0) {  // Valid range reading
                float obs_x = robot_x + range * cos(angle);
                float obs_y = robot_y + range * sin(angle);

                // Calculate distance to this obstacle
                float dist_to_obs = sqrt(pow(world_x - obs_x, 2) + pow(world_y - obs_y, 2));

                // Add cost based on proximity to obstacle
                if (dist_to_obs < 0.5) {  // 0.5m clearance
                    cost = max(cost, (unsigned char)(254 * (1.0 - dist_to_obs / 0.5)));
                }
            }
        }

        // Apply inflation to costmap
        costmap[y * width + x] = cost;
    }
};

// Example usage in ROS 2 node
class IsaacCostmapNode : public rclcpp::Node {
public:
    IsaacCostmapNode() : Node("isaac_costmap_node") {
        // Initialize Isaac costmap generator
        costmap_gen_ = std::make_unique<IsaacCostmapGenerator>(1000, 1000, 0.05); // 5cm resolution

        // Create subscription to laser scan
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->processScan(msg);
            });

        // Create publisher for costmap
        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "isaac_costmap", 10);
    }

private:
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // Convert ROS LaserScan to vector
        std::vector<float> ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());

        // Get robot pose (simplified - in real implementation, get from TF)
        float robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;

        // Generate costmap using Isaac GPU acceleration
        costmap_gen_->generateCostmap(ranges, robot_x, robot_y, robot_yaw);

        // Get the generated costmap
        std::vector<unsigned char> costmap_data;
        costmap_gen_->getCostmap(costmap_data);

        // Publish as OccupancyGrid message
        auto grid_msg = nav_msgs::msg::OccupancyGrid();
        grid_msg.header.stamp = this->get_clock()->now();
        grid_msg.header.frame_id = "map";
        grid_msg.info.resolution = 0.05;
        grid_msg.info.width = 1000;
        grid_msg.info.height = 1000;
        grid_msg.info.origin.position.x = -25.0;  // Center of map
        grid_msg.info.origin.position.y = -25.0;
        grid_msg.data = std::vector<int8_t>(costmap_data.begin(), costmap_data.end());

        costmap_publisher_->publish(grid_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    std::unique_ptr<IsaacCostmapGenerator> costmap_gen_;
};
```

### Advanced Perception Integration
Isaac's perception capabilities enhance navigation through:

- Real-time obstacle detection and classification
- Semantic mapping for improved path planning
- Multi-sensor fusion for robust navigation
- Dynamic environment modeling

### Simulation Integration
Isaac Sim provides:

- Realistic navigation testing environments
- Synthetic sensor data for perception system training
- Multi-robot navigation scenario simulation
- Safety-critical navigation validation