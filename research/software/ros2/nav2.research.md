# Nav2 Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/software/ros2/nav2.mdx

## Overview

**Nav2 (Navigation2)** is ROS 2's professional-grade navigation framework for autonomous mobile robots. It is the successor to the ROS 1 Navigation Stack, redesigned from the ground up for ROS 2 with modern architecture, pluggable components, and behavior tree-based orchestration. Nav2 enables robots to navigate through complex environments, avoiding obstacles and reaching specified goals.

Nav2 is trusted by 100+ companies worldwide and is used in both research and industrial applications. It provides perception, planning, control, localization, visualization, and behavior management to build highly reliable autonomous systems.

## Key Concepts to Cover

### 1. Core Architecture

Nav2 uses a modular, server-based architecture where specialized lifecycle nodes communicate via ROS 2 actions and services:

| Server | Purpose |
|--------|---------|
| `planner_server` | Computes global path from start to goal using global costmap |
| `controller_server` | Follows the planned path, generates velocity commands using local costmap |
| `behavior_server` | Executes recovery behaviors (spin, wait, backup) |
| `smoother_server` | Smooths planned paths for better trajectory quality |
| `bt_navigator` | Orchestrates navigation using behavior trees |
| `waypoint_follower` | Navigates through multiple waypoints |
| `velocity_smoother` | Smooths velocity commands to reduce jerk |

### 2. Behavior Trees

- **Industry-standard orchestration**: Uses BehaviorTree.CPP library (v4 in Jazzy)
- **XML-based configuration**: Trees defined in XML files, loaded at runtime
- **Modular nodes**: Action, Condition, Control, and Decorator nodes
- **Default tree**: `navigate_to_pose_w_replanning_and_recovery.xml`
- **Recovery subtree**: Handles system-level failures with costmap clearing, spinning, waiting, backing up
- **Contextual recoveries**: Planner and controller have their own recovery logic
- **Goal updates**: `GoalUpdated` node enables responsive preemption during recovery

### 3. Costmap 2D

The environmental representation used by planners and controllers:

- **2D grid-based costmap**: Each cell has a cost value (0-254, 255 = lethal)
- **Layer architecture**: Multiple plugins combined into final costmap
- **Standard layers**:
  - `static_layer`: Map from map_server (/map topic)
  - `obstacle_layer`: Dynamic obstacles from sensor data
  - `inflation_layer`: Inflates obstacles for safety margin based on robot footprint
  - `voxel_layer`: 3D obstacle representation projected to 2D
- **Global costmap**: Used by planner, typically larger, updated less frequently
- **Local costmap**: Used by controller, rolling window around robot, updated frequently
- **Costmap filters**: Spatial-dependent features (keepout zones, speed limits, preferred lanes)

### 4. Planners

Global path planning algorithms (plugins for `planner_server`):

| Planner | Algorithm | Robot Types | Notes |
|---------|-----------|-------------|-------|
| `NavFn` | Dijkstra/A* | Circular robots | Legacy, from ROS 1 |
| `SmacPlanner2D` | Cost-aware A* | Differential, omnidirectional | 8-connected grid |
| `SmacPlannerHybrid` | Hybrid-A* | Ackermann, legged | Dubin/Reeds-Shepp models, respects min turning radius |
| `SmacPlannerLattice` | State Lattice | All types | Pre-computed motion primitives |
| `ThetaStar` | Theta* | Circular robots | Any-angle pathfinding |

### 5. Controllers

Local trajectory following (plugins for `controller_server`):

| Controller | Type | Robot Types | Notes |
|------------|------|-------------|-------|
| `DWB` | Dynamic Window | Differential, omnidirectional | Plugin-based critics, highly configurable |
| `MPPI` | Model Predictive | Differential, omnidirectional, Ackermann | Sampling-based optimization, GPU-friendly, recommended |
| `TEB` | Timed Elastic Band | All types | Time-optimal MPC |
| `RPP` | Regulated Pure Pursuit | Differential, Ackermann, legged | Industrial/service robots |

**MPPI vs DWB:**
- MPPI is more modern, optimization-based, handles dynamic obstacles better
- MPPI runs at 100+ Hz on modest Intel processors (4th gen i5)
- DWB is more configurable but requires more tuning
- MPPI recommended for most new applications

### 6. Localization

Two primary approaches:

**AMCL (Adaptive Monte Carlo Localization):**
- Particle filter-based localization on known map
- Publishes `map → odom` transform
- Requires pre-built map from map_server
- Good out-of-box experience for beginners
- Configuration: `nav2_amcl` package

**SLAM Toolbox (Localization Mode):**
- Elastic pose-graph localization
- Can replace AMCL entirely
- No need for .pgm maps
- Set `mode: localization` in config
- Uses Ceres optimizer for performance

### 7. SLAM Integration

**SLAM Toolbox:**
- Official Nav2-supported SLAM library
- Packages: `slam_toolbox`
- Modes: online_async, online_sync, offline
- Publishes to `/map` topic and `map → odom` transform
- Launch: `ros2 launch slam_toolbox online_async_launch.py`

**Navigating while Mapping:**
- Can run Nav2 with SLAM simultaneously
- Real-time mapping and navigation
- Useful for exploration tasks

### 8. Robot Kinematics Support

Nav2 supports all major robot types:

- **Differential drive**: Two-wheel robots (TurtleBot, etc.)
- **Omnidirectional/Holonomic**: Mecanum wheels, omni wheels
- **Ackermann**: Car-like steering (minimum turning radius)
- **Legged**: Walking robots with similar kinematic constraints

Support varies by planner/controller plugin selection.

### 9. Recovery Behaviors

Built-in behaviors in `behavior_server`:

| Behavior | Action |
|----------|--------|
| `Spin` | Rotate in place to clear surroundings |
| `Wait` | Pause and wait (for dynamic obstacles to clear) |
| `BackUp` | Drive backwards a specified distance |
| `DriveOnHeading` | Drive forward a specified distance |
| `AssistedTeleop` | Semi-autonomous teleoperation |

Default recovery sequence: clear costmap → spin → wait → backup → retry navigation

### 10. TF2 Requirements

Nav2 requires a properly configured TF tree per REP-105:

```
map
 └── odom
      └── base_link
           ├── [sensor frames]
           └── [robot links]
```

- `map → odom`: Provided by localization (AMCL, SLAM Toolbox)
- `odom → base_link`: Provided by odometry source
- `base_link → sensors`: Provided by robot_state_publisher (from URDF)

## Current Version Numbers (January 2026)

### ROS 2 Distributions with Nav2

| Distribution | Nav2 Status | Support End |
|--------------|-------------|-------------|
| **Jazzy Jalisco** | Full support, BT.CPP v4, GZ Harmonic | May 2029 |
| **Kilted Kaiju** | Latest features | November 2026 |
| **Humble Hawksbill** | Stable, LTS | May 2027 |
| **Rolling** | Development branch | Ongoing |

### Key Changes in Jazzy Release (June 2024)
- Upgrade to BehaviorTree.CPP v4 (from v3)
- Upgrade to Gazebo Harmonic (from Gazebo Classic)
- TwistStamped publication option throughout stack
- Autodocking server and integration
- `plugin_lib_names` auto-includes Nav2 BT libraries

### Installation
```bash
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
```

## Code Examples to Include

### 1. Basic Nav2 Launch Configuration

```python
# nav2_params.yaml - Key parameters overview
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_nav_to_pose_bt_xml: ''  # Uses built-in default
    plugin_lib_names: []  # Custom BT plugins only (Nav2 auto-included in Jazzy)

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # MPPI parameters...

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      # SmacPlanner parameters...
```

### 2. Navigate to Pose (Python Client)

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def go_to_pose(self, x: float, y: float, yaw: float = 0.0):
        """Send robot to specified pose."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Convert yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete!')

def main():
    rclpy.init()
    client = Nav2Client()
    client.go_to_pose(2.0, 1.0, 0.5)  # x=2m, y=1m, yaw=0.5rad
    rclpy.spin(client)
```

### 3. Navigate Through Poses (Waypoints)

```python
from nav2_msgs.action import NavigateThroughPoses

def navigate_waypoints(self, waypoints: list):
    """Navigate through multiple waypoints."""
    goal = NavigateThroughPoses.Goal()
    goal.poses = []

    for wp in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.orientation.w = 1.0
        goal.poses.append(pose)

    self.nav_client.wait_for_server()
    self.nav_client.send_goal_async(goal)
```

### 4. Costmap Configuration

```yaml
# Global costmap
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
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
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

# Local costmap
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["voxel_layer", "inflation_layer"]
```

### 5. MPPI Controller Configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.5
      wz_max: 1.9
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      motion_model: "DiffDrive"  # or "Omni", "Ackermann"
      critics: ["ConstraintCritic", "GoalCritic", "GoalAngleCritic",
                "PreferForwardCritic", "ObstaclesCritic", "PathFollowCritic",
                "PathAngleCritic", "PathAlignCritic"]
```

### 6. Custom Behavior Tree (XML)

```xml
<!-- custom_nav_bt.xml - Simple navigation with recovery -->
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap"
                                service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap"
                                service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5.0"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### 7. C++ Action Client

```cpp
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class Nav2Client : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2Client() : Node("nav2_client") {
        client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");
    }

    void send_goal(double x, double y) {
        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->get_clock()->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.w = 1.0;

        auto send_goal_options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&Nav2Client::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal, send_goal_options);
    }

private:
    void result_callback(const GoalHandleNav::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Goal aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(get_logger(), "Goal canceled");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};
```

## CLI Tools

### Navigation Commands
```bash
# Launch Nav2 with default parameters
ros2 launch nav2_bringup navigation_launch.py

# Launch with custom params
ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/params.yaml

# Launch full simulation (with localization and map server)
ros2 launch nav2_bringup tb3_simulation_launch.py

# Launch with SLAM
ros2 launch nav2_bringup navigation_launch.py slam:=True
```

### Send Goals via CLI
```bash
# Send NavigateToPose goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# Check action server status
ros2 action info /navigate_to_pose
```

### Lifecycle Management
```bash
# Check lifecycle state of Nav2 nodes
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server

# Manually activate nodes (usually done by lifecycle_manager)
ros2 lifecycle set /controller_server activate
```

### Costmap Inspection
```bash
# View costmap in RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# Clear costmaps
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

## NVIDIA Isaac Integration

### Isaac ROS Nvblox for Nav2

**Purpose:** GPU-accelerated 3D reconstruction and costmap generation

**Architecture:**
- Takes depth images + pose as input
- Computes 3D scene reconstruction using TSDF (Truncated Signed Distance Function)
- Outputs 2D costmap directly to Nav2 via costmap layer plugin
- Replaces traditional Voxel Layer, Obstacle Layer, etc.

**Key Benefits:**
- GPU-accelerated obstacle detection
- Better detection of thin/low obstacles (forklift tines)
- 3D voxel grid projected to 2D costmap
- Distance field useful for path planning collision checking

**Package:** `isaac_ros_nvblox`
- `nvblox_nav2`: Costmap2D plugin for Nav2 integration
- `nvblox_ros`: ROS 2 wrapper for reconstruction
- Platform: Jetson or x86_64 with NVIDIA GPU

**Integration:**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["nvblox_layer", "inflation_layer"]
      nvblox_layer:
        plugin: "nvblox::NvbloxCostmapLayer"
        enabled: True
```

### Vision-Based Navigation with Isaac Perceptor

**Lidar-Free Navigation:**
- Uses only passive stereo cameras (no LiDAR, no active depth)
- Powered by NVIDIA Jetson, Isaac ROS, Isaac Perceptor, Nova
- cuVSLAM for visual localization and mapping
- cuVGL (Visual Global Localization) for initial pose estimation
- nvblox for obstacle detection and costmap

**Components:**
- `cuVSLAM`: Visual SLAM for pose estimation (map → odom → base_link)
- `nvblox`: 3D reconstruction and costmap generation
- `Isaac Perceptor`: Integrated perception pipeline

### Isaac Sim Integration

**Simulation Setup:**
- Access: Window > Examples > Robotics Examples > ROS2 > Navigation > Nova Carter
- Generate occupancy map using Occupancy Map Generator extension
- Set navigation goals via Navigation2 Goal button in Isaac Sim

**Features:**
- Full Nav2 stack runs alongside Isaac Sim
- Dynamic obstacle avoidance (obstacles not in initial map)
- Supports TF publishing via ROS 2 bridge

### Autonomous Docking (Nav2 + Isaac ROS)

- Nav2 docking feature with Isaac ROS perception
- Custom pose detection algorithms for infrastructure docking
- Uses Isaac ROS for enhanced perception capabilities

## Diagrams Needed

### 1. Nav2 Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        BT Navigator (Behavior Tree)                  │
│         ┌─────────────────────────────────────────────────┐         │
│         │           navigate_to_pose_bt.xml               │         │
│         └─────────────────────────────────────────────────┘         │
└───────────────────────────────────────────────────────────────────────┘
              │                    │                    │
              ▼                    ▼                    ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Planner Server │    │Controller Server│    │ Behavior Server │
│  ┌───────────┐  │    │  ┌───────────┐  │    │  ┌───────────┐  │
│  │  NavFn    │  │    │  │   MPPI    │  │    │  │   Spin    │  │
│  │  Smac2D   │  │    │  │   DWB     │  │    │  │   Wait    │  │
│  │  Hybrid-A*│  │    │  │   TEB     │  │    │  │  Backup   │  │
│  └───────────┘  │    │  └───────────┘  │    │  └───────────┘  │
│        ▲        │    │        ▲        │    └─────────────────┘
│        │        │    │        │        │
│ Global Costmap  │    │ Local Costmap   │
└─────────────────┘    └─────────────────┘
```

### 2. Costmap Layer Stack

```
┌─────────────────────────────────────────┐
│            Final Costmap                 │
├─────────────────────────────────────────┤
│         Inflation Layer                  │  ← Expands obstacles
├─────────────────────────────────────────┤
│         Obstacle Layer                   │  ← Dynamic obstacles (sensors)
├─────────────────────────────────────────┤
│          Static Layer                    │  ← Pre-built map
└─────────────────────────────────────────┘
        Combined via plugin system
```

### 3. Behavior Tree Flow

```
                    ┌──────────────┐
                    │ NavigateToPose│
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │RecoveryNode  │ (6 retries)
                    └──────┬───────┘
              ┌────────────┴────────────┐
              │                         │
      ┌───────▼───────┐         ┌──────▼──────┐
      │  Navigation   │         │  Recovery   │
      │   Subtree     │         │   Subtree   │
      └───────┬───────┘         └──────┬──────┘
              │                        │
    ┌─────────┴─────────┐    ┌─────────┴─────────┐
    │ ComputePathToPose │    │  Clear Costmaps   │
    │    FollowPath     │    │  Spin │ Wait      │
    └───────────────────┘    │  Backup           │
                             └───────────────────┘
```

## Prerequisites

- [ROS 2](/robotics-glossary/software/frameworks/ros2/) - Core middleware
- [TF2](/robotics-glossary/software/ros2/tf2/) - Transform library (critical for Nav2)
- [Coordinate Frames](/robotics-glossary/concepts/fundamentals/coordinate-frames/) - REP-105 conventions
- [SLAM](/robotics-glossary/concepts/perception/slam/) - For map creation/localization

## Related Concepts

- [Isaac ROS](/robotics-glossary/software/isaac/isaac-ros/) - GPU-accelerated perception for Nav2
- [Isaac Sim](/robotics-glossary/software/simulation/isaac-sim/) - Simulation environment
- [LiDAR](/robotics-glossary/hardware/sensors/lidar/) - Common sensor for obstacle detection
- [Motion Planning](/robotics-glossary/concepts/control/motion-planning/) - Path planning foundations
- [PID](/robotics-glossary/concepts/control/pid/) - Control theory basics

## Source URLs for Citations

### Official Nav2 Documentation
- Nav2 Home: https://docs.nav2.org/
- Navigation Concepts: https://docs.nav2.org/concepts/index.html
- Configuration Guide: https://docs.nav2.org/configuration/
- Behavior Trees: https://docs.nav2.org/behavior_trees/index.html
- BT Walkthrough: https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html
- Tuning Guide: https://docs.nav2.org/tuning/index.html

### Plugin Documentation
- Planner Server: https://docs.nav2.org/configuration/packages/configuring-planner-server.html
- Controller Server: https://docs.nav2.org/configuration/packages/configuring-controller-server.html
- Costmap 2D: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- MPPI Controller: https://docs.nav2.org/configuration/packages/configuring-mppic.html
- Smac Planner: https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- AMCL: https://docs.nav2.org/configuration/packages/configuring-amcl.html

### GitHub
- Navigation2 Repository: https://github.com/ros-navigation/navigation2
- Nav2 Issues: https://github.com/ros-navigation/navigation2/issues

### NVIDIA Isaac Integration
- Isaac ROS Nvblox: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html
- Nvblox GitHub: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
- Nav2 with Isaac ROS GEMs: https://github.com/NVIDIA-AI-IOT/Nav2-with-Isaac-ROS-GEMs
- Isaac Integration Blog: https://developer.nvidia.com/blog/integrating-the-nav2-stack-with-nvidia-isaac-ros-gems/
- Vision-Based Navigation Tutorial: https://docs.nav2.org/tutorials/docs/using_isaac_perceptor.html
- Isaac Sim Nav2 Tutorial: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_navigation.html

### SLAM and Localization
- SLAM Toolbox GitHub: https://github.com/SteveMacenski/slam_toolbox
- Mapping and Localization Guide: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- Navigating While Mapping: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

### Tutorials and Community
- Foxglove Nav2 Tutorial: https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps
- Automatic Addison Guide: https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/
- Robotics Backend Tutorial: https://roboticsbackend.com/ros2-nav2-tutorial/
- Husarion Navigation Tutorial: https://husarion.com/tutorials/ros2-tutorials/9-navigation/

### ROS 2 General
- ROS 2 Releases: https://docs.ros.org/en/jazzy/Releases.html
- REP-105 (Coordinate Frames): https://www.ros.org/reps/rep-0105.html
- Nav2 Jazzy Release Announcement: https://discourse.openrobotics.org/t/nav2-jazzy-release/38321

### Conference Materials
- MPPI Controller ROSCon 2023: https://roscon.ros.org/2023/talks/On_Use_of_Nav2_MPPI_Controller.pdf

## Common Pitfalls to Document

1. **TF tree not configured properly**: Nav2 requires map → odom → base_link chain
2. **Missing sensor data**: Costmap layers need proper topic configuration
3. **Timing issues**: Transforms must be available before navigation starts
4. **Wrong frame IDs**: Goals must be in correct frame (usually 'map')
5. **Costmap not clearing**: Obstacles remain after they move (check clearing configuration)
6. **Recovery loop**: Robot stuck in recovery if underlying issue not resolved
7. **Planner/controller mismatch**: Using circular robot planner with Ackermann robot
8. **MPPI batch size too small**: Poor trajectory quality with small batch sizes

## Summary of Entry Structure

### Recommended Sections
1. **Introduction** - What Nav2 is and why it matters for autonomous robots
2. **Prerequisites** - ROS 2, TF2, Coordinate Frames, basic SLAM understanding
3. **Core Architecture** - Servers, their roles, plugin system
4. **Behavior Trees** - Orchestration, default tree, recovery logic
5. **Costmap 2D** - Environmental representation, layers, configuration
6. **Planning** - Available planners, when to use each
7. **Control** - Controllers, MPPI vs DWB comparison
8. **Localization** - AMCL, SLAM Toolbox modes
9. **Code Examples** - Python client, configuration examples
10. **CLI Tools** - Launch commands, debugging
11. **NVIDIA Isaac Integration** - Nvblox, Isaac Perceptor, Isaac Sim
12. **Related Terms** - Links
13. **Learn More** - External resources
14. **Sources** - Citations

### Badge
- **Type:** Practical
- **Variant:** success

### Suggested Links
- Prerequisites: ROS 2, TF2, Coordinate Frames, SLAM
- Related: Isaac ROS, Isaac Sim, LiDAR, Motion Planning
