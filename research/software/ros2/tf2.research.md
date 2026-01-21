# TF2 Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/software/ros2/tf2.mdx

## Overview

TF2 (Transform Library 2) is ROS 2's coordinate frame management system. It tracks the relationships between multiple coordinate frames over time, storing them in a time-buffered tree structure. TF2 enables robots to answer spatial questions like "Where was the gripper relative to the world 5 seconds ago?" or "What is the current pose of the camera in the robot base frame?"

TF2 is the **only** transform library in ROS 2 (unlike ROS 1 which supported both tf and tf2). It operates in a distributed system where all transform information is available to all ROS 2 components on any computer in the network.

## Key Concepts to Cover

### 1. Core Architecture
- **Tree Structure**: Frames form a directed tree (each frame has exactly one parent, except root)
- **Time Buffered**: Stores transforms over time (default 10 seconds of history)
- **Distributed**: All nodes can access transforms from anywhere in the system
- **Two Main Tasks**: Broadcasting transforms and listening for transforms

### 2. TF2 Packages (geometry2 repository)
| Package | Purpose |
|---------|---------|
| `tf2` | Core library, ROS-agnostic, templated datatype support |
| `tf2_ros` | ROS 2 bindings (Buffer, TransformListener, TransformBroadcaster) |
| `tf2_geometry_msgs` | Conversions for geometry_msgs types |
| `tf2_sensor_msgs` | Conversions for sensor_msgs (PointCloud2) |
| `tf2_kdl` | KDL (Kinematics and Dynamics Library) conversions |
| `tf2_eigen` | Eigen library conversions |
| `tf2_tools` | Debugging utilities (view_frames, tf2_echo) |

### 3. Broadcasting Transforms
- **TransformBroadcaster**: Publishes dynamic transforms to `/tf` topic
- **StaticTransformBroadcaster**: Publishes static transforms to `/tf_static` topic (latched, sent once)
- **static_transform_publisher**: CLI/node for publishing static transforms

### 4. Listening for Transforms
- **Buffer**: Stores received transforms, provides lookup API
- **TransformListener**: Subscribes to `/tf` and `/tf_static`, populates buffer
- **lookup_transform()**: Query transform between two frames at a specific time
- **can_transform()**: Check if transform is available

### 5. Time and Transform Lookup
- **Time 0 (tf2::TimePointZero)**: Means "latest available" transform
- **Timeout parameter**: Blocks waiting for transform to become available
- **Time travel**: Query transforms at past timestamps (within buffer history)
- **Advanced API**: 6-parameter lookup for complex time relationships

### 6. Static vs Dynamic Transforms
- **Static** (`/tf_static`): Published once, assumed constant, no time history stored
- **Dynamic** (`/tf`): Continuously published, time-stamped, stored in buffer
- Static transforms save storage, lookup time, and reduce publishing overhead

### 7. Common Exceptions
| Exception | Cause |
|-----------|-------|
| `LookupException` | Frame not in graph (not published or tree broken) |
| `ConnectivityException` | No path between frames in tree |
| `ExtrapolationException` | Requested time outside buffer (future or too far past) |
| `InvalidArgumentException` | Invalid frame ID or arguments |
| `TimeoutException` | Transform not available within timeout |

### 8. Topics
- `/tf` (tf2_msgs/TFMessage): Dynamic transforms
- `/tf_static` (tf2_msgs/TFMessage): Static transforms (transient local QoS)

## Current Version Numbers (January 2026)

### ROS 2 Distributions with TF2
- **Jazzy Jalisco** (LTS until 2029): tf2 version 0.36.x
- **Kilted Kaiju** (stable until Nov 2026): Latest tf2 features
- **Humble Hawksbill** (LTS until 2027): tf2 version 0.25.x
- **Rolling**: Development branch, latest features

### geometry2 Repository
- GitHub: https://github.com/ros2/geometry2
- Contains all tf2_* packages
- Jazzy: tf2 version 0.36.17

### Related Packages
- `tf_transformations`: Euler/quaternion conversions (Python)
- `robot_state_publisher`: Publishes transforms from URDF + joint states

## Code Examples to Include

### 1. Transform Broadcaster (Python)

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class FrameBroadcaster(Node):
    def __init__(self):
        super().__init__('frame_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot_base'

        # Translation
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0

        # Rotation (quaternion from yaw angle)
        q = tf_transformations.quaternion_from_euler(0, 0, 0.5)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
```

### 2. Static Transform Broadcaster (Python)

```python
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # Camera is 10cm forward, 20cm up from base
        t.transform.translation.x = 0.1
        t.transform.translation.z = 0.2
        t.transform.rotation.w = 1.0  # No rotation

        self.tf_static_broadcaster.sendTransform(t)
```

### 3. Transform Listener with Exception Handling (Python)

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            # Get latest transform with 1 second timeout
            transform = self.tf_buffer.lookup_transform(
                'base_link',        # Target frame
                'camera_link',      # Source frame
                rclpy.time.Time(),  # Time 0 = latest
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().info(
                f'Translation: ({transform.transform.translation.x:.2f}, '
                f'{transform.transform.translation.y:.2f}, '
                f'{transform.transform.translation.z:.2f})'
            )
        except LookupException as e:
            self.get_logger().warn(f'Frame not found: {e}')
        except ConnectivityException as e:
            self.get_logger().warn(f'Frames not connected: {e}')
        except ExtrapolationException as e:
            self.get_logger().warn(f'Extrapolation error: {e}')
```

### 4. Transform Listener (C++)

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class FrameListener : public rclcpp::Node {
public:
    FrameListener() : Node("frame_listener") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FrameListener::on_timer, this));
    }

private:
    void on_timer() {
        try {
            auto transform = tf_buffer_->lookupTransform(
                "base_link", "camera_link",
                tf2::TimePointZero,
                tf2::durationFromSec(1.0));
            RCLCPP_INFO(this->get_logger(), "Transform received");
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### 5. Time Travel (Past Transforms)

```python
# Get where camera_link was 5 seconds ago relative to base_link now
from rclpy.time import Time
from rclpy.duration import Duration

now = self.get_clock().now()
past = now - Duration(seconds=5.0)

# Basic past lookup
transform = self.tf_buffer.lookup_transform(
    'base_link',
    'camera_link',
    past.to_msg(),
    timeout=Duration(seconds=1.0)
)

# Advanced 6-parameter lookup for complex time relationships
# "Where was source_frame at source_time, expressed in target_frame at target_time?"
transform = self.tf_buffer.lookup_transform_full(
    'base_link',      # Target frame
    now.to_msg(),     # Target time
    'camera_link',    # Source frame
    past.to_msg(),    # Source time
    'world',          # Fixed frame (reference for time travel)
    timeout=Duration(seconds=1.0)
)
```

### 6. Transforming Points and Poses

```python
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

# Create a pose in camera_link frame
pose = PoseStamped()
pose.header.frame_id = 'camera_link'
pose.header.stamp = self.get_clock().now().to_msg()
pose.pose.position.x = 1.0
pose.pose.orientation.w = 1.0

# Transform to base_link frame
transform = self.tf_buffer.lookup_transform(
    'base_link', 'camera_link', rclpy.time.Time())
pose_in_base = do_transform_pose(pose, transform)
```

## CLI Tools

### view_frames
```bash
# Generate PDF visualization of transform tree
ros2 run tf2_tools view_frames
# Creates frames.pdf and frames.gv in current directory
```

### tf2_echo
```bash
# Print live transform between two frames
ros2 run tf2_ros tf2_echo base_link camera_link

# Output:
# At time 1234567890.123456789
# - Translation: [0.100, 0.000, 0.200]
# - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
#             in RPY (radian) [0.000, -0.000, 0.000]
#             in RPY (degree) [0.000, -0.000, 0.000]
```

### static_transform_publisher
```bash
# Publish static transform from command line
ros2 run tf2_ros static_transform_publisher \
    --x 0.1 --y 0.0 --z 0.2 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id base_link \
    --child-frame-id camera_link

# Alternative with quaternion
ros2 run tf2_ros static_transform_publisher \
    --x 0.1 --y 0.0 --z 0.2 \
    --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 \
    --frame-id base_link \
    --child-frame-id camera_link
```

### tf2_monitor
```bash
# Monitor transform tree health and timing
ros2 run tf2_ros tf2_monitor
```

## NVIDIA Isaac Integration

### Isaac Sim TF Publishing
- Isaac Sim can publish transform trees via ROS 2 bridge
- Access: Tools > Robotics > ROS 2 OmniGraphs > TF Publisher
- Ros2BufferCore class provides tf2 buffer interface
- TF Viewer extension for visualizing transforms

### Isaac ROS and TF2
- Isaac ROS packages use standard TF2 for coordinate frames
- cuVSLAM publishes transforms: map → odom → base_link
- All Isaac ROS perception packages expect properly configured TF tree
- NITROS accelerated packages maintain standard TF compatibility

### Best Practice for Isaac Sim
- Use `robot_state_publisher` for robot static transforms (from URDF)
- Let Isaac Sim publish joint states
- Avoids performance overhead of publishing all TFs from simulation

## Diagrams Needed

1. **TF2 Architecture Diagram**
   ```
   ┌─────────────────────────────────────────────────────────────┐
   │                      Your ROS 2 Nodes                        │
   ├─────────────────────────────────────────────────────────────┤
   │  ┌─────────────────┐          ┌─────────────────────────┐   │
   │  │ TransformBroadcaster │     │   TransformListener     │   │
   │  │ (publishes to /tf)   │     │   (subscribes to /tf)   │   │
   │  └─────────────────┘          └─────────────────────────┘   │
   │           │                              │                   │
   │           ▼                              ▼                   │
   │     ┌──────────┐                   ┌──────────┐              │
   │     │   /tf    │                   │  Buffer  │              │
   │     │/tf_static│                   │ (stores) │              │
   │     └──────────┘                   └──────────┘              │
   ├─────────────────────────────────────────────────────────────┤
   │                    tf2_ros Package                           │
   ├─────────────────────────────────────────────────────────────┤
   │                    tf2 Core Library                          │
   └─────────────────────────────────────────────────────────────┘
   ```

2. **Transform Tree Example**
   ```
   map
    └── odom
         └── base_link
              ├── camera_link
              │    └── camera_optical_frame
              ├── lidar_link
              ├── imu_link
              └── arm_base_link
                   └── arm_link_1
                        └── ... → gripper_link
   ```

3. **Time Buffer Visualization**
   - Show 10-second sliding window of stored transforms
   - Illustrate lookups at different timestamps

## Prerequisites

- [ROS 2](/robotics-glossary/software/frameworks/ros2/) - Core ROS 2 concepts
- [Coordinate Frames](/robotics-glossary/concepts/fundamentals/coordinate-frames/) - Understanding frames
- [Transforms](/robotics-glossary/concepts/fundamentals/transforms/) - Mathematical foundations

## Related Concepts

- [SLAM](/robotics-glossary/concepts/perception/slam/) - Generates map→odom→base_link transforms
- [Isaac ROS](/robotics-glossary/software/isaac/isaac-ros/) - GPU-accelerated ROS 2 packages
- [URDF](/robotics-glossary/software/urdf/) - Robot description that defines static transforms
- [Nav2](/robotics-glossary/software/nav2/) - Navigation stack relies heavily on TF2

## Source URLs for Citations

### Official ROS 2 Documentation
- TF2 Concepts (Jazzy): https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html
- TF2 Concepts (Humble): https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html
- TF2 Introduction Tutorial: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- Writing a Broadcaster (Python): https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
- Writing a Static Broadcaster: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
- Writing a Listener (Python): https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
- Time Travel Tutorial: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Py.html

### API Documentation
- tf2_ros Package: https://docs.ros.org/en/rolling/p/tf2_ros/doc/index.html
- tf2 C++ API (Jazzy): https://docs.ros.org/en/jazzy/p/tf2/generated/index.html
- tf2_ros::Buffer: https://docs.ros2.org/foxy/api/tf2_ros/classtf2__ros_1_1Buffer.html
- tf2_ros::TransformBroadcaster: https://docs.ros2.org/foxy/api/tf2_ros/classtf2__ros_1_1TransformBroadcaster.html

### GitHub Repositories
- geometry2 (ROS 2): https://github.com/ros2/geometry2
- tf2_geometry_msgs Changelog: https://github.com/ros2/geometry2/blob/jazzy/tf2_geometry_msgs/CHANGELOG.rst
- tf_transformations: https://github.com/DLu/tf_transformations

### ROS Package Index
- tf2: https://index.ros.org/p/tf2/
- tf2_geometry_msgs: https://index.ros.org/p/tf2_geometry_msgs/
- tf2_sensor_msgs: https://index.ros.org/p/tf2_sensor_msgs/

### NVIDIA Isaac
- Isaac Sim TF Tutorial: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.html
- Isaac Sim TF Viewer: https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.ros2.tf_viewer/docs/index.html
- Isaac ROS Navigation (TF Setup): https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_navigation.html

### Tutorials and Community
- ROS Industrial TF2 Workshop: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html
- Articulated Robotics TF Tutorial: https://articulatedrobotics.xyz/tutorials/ready-for-ros/tf/
- Automatic Addison TF2 Tutorial: https://automaticaddison.com/how-to-create-a-tf-listener-using-ros-2-and-python/

### Migration and History
- TF vs TF2 Differences: https://answers.ros.org/question/11242/what-is-the-difference-between-tf-and-tf2/
- tf2 Introduction (ROSCon 2013): https://www.osrfoundation.org/wordpress2/wp-content/uploads/2015/04/tf2.pdf

### ROS Enhancement Proposals
- REP-103 (Units and Conventions): https://www.ros.org/reps/rep-0103.html
- REP-105 (Coordinate Frames): https://www.ros.org/reps/rep-0105.html

## Common Pitfalls to Document

1. **Extrapolation into the future**: Using `Time()` (now) without timeout when transform hasn't arrived yet
2. **Mixing simulated and wall clock time**: Causes extrapolation errors
3. **Missing parent frame**: Tree becomes disconnected
4. **Circular dependencies**: TF2 enforces tree structure (will reject)
5. **Not waiting for transforms**: Transforms take time to propagate
6. **Wrong frame order in lookup**: `lookup_transform(target, source)` returns transform to express source points in target frame

## Summary of Entry Structure

### Recommended Sections
1. **Introduction** - What TF2 is and why it matters
2. **Prerequisites** - ROS 2, Coordinate Frames, Transforms concepts
3. **Core Concepts** - Tree structure, time buffering, distributed system
4. **Package Overview** - tf2, tf2_ros, tf2_geometry_msgs, etc.
5. **Broadcasting Transforms** - Static vs Dynamic, Python/C++ examples
6. **Listening for Transforms** - Buffer, Listener, lookup API
7. **Time and Transforms** - Time 0, timeouts, time travel
8. **Exception Handling** - Common errors and solutions
9. **CLI Tools** - view_frames, tf2_echo, static_transform_publisher
10. **NVIDIA Integration** - Isaac Sim, Isaac ROS
11. **Related Terms** - Links
12. **Learn More** - External resources
13. **Sources** - Citations

### Badge
- **Type:** Practical (matches ROS 2, Isaac ROS entries)
- **Variant:** success

### Suggested Links
- Prerequisites: ROS 2, Coordinate Frames, Transforms
- Related: SLAM, Isaac ROS, URDF, Nav2
