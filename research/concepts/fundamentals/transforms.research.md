# Transforms Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/concepts/fundamentals/transforms.mdx

## Overview

Coordinate transformations are the mathematical foundation of robotics, enabling robots to understand spatial relationships between different reference frames (e.g., world, robot base, sensors, end-effector). Transforms describe both position (translation) and orientation (rotation) of one coordinate frame relative to another.

## Key Concepts to Cover

### 1. Coordinate Frames
- A frame is a coordinate system with an origin and orientation (x, y, z axes)
- Every robot component, sensor, and object has its own frame
- ROS convention: x-forward, y-left, z-up (right-hand rule, REP-103)
- Geographic convention: East-North-Up (ENU)
- Camera convention: z-forward, y-down (optical frame)

### 2. Homogeneous Transformation Matrices (SE(3))
- 4x4 matrices encoding rotation (3x3) + translation (3x1)
- Structure:
  ```
  T = | R  p |    where R ∈ SO(3), p ∈ ℝ³
      | 0  1 |
  ```
- Part of Special Euclidean group SE(3) - the group of rigid body motions
- Key properties:
  - Closure: T₁ × T₂ ∈ SE(3)
  - Inverse exists: T⁻¹ represents reverse motion
  - Non-commutative: T₁ × T₂ ≠ T₂ × T₁
- Enables chaining transforms by matrix multiplication

### 3. Rotation Representations
| Representation | Size | Pros | Cons |
|---------------|------|------|------|
| Rotation Matrix | 3x3 (9 values) | Fast composition, direct use | Redundant, drift |
| Euler Angles | 3 values | Intuitive | Gimbal lock, 24 conventions |
| Quaternion | 4 values | Compact, no gimbal lock, smooth interpolation | Less intuitive |
| Axis-Angle | 4 values | Intuitive for single rotations | Less efficient composition |
| Rodrigues | 3 values | Most compact | Singularity at π |

### 4. Gimbal Lock Problem
- Loss of one DOF when two rotation axes align
- Occurs with Euler angles at certain configurations (e.g., pitch = ±90°)
- Solution: Use quaternions for internal representation
- Rule: "Read Euler angles, code with quaternions"

### 5. ROS 2 TF2 System
- **tf2**: The transform library for ROS 2
- Maintains a tree of transforms buffered over time
- Key concepts:
  - **TransformBroadcaster**: Publishes transforms
  - **TransformListener**: Receives and buffers transforms
  - **Buffer**: Stores transforms, provides lookup
  - **Static vs Dynamic**: `/tf_static` vs `/tf` topics
- Standard frames (REP-105):
  - `map`: Global, world-fixed frame
  - `odom`: Odometry frame (continuous but drifts)
  - `base_link`: Robot body frame
  - `sensor_frame`: Individual sensor frames

### 6. URDF and Transforms
- URDF defines static transforms between robot links
- `<joint>` elements define frame relationships
- `robot_state_publisher` broadcasts transforms from URDF + joint states
- Frame naming conventions (REP-103, REP-105, REP-199)

## Current Version Numbers (January 2026)

### ROS 2 TF2
- **ROS 2 Kilted Kaiju**: Latest distribution (May 2025)
- **ROS 2 Jazzy Jalisco**: LTS (May 2024)
- **tf2_ros package**: Part of geometry2 repository
- **geometry_tutorials**: https://github.com/ros/geometry_tutorials

### NVIDIA Isaac ROS
- **Isaac ROS 4.0.1**: December 10, 2025
- **cuVSLAM 14**: Uses transform frames extensively
  - `map_frame`, `odom_frame`, `base_frame`, `camera_optical_frames`
- Supports ROS 2 Jazzy with Humble compatibility

### SciPy Spatial Transform
- **SciPy 1.16.0+**: Added `RigidTransform` class (SE(3))
- **SciPy 1.17.0**: Framework-agnostic (JAX, PyTorch, CuPy support)
- `scipy.spatial.transform.Rotation`: Full rotation support

### Eigen (C++)
- **Eigen 3.4**: Current stable
- `Eigen::Isometry3d`: 4x4 rigid transform
- `Eigen::Quaterniond`: Unit quaternion
- `Eigen::AngleAxisd`: Axis-angle representation

## Code Examples to Include

### 1. Python: Creating and Composing Transforms

```python
import numpy as np
from scipy.spatial.transform import Rotation, RigidTransform

# Create a rotation (90° about Z-axis)
rot = Rotation.from_euler('z', 90, degrees=True)

# Create a rigid transform (rotation + translation)
transform = RigidTransform.from_components(
    rotation=rot,
    translation=[1.0, 0.0, 0.5]
)

# Apply to a point
point = np.array([1.0, 0.0, 0.0])
transformed_point = transform.apply(point)

# Compose transforms (T_world_gripper = T_world_base @ T_base_gripper)
T_world_gripper = T_world_base @ T_base_gripper

# Inverse transform
T_gripper_world = T_world_gripper.inv()
```

### 2. ROS 2: Static Transform Publisher (Launch File)

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.1',
                '--y', '0.0',
                '--z', '0.2',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        ),
    ])
```

### 3. ROS 2: Transform Broadcaster (Python)

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class DynamicBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot_base'

        # Set translation
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0

        # Set rotation (quaternion)
        q = tf_transformations.quaternion_from_euler(0, 0, 0.5)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)
```

### 4. ROS 2: Transform Listener (Python)

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class TransformLookup(Node):
    def __init__(self):
        super().__init__('transform_lookup')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_transform(self, target_frame, source_frame):
        try:
            # Wait up to 1 second for transform
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            return transform
        except Exception as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
            return None
```

### 5. C++: Eigen Transforms

```cpp
#include <Eigen/Geometry>

// Create an isometry (rigid transform)
Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

// Set rotation (90° about Z-axis)
T.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));

// Set translation
T.pretranslate(Eigen::Vector3d(1.0, 0.0, 0.5));

// Apply to a point
Eigen::Vector3d point(1.0, 0.0, 0.0);
Eigen::Vector3d transformed = T * point;

// Get rotation matrix and quaternion
Eigen::Matrix3d R = T.rotation();
Eigen::Quaterniond q(T.rotation());

// Compose transforms
Eigen::Isometry3d T_result = T1 * T2;

// Inverse
Eigen::Isometry3d T_inv = T.inverse();
```

## Diagrams Needed

1. **Transform Tree Diagram**: Show typical robot TF tree
   ```
   map
    └── odom
         └── base_link
              ├── camera_link
              │    └── camera_optical
              ├── lidar_link
              └── arm_base
                   └── arm_link1
                        └── arm_link2
                             └── gripper
   ```

2. **Homogeneous Transform Structure**: 4x4 matrix visualization

3. **Gimbal Lock Illustration**: Show how 2 axes aligning loses DOF

4. **ROS Coordinate Convention**: x-forward, y-left, z-up

## Prerequisites

- [Degrees of Freedom](/robotics-glossary/concepts/fundamentals/degrees-of-freedom/)
- [Kinematics](/robotics-glossary/concepts/fundamentals/kinematics/)
- Linear algebra basics (matrices, vectors)

## Related Concepts

- [SLAM](/robotics-glossary/concepts/perception/slam/) - Uses transforms for localization
- [ROS 2](/robotics-glossary/software/frameworks/ros2/) - TF2 is core ROS infrastructure
- [Isaac ROS](/robotics-glossary/software/isaac/isaac-ros/) - GPU-accelerated transform support

## Source URLs for Citations

### ROS Official Documentation
- REP-103 (Units and Conventions): https://www.ros.org/reps/rep-0103.html
- REP-105 (Coordinate Frames): https://www.ros.org/reps/rep-0105.html
- TF2 Introduction (Rolling): https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- TF2 Introduction (Humble): https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- TF2 About Page: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html
- Writing a TF2 Broadcaster (Python): https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
- tf2_ros Buffer API: https://docs.ros.org/en/ros2_packages/humble/api/tf2_ros/generated/classtf2__ros_1_1Buffer.html

### NVIDIA Isaac ROS
- Isaac ROS Visual SLAM (frames): https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html
- cuVSLAM Concepts: https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html

### Academic/Tutorials
- Modern Robotics - Homogeneous Transforms: https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
- Articulated Robotics - TF2 Tutorial: https://articulatedrobotics.xyz/tutorials/ready-for-ros/tf/
- Articulated Robotics - Transform Matrices: https://articulatedrobotics.xyz/tutorials/coordinate-transforms/transformation-matrices/
- Mecharithm - Homogeneous Transforms: https://mecharithm.com/learning/lesson/homogenous-transformation-matrices-configurations-in-robotics-12
- Foxglove - Understanding ROS Transforms: https://foxglove.dev/blog/understanding-ros-transforms

### Math/Library Documentation
- SciPy RigidTransform: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.RigidTransform.html
- SciPy Rotation: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
- Eigen Geometry Tutorial: https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html
- Eigen Transform Class: https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html
- spatialmath-python: https://github.com/bdaiinstitute/spatialmath-python

### Rotation Representations
- Gimbal Lock (Wikipedia): https://en.wikipedia.org/wiki/Gimbal_lock
- Quaternions and Spatial Rotation: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
- SE(3) Tutorial (arXiv): https://arxiv.org/pdf/2103.15980

### Nav2 / URDF
- Nav2 Transform Setup: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- Nav2 URDF Setup: https://docs.nav2.org/setup_guides/urdf/setup_urdf.html
- Articulated Robotics - URDF: https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf/

### Tools
- ROS Industrial TF2 Workshop: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html
- MATLAB TF Access: https://www.mathworks.com/help/ros/ug/access-the-tf-transformation-in-ros-2.html

## Visualization Tools

### view_frames
```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf showing transform tree
```

### tf2_echo
```bash
ros2 run tf2_ros tf2_echo base_link camera_link
# Shows live transform between two frames
```

### RViz2
- Add TF display to visualize all transforms
- Set fixed frame to visualize relative to a specific frame

## Summary of Entry Structure

### Recommended Sections
1. **Introduction** - What transforms are and why they matter
2. **Prerequisites** - DOF, Kinematics, basic linear algebra
3. **Coordinate Frames** - Frames, conventions (REP-103, REP-105)
4. **Homogeneous Transformation Matrices** - SE(3), structure, properties
5. **Rotation Representations** - Comparison table with pros/cons
6. **Gimbal Lock** - Problem and quaternion solution
7. **TF2 in ROS 2** - Broadcasters, listeners, buffers, topics
8. **Code Examples** - Python (SciPy + ROS 2), C++ (Eigen)
9. **Visualization** - view_frames, tf2_echo, RViz2
10. **Related Terms** - Links to DOF, Kinematics, SLAM, ROS 2
11. **Learn More** - External resources
12. **Sources** - Citations

### Badge
- **Type:** Conceptual (matches Kinematics, DOF)
- **Variant:** note

### Suggested Links in Entry
- Forward link TO: SLAM, ROS 2, Isaac ROS, Motion Planning
- Backward link FROM: Kinematics, DOF (as related terms)
