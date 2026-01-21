# Coordinate Frames Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/concepts/fundamentals/coordinate-frames.mdx
**Description:** Reference frames and how robots understand position in space

## Overview

Coordinate frames (also called reference frames) are the foundational concept for how robots understand and represent positions in space. Every point, velocity, or orientation in robotics is defined *relative to some frame*. A coordinate frame consists of an origin point and three orthogonal axes (x, y, z) that define the directions of measurement.

**Key Insight:** This entry should complement the existing `transforms.mdx` entry. While transforms focuses on the *mathematical operations* to convert between frames, this entry focuses on *what frames are*, *why they matter*, and *how to think about them*.

## Key Concepts to Cover

### 1. What Is a Coordinate Frame?

- A **coordinate frame** is a reference system consisting of:
  - An **origin** point (where measurements start)
  - Three **orthogonal axes** (x, y, z) defining directions
  - A **handedness** (typically right-handed in robotics)
- Every measurement (position, velocity, force) must be expressed in *some* frame
- Frames can be:
  - **Fixed** (attached to the world, a wall, the ground)
  - **Moving** (attached to a robot, a gripper, a sensor)

### 2. Why Frames Matter in Robotics

- **Sensors see the world differently**: A camera sees objects in its own frame; you need to convert to robot frame to act
- **Multiple perspectives**: "1 meter to the left" depends on whose left
- **Composition**: Robot arm has base → link1 → link2 → ... → gripper, each with its own frame
- **Without frames**: Impossible to fuse sensor data, plan motions, or coordinate multiple robots

### 3. Standard Frames in Robotics

#### World / Map Frame
- Fixed frame representing the global environment
- Origin at some known reference point (corner of room, building entrance)
- Does not move; all other frames are ultimately relative to this
- In ROS: `map`

#### Odometry Frame
- Tracks robot's motion from its starting point
- Continuous (smooth, no jumps) but **drifts over time**
- Useful for short-term motion but unreliable long-term
- In ROS: `odom`

#### Robot Base Frame
- Attached to the robot's body (typically center of rotation)
- Moves with the robot through the world
- All robot components are defined relative to this
- In ROS: `base_link`

#### Sensor Frames
- Each sensor has its own frame (camera, LiDAR, IMU)
- Measurements reported in sensor's local frame
- Must transform to robot/world frame for use
- Camera optical frame: z-forward, y-down (different from robot convention!)

#### End-Effector / Tool Frame
- Located at the robot's "hand" or tool tip
- Critical for manipulation tasks ("grasp the object")
- May have a Tool Center Point (TCP) offset from physical mount

### 4. Frame Conventions

#### ROS Convention (REP-103, REP-105)
- **Right-hand rule**: thumb=x, index=y, middle=z
- **Axis alignment**: x=forward, y=left, z=up (FLU)
- **Units**: meters, radians (SI)
- **Orientation**: Quaternions preferred (no gimbal lock)

#### NVIDIA Isaac Sim / USD Convention
- Uses **Y-up, -Z forward** (computer graphics convention)
- Must convert when bridging to ROS (USD ↔ ROS transforms needed)
- Stage units typically 1.0 = 1 meter

#### Aerospace Conventions
- **NED (North-East-Down)**: Used by drones/aircraft, z points down
- **ENU (East-North-Up)**: Common in geographic applications, similar to ROS
- **FRD (Forward-Right-Down)**: Body frame for aircraft (PX4)
- **FLU (Forward-Left-Up)**: Body frame for ground robots (ROS)

#### Camera Optical Frame
- **z-forward, y-down, x-right**: Different from robot conventions!
- Matches how images are processed (origin at top-left)
- ROS convention: `camera_optical_frame` uses this orientation

### 5. The TF2 Frame Tree

#### Tree Structure
- TF2 maintains frames as a **directed tree** (no cycles/loops)
- Each frame has **exactly one parent** (except root)
- Typical hierarchy:
  ```
  earth
   └── map
        └── odom
             └── base_link
                  ├── camera_link
                  │    └── camera_optical
                  ├── lidar_link
                  ├── imu_link
                  └── arm_base
                       └── ... → gripper
  ```

#### Why a Tree?
- Unambiguous path between any two frames
- Efficient lookup (single path to traverse)
- Supports time-buffered transforms (where was frame X 2 seconds ago?)

#### Static vs Dynamic Frames
- **Static**: Fixed relationship (sensor mount, link geometry) → `/tf_static`
- **Dynamic**: Changes over time (odometry, joint positions) → `/tf`

### 6. Common Frame Relationships

| Relationship | Description | Example |
|--------------|-------------|---------|
| map → odom | SLAM correction (may jump) | Localization update |
| odom → base_link | Odometry (continuous, drifts) | Wheel encoders |
| base_link → sensor | Fixed mount position | Camera at (0.1, 0, 0.2) |
| base_link → gripper | Kinematic chain | Arm FK computation |

### 7. Thinking in Frames

#### The Fundamental Question
When working with any position/orientation data, always ask:
1. **What frame is this expressed in?**
2. **What frame do I need it in?**
3. **Do I have the transform between them?**

#### Common Mistakes
- Assuming data is in a different frame than it actually is
- Forgetting camera optical frames are oriented differently
- Mixing up frame naming conventions across systems
- Not accounting for time (transforms change!)

## Current Version Numbers (January 2026)

### ROS 2
- **ROS 2 Kilted Kaiju**: Latest (May 2025)
- **ROS 2 Jazzy Jalisco**: LTS (May 2024 - May 2029)
- **tf2_ros**: Part of geometry2 stack (note: repo archived May 2025, code lives on)

### NVIDIA Isaac
- **Isaac Sim 4.5.0**: Current version
- **Isaac ROS 4.0.1**: December 2025 release
- **cuVSLAM 14**: Visual SLAM with frame outputs (map_frame, odom_frame, base_frame)

### REP Standards
- **REP-103**: Standard Units of Measure and Coordinate Conventions
- **REP-105**: Coordinate Frames for Mobile Platforms
- **REP-199**: Coordinate Frames for Serial Industrial Manipulators (proposed)

## Code Examples to Include

### 1. Visualizing the TF Tree
```bash
# Generate PDF of current transform tree
ros2 run tf2_tools view_frames

# Print transform between two frames continuously
ros2 run tf2_ros tf2_echo map base_link
```

### 2. Publishing a Static Frame (CLI)
```bash
ros2 run tf2_ros static_transform_publisher \
    --x 0.1 --y 0.0 --z 0.2 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id base_link \
    --child-frame-id camera_link
```

### 3. Querying Frame Info in Python
```python
import rclpy
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('frame_query')
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

# After spinning briefly to receive transforms...
# Check if transform exists
can_transform = tf_buffer.can_transform('map', 'base_link', rclpy.time.Time())

# Get all known frames
frames = tf_buffer.all_frames_as_yaml()
print(frames)
```

### 4. Frame Naming Best Practices
```
# Good: Descriptive, follows conventions
base_link
front_camera_link
front_camera_optical_frame
left_wheel_link

# Bad: Ambiguous, inconsistent
camera1
Camera_Frame
my_robot
link_a
```

## Diagrams Needed

### 1. What Is a Frame
```
           z (up)
           │
           │
           │
           ●───────► y (left)
          /origin
         /
        ↙
       x (forward)

    Right-Hand Rule (ROS Convention)
```

### 2. Multiple Frames in a Robot System
```
World Frame (map)          Robot sees a cup
    ↓                      "Cup is at (0.3, 0.1, 0.0) in camera_frame"
    ↓  SLAM
    ↓                      But robot arm needs position in base_link!
Odometry Frame (odom)
    ↓                      Transform: camera_frame → base_link
    ↓  wheel encoders      "Cup is at (0.5, 0.2, 0.3) in base_link"
    ↓
Robot Frame (base_link)    Now gripper can plan a path
    ├── camera_frame
    ├── lidar_frame
    └── arm → gripper_frame
```

### 3. Frame Tree Example
```
earth (optional)
 └── map                    ← World-fixed (SLAM corrected)
      └── odom              ← Robot starting point (drifts)
           └── base_link    ← Robot body center
                ├── front_camera
                │    └── front_camera_optical
                ├── lidar
                └── arm_base
                     └── link1
                          └── link2
                               └── gripper
```

### 4. Convention Comparison Table
```
Convention    X-axis     Y-axis     Z-axis     Used By
───────────────────────────────────────────────────────
FLU (ROS)     Forward    Left       Up         ROS, ground robots
FRD           Forward    Right      Down       PX4, aircraft
NED           North      East       Down       Aviation, GPS
ENU           East       North      Up         Geographic systems
Camera        Right      Down       Forward    Computer vision
USD/Graphics  Right      Up         -Forward   NVIDIA, Blender
```

## Prerequisites

This entry should link to:
- None required (this is a fundamental concept)
- Optional: Basic 3D geometry understanding

## Related Concepts

This entry should link TO (forward references):
- [Transforms](/robotics-glossary/concepts/fundamentals/transforms/) — Mathematical operations to convert between frames
- [Kinematics](/robotics-glossary/concepts/fundamentals/kinematics/) — Using frames to compute robot positions
- [SLAM](/robotics-glossary/concepts/perception/slam/) — Uses coordinate frames for localization
- [ROS 2](/robotics-glossary/software/frameworks/ros2/) — TF2 is core infrastructure

Other entries should link FROM (backward references):
- Transforms entry already has a section on coordinate frames
- SLAM entry references coordinate frames in prerequisites

## Source URLs for Citations

### ROS Official Documentation
- REP-103 (Units and Conventions): https://www.ros.org/reps/rep-0103.html
- REP-105 (Coordinate Frames for Mobile Platforms): https://www.ros.org/reps/rep-0105.html
- REP-199 (Coordinate Frames for Industrial Manipulators): https://gavanderhoorn.github.io/rep/rep-0199.html
- TF2 Concepts (Humble): https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html
- TF2 Introduction Tutorial: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
- Nav2 Transform Setup: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html

### NVIDIA Isaac Documentation
- Isaac Sim Conventions: https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/reference_conventions.html
- Isaac Sim 4.5.0 Conventions: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/reference_material/reference_conventions.html
- Coordinate Frames (Isaac Archive): https://docs.nvidia.com/isaac/archive/2020.1/packages/perception/doc/coord_frame.html
- cuVSLAM Frames: https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html
- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html

### Academic / Educational Resources
- Engineering LibreTexts - Coordinate Systems: https://eng.libretexts.org/Bookshelves/Mechanical_Engineering/Introduction_to_Autonomous_Robots_(Correll)/03:_Forward_and_Inverse_Kinematics/3.01:_Coordinate_Systems_and_Frames_of_Reference
- Standard Bots - Robot Frames Explained: https://standardbots.com/blog/robot-frames
- Automatic Addison - Coordinate Frame Basics: https://automaticaddison.com/coordinate-frame-basics-and-the-right-hand-rule-of-robotics/
- Robot Academy - Base and Tool Transforms: https://robotacademy.net.au/lesson/base-and-tool-transforms/
- Patti Engineering - Industrial Robot Frames: https://www.pattiengineering.com/blog/coordinate-system-frames-industrial-robots/
- Solis PLC - Robot Coordinate Frames and Points: https://www.solisplc.com/tutorials/robot-coordinate-frames-and-points
- Moonpreneur - Coordinate Frames in Robotics: https://mp.moonpreneur.com/blog/coordinate-frames-and-transformations-in-robotics/

### Aerospace Conventions
- Wikipedia - Axes Conventions: https://en.wikipedia.org/wiki/Axes_conventions
- NPS Wiki - Local Coordinate Frames: https://wiki.nps.edu/display/RC/Local+Coordinate+Frames
- MATLAB UAV Toolbox Coordinate Systems: https://www.mathworks.com/help/uav/ug/coordinate-systems-in-uav-toolbox.html
- FIRST Robotics - Coordinate System: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

### TF2 Tutorials and Tools
- Articulated Robotics - TF Tutorial: https://articulatedrobotics.xyz/tutorials/ready-for-ros/tf/
- ROS Industrial TF2 Workshop: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html
- ROS-I Academy TF2: https://ros2.fh-aachen.de/summer_school/Day4/Tutorials/ros_tf2.html
- MATLAB - Access TF Transformation in ROS 2: https://www.mathworks.com/help/ros/ug/access-the-tf-transformation-in-ros-2.html

### Calibration / Frame Alignment
- robot_calibration (GitHub): https://github.com/mikeferguson/robot_calibration
- Camera-Robot Extrinsic Calibration: https://samarth-robo.github.io/blog/2020/11/18/robot_camera_calibration.html
- MATLAB Hand-Eye Calibration: https://www.mathworks.com/help/robotics/ug/estimate-pose-of-fixed-camera-relative-to-robot-base.html

### Unity / Game Engine Integration
- Unity ROS Geometry: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/ROSGeometry.md
- Unity Working with Coordinate Spaces: https://docs.unity3d.com/Simulation/manual/author/working-with-coordinate-spaces.html

### Homogeneous Transforms (related)
- Modern Robotics - Homogeneous Transformation Matrices: https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/
- Mecharithm - Homogeneous Transforms: https://mecharithm.com/learning/lesson/homogenous-transformation-matrices-configurations-in-robotics-12
- Medium - Homogeneous Transformations Tutorial: https://medium.com/@idelossantosruiz/homogeneous-transformations-in-robotics-a-comprehensive-tutorial-with-examples-e36cd3709f41

## Summary of Entry Structure

### Recommended Sections
1. **Introduction** — What coordinate frames are, why they're fundamental
2. **What Is a Coordinate Frame?** — Origin, axes, handedness
3. **Standard Frames in Robotics** — map, odom, base_link, sensors, end-effector
4. **Frame Conventions** — ROS (REP-103/105), Isaac Sim/USD, aerospace (NED/ENU/FLU/FRD)
5. **The TF2 Frame Tree** — Tree structure, static vs dynamic, typical hierarchy
6. **Thinking in Frames** — Mental model, common questions, common mistakes
7. **Visualization Tools** — view_frames, tf2_echo, rviz
8. **Related Terms** — Links to transforms, kinematics, SLAM
9. **Learn More** — External resources
10. **Sources** — Citations

### Badge
- **Type:** Foundational (new category for truly fundamental concepts)
- **Variant:** caution (to emphasize importance)
- OR **Type:** Conceptual (like Kinematics)
- **Variant:** note

### Differentiation from transforms.mdx
| Topic | coordinate-frames.mdx (this entry) | transforms.mdx (existing) |
|-------|-----------------------------------|---------------------------|
| Focus | What frames ARE, how to think about them | HOW to convert between frames |
| Math level | Low (conceptual) | Medium (matrices, quaternions) |
| Code | Visualization, frame queries | Broadcasting, listening, composition |
| Conventions | Heavy emphasis on conventions | Brief mention |
| Prerequisites | None | DOF, Kinematics |

### Notes on Relationship with transforms.mdx
The existing transforms.mdx already has a "Coordinate Frames" section. Options:
1. **Keep both**: This entry is conceptual introduction, transforms is implementation
2. **Cross-link heavily**: This entry → transforms for "how to convert"
3. **Update transforms.mdx**: Remove/reduce its coordinate frames section, link here instead

Recommended: Option 2 — Both entries serve different purposes. This entry explains the "what and why" of frames; transforms explains the "how" of conversions.
