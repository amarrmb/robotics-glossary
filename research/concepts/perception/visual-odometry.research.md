# Visual Odometry Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/concepts/perception/visual-odometry.mdx

## Definition

**Visual Odometry (VO)** is the process of estimating the egomotion (position and orientation) of a robot or agent using only camera images. Unlike full SLAM, VO focuses solely on motion estimation without building a persistent map.

## Key Concepts to Cover

### 1. What is Visual Odometry
- Motion estimation from sequential camera images
- Outputs 6-DoF pose (position + orientation)
- Relative position error typically 0.1-2%
- Does not maintain a persistent map (unlike SLAM)
- "Odometry" because it provides incremental motion estimates like wheel encoders

### 2. VO vs SLAM vs VIO

| System | Map Building | IMU Fusion | Drift Correction |
|--------|--------------|------------|------------------|
| Visual Odometry | No | No | No |
| Visual SLAM | Yes | Optional | Yes (loop closure) |
| Visual-Inertial Odometry (VIO) | No | Yes | Partial (IMU helps) |

### 3. Camera Configurations

#### Monocular VO
- Single camera
- **Pros:** Minimal hardware, low cost, lightweight
- **Cons:** Scale ambiguity (cannot determine absolute distances), requires motion for initialization
- Use case: Drones where weight matters, simple robots

#### Stereo VO
- Two cameras with known baseline
- **Pros:** Metric scale from triangulation, more robust
- **Cons:** Higher cost, calibration required, degrades to monocular at long distances
- Use case: Ground robots, autonomous vehicles

#### RGB-D VO
- RGB camera + depth sensor
- **Pros:** Direct depth measurement, simplified algorithm
- **Cons:** Limited range, affected by sunlight (structured light), sensor cost
- Use case: Indoor robots, manipulation

### 4. Algorithmic Approaches

#### Feature-Based (Indirect) Methods
- Extract keypoints (FAST, ORB, SIFT, SURF)
- Track features across frames
- Estimate motion from correspondences
- **Examples:** ORB-SLAM (odometry component), RTAB-Map
- **Pros:** Robust to moderate lighting changes, efficient
- **Cons:** Fails in textureless environments

#### Direct (Appearance-Based) Methods
- Use raw pixel intensities
- Minimize photometric error between frames
- **Examples:** LSD-SLAM, DSO (Direct Sparse Odometry)
- **Pros:** Works in low-texture environments, dense reconstruction possible
- **Cons:** Sensitive to lighting changes, exposure, rolling shutter

#### Hybrid Methods
- Combine feature-based and direct approaches
- **Examples:** SVO (Semi-direct Visual Odometry)

### 5. Feature Extractors Comparison

| Extractor | Speed | Accuracy | Notes |
|-----------|-------|----------|-------|
| FAST | Fastest | Lower | No scale invariance |
| ORB | Fast | Good | Best speed/accuracy tradeoff, used in ORB-SLAM |
| SURF | Medium | High | Scale/rotation invariant |
| SIFT | Slow | Highest | Most robust, computationally expensive |
| AKAZE | Medium | Good | Binary descriptor, good balance |

### 6. Processing Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    Visual Odometry Pipeline                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. IMAGE ACQUISITION                                            │
│  ┌──────────────┐   ┌──────────────┐                            │
│  │   Camera     │──►│  Undistort   │                            │
│  │   Frame(s)   │   │  + Rectify   │                            │
│  └──────────────┘   └──────────────┘                            │
│                            │                                     │
│  2. FEATURE PROCESSING     ▼                                     │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │   Feature    │──►│   Feature    │──►│   Outlier    │         │
│  │  Detection   │   │   Matching   │   │  Rejection   │         │
│  │ (ORB, FAST)  │   │  (BF/FLANN)  │   │  (RANSAC)    │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│                                               │                  │
│  3. MOTION ESTIMATION                         ▼                  │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │    Pose      │◄──│   Bundle     │◄──│   Essential/ │         │
│  │   Output     │   │  Adjustment  │   │  Fundamental │         │
│  │   (6-DoF)    │   │  (optional)  │   │    Matrix    │         │
│  └──────────────┘   └──────────────┘   └──────────────┘         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 7. Visual-Inertial Odometry (VIO)

Fusion of camera + IMU for robust tracking.

#### Coupling Approaches
- **Loosely-coupled:** Independent visual and inertial estimates, fused via Kalman filter
- **Tightly-coupled:** Joint optimization over all states (state-of-the-art)

#### IMU Preintegration
- Summarize multiple IMU measurements into single constraint
- Reduces computational cost for optimization
- Essential for real-time operation

#### Notable VIO Systems
| System | Type | Coupling | Notes |
|--------|------|----------|-------|
| VINS-Mono/Fusion | Feature | Tight | ROS 1 focused, highly cited |
| MSCKF | Feature | Tight | Low computational load |
| OpenVINS | Feature | Tight | Active ROS 2 support |
| OKVIS | Feature | Tight | Keyframe-based |
| S-MSCKF | Feature | Tight | Stereo support |

### 8. Challenges and Limitations

#### Environmental Challenges
- **Low texture:** Blank walls, uniform surfaces → insufficient features
- **Repetitive patterns:** Feature matching ambiguity
- **Dynamic objects:** Moving people/vehicles corrupt motion estimate
- **Lighting changes:** Day/night, shadows, flickering lights

#### Motion Challenges
- **Motion blur:** Fast rotation or translation during exposure
- **Fast motion:** Insufficient overlap between frames
- **Pure rotation:** Scale ambiguity in monocular

#### Hardware Challenges
- **Rolling shutter:** Distortion during fast motion
- **Auto-exposure:** Intensity changes between frames
- **Calibration errors:** Inaccurate intrinsics/extrinsics

### 9. Applications

| Domain | Use Case | Why VO |
|--------|----------|--------|
| **Drones/UAVs** | Autonomous flight, inspection | GPS-denied environments (indoors, urban canyons) |
| **Ground robots** | Navigation, warehouse AGVs | Wheel slip compensation |
| **Autonomous vehicles** | Self-driving cars | GPS complement, tunnel navigation |
| **AR/VR** | Head tracking | Low latency, 6-DoF |
| **Underwater robots** | Inspection, mapping | GPS unavailable underwater |

## NVIDIA + ROS2 Ecosystem

### Isaac ROS Visual SLAM (cuVSLAM)

**Current Version:** Isaac ROS 3.2 / cuVSLAM (as of January 2026)
**Source:** https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html

#### Key Features
- GPU-accelerated visual odometry and SLAM
- Stereo visual-inertial odometry (SVIO)
- Multi-camera support (up to 32 cameras / 16 stereo pairs)
- Automatic IMU fallback when visual tracking fails
- Loop closure for drift correction (SLAM mode)
- Sub-1% trajectory error on KITTI benchmark

#### Performance (Jetson Orin AGX)
- Mono mode: 2.7 ms per frame
- Stereo mode: 2.7 ms per frame
- Multi-camera: 2.8 ms per frame
- Stereo-Inertial: 30 FPS camera, 200 Hz IMU

#### ROS 2 Topics
- `/visual_slam/tracking/odometry` — nav_msgs/Odometry
- `/visual_slam/vis/observations_cloud` — Feature point cloud
- `/visual_slam/vis/landmarks_cloud` — Map landmarks
- `/visual_slam/status` — Tracking status

#### Launch Example
```bash
# Launch with RealSense camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# Launch with ZED camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_zed.launch.py

# Multi-camera (HAWK)
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_hawk.launch.py
```

### PyCuVSLAM
- Python wrapper for cuVSLAM library
- Supports monocular, stereo, RGB-D, multi-camera, and VIO modes
- Direct camera stream interface
- Real-time pose, map points, and loop closure output

### RTAB-Map (ROS 2)
**Versions:**
- ROS 2 Jazzy/Rolling: Active support
- ROS 2 Humble: Active support

**Visual Odometry Strategies:**
- F2M (Frame-to-Map) — default, best accuracy
- F2F (Frame-to-Frame) — faster
- ORB-SLAM2 integration (`Odom/Strategy=5`)
- Fovis, viso2, DVO-SLAM, OpenVINS, VINS-Fusion

**ROS 2 Launch:**
```bash
# Stereo odometry
ros2 launch rtabmap_ros stereo_odometry.launch.py

# RGB-D odometry
ros2 launch rtabmap_ros rgbd_odometry.launch.py
```

### robot_localization
EKF/UKF sensor fusion package for combining:
- Visual odometry (velocity recommended)
- Wheel odometry
- IMU data
- GPS (if available)

```bash
ros2 launch robot_localization ekf.launch.py
```

## Sensor Fusion with Wheel Odometry

### Why Fuse VO + Wheel Odometry?
- **Wheel odometry:** Drifts due to slip, especially during rotation
- **Visual odometry:** Fails in poor visibility, sensitive to features
- **Combined:** Complementary strengths, more robust overall

### Fusion Architecture
```
┌────────────────┐
│ Wheel Encoders │────┐
└────────────────┘    │    ┌─────────────┐
                      ├───►│   EKF /     │───► Fused Odometry
┌────────────────┐    │    │   UKF       │     (position, velocity,
│ Visual Odometry│────┤    └─────────────┘      orientation)
└────────────────┘    │
                      │
┌────────────────┐    │
│      IMU       │────┘
└────────────────┘
```

### robot_localization Configuration
- Fuse VO velocity (not position) for best results
- Fuse IMU orientation and angular velocity
- Fuse wheel odometry velocity

## Code Examples to Include

### 1. Basic VO Pipeline Concept
```python
# Simplified feature-based VO pseudocode
def visual_odometry_step(prev_frame, curr_frame, K):
    # 1. Detect features
    prev_kp = detect_features(prev_frame)  # ORB, FAST, etc.
    curr_kp = detect_features(curr_frame)

    # 2. Match features
    matches = match_features(prev_kp, curr_kp)

    # 3. Estimate essential matrix (for calibrated cameras)
    E, mask = cv2.findEssentialMat(prev_pts, curr_pts, K, method=cv2.RANSAC)

    # 4. Recover pose from essential matrix
    _, R, t, mask = cv2.recoverPose(E, prev_pts, curr_pts, K)

    return R, t  # Rotation and translation (up to scale for monocular)
```

### 2. Isaac ROS Visual SLAM Launch
```bash
# Launch cuVSLAM with stereo camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# Check odometry output
ros2 topic echo /visual_slam/tracking/odometry
```

### 3. RTAB-Map Stereo Odometry
```bash
# Launch stereo visual odometry
ros2 launch rtabmap_ros stereo_odometry.launch.py \
    left_image_topic:=/stereo/left/image_rect \
    right_image_topic:=/stereo/right/image_rect \
    left_camera_info_topic:=/stereo/left/camera_info \
    right_camera_info_topic:=/stereo/right/camera_info
```

## Diagrams Needed

1. **VO vs SLAM comparison diagram** — show VO outputs pose only, SLAM outputs pose + map
2. **Monocular vs Stereo** — illustrate scale ambiguity in monocular
3. **Feature-based vs Direct pipeline** — side-by-side comparison
4. **VIO sensor fusion diagram** — show camera + IMU fusion with EKF/optimization
5. **VO processing pipeline** — detailed flowchart (provided above)

## Source URLs for Citations

### NVIDIA Official
- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
- cuVSLAM Concepts: https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html
- cuVSLAM Paper (arXiv): https://arxiv.org/abs/2506.04359
- Isaac ROS GitHub: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
- Multi-Camera Tutorial: https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_multi_hawk.html

### ROS 2 Official
- RTAB-Map ROS 2: https://github.com/introlab/rtabmap_ros (ros2 branch)
- RTAB-Map Documentation: http://introlab.github.io/rtabmap/
- robot_localization: https://index.ros.org/p/robot_localization/
- sensor_msgs/Image: https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html

### Academic/Reference
- Visual Odometry Tutorial (Scaramuzza): https://rpg.ifi.uzh.ch/docs/VO_Part_I_Scaramuzza.pdf
- Visual Odometry Review (SpringerPlus): https://link.springer.com/article/10.1186/s40064-016-3573-7
- ORB Paper: https://sites.cc.gatech.edu/classes/AY2024/cs4475_summer/images/ORB_an_efficient_alternative_to_SIFT_or_SURF.pdf
- DSO Paper: https://arxiv.org/abs/1607.02565
- LSD-SLAM: https://www.researchgate.net/publication/319770169_LSD-SLAM_large-scale_direct_monocular_SLAM
- VINS-Mono: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
- OpenVINS: https://docs.openvins.com/
- IMU Preintegration Paper: https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf

### Wikipedia
- Visual Odometry: https://en.wikipedia.org/wiki/Visual_odometry

## Prerequisites (for content file)

- [Cameras](/robotics-glossary/hardware/sensors/cameras/) — Understanding camera types and calibration
- [IMU](/robotics-glossary/hardware/sensors/imu/) — For VIO fusion
- [Coordinate Frames](/robotics-glossary/concepts/fundamentals/coordinate-frames/) — Understanding transforms
- [SLAM](/robotics-glossary/concepts/perception/slam/) — Related concept (VO is SLAM without mapping)

## Related Terms (for content file)

- [SLAM](/robotics-glossary/concepts/perception/slam/) — Full SLAM with mapping and loop closure
- [Isaac ROS](/robotics-glossary/software/isaac/isaac-ros/) — GPU-accelerated VO
- [Nav2](/robotics-glossary/software/ros2/nav2/) — Uses odometry for navigation
- [TF2](/robotics-glossary/software/ros2/tf2/) — Transform broadcasting

## Badge Recommendation

**Badge:** Deep Dive
**Rationale:** Visual odometry involves multiple algorithmic approaches (feature-based, direct, VIO), mathematical concepts (essential matrix, bundle adjustment), and practical considerations (sensor choice, fusion). This warrants a "Deep Dive" badge similar to SLAM.

## Content Structure Recommendation

1. **Definition** — What is VO, how it differs from SLAM
2. **Camera Configurations** — Monocular, stereo, RGB-D (with pros/cons table)
3. **Algorithmic Approaches** — Feature-based vs Direct (tabs like SLAM entry)
4. **Processing Pipeline** — ASCII diagram
5. **Visual-Inertial Odometry** — VIO basics, tight vs loose coupling
6. **Challenges** — Aside with common failure modes
7. **NVIDIA Isaac ROS** — cuVSLAM section with code examples
8. **ROS 2 Integration** — RTAB-Map, robot_localization
9. **Prerequisites** — CardGrid
10. **Related Terms** — CardGrid
11. **Sources** — Properly formatted source list
