# Depth Cameras Research Notes

**Date:** 2026-01-21
**File to create:** src/content/docs/hardware/sensors/depth-cameras.mdx

## Key Concepts to Cover

### Three Main Depth Sensing Technologies

1. **Stereo Vision**
   - Uses two or more cameras separated by known baseline distance
   - Calculates depth via disparity (difference in pixel position between left/right images)
   - Uses epipolar geometry and triangulation
   - Can be passive (ambient light) or active (with IR projector)

2. **Structured Light**
   - Projects known IR pattern (coded patterns, Gray codes, phase-shifted stripes)
   - Single camera analyzes pattern distortion on objects
   - Triangulation-based depth calculation
   - High precision at short to medium range

3. **Time-of-Flight (ToF)**
   - Emits modulated IR light, measures phase shift or direct time delay
   - Two variants: continuous wave (AMCW) and pulsed
   - Fast capture, works in low light
   - Less affected by texture-less surfaces

### Technology Comparison Table

| Aspect | Stereo Vision | Structured Light | Time-of-Flight |
|--------|---------------|------------------|----------------|
| **Range** | 0.5-20m | 0.3-5m | 0.2-8m |
| **Accuracy** | Medium (error increases with distance squared) | High (at short range) | Medium |
| **Speed** | Medium (compute intensive) | Medium | Fast (real-time) |
| **Outdoor use** | Good (passive stereo) | Poor (IR pattern washed out) | Moderate |
| **Texture dependency** | High (needs features) | Low | None |
| **Cost** | Low-Medium | Medium | Medium-High |
| **Power consumption** | Low | Medium | Medium |
| **Form factor** | Larger (baseline needed) | Compact | Compact |

### Best Use Cases

- **Stereo Vision**: Outdoor navigation, long-range perception, autonomous vehicles
- **Structured Light**: Indoor manipulation, 3D scanning, facial recognition, short-range high-accuracy
- **ToF**: AMR navigation, pick-and-place, gesture recognition, low-light environments

## Version Numbers & Current Status

### Popular Depth Cameras

**Intel RealSense (Stereo + Active IR)**
- D455: 95mm baseline, 86° FoV, up to 20m range, USB 3.1, up to 90fps
- D456: IP65-rated outdoor version, same optics as D455
- D405: Short-range (0.1-0.5m), manipulation focus, 87° FoV
- D435i: Compact, built-in IMU, 50mm baseline
- ROS2 Support: Humble, Iron, Jazzy, Kilted (via realsense-ros)
- SDK: librealsense2 v2.56.5 (as of Jazzy packages)
- Source: [Intel RealSense ROS2](https://github.com/realsenseai/realsense-ros)

**Stereolabs ZED (Stereo + Neural Depth)**
- ZED 2i: IP66, 12cm baseline, 0.2-20m, neural depth engine
- ZED SDK 5.1 (October 2025): Centimeter-level close-range sensing, NITROS integration
- ROS2: zed-ros2-wrapper with Isaac ROS NITROS support (10x lower latency)
- Jetson Thor support with JetPack 7
- Source: [Stereolabs ZED SDK 5.1 Blog](https://www.stereolabs.com/blog/introducing-zed-sdk-51-pushing-robotics-multi-camera-perception-further-than-ever)

**Orbbec (ToF and Active Stereo)**
- Femto Bolt (ToF): 1MP ToF sensor, 120° FoV, 1024x1024@15fps or 640x576@30fps
  - Microsoft-partnered Azure Kinect replacement
  - AMCW time-of-flight principle
  - Source: [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/)
- Femto Mega (ToF): Built-in compute, PoE/USB-C
- Gemini 330 series (Active Stereo): MX6800 ASIC, indoor/outdoor operation
- Gemini 335Lg: Designed for mobile robots, ROS2/Isaac compatible
- Astra series (Structured Light): Built-in depth processing
- ROS2: OrbbecSDK_ROS2 v2 supports Foxy, Humble, Jazzy
- Source: [OrbbecSDK ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)

### NVIDIA Isaac ROS Depth Integration

**Current Version:** Isaac ROS 3.2 (compatible with ROS 2 Jazzy)

**Key Packages:**
- `isaac_ros_image_pipeline`: GPU-accelerated resize, rectify, color convert
- `isaac_ros_stereo_image_proc`: Disparity computation
- `isaac_ros_depth_image_proc`: Depth to point cloud conversion
- `isaac_ros_dnn_stereo_depth`: DNN-based stereo depth estimation

**DNN Stereo Depth Models:**
1. **ESS (Efficient Semi-Supervised)**:
   - Fast, suitable for compute-limited scenarios
   - Outputs disparity + confidence map
   - Robust to unseen environments
   - Best for real-time applications

2. **FoundationStereo** (Added October 2025):
   - Foundation model for stereo depth estimation
   - Transformer-based architecture
   - Uses Depth Anything V2 as feature extractor
   - Zero-shot generalization across scenes
   - Best Paper Nomination at CVPR 2025
   - No confidence output, all disparity values provided
   - Source: [FoundationStereo](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html)

**Supported Cameras:**
- Intel RealSense (D4xx series)
- ZED cameras (with NITROS support)
- Orbbec cameras
- Isaac Sim simulated cameras

Source: [Isaac ROS DNN Stereo Depth](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html)

### ROS 2 Integration

**Standard Message Types:**
- `sensor_msgs/Image`: Depth image (32-bit float meters or 16-bit mm)
- `sensor_msgs/CameraInfo`: Intrinsic/extrinsic calibration
- `sensor_msgs/PointCloud2`: 3D point cloud representation

**Common Topics:**
- `/camera/depth/image_raw`: Raw depth image
- `/camera/depth/camera_info`: Depth camera calibration
- `/camera/depth_registered/points`: Colored point cloud
- `/camera/color/image_raw`: RGB image

**Key ROS 2 Packages:**
| Package | Purpose |
|---------|---------|
| `realsense2_camera` | Intel RealSense driver |
| `zed_wrapper` | ZED camera driver |
| `orbbec_camera` | Orbbec camera driver |
| `depth_image_proc` | Depth processing (CPU) |
| `image_pipeline` | Image processing suite |
| `pointcloud_to_laserscan` | Convert 3D to 2D |

## Code Examples to Include

### Basic ROS 2 Depth Subscription (Python)
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, 10)
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        # Convert to numpy array (meters, 32-bit float)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        # Get depth at center pixel
        h, w = depth_image.shape
        center_depth = depth_image[h//2, w//2]
        self.get_logger().info(f'Center depth: {center_depth:.3f}m')
```

### Launch RealSense with Depth
```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    depth_module.profile:=640x480x30 \
    pointcloud.enable:=true
```

### Launch ZED Camera
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i
```

### Launch Orbbec Camera
```bash
ros2 launch orbbec_camera femto_bolt.launch.py
```

### Visualize Depth in RViz2
```bash
# Add PointCloud2 display
# Set topic to /camera/depth_registered/points
# Set Fixed Frame to camera_link or camera_depth_frame
```

## Diagrams Needed

1. **Technology Comparison Diagram**
```
Stereo Vision:
[Left Camera]---baseline---[Right Camera]
      │                          │
      └──────┬───────────────────┘
             │ Disparity Matching
             ▼
        [Depth Map]

Structured Light:
[IR Projector]──pattern──►[Object]
                              │
                         distortion
                              │
                    ◄────────[Camera]
                              │
                         triangulation
                              ▼
                         [Depth Map]

Time-of-Flight:
[Emitter]───IR pulse───►[Object]
    │                        │
    │                   reflection
    │                        │
    └◄───────────────────────┘
    │
    ▼ measure time/phase
[Depth Map]
```

2. **Depth Camera Pipeline**
```
┌─────────────┐    ┌──────────────┐    ┌───────────────┐
│   Depth     │ →  │  Undistort   │ →  │   Register    │
│   Sensor    │    │  (calibrate) │    │   to RGB      │
└─────────────┘    └──────────────┘    └───────────────┘
                                              │
                                              ▼
┌─────────────┐    ┌──────────────┐    ┌───────────────┐
│  Obstacle   │ ←  │ Point Cloud  │ ←  │   Depth to    │
│  Detection  │    │ (PCL2)       │    │   3D Points   │
└─────────────┘    └──────────────┘    └───────────────┘
```

3. **Depth Camera Specifications Table** (for visual comparison)

## Camera Calibration

### Intrinsic Parameters
- Focal length (fx, fy)
- Principal point (cx, cy)
- Distortion coefficients (k1, k2, p1, p2, k3)

### Extrinsic Parameters
- Rotation matrix R (3x3)
- Translation vector t (3x1)
- Transforms world coordinates to camera coordinates

### RGB-D Calibration
For depth cameras with RGB, need to calibrate:
1. RGB intrinsics
2. Depth intrinsics
3. RGB-Depth extrinsics (registration)

Source: [MathWorks Camera Calibration](https://www.mathworks.com/help/vision/ug/camera-calibration.html)

## Prerequisites and Related Concepts

**Prerequisites:**
- [Cameras](/robotics-glossary/hardware/sensors/cameras/) - General camera concepts
- [Coordinate Frames](/robotics-glossary/concepts/fundamentals/coordinate-frames/) - Understanding 3D coordinates
- [Transforms](/robotics-glossary/concepts/fundamentals/transforms/) - TF2 for sensor frames

**Related Concepts:**
- [SLAM](/robotics-glossary/concepts/perception/slam/) - Uses depth for mapping
- [Visual Odometry](/robotics-glossary/concepts/perception/visual-odometry/) - Motion estimation
- [Sensor Fusion](/robotics-glossary/concepts/perception/sensor-fusion/) - Combining with other sensors
- [LiDAR](/robotics-glossary/hardware/sensors/lidar/) - Alternative 3D sensing
- [nvblox](/robotics-glossary/software/nvidia/nvblox/) - GPU-accelerated 3D reconstruction

## Source URLs for Citations

### Official Documentation
- [Intel RealSense SDK](https://github.com/realsenseai/realsense-ros)
- [Intel RealSense ROS2 Sample](https://docs.openedgeplatform.intel.com/dev/edge-ai-suites/robotics-ai-suite/robotics/dev_guide/tutorials_amr/perception/realsense-ros2-sample-application.html)
- [Stereolabs ZED ROS2 Docs](https://www.stereolabs.com/docs/ros2)
- [Stereolabs Depth Sensing](https://www.stereolabs.com/docs/ros2/depth-sensing)
- [Orbbec ROS2 Wrapper](https://github.com/orbbec/OrbbecSDK_ROS2)
- [Orbbec Femto Bolt Docs](https://www.orbbec.com/documentation/depth-camera/)

### NVIDIA Isaac ROS
- [Isaac ROS Stereo Depth](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/index.html)
- [Isaac ROS FoundationStereo](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html)
- [Isaac ROS ESS](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/ess/index.html)
- [Isaac ROS DNN Stereo Depth](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html)
- [Isaac ROS Image Pipeline](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_pipeline/)
- [NVIDIA Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

### ROS 2 Documentation
- [sensor_msgs/PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)
- [librealsense2 Jazzy](https://docs.ros.org/en/jazzy/p/librealsense2/)
- [ROS-Industrial 3D Camera Survey](https://rosindustrial.org/3d-camera-survey)

### Technical References
- [DFRobot Depth Camera Principles](https://wiki.dfrobot.com/brief_analysis_of_camera_principles)
- [LIPS How 3D Depth Cameras Work](https://www.lips-hci.com/post/how-3d-depth-cameras-work)
- [e-con Systems ToF Comparison](https://www.e-consystems.com/blog/camera/technology/how-time-of-flight-tof-compares-with-other-3d-depth-mapping-technologies/)
- [Basler ToF vs Stereovision](https://www.baslerweb.com/en/learning/time-of-flight-stereovision/)
- [Edge AI Vision 3D Camera Technologies](https://www.edge-ai-vision.com/2025/04/understanding-3d-camera-technologies-stereo-vision-structured-light-and-time-of-flight/)

### Product Pages
- [Intel RealSense D455](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html)
- [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/)
- [Orbbec Gemini 335Lg](https://www.therobotreport.com/gemini-335lg-stereo-vision-3d-camera-orbbec-launches-collaborative-mobile-robots/)
- [Stereolabs ZED SDK 5.1](https://www.stereolabs.com/blog/introducing-zed-sdk-51-pushing-robotics-multi-camera-perception-further-than-ever)

## Notes for Content Creation

1. **Differentiation from cameras.mdx**: The cameras entry already covers depth cameras briefly. This entry should go deeper into:
   - Detailed technology explanations (how each works)
   - Selection criteria and trade-offs
   - Integration with Isaac ROS DNN models (ESS, FoundationStereo)
   - Practical code examples for common tasks

2. **NVIDIA + ROS2 Focus**: Emphasize:
   - Isaac ROS depth packages
   - NITROS integration for ZED cameras
   - FoundationStereo as new foundation model
   - GPU-accelerated depth processing

3. **Practical Badge**: This is a hardware/practical topic, use appropriate badge

4. **Cross-references**: Link to cameras.mdx, SLAM, visual-odometry, sensor-fusion, nvblox
