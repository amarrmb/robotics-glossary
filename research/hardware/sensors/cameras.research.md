# Cameras Research Notes

**Date:** 2026-01-21
**File:** src/content/docs/hardware/sensors/cameras.mdx

## Version Numbers & Current Status

### NVIDIA Isaac ROS

**Current Version:** Isaac ROS 3.2 (Update 15 latest, tagged v3.2-15 on GitHub)
- Compatible with ROS 2 Jazzy
- Runs on Jetson or x86_64 with NVIDIA GPU
- Source: [Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

**Key Packages (verified):**
- `isaac_ros_image_pipeline` - GPU-accelerated resize, rectify, color convert
- `isaac_ros_stereo_image_proc` - Disparity processing
- `isaac_ros_depth_image_proc` - Depth processing
- `isaac_ros_visual_slam` - Visual SLAM/tracking
- `isaac_ros_detectnet` - Detection
- `isaac_ros_argus_camera` - CSI camera support via libArgus
- Source: [Isaac ROS Repositories](https://nvidia-isaac-ros.github.io/repositories_and_packages/index.html)

### ROS 2 Distributions

**Current:**
- **ROS 2 Kilted Kaiju** - Latest release (May 2025), supported until Nov 2026
- **ROS 2 Jazzy Jalisco** - LTS release (May 2024), supported until May 2029
- **ROS 2 Lyrical Luth** - Expected May 2026
- Source: [ROS 2 Distributions](https://docs.ros.org/en/jazzy/Releases.html)

### Depth Cameras

**Intel RealSense D400 Series:**
- D455 - Current model, stereo + IMU, 95mm baseline, 86 deg FOV
  - Range: up to 20m, depth error <2% at 4m
  - Interface: USB 3.1, up to 90fps
- D456 - **IP65-rated** version (dust/water resistant), same optics as D455
  - Same D450 optical module, D4 Vision Processor V3
  - Designed for outdoor robotics, automotive, industrial
- Source: [Intel RealSense D455](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html), [RealSense D456](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d456.html)

**Orbbec Femto (Azure Kinect Replacement):**
- Azure Kinect DK - **DISCONTINUED** (Oct 2023, SDK unmaintained)
- **Femto Bolt** - Direct replacement, co-developed with Microsoft
  - Same iToF technology and operating modes as Azure Kinect
  - 1MP ToF sensor, 120 deg FOV, up to 1024x1024@15fps or 640x576@30fps
  - RGB: 3840x2160@30fps with HDR support
  - USB-C, more compact (half depth of Azure Kinect)
  - Compatible with Azure Kinect Sensor SDK via Orbbec wrapper
- **Femto Mega** - Built-in NVIDIA Jetson Nano, PoE/USB-C
- Source: [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/), [Azure Kinect Comparison](https://www.orbbec.com/documentation/comparison-with-azure-kinect-dk/)

**Stereolabs ZED:**
- **ZED 2i** - Current primary model, IP66-rated
  - Stereo vision + neural depth engine 2
  - Range: 0.2-20m, 110 deg H / 70 deg V / 120 deg diagonal FOV
  - Resolutions: up to 2208x1242@15fps, 1920x1080@30fps, 1280x720@60fps
  - Baseline: 12cm, IMU (gyro, accel, magnetometer), barometer
  - USB Type-C, 166g
  - Lens options: 2.1mm wide or 4mm narrow, optional polarizing filter
- Source: [Stereolabs ZED 2i](https://www.stereolabs.com/store/products/zed-2i)

### Event Cameras

**Prophesee EVK4 HD:**
- Sensor: Sony IMX636 stacked event-based sensor
- Resolution: 1280x720 HD
- Dynamic range: >86dB (5-100000 lux), >120dB (0.08-100000 lux)
- Size: 30x30x36mm, Weight: 40g
- Bandwidth: up to 1.6 Gbps, Power: 1.5W via USB
- C/CS mount compatible
- Includes Metavision SDK PRO
- Source: [Prophesee EVK4](https://www.prophesee.ai/event-camera-evk4/)

**iniVation DVS:**
- **DVXplorer** - VGA resolution, 110dB dynamic range, 165M events/sec
- **DVXplorer Lite** - QVGA (320x240), 110dB, sub-1ms latency, 100M events/sec
- **DAVIS 346** - Combined event + frame output, 120dB
- **DVXplorer S Duo** - Event + global shutter color, built-in Jetson Nano
- Common features: 40x60x25mm aluminum housing, USB3, GPIO sync
- Source: [iniVation Cameras](https://inivation.com/solutions/cameras/)

### Jetson Camera Support

**MIPI CSI-2:**
- Native IMX219 (Pi Camera v2) driver support
- Up to 6-12 cameras depending on Jetson model
- Third-party drivers: Vision Components, Allied Vision (Alvium)
- JetPack 5.1+ for Orin series

**Supported Interfaces:**
- MIPI CSI-2 (lowest latency, ISP processing)
- USB 3.0 (plug-and-play)
- GigE Vision (industrial, long cable runs)
- Source: [NVIDIA Jetson Camera Tutorial](https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera)

### ROS 2 Camera Packages

| Package | Status | Notes |
|---------|--------|-------|
| `usb_cam` | Active | V4L2 driver |
| `realsense2_camera` | Active | ROS 2 focus, ROS 1 legacy branch |
| `zed_wrapper` | Active | Stereolabs maintained |
| `image_proc` | Active | Part of image_pipeline |
| `cv_bridge` | Active | ROS/OpenCV conversion |
| `camera_calibration` | Active | OpenCV-based, v0.1.0 for Jazzy |

**Sources:**
- [realsense-ros GitHub](https://github.com/realsenseai/realsense-ros)
- [ROS 2 Jazzy camera_calibration](https://docs.ros.org/en/ros2_packages/jazzy/api/camera_calibration/)

## Validation Summary

### Content File Status: Mostly Accurate

**Verified Correct:**
- Camera types and technologies (RGB, depth, event)
- Depth camera technology comparison (stereo, structured light, ToF)
- Intel RealSense D455/D456 mentioned correctly
- ZED 2i specs (IP66, 0.2-20m range)
- Orbbec Femto Bolt/Mega as Azure Kinect replacement
- Prophesee EVK4, iniVation DVS
- ROS 2 message types (sensor_msgs/Image, CameraInfo, PointCloud2)
- Camera calibration command syntax
- Isaac ROS 3.2 compatible with ROS 2 Jazzy

**Minor Corrections Needed:**
1. **Isaac ROS version** - File says "3.2", current is 3.2 Update 15 (v3.2-15)
2. **ROS 2 note** - Kilted is now latest (May 2025), Jazzy is LTS
3. **Event camera dynamic range** - EVK4 is >86dB (up to 120dB), file says 140dB (this may be spec for different event cameras in general)

**No Major Issues Found** - Content is current and accurate as of January 2026

## Source URLs

- [Intel RealSense D455 Specs](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html)
- [Intel RealSense D456 Store](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d456.html)
- [Orbbec Femto Bolt](https://www.orbbec.com/products/tof-camera/femto-bolt/)
- [Orbbec Azure Kinect Comparison](https://www.orbbec.com/documentation/comparison-with-azure-kinect-dk/)
- [Stereolabs ZED 2i](https://www.stereolabs.com/store/products/zed-2i)
- [Prophesee EVK4](https://www.prophesee.ai/event-camera-evk4/)
- [iniVation Cameras](https://inivation.com/solutions/cameras/)
- [Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)
- [Isaac ROS Packages](https://nvidia-isaac-ros.github.io/repositories_and_packages/index.html)
- [ROS 2 Distributions](https://docs.ros.org/en/jazzy/Releases.html)
- [ROS 2 camera_calibration](https://docs.ros.org/en/ros2_packages/jazzy/api/camera_calibration/)
- [NVIDIA Jetson Camera Tutorial](https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera)
- [realsense-ros GitHub](https://github.com/realsenseai/realsense-ros)
