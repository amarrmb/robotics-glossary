# Cameras Research Notes

**Date:** 2026-01-20
**File:** src/content/docs/hardware/sensors/cameras.mdx

## Version Numbers & Current Status

### Depth Cameras

**Intel RealSense D400 Series:**
- D455 - Current model, still available
- D456 - IP65-rated version (dustproof/waterproof)
- D457 - GMSL/FAKRA IP65 version
- D455f - IR filter variant for improved depth accuracy
- Source: [Intel RealSense D455](https://www.intelrealsense.com/depth-camera-d455/), [Intel RealSense D456](https://www.intelrealsense.com/depth-camera-d456/)

**Azure Kinect:**
- **DISCONTINUED** - End of sale October 2023
- SDK unmaintained since ~2020
- Recommended replacement: **Orbbec Femto Bolt** (uses same Microsoft ToF technology)
- Alternative: **Orbbec Femto Mega** (Ethernet/USB, built on NVIDIA Jetson)
- Source: [Microsoft Azure Kinect Discontinuation](https://redmondmag.com/articles/2023/08/17/microsoft-ending-azure-kinect.aspx), [Orbbec Partnership](https://www.orbbec.com/microsoft-collaboration/)

**ZED Cameras:**
- ZED 2i - Current primary model (IP66-rated, neural depth engine 2)
- ZED Mini - Compact model for mixed reality
- Depth range: 0.2-20m
- Source: [Stereolabs ZED 2](https://www.stereolabs.com/products/zed-2)

**Orbbec Femto:**
- Femto Bolt - Azure Kinect replacement (iToF, same performance)
- Femto Mega - Programmable camera with NVIDIA Jetson platform
- Source: [Orbbec Femto Bolt](https://www.orbbec.com/documentation/comparison-with-azure-kinect-dk/)

### Event Cameras

**Prophesee:**
- EVK4 HD - Current evaluation kit
- Uses Sony IMX636 stacked event-based sensor
- Dynamic range: >86dB (up to >120dB)
- Weight: 40g, Size: 30x30x36mm
- Source: [Prophesee EVK4](https://www.prophesee.ai/event-camera-evk4/)

**iniVation DVS:**
- File mentions iniVation DVS - still current
- No deprecation found

## Isaac ROS Packages

**Current Version:** 3.2 (Update 15 as of search date)
- Compatible with ROS 2 Jazzy
- Runs on Jetson or x86_64 with NVIDIA GPU

**Image Processing Packages:**
- `isaac_ros_image_pipeline` - Metapackage for GPU-accelerated image processing
  - Includes: crop, resize, rectify, color conversion
  - Uses NITROS for zero-copy optimization
- `isaac_ros_stereo_image_proc` - Stereo disparity processing
- `isaac_ros_visual_slam` - Visual SLAM
- `isaac_ros_detectnet` - Object detection (also: isaac_ros_rtdetr, isaac_ros_yolov8)
- Source: [Isaac ROS Image Pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline), [Isaac ROS Docs](https://nvidia-isaac-ros.github.io/)

## ROS 2 Camera Packages

**Tested with ROS 2 Jazzy:**

| Package | Status | Notes |
|---------|--------|-------|
| `usb_cam` | Active | V4L2 driver, binary available all ROS 2 distros |
| `realsense2_camera` | Active | ROS 1 wrapper deprecated, ROS 2 focus |
| `zed_wrapper` | Active | Stereolabs maintained |
| `image_proc` | Active | Part of image_pipeline |
| `cv_bridge` | Active | ROS/OpenCV conversion |
| `camera_calibration` | Active | OpenCV-based calibration |

**Sources:**
- [usb_cam GitHub](https://github.com/ros-drivers/usb_cam)
- [realsense-ros GitHub](https://github.com/IntelRealSense/realsense-ros)
- [ROS 2 Jazzy camera_calibration](https://docs.ros.org/en/ros2_packages/jazzy/api/camera_calibration/)

## Jetson Camera Support

**MIPI CSI-2:**
- Orin Nano: 8 lanes MIPI CSI-2 D-PHY 2.1 (up to 20Gbps)
- Supports up to 4 cameras (8 via virtual channels on Orin Nano)
- Dev kit exposes 2 CSI interfaces (CSI-0, CSI-2)
- Source: [NVIDIA Jetson Partner Cameras](https://developer.nvidia.com/embedded/jetson-partner-supported-cameras)

**Supported formats (Xavier NX):**
- RGB: RGB888, RGB666, RGB565, RGB555, RGB444
- YUV: YUV422-8b, YUV420-8b
- RAW: RAW6-RAW20

## Corrections Needed

1. **Azure Kinect** - Add note that it's discontinued; recommend Orbbec Femto Bolt as replacement
2. **Isaac ROS packages** - Current list is accurate but could add:
   - `isaac_ros_depth_image_proc` for depth processing
   - Note Isaac ROS 3.2 version compatibility with ROS 2 Jazzy
3. **RealSense models** - D456 (IP65) and D457 (GMSL) variants could be mentioned
4. **ZED cameras** - ZED 2i is current primary model (IP66), ZED 2 may be older
5. **Orbbec Femto** - Update to mention it's the Azure Kinect replacement

## Minor Updates

- Isaac ROS compatible with ROS 2 Jazzy (verified)
- Camera calibration package syntax is correct for Jazzy
- usb_cam supports all active ROS 2 distros via binary install
