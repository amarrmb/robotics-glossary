# Isaac ROS Research Notes

**Research Date:** 2026-01-21
**Previous Research:** 2026-01-20
**Current File:** src/content/docs/software/isaac/isaac-ros.mdx
**File Last Validated:** 2026-01-20

## Current Version Status

### Isaac ROS 4.0 (Confirmed Current)
- **Latest Release:** Isaac ROS 4.0 Update 1 (v4.0-1) on GitHub
- **Base Release:** Isaac ROS 4.0.0 (v4.0.0)
- **ROS 2 Support:** ROS 2 Jazzy (all packages designed and tested for Jazzy)
- **Isaac Sim Compatibility:** Isaac Sim 5.1
- Source: [Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

### Platform Support (Verified)
| Platform | JetPack | Status |
|----------|---------|--------|
| Jetson AGX Thor | JetPack 7.0/7.1 | Supported (new in 4.0) |
| Jetson AGX Orin | JetPack 6.x | Supported |
| Jetson Orin NX | JetPack 6.x | Supported |
| Jetson Orin Nano | JetPack 6.x | Supported |
| Jetson Xavier (all) | N/A | Removed in Isaac ROS 2.1 |
| x86_64 + NVIDIA GPU | N/A | Supported |

- **JetPack 7.1:** Latest for Thor - Jetson Linux 38.4, Ubuntu 24.04 LTS, CUDA 13.0
- **JetPack 6.2:** Latest for Orin - Jetson Linux 36.4.3, Ubuntu 22.04, CUDA 12.6, TensorRT 10.3
- Source: [JetPack SDK](https://developer.nvidia.com/embedded/jetpack), [Getting Started](https://nvidia-isaac-ros.github.io/getting_started/index.html)

## Content File Verification

### Version Badge (Line 17-19) ✓
- **Content:** "Isaac ROS 4.0 — Supports ROS 2 Jazzy, JetPack 6.x (Orin) and JetPack 7.x (Thor)"
- **Status:** CORRECT

### Xavier Deprecation (Line 224-225) ✓
- **Content:** "Jetson Xavier (AGX Xavier, Xavier NX) is no longer supported as of Isaac ROS 2.1"
- **Status:** CORRECT
- Source: [Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

### Docker Requirements (Line 140) ✓
- **Content:** "Requires Docker Engine 27.2.0+"
- **Status:** ACCEPTABLE - Dev container functionality moved to Isaac ROS CLI (Oct 2025)
- Source: [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)

### NITROS Zero-Copy (Lines 112-136) ✓
- **Content:** Accurate description of zero-copy GPU memory sharing
- **Status:** CORRECT
- **Key requirement:** All NITROS-accelerated nodes must run in same process for zero-copy
- Source: [NITROS Documentation](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html)

### Key Packages ✓
All listed packages verified:
- `isaac_ros_visual_slam` (cuVSLAM) - GPU-accelerated Visual SLAM
- `isaac_ros_apriltag` - Fiducial marker detection
- `isaac_ros_centerpose` - 6-DOF object pose estimation
- `isaac_ros_foundationpose` - Zero-shot pose estimation (>120 FPS tracking on Thor)
- `isaac_ros_nvblox` - Real-time 3D reconstruction
- `isaac_ros_cumotion` - GPU-accelerated motion planning (MoveIt 2 integration)
- `isaac_ros_yolov8` - YOLOv8 object detection
- Source: [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)

## Known Limitations (Isaac ROS 4.0)

- Isaac Perceptor and Nova packages not yet optimized for AGX Thor
- ZED SDK not updated for Jetson Thor compatibility
- ZED ROS2 Wrapper only compatible with Isaac ROS 3.2.x (not 4.x yet)
- Nova Calibration Tool is deprecated
- Source: [Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

## New Features in Isaac ROS 4.0

- `isaac_ros_mapping_and_localization` - consolidated repository
- Improved localization robustness with learning-based descriptors
- Improved map creation with FoundationStereo
- Manipulation and multi-object pick-and-place workflows
- NITROS bridge communication with Isaac Sim fixes
- Support for third party RT-DETR models
- FoundationPose enhancements: symmetry_axes, fixed_axis_angles, fixed_translations options
- Source: [Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

## Performance Notes

| Capability | FoundationPose Tracking |
|------------|------------------------|
| Jetson Thor | >120 FPS |

Note: Performance numbers in content file are illustrative CPU vs GPU comparisons, not official benchmarks.
Source: [FoundationPose](https://nvidia-isaac-ros.github.io/concepts/pose_estimation/foundationpose/index.html)

## Corrections Needed

**None identified** - Content appears accurate as of 2026-01-21

The file was validated on 2026-01-20 and reflects current information:
- Version 4.0 with v4.0-1 update
- ROS 2 Jazzy support
- JetPack 6.x (Orin) and 7.x (Thor) platforms
- Xavier deprecation noted
- NITROS documentation accurate

## Sources

- [Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)
- [Isaac ROS Getting Started](https://nvidia-isaac-ros.github.io/getting_started/index.html)
- [NITROS Documentation](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html)
- [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html)
- [FoundationPose](https://nvidia-isaac-ros.github.io/concepts/pose_estimation/foundationpose/index.html)
- [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [NVIDIA Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [NVIDIA Developer Portal - Isaac ROS](https://developer.nvidia.com/isaac/ros)
- [Isaac ROS 4.0 Announcement](https://forums.developer.nvidia.com/t/nvidia-isaac-ros-4-0-for-thor-has-arrived/352109)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/Related-Projects/Nvidia-ROS2-Projects.html)
