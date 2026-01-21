# SLAM Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/concepts/perception/slam.mdx

## Current Version Numbers (January 2026)

### NVIDIA Isaac ROS
- **Isaac ROS Version:** 4.0.1 (December 10, 2025), 4.0.0 (October 24, 2025)
  - Source: https://nvidia-isaac-ros.github.io/releases/index.html
- **cuVSLAM:** Version 14 (as of October 24, 2025)
  - Source: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
- **ROS 2 Support:** Jazzy (primary), Humble
- **Supported Platforms:**
  - Jetson AGX Xavier/Orin with JetPack
  - x86_64 with NVIDIA GPU

### ROS 2 SLAM Toolbox
- **Kilted:** Version 2.9.0 (ROS 2 Kilted released May 2025)
- **Jazzy:** Version 2.8.3
- **Humble:** Version 2.6.10
  - Source: https://docs.ros.org/en/jazzy/p/slam_toolbox/

### Other SLAM Systems
- **ORB-SLAM3:** Active, paper from 2020 (arXiv:2007.11898)
- **DROID-SLAM:** Maintained, tested up to PyTorch 2.7
- **VINS-Mono/VINS-Fusion:** ROS 1 focused (Kinetic/Melodic)
- **Google Cartographer:** **No longer actively maintained**

## Corrections Needed

### Version/Feature Updates
1. **cuVSLAM multi-camera support:** Now supports up to 32 cameras (16 stereo pairs)
2. **cuVSLAM performance:** 0.94% translation error, 0.0019 deg/m rotation on KITTI at 0.007s runtime
3. **nvblox enhancements:**
   - Fine-grained ~1cm voxel resolution
   - Bounded workspace support
   - Multi-camera (up to 3 cameras) and 3D LiDAR input
   - Dynamic mode and people detection

### Google Cartographer Status
- **Current file:** Does not mention Cartographer
- **Status:** Google Cartographer is **no longer actively maintained**
- **Source:** https://github.com/cartographer-project/cartographer
- **Note:** File correctly lists SLAM Toolbox which is actively maintained

### Minor Items
- File correctly references SLAM Toolbox Jazzy documentation
- ORB-SLAM3 and DROID-SLAM links are valid

## Key Facts Verified

### cuVSLAM Features (Confirmed)
- GPU-accelerated Visual SLAM
- Supports stereo RGB cameras with multi-camera mode (up to 32 cameras)
- IMU fusion support (Visual-Inertial)
- Map saving and localization capabilities
- Best-in-class performance on KITTI benchmark
- Source: https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html

### nvblox Features (Confirmed)
- TSDF-based 3D reconstruction
- Real-time mesh and occupancy grid generation
- Three modes: static, people reconstruction, dynamic objects
- 100x faster than CPU methods (NVIDIA claim)
- Nav2 costmap integration
- Multi-sensor: up to 3 cameras + 3D LiDAR
- Source: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html

### SLAM Approaches Status
| Approach | Status | Notes |
|----------|--------|-------|
| ORB-SLAM3 | Active | Research ongoing (SuperPoint-SLAM3 June 2025) |
| Cartographer | **Deprecated** | No longer actively maintained by Google |
| SLAM Toolbox | Active | Recommended for ROS 2, actively maintained |
| LIO-SAM | Active | Multiple ROS 2 ports available |
| DROID-SLAM | Active | Deep learning SLAM, MINI-DROID-SLAM published Sept 2025 |
| VINS-Mono/Fusion | Stable | ROS 1 only, research variants for ROS 2 exist |

## Source URLs

### NVIDIA Official
- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
- cuVSLAM Concepts: https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html
- Isaac ROS Nvblox: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html
- Nvblox Concepts: https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html
- Isaac ROS Releases: https://nvidia-isaac-ros.github.io/releases/index.html
- GitHub (Visual SLAM): https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
- GitHub (Nvblox): https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox

### ROS Official
- SLAM Toolbox (Kilted): https://docs.ros.org/en/kilted/p/slam_toolbox/
- SLAM Toolbox (Jazzy): https://docs.ros.org/en/jazzy/p/slam_toolbox/
- SLAM Toolbox (Humble): https://docs.ros.org/en/ros2_packages/humble/api/slam_toolbox/
- SLAM Toolbox GitHub: https://github.com/SteveMacenski/slam_toolbox
- Nav2 SLAM Tutorial: https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html
- ROS 2 Kilted Release: https://docs.ros.org/en/jazzy/Releases/Release-Kilted-Kaiju.html

### Academic/Other
- ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3
- ORB-SLAM3 Paper: https://arxiv.org/abs/2007.11898
- Cartographer (deprecated): https://github.com/cartographer-project/cartographer
- LIO-SAM GitHub: https://github.com/TixiaoShan/LIO-SAM
- DROID-SLAM GitHub: https://github.com/princeton-vl/DROID-SLAM
- DROID-SLAM Paper: https://arxiv.org/abs/2108.10869
- VINS-Fusion GitHub: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

## Summary of Validation

### Current File is Accurate
- cuVSLAM description and links: Valid
- nvblox description: Valid (could add multi-sensor details)
- SLAM Toolbox link: Valid (Jazzy 2.8.3)
- ORB-SLAM3 paper link: Valid
- DROID-SLAM link: Valid
- Isaac ROS Visual SLAM launch examples: Should be verified for 4.0

### Recommended Minor Updates
1. Note that cuVSLAM supports multi-camera (up to 32 cameras)
2. Note nvblox multi-sensor support (3D LiDAR + multiple cameras)
3. VINS-Mono/VINS-Fusion are ROS 1 focused - add clarification
4. Isaac ROS now at 4.0.1 - verify launch file commands still current

### No Major Corrections Required
The current content file is accurate. The information about SLAM types, approaches, and NVIDIA solutions is valid. All source URLs are functional.
