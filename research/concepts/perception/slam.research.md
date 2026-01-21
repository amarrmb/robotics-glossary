# SLAM Research Notes

**Research Date:** 2026-01-20

## Current Version Numbers

### NVIDIA Isaac ROS
- **Isaac ROS Version:** 4.0.1 (released December 10, 2025)
  - Source: https://nvidia-isaac-ros.github.io/releases/index.html
- **cuVSLAM:** Version 14 (as of October 24, 2025)
  - Source: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
- **ROS 2 Support:** Jazzy (primary), running on Jetson or x86_64 with NVIDIA GPU
- **Supported Platforms:**
  - Jetson Thor with JetPack 7.0
  - x86_64: Ampere or newer GPU, Ubuntu 24.04, CUDA 13.0+, NVIDIA Driver 580+

### ROS 2 SLAM Toolbox
- **Kilted:** Version 2.9.0
- **Jazzy:** Version 2.8.3
- **Humble:** Version 2.6.10
  - Source: https://docs.ros.org/en/kilted/p/slam_toolbox/

## Corrections Needed

### cuVSLAM Performance Claims
- **Current claim:** "60+ FPS on Jetson Orin"
- **Actual:** Documentation shows benchmarks on Jetson AGX Xavier achieving 0.94% translation error, 0.0019 deg/m rotation at 0.007s runtime. No specific "60+ FPS on Orin" claim found in official docs.
- **Recommendation:** Verify or update performance claims with official benchmarks

### Google Cartographer Status
- **Current:** Listed as active option for 2D/3D SLAM
- **Actual:** Google Cartographer is **no longer actively maintained**. Only critical PRs may be merged occasionally.
- **Source:** https://github.com/cartographer-project/cartographer
- **Recommendation:** Add note about maintenance status or suggest SLAM Toolbox as actively maintained alternative

### Launch Command Verification
- Current example: `ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py`
- Should verify this is still the current launch file name in Isaac ROS 4.0

## Key Facts Verified

### cuVSLAM Features (Confirmed)
- GPU-accelerated Visual SLAM
- Supports stereo RGB cameras with multi-camera mode
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
- Source: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html

### SLAM Approaches Listed (Status)
| Approach | Status | Notes |
|----------|--------|-------|
| ORB-SLAM3 | Active | Last major release included Eigen optimization (16% speedup). Research ongoing (SuperPoint-SLAM3 in 2025) |
| Cartographer | **Deprecated** | No longer actively maintained by Google |
| SLAM Toolbox | Active | Recommended for ROS 2, actively maintained |
| LIO-SAM | Active | Multiple ROS 2 ports available |
| DROID-SLAM | Active | Deep learning SLAM, derivatives being published (MINI-DROID-SLAM 2025) |
| VINS-Mono | Active | Visual-inertial |

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
- Nav2 SLAM Tutorial: https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html

### Academic/Other
- ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3
- ORB-SLAM3 Paper: https://arxiv.org/abs/2007.11898
- Cartographer (deprecated): https://github.com/cartographer-project/cartographer
- LIO-SAM GitHub: https://github.com/TixiaoShan/LIO-SAM
- DROID-SLAM GitHub: https://github.com/princeton-vl/DROID-SLAM
- DROID-SLAM Paper: https://arxiv.org/abs/2108.10869

## Summary of Required Updates

1. **Add deprecation note for Cartographer** - Google no longer maintains it
2. **Verify cuVSLAM "60+ FPS on Orin" claim** - Not found in official docs
3. **Update Isaac ROS version context** - Now at 4.0.1, ROS 2 Jazzy primary
4. **Verify launch file commands** - May have changed in Isaac ROS 4.0
5. **Consider adding SLAM Toolbox** - Actively maintained ROS 2 native SLAM
