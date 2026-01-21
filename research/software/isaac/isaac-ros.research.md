# Isaac ROS Research Notes

**Research Date:** 2026-01-20
**Current File:** src/content/docs/software/isaac/isaac-ros.mdx

## Current Version Status

### File Claims (Needs Update)
- States: Isaac ROS 3.2, JetPack 6.1, ROS 2 Humble

### Actual Current Versions
- **Isaac ROS 4.0.0** (latest stable) - released October/November 2025
  - Available as v4.0.0 and v4.0-1 on GitHub
- **ROS 2 Jazzy** - now the supported ROS distribution (not Humble)
- **JetPack 7.0/7.1** - for Jetson Thor support
- **JetPack 6.x** - still supported for Orin platforms
- Source: [Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

## Key Corrections Needed

### Version Badge (Line 16-18)
- **Current:** "Isaac ROS 3.2 — Aligned with JetPack 6.1 and ROS 2 Humble"
- **Should be:** "Isaac ROS 4.0 — Supports ROS 2 Jazzy, JetPack 6.x (Orin) and JetPack 7.x (Thor)"

### ROS 2 Distribution
- File references ROS 2 Humble throughout
- Isaac ROS 4.0 is designed and tested for **ROS 2 Jazzy**
- Source: [Getting Started](https://nvidia-isaac-ros.github.io/getting_started/index.html)

## Platform Support Changes

### New Support (Isaac ROS 4.0)
- **Jetson AGX Thor** with JetPack 7.0/7.1
- **x86_64 with RTX 5090**
- CUDA 13.0 on Thor platforms
- Source: [JetPack 7.1 Announcement](https://jetsonhacks.com/2026/01/12/jetpack-7-1-and-jetson-t4000-now-available/)

### Removed Support (Since Isaac ROS 2.1)
- JetPack 5.1.2
- Ubuntu 20.04
- **Jetson Xavier** (AGX Xavier, Xavier NX) - no longer supported
- CenterPose on Jetson - removed
- Source: [Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

### Limited Support
- **ZED cameras** - only compatible with Isaac ROS 3.2.x; awaiting ZED SDK update for JetPack 7/Thor
- **RealSense SDK** - stability issues on JetPack 7
- Source: [ZED Isaac ROS Integration](https://www.stereolabs.com/docs/isaac-ros)

## Performance Benchmarks (Isaac ROS 4.0)

| Node | AGX Thor | x86_64 + RTX 5090 |
|------|----------|-------------------|
| AprilTag (720p) | 392 fps | 596 fps |
| FoundationPose (720p) | 3.92 fps | 10.1 fps |
| Stereo Disparity (1080p) | 217 fps | 863 fps |

Source: [isaac_ros_benchmark GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark)

## NITROS Updates

- Core concept unchanged: zero-copy GPU memory transport
- Still uses type adaptation (REP-2007) and type negotiation
- **Requirement:** All NITROS-accelerated nodes must run in same process for zero-copy
- New packages:
  - `isaac_ros_managed_nitros` - wrapper for adding NITROS to third-party CUDA nodes
  - `isaac_ros_nitros_bridge` - inter-process communication
  - `isaac_ros_pynitros` - Python NITROS implementation
- Source: [NITROS Documentation](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html)

## Package Updates

### New/Updated Packages
- `isaac_ros_mapping_and_localization` - consolidated from isaac_ros_map_localization
  - Improved localization with learning-based descriptors
  - Improved map creation with FoundationStereo
- `isaac_ros_visual_slam` - updated with cuVSLAM 11
- `isaac_ros_cumotion` - MoveIt 2.5.8 compatibility fix
- Source: [Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

### Package Table Verification
- Listed packages appear current
- `isaac_ros_yolov8` - confirmed present
- `isaac_ros_foundationpose` - confirmed present
- `isaac_ros_cumotion` - confirmed present

## Installation Changes

- **Docker Engine 27.2.0+** required (previously not specified)
- Dev container functionality moved to **Isaac ROS CLI**
- For Thor: "-igpu" container tag no longer required (SBSA stack)
- Source: [Getting Started](https://nvidia-isaac-ros.github.io/getting_started/index.html)

## Documentation Links (Verified)

- Main docs: https://nvidia-isaac-ros.github.io/ (valid)
- GitHub: https://github.com/NVIDIA-ISAAC-ROS (valid)
- NITROS: https://nvidia-isaac-ros.github.io/concepts/nitros/index.html (valid)
- Release notes: https://nvidia-isaac-ros.github.io/releases/index.html (valid)

## Summary of Required Changes

1. **Update version badge** from 3.2 to 4.0
2. **Update ROS distribution** from Humble to Jazzy
3. **Update JetPack references** to include JetPack 7.x for Thor
4. **Note deprecation** of Jetson Xavier support
5. **Update Docker requirement** to 27.2.0+
6. **Consider adding** Thor benchmarks to performance comparison table
7. **Update prerequisites** to reflect current platform support
