# LiDAR Research Notes

**Research Date:** January 2026
**File:** src/content/docs/hardware/sensors/lidar.mdx

---

## NVIDIA Isaac ROS

### Current Versions
- **Isaac ROS 4.0** - Latest release (tagged v4.0.0 on GitHub)
- **Isaac ROS 3.2** - Still receiving updates (currently at v3.2-15)
- **Isaac ROS 2.1** - Deprecated as of June 30, 2025; Ubuntu 20.04 Focal no longer supported

### Key Information
- Isaac ROS packages designed and tested for **ROS 2 Jazzy**
- Compatible with Jetson or x86_64 systems with NVIDIA GPU
- nvblox provides GPU-accelerated 3D reconstruction from depth cameras and/or 3D LiDAR
- Uses TSDF (Truncated Signed Distance Function) stored in 3D voxel grid

### Corrections Needed
- Current file mentions `isaac_ros_nvblox` and `isaac_ros_pointcloud_utils` - these are accurate
- Should note ROS 2 Jazzy compatibility requirement

### Sources
- https://nvidia-isaac-ros.github.io/
- https://nvidia-isaac-ros.github.io/releases/index.html
- https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html

---

## ROS 2 Packages

### Current ROS 2 Distributions (January 2026)
- **Kilted Kaiju** - Released May 2025, supported until November 2026
- **Jazzy Jalisco** - Released May 2024, supported until May 2029 (LTS)
- **Humble Hawksbill** - Released May 2022, supported until May 2027 (LTS)
- **Lyrical Luth** - Planned for May 2026

### pointcloud_to_laserscan
- Current version: 2.1.0 (Rolling)
- Converts sensor_msgs/PointCloud2 to sensor_msgs/LaserScan
- Available for Jazzy, Rolling distributions

### laser_filters
- Released into Humble distro: July 3, 2025
- Released into Rolling distro: June 30, 2025
- Released into Kilted distro: May 30, 2025

### pcl_ros
- Part of PCL (Point Cloud Library) ROS interface stack
- Preferred bridge for 3D point cloud applications in ROS

### Corrections Needed
- Package list in current file is accurate
- No changes needed for package names

### Sources
- https://docs.ros.org/en/jazzy/p/pointcloud_to_laserscan/
- https://index.ros.org/p/laser_filters/
- https://index.ros.org/p/pcl_ros/
- https://docs.ros.org/en/kilted/Releases.html

---

## LiDAR Sensors

### Velodyne VLP-16 (Puck)
- **Current Status:** Now sold through Ouster (Velodyne merged with Ouster in 2023)
- **Specifications:**
  - 16 channels
  - Range: up to 100m (some sources cite 200m)
  - ~300,000 points/second
  - 360° horizontal FOV, 30° vertical FOV (±15° up/down)
  - Rotational frequency: 5-20 Hz
  - Distance precision: ±3 cm
  - Power: ~8W
  - Weight: 590-830g
  - IP67 rated
  - Operating temp: -10°C to +60°C
- **Price:** ~$4,000 (file states $4,000 - accurate)

### Ouster OS1
- **Current Status:** Rev 7 is latest generation, uses L3 chip
- **Specifications:**
  - Up to 128 channels
  - Up to 5.2M points/second
  - Range: 90m @ 10% reflectivity, 200m+ max, up to 400m some configs
  - 360° horizontal, 45° vertical FOV
  - IP68 and IP69K rated
  - NDAA compliant
- **Price:** Not publicly listed; historical $18,000 for OS1-128 (2019)
- **Correction Needed:** File states $4,000-8,000 - may need verification

### Livox Mid-360
- **Specifications:**
  - 360° horizontal FOV (first Livox to achieve this)
  - 59° vertical FOV
  - 200,000 points/second
  - Range: 40m (8-15m practical for people detection)
  - Min detection: 10 cm
  - Dimensions: 65 × 65 × 60 mm
  - Weight: 265g
  - Power: up to 14W in self-heating mode
  - Operating temp: -20°C to +55°C
- **Price:** File states ~$1,000 - consistent with "lowest cost" description
- **Technology:** Rotating mirror hybrid-solid technology (not pure solid-state)

### Hesai AT128
- **Specifications:**
  - 128 channels
  - Range: 200m @ 10% reflectivity
  - Ground detection: up to 70m
  - 120° FOV
  - 1.53M points/second
  - Resolution: 1200x128
  - 905nm VCSEL arrays
  - IP6K7 and IP6K9K rated
  - Automotive Ethernet interface
  - AEC-Q certified (automotive grade)
- **Current Use:** Primary LiDAR for Pony.ai 7th-gen Robotaxi, DiDi/GAC Aion robotaxi (mass production 2026)
- **Price:** File states $1,000+ - verify with current pricing

### RPLidar A1/A2 (SLAMTEC)
- **A1 Specifications:**
  - 2D 360° laser scanner
  - Range: up to 12 meters
  - Scan frequency: up to 10 Hz
  - 8,000+ measurements/second
  - Dimensions: 96.8mm × 55mm
  - Weight: 170g
  - Power: 5V
  - OPTMAG wireless power/optical communication
- **A2 Specifications:**
  - Dimensions: 76mm × 41mm
  - Weight: 190g
- **Price:** File states $100-200 - consistent with "cost-effective" description
- **Compatibility:** ROS1, ROS2, SDK support

### Sources
- https://ouster.com/products/hardware/vlp-16
- https://ouster.com/products/hardware/os1-lidar-sensor
- https://www.livoxtech.com/mid-360
- https://www.hesaitech.com/product/at128/
- https://www.slamtec.com/en/lidar/a1

---

## Intel RealSense L515

### Discontinued Status
- **EOL Date:** February 2022
- Last order date: February 28, 2022
- Last ship date: March 31, 2022
- Intel wound down RealSense business

### Recommended Replacements
- Intel RealSense D455
- Intel RealSense D435i
- Note: D455/D435i are stereo depth cameras, NOT LiDAR

### Correction Needed
- File correctly notes "(discontinued)" - accurate
- Consider adding note about recommended replacements

### Sources
- https://support.intelrealsense.com/hc/en-us/articles/10705277802643-Are-the-Intel-RealSense-LiDAR-and-Tracking-Product-Families-Being-Discontinued
- https://www.therobotreport.com/intel-issues-end-of-life-notice-realsense-lidar/

---

## Summary of Corrections Needed

1. **Velodyne/Ouster:** Note that Velodyne merged with Ouster in 2023; VLP-16 now sold through Ouster
2. **Ouster OS1 pricing:** Verify $4,000-8,000 range (may vary by channel count)
3. **Livox Mid-360:** Clarify as "hybrid solid-state" not pure solid-state
4. **Intel L515:** Consider adding replacement recommendations (D455/D435i)
5. **Isaac ROS:** Update to mention ROS 2 Jazzy compatibility, note version 4.0 is current
6. **ROS 2:** Current stable LTS is Jazzy (supported until 2029)

## Information Verified as Accurate
- Basic LiDAR working principle (time-of-flight)
- LiDAR types (mechanical, solid-state, flash)
- Key specifications ranges
- ROS 2 message types (sensor_msgs/PointCloud2, sensor_msgs/LaserScan)
- Package names (pointcloud_to_laserscan, laser_filters, pcl_ros, nvblox)
- Comparison table (LiDAR vs Camera vs Radar)
- SLAM algorithms (ICP, NDT)
