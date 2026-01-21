# LiDAR Research Notes

**Research Date:** January 21, 2026
**File:** src/content/docs/hardware/sensors/lidar.mdx
**Validated By:** Claude Code

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

### Isaac Sim
- **Isaac Sim** supports ROS 2 Jazzy and ROS 2 Humble
- On Ubuntu 24.04, automatically loads internal ROS 2 Jazzy libraries
- On Ubuntu 22.04, loads ROS 2 Humble libraries
- Can simulate LiDAR sensors with realistic noise models

### ROSCon 2025 Announcements
- Isaac ROS 4.0 announced at ROSCon October 2025
- Now available on NVIDIA Jetson Thor platform
- NVIDIA contributing GPU-aware abstractions directly to ROS 2
- Open-sourcing Greenwave Monitor for performance bottleneck identification

### Corrections Needed
- Current file mentions `isaac_ros_nvblox` and `isaac_ros_pointcloud_utils` - these are accurate
- File states "Isaac ROS 4.0 (current release, requires ROS 2 Jazzy)" - **ACCURATE**
- nvblox works with depth cameras AND/OR 3D LiDAR (file could clarify)

### Sources
- https://nvidia-isaac-ros.github.io/
- https://nvidia-isaac-ros.github.io/releases/index.html
- https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html
- https://developer.nvidia.com/isaac/ros
- https://blogs.nvidia.com/blog/roscon-2025-open-framework-robotics/

---

## ROS 2 Packages

### Current ROS 2 Distributions (January 2026)
- **Kilted Kaiju** - Released May 23, 2025, supported until November 2026 (non-LTS)
  - Recommended Gazebo release: Ionic
  - sensor_msgs version: 5.5.1
- **Jazzy Jalisco** - Released May 2024, supported until May 2029 (LTS)
  - sensor_msgs version: 5.3.6
- **Humble Hawksbill** - Released May 2022, supported until May 2027 (LTS)
- **Lyrical Luth** - Planned for May 2026

### pointcloud_to_laserscan
- Current version: 2.0.2 (Jazzy)
- Converts sensor_msgs/PointCloud2 to sensor_msgs/LaserScan and back
- Available for Jazzy, Kilted, Rolling distributions
- Key parameters: min_height, max_height, angle_min, angle_max, angle_increment

### laser_filters
- Current version: 2.0.9-1 (Jazzy, as of July 3, 2025)
- Released into Humble distro: July 3, 2025
- Released into Jazzy distro: July 3, 2025
- Released into Kilted distro: May 30, 2025
- Released into Rolling distro: June 30, 2025
- Filters for 2D planar laser scanners (sensor_msgs/LaserScan type)

### pcl_ros
- Part of PCL (Point Cloud Library) ROS interface stack
- Preferred bridge for 3D point cloud applications in ROS
- Maintained as of 2025-01-22 per index.ros.org
- Recent fixes: PCD timestamp format output, split pcl_ros_filter into separate library

### sensor_msgs Notes
- **PointCloud** message deprecated as of Foxy - use PointCloud2 instead
- **PointCloud2** holds N-dimensional points with optional normals, intensity, etc.
- Utilities: point_cloud2_iterator.hpp, point_cloud_conversion.hpp

### Corrections Needed
- Package list in current file is accurate
- No changes needed for package names

### Sources
- https://docs.ros.org/en/jazzy/p/pointcloud_to_laserscan/
- https://docs.ros.org/en/jazzy/p/sensor_msgs/
- https://docs.ros.org/en/ros2_packages/kilted/api/sensor_msgs/msg/PointCloud2.html
- https://index.ros.org/p/laser_filters/
- https://index.ros.org/p/pcl_ros/
- https://docs.ros.org/en/kilted/Releases.html
- https://docs.ros.org/en/jazzy/Releases/Release-Kilted-Kaiju.html

---

## LiDAR Sensors

### Velodyne VLP-16 (Puck)
- **Current Status:** Now sold through Ouster (Velodyne merged with Ouster in 2023)
- **Specifications (from Ouster 2025):**
  - 16 channels
  - Range: up to 200m
  - ~600,000 points/second
  - 360° horizontal FOV, 40° vertical FOV
  - Supports dual return mode
  - Weight: 590g (without cabling/interfacing boxes)
  - Dimensions: 72mm × 89mm (height × width)
- **Price:** ~$4,000 (file states $4,000 - accurate)
- **Note:** Good for drone/UAV applications due to size/weight

### Ouster OS1
- **Current Status:** Rev 7 is latest generation, uses L3 chip
- **2025 Update:** July 2025 - NDAA compliant OS1 announced for drones/UAVs
- **Specifications (from Ouster datasheet Rev 7):**
  - Up to 128 channels (64 or 128 options)
  - Up to 5.2M points/second
  - Range: 90m @ 10% reflectivity, up to 400m max
  - 360° horizontal, 42.4° vertical FOV
  - IP68 and IP69K rated
  - Operating voltage: 9V-34V (12V or 24V nominal)
  - NDAA compliant
- **Price:** File states $4,000-8,000 - pricing varies by channel count
- **Use cases:** Mid-range sensor for security, smart infrastructure, autonomous vehicles

### Livox Mid-360
- **Specifications (from Livox official):**
  - 360° horizontal FOV (first Livox to achieve this)
  - 59° vertical FOV
  - 200,000 points/second
  - Range: 40m typical
  - Min detection: 0.1-0.2m (zero blind zone)
  - Dimensions: 65 × 65 × 60 mm
  - Weight: 265g
  - Power: up to 14W in self-heating mode (-20°C to 0°C)
  - Operating temp: -20°C to +55°C (ensure shell <80°C)
- **Technology:** Rotating mirror hybrid-solid technology (not pure solid-state)
- **Compatibility:** ROS, PTPv2, GPS synchronization
- **Price:** File states ~$1,000 - available through DJI Store
- **Applications:** Mobile robotics, autonomous navigation, SLAM, obstacle avoidance

### Hesai AT128
- **Specifications (from Hesai official):**
  - 128 channels (128 VCSEL arrays at 905nm)
  - Range: 200m @ 10% reflectivity, 260m max
  - Effective ground detection: up to 70m
  - 120° FOV
  - 1.53M points/second
  - Resolution: 1200×128
  - Accuracy: ±5cm
  - IP6K7 and IP6K9K rated
  - AEC-Q certified (automotive grade)
  - Over 50 DV tests (electrical, mechanical, environmental, EMC)
- **2025-2026 Deployments:**
  - Primary LiDAR for Pony.ai 7th-gen Robotaxi (4 units per vehicle)
  - DiDi/GAC Aion L4 Robotaxi (mass production 2025, deployment 2026)
  - Selected by NVIDIA for DRIVE Hyperion 10 (L4 fleet deployment)
- **Price:** File states $1,000+ - consistent with automotive-grade pricing

### RPLidar A1/A2 (SLAMTEC)
- **A1 Specifications (from SLAMTEC official):**
  - 2D 360° laser scanner (triangulation ranging)
  - Range: up to 12 meters (6m for A1M8-R4 and earlier)
  - Scan frequency: up to 10 Hz (configurable)
  - 8,000+ measurements/second (RPVision engine)
  - Power: 5V, 3V TTL serial data
  - OPTMAG technology (wireless power/optical communication) - prolongs lifespan
- **A2 Specifications (from SLAMTEC official):**
  - Dimensions: 76mm × 41mm
  - Weight: 190g
  - Range: up to 16 meters (improved from A1)
  - 8,000+ measurements/second (RPVision3.0 engine)
  - Brushless motor - 5 year lifespan (24/7 operation)
  - Silent operation due to brushless motor
- **Price:** File states $100-200 - consistent with "cost-effective" description
- **Compatibility:** ROS1, ROS2, SDK support, Jetson NANO, Raspberry Pi

### Luminar Iris
- **Specifications:**
  - Range: 250m (dark objects), up to 500m (bright objects)
  - 120° horizontal FoV, 26° dynamic vertical FoV
  - 2-axis scanning mirrors
  - 1550nm wavelength (40x higher power than 905nm, eye-safe)
  - Operating temp: -40°C to +85°C
  - Weight: <2 lbs
  - Camera-like resolution: >300 points per square degree
- **CRITICAL UPDATE (December 2025):**
  - Volvo dropped Luminar from 2026 ES90/EX90 models
  - Luminar filed Chapter 11 bankruptcy in December 2025
  - Assets: $100-500M, Liabilities: $500M-$1B
- **Correction Needed:** File lists "Luminar Iris — Long-range automotive" - consider adding note about bankruptcy status or replacing with alternative

### Sources
- https://ouster.com/products/hardware/vlp-16
- https://ouster.com/products/hardware/os1-lidar-sensor
- https://www.livoxtech.com/mid-360
- https://www.livoxtech.com/mid-360/specs
- https://www.hesaitech.com/product/at128/
- https://www.slamtec.com/en/lidar/a1
- https://www.slamtec.com/en/Lidar/a2
- https://en.wikipedia.org/wiki/Luminar_Technologies

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

1. **Velodyne/Ouster:** File correctly notes "(now sold through Ouster)" - **ACCURATE**
2. **Ouster OS1 pricing:** File states $4,000-8,000 - reasonable range by channel count
3. **Livox Mid-360:** File correctly notes "(hybrid)" - **ACCURATE**
4. **Intel L515:** File correctly notes "(discontinued Feb 2022)" and alternatives - **ACCURATE**
5. **Isaac ROS:** File states "Isaac ROS 4.0 (current release, requires ROS 2 Jazzy)" - **ACCURATE**
6. **Luminar Iris:** **NEEDS UPDATE** - Luminar filed bankruptcy December 2025, dropped by Volvo
7. **VLP-16 specs:** Points/sec should be ~600,000 (not 300K implied by general table)

## Information Verified as Accurate
- Basic LiDAR working principle (time-of-flight)
- LiDAR types (mechanical, solid-state, flash)
- Key specifications ranges (mostly accurate)
- ROS 2 message types (sensor_msgs/PointCloud2, sensor_msgs/LaserScan)
- Package names (pointcloud_to_laserscan, laser_filters, pcl_ros, nvblox)
- Comparison table (LiDAR vs Camera vs Radar)
- SLAM algorithms (ICP, NDT)
- Isaac ROS version and ROS 2 Jazzy requirement
- Most sensor examples and pricing

## Priority Corrections
1. **HIGH:** Update Luminar Iris entry - add bankruptcy note or replace with alternative (e.g., Hesai FT120 for automotive)
