# IMU Research Notes

**Research Date:** 2026-01-21
**File:** src/content/docs/hardware/sensors/imu.mdx

---

## NVIDIA Isaac ROS

### Current Versions
- **Isaac ROS 4.0** - Latest release (v4.0.0, October 2025)
  - Update 1 available: v4.0-1
  - Added ROS 2 Jazzy support (October 24, 2025)
- **Isaac ROS 3.2** - Released at CES 2025 (January 7, 2025)
- **cuVSLAM** - Version 11+ (renamed from ELBRUS VSLAM in DP3.1 release)
  - 2025 arXiv paper (2506.04359)

### Visual-Inertial SLAM
- `isaac_ros_visual_slam` uses cuVSLAM for GPU-accelerated VSLAM
- Supports IMU fusion via `enable_imu_fusion` parameter
- Subscribes to `/visual_slam/imu: sensor_msgs/msg/Imu`
- Supports stereo visual-inertial odometry (SVIO)
- Sub-1% trajectory errors claimed on KITTI benchmark

### Corrections Needed
- File mentions `isaac_ros_visual_slam` - accurate
- Consider updating to reference Isaac ROS 4.0 availability

### Sources
- https://nvidia-isaac-ros.github.io/releases/index.html
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
- https://developer.nvidia.com/isaac/ros
- https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html
- https://forums.developer.nvidia.com/t/ces-2025-isaac-ros-3-2-and-platform-updates/319021
- https://blogs.nvidia.com/blog/roscon-2025-open-framework-robotics/

---

## ROS 2 Packages

### Current ROS 2 Distributions (January 2026)
- **Kilted Kaiju** - Released May 23, 2025, supported until November 2026
- **Jazzy Jalisco** - Released May 2024, supported until May 2029 (LTS)
- **Humble Hawksbill** - Released May 2022, supported until May 2027 (LTS)

### robot_localization
- Available for: humble, jazzy, kilted, rolling
- Released into kilted: August 29, 2025
- Released into jazzy: March 21, 2025
- Provides EKF/UKF sensor fusion (ekf_node, ukf_node)
- Install: `sudo apt install ros-<distro>-robot-localization`

### imu_filter_madgwick
- Latest version: 2.2.0 (Rolling), 2.1.5 (Humble/Iron/Jazzy)
- Version 1.2.7 released: April 29, 2025
- Based on Sebastian Madgwick's algorithm
- Fuses accelerometer, gyroscope, and optional magnetometer into orientation
- Part of `imu_tools` metapackage

### imu_complementary_filter
- Part of `imu_tools` metapackage
- Available for: humble, jazzy, kilted, rolling
- Based on Roberto G. Valenti et al. algorithm
- Install: `sudo apt install ros-<distro>-imu-tools`

### sensor_msgs/Imu Message
- Fields confirmed accurate:
  - `std_msgs/Header header`
  - `geometry_msgs/Quaternion orientation`
  - `geometry_msgs/Vector3 angular_velocity`
  - `geometry_msgs/Vector3 linear_acceleration`
  - Plus covariance arrays for each measurement

### Corrections Needed
- Package list in current file is accurate
- Message fields description is accurate

### Sources
- https://index.ros.org/p/robot_localization/
- https://docs.ros.org/en/ros2_packages/humble/api/imu_filter_madgwick/
- https://index.ros.org/p/imu_complementary_filter/
- https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html
- https://docs.ros.org/en/kilted/Releases/Release-Kilted-Kaiju.html

---

## VIO Tools

### VINS-Mono
- Published: IEEE Transactions on Robotics, 2018
- Monocular visual-inertial state estimator
- Can handle unsynchronized sensors (unlike ORB-SLAM3 mono-inertial)
- No major version updates found for 2025-2026

### ORB-SLAM3
- Published: IEEE Transactions on Robotics, 2021
- Supports visual, visual-inertial, and multimap SLAM
- Works with monocular, stereo, RGB-D cameras
- Supports pin-hole and fisheye lens models
- Generally outperforms VINS-Mono (2x better in some studies)
- No major version updates found for 2025-2026

### Newer Developments
- **VINGS-Mono** (2025) - Visual-Inertial Gaussian Splatting SLAM
- Emerging methods using neural representations

### Corrections Needed
- File mentions VINS-Mono, ORB-SLAM3 - accurate, still widely used
- Consider noting these are mature/stable systems (2018, 2021)

### Sources
- https://github.com/HKUST-Aerial-Robotics/VINS-Mono
- https://arxiv.org/html/2501.08286v1 (VINGS-Mono)

---

## IMU Chips

### MPU-6050
- Manufacturer: TDK InvenSense
- Grade: Consumer
- Type: 6-axis (accelerometer + gyroscope)
- Very common, low-cost option for simple projects
- Note: Older design, still widely used in hobbyist projects

### BMI088
- Manufacturer: Bosch Sensortec
- Grade: Industrial
- Type: 6-axis IMU
- Accelerometer: ±3g, ±6g, ±12g, ±24g (extended range for vibration)
- Gyroscope: ±125°/s to ±2000°/s
- Package: LGA-16 (3x4.5mm)
- Good for drones (designed for vibration rejection)
- Widely used in flight controllers

### ICM-42688-P
- Manufacturer: TDK InvenSense
- Grade: Industrial
- Type: 6-axis MotionTracking device
- Accelerometer: ±2g, ±4g, ±8g, ±16g
- Gyroscope: ±15.625°/s to ±2000°/s (8 configurable ranges)
- Package: 2.5x3x0.91mm (14-pin LGA)
- 2KB FIFO buffer
- Low noise specifications
- Supports I3C, I2C, SPI interfaces
- APEX features: pedometer, tilt detection
- Note: Some temperature sensitivity reported

### ADIS16470
- Manufacturer: Analog Devices
- Grade: Tactical
- Type: High-performance 6-DOF IMU
- SPI interface
- Higher accuracy than consumer/industrial grades

### Corrections Needed
- Chip table is accurate
- Grade classifications are appropriate

### Sources
- https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=MPU-6050
- https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/
- https://invensense.tdk.com/wp-content/uploads/2022/12/DS-000347-ICM-42688-P-v1.7.pdf
- https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/
- https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
- https://www.utmel.com/components/bmi088-imu-bmi088-datasheet-pinout-bmi088-vs-mpu6050

---

## Summary of Corrections Needed

1. **Isaac ROS:** Isaac ROS 4.0 is now available (October 2025 release)
2. **ROS 2:** Kilted Kaiju released May 2025; all mentioned packages support it
3. **VIO tools:** VINS-Mono (2018) and ORB-SLAM3 (2021) are mature, stable systems
4. **No major errors found** in current content

## Information Verified as Accurate

- IMU component description (accelerometer, gyroscope, magnetometer)
- Sensor characteristics and problems (drift, magnetic interference)
- Specifications table (noise density, bias stability, sample rate, range)
- IMU grade classifications and cost ranges
- Sensor fusion methods (complementary filter, Kalman filter)
- ROS 2 message type (`sensor_msgs/Imu`)
- Package names (`robot_localization`, `imu_filter_madgwick`, `imu_complementary_filter`, `isaac_ros_visual_slam`)
- IMU chip selection table
- Calibration requirements
