# Motion Planning Research Notes

**Researched:** 2026-01-20

## Current Version Numbers

### NVIDIA Isaac Platform
- **Isaac Sim:** 4.5.0 (current documented), 5.1.0 (latest)
- **Isaac ROS:** 3.2 (announced CES 2025)
- **cuMotion:** Developer Preview in Isaac 3.0, uses cuRobo backend
- **nvblox:** Part of Isaac Perceptor, CUDA-accelerated 3D reconstruction
- Sources:
  - https://docs.isaacsim.omniverse.nvidia.com/4.5.0/manipulators/manipulators_curobo.html
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/manipulators_curobo.html
  - https://forums.developer.nvidia.com/t/ces-2025-isaac-ros-3-2-and-platform-updates/319021

### ROS 2 Distributions
- **Kilted Kaiju:** Released May 23, 2025, EOL November 2026 (standard release)
  - First ROS release with Zenoh as Tier 1 middleware
  - Recommended Gazebo: Ionic
- **Jazzy Jalisco:** LTS release, still supported
- Sources:
  - https://docs.ros.org/en/kilted/Releases/Release-Kilted-Kaiju.html
  - https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released

### MoveIt 2
- **MoveIt 2.10:** LTS release for ROS 2 Jazzy
- Binary installs available for: Jazzy, Rolling (Ubuntu 24.04), Humble (Ubuntu 22.04)
- Humble support being phased out
- Sources:
  - https://moveit.ai/release/jazzy/rolling/2024/06/30/New-MoveIt-LTS-release-for-ROS-2-Jazzy.html
  - https://moveit.ai/install-moveit2/binary/

### OMPL
- **OMPL 1.7.0:** Current release
- PyPI package last released April 12, 2025
- GitHub active (plannerarena updated Jan 2, 2026)
- Sources:
  - https://ompl.kavrakilab.org/
  - https://github.com/ompl/ompl

### Drake
- Actively developed by Toyota Research Institute
- GCS (Graphs of Convex Sets) trajectory optimization available
- GitHub active as of January 2026
- MoveIt 2 integration discussions ongoing (March 2025)
- Sources:
  - https://drake.mit.edu/
  - https://github.com/RobotLocomotion/drake
  - https://picknik.ai/2025/03/11/How-to-Use-a-Dragon-s-Algorithm-Integrating-Drake-with-MoveIt-2.html

### Tesseract
- ROS-Industrial motion planning environment
- Python packages available on PyPI (Python 3.7-3.10)
- Supports: Ubuntu 20.04, 22.04, Windows 10+
- Integrates: OMPL, TrajOpt, TrajOpt IFOPT, Descartes
- Sources:
  - https://github.com/tesseract-robotics/tesseract
  - https://github.com/tesseract-robotics/tesseract_planning
  - https://pypi.org/project/tesseract-robotics/

## Corrections Needed

### Code Example (lines 99-113)
- **Issue:** Code example shows `from isaac_ros_cumotion import CumotionPlanner` - this may not be accurate API
- **Reality:** cuMotion integrates as a MoveIt 2 plugin, not typically used directly via Python import
- **Recommendation:** Verify exact API or replace with conceptual example showing MoveIt 2 plugin usage

### Common Tools Table (lines 159-168)
- **MoveIt 2:** Accurate - most popular, multiple backends
- **OMPL:** Accurate - sampling-based planners, version 1.7.0
- **cuMotion:** Accurate - NVIDIA GPU-accelerated
- **Drake:** Accurate - optimization-based, TRI-developed
- **Tesseract:** Accurate - industrial focused, ROS-Industrial

### Minor Notes
- nvblox mentioned on line 127 is correct (GPU-accelerated SDF)
- cuCollision reference on line 127 - verify this is the correct name (may be part of cuMotion/cuRobo)

## Deprecation Notes
- NVIDIA Omniverse Launcher deprecated October 1, 2025
- Isaac Sim 4.2.0 is outdated, use 4.5.0+ documentation
- ROS 2 Humble support in MoveIt being phased out

## Summary
The glossary entry is largely accurate. Main concerns:
1. The Python code example for cuMotion may need updating to reflect actual MoveIt 2 plugin usage
2. All tool versions and descriptions in the Common Tools table are current and accurate
3. Core motion planning concepts (RRT, PRM, A*, configuration space) are timeless and accurate
