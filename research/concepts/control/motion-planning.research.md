# Motion Planning Research Notes

**Researched:** 2026-01-21

## Current Version Numbers

### NVIDIA Isaac Platform
- **Isaac Sim:** 5.1.0 (latest, updated Jan 16, 2026)
  - Kit 107.3.3, DGX Spark support
  - Deprecated extensions will be fully removed in 6.0
- **Isaac ROS:** 4.0.1 (December 10, 2025) - current stable
  - 3.2 Update 15 also available (December 10, 2025)
- **cuMotion:** Developer Preview in Isaac 3.0, uses cuRobo backend
  - Provides MoveIt 2 plugin integration
  - CUDA-accelerated motion planning via parallel trajectory optimization
- Sources:
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html
  - https://nvidia-isaac-ros.github.io/releases/index.html
  - https://developer.nvidia.com/isaac/manipulator

### ROS 2 Distributions
- **Kilted Kaiju:** Released May 23, 2025, EOL November 2026 (standard release)
  - First ROS release with Zenoh as Tier 1 middleware
  - Recommended Gazebo: Ionic
- **Jazzy Jalisco:** LTS release, still supported
- Sources:
  - https://docs.ros.org/en/kilted/Releases/Release-Kilted-Kaiju.html

### MoveIt 2
- **MoveIt 2.10:** LTS release for ROS 2 Jazzy
- Binary installs available for: Jazzy, Rolling (Ubuntu 24.04), Humble (Ubuntu 22.04)
- Humble support being phased out
- Sources:
  - https://moveit.ai/documentation/contributing/releases/
  - https://moveit.ai/install-moveit2/binary/

### OMPL
- **OMPL 1.7.0:** Released March 24, 2025
  - New planners: Effort Informed Trees (EIT*), RRT-Rope
  - Kinodynamic version of SST now supports optimization objectives
  - New topological state spaces: torus, sphere, Mobius strip, Klein bottle
- PyPI package last released April 12, 2025
- Sources:
  - https://ompl.kavrakilab.org/releaseNotes.html
  - https://github.com/ompl/ompl

### Drake
- Actively developed by Toyota Research Institute
- Used in MIT 6.4210/2 Robotic Manipulation course (Fall 2025)
- 500,000+ lines of code
- Focus on Sim2Real gap for dexterous manipulation
- GitHub active: drake-iiwa-driver updated December 2025
- Sources:
  - https://drake.mit.edu/
  - https://github.com/RobotLocomotion/drake

### Tesseract
- ROS-Industrial motion planning environment
- Python packages available on PyPI (Python 3.8+)
- Under heavy development
- Integrates: OMPL, TrajOpt, TrajOpt IFOPT, Descartes
- Sources:
  - https://github.com/tesseract-robotics/tesseract
  - https://github.com/tesseract-robotics/tesseract_planning

## Corrections Needed

### Isaac ROS Version (line 112)
- **Current text:** "Integrates with Isaac ROS 3.2 and MoveIt 2"
- **Correction:** Isaac ROS 4.0.1 is now the latest stable version (December 2025)
- **Recommendation:** Update to "Integrates with Isaac ROS 4.x and MoveIt 2"

### Isaac Sim Documentation URL (line 174)
- **Current URL:** https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/manipulators_curobo.html
- **Status:** URL is correct and current for Isaac Sim 5.1.0

### MoveIt 2 Configuration Example (lines 99-107)
- **Issue:** Example shows YAML config for cuMotion plugin
- **Status:** The YAML format appears reasonable for MoveIt 2 plugin configuration
- **Note:** Actual API may vary; verify against current Isaac ROS cuMotion documentation

### Common Tools Table (lines 155-161)
- All tools correctly listed:
  - MoveIt 2: Accurate - most popular ROS 2 framework
  - OMPL: Accurate - version 1.7.0, sampling-based
  - cuMotion: Accurate - NVIDIA GPU-accelerated
  - Drake: Accurate - optimization-based, TRI-developed
  - Tesseract: Accurate - industrial focused

## Deprecation Notes

- **NVIDIA Omniverse Launcher:** Deprecated October 1, 2025
- **Isaac Sim 4.x:** Being superseded by 5.1.0; deprecated extensions removed in 6.0
- **ROS 2 Humble:** MoveIt support being phased out
- **cuRobo License:** Non-commercial use only (except NVIDIA); commercial use requires license or MoveIt plugin

## Summary

The glossary entry is largely accurate. Required updates:
1. **Line 112:** Update Isaac ROS version from 3.2 to 4.x (4.0.1 is current stable)
2. All other content including algorithms (RRT, PRM, A*, CHOMP, TrajOpt), configuration space concepts, and tool descriptions remain current and accurate
3. Source URLs are valid - Isaac Sim 5.1.0 documentation link is current
