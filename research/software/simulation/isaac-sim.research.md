# Isaac Sim Research Notes

**Research Date:** January 21, 2026
**Current File:** src/content/docs/software/simulation/isaac-sim.mdx

## Current Version Information

### Isaac Sim
- **Latest Stable:** Isaac Sim 5.1.0 (GA release)
- **Early Preview:** Isaac Sim 6.0.0 (early developer release, October 2025)
- **GA Release:** Isaac Sim 5.0 announced at SIGGRAPH 2025
- **Open Source:** Isaac Sim is now open source (extensions on GitHub; Omniverse Kit remains closed source)
- Source: [Isaac Sim Release Notes 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)
- Source: [NVIDIA Blog - Isaac Sim 5.0 GA](https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/)
- Source: [Isaac Sim GitHub](https://github.com/isaac-sim/IsaacSim)

### Isaac Lab
- **Latest:** Isaac Lab 2.3.0 (built on Isaac Sim 5.1)
- **Previous:** Isaac Lab 2.2 (released with Isaac Sim 5.0 at SIGGRAPH 2025)
- Source: [Isaac Lab Release Notes](https://isaac-sim.github.io/IsaacLab/main/source/refs/release_notes.html)
- Source: [Isaac Lab GitHub](https://github.com/isaac-sim/IsaacLab)
- Source: [NVIDIA Blog - Isaac Lab 2.3](https://developer.nvidia.com/blog/streamline-robot-learning-with-whole-body-control-and-enhanced-teleoperation-in-nvidia-isaac-lab-2-3/)

## Validation of Current Entry

### Correct in Current Entry
- Version: Isaac Sim 5.1.0 ✓
- Isaac Lab: 2.3 ✓
- ROS 2 Humble/Jazzy support ✓
- Newton physics engine mentioned ✓
- Open source status mentioned ✓
- PhysX 5 for physics ✓

### System Requirements (Current Entry vs Official Docs)

| Component | Entry Says | Official 5.1 Docs |
|-----------|-----------|-------------------|
| GPU Minimum | RTX 4080 | RTX 3070+ (recommend RTX 4080) |
| GPU Recommended | RTX 5080 / RTX PRO 6000 / DGX Spark | RTX 40-series or professional GPUs |
| VRAM | 16GB | 16GB minimum, 16-48GB for complex |
| RAM Minimum | 32GB | 32GB ✓ |
| RAM Recommended | 64GB+ | 64GB ✓ |
| OS | Ubuntu 22.04/24.04 | Ubuntu 22.04/24.04 ✓ |

- Source: [Isaac Sim 5.1 Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)

### Minor Corrections Needed

1. **GPU Minimum**: Entry says "RTX 4080" but official docs say "RTX 3070 or higher recommended"
   - Recommendation: Keep RTX 4080 as minimum since the docs say RTX 3070+ but recommend higher

2. **DGX Spark**: Entry mentions it as recommended GPU option
   - Confirmed: DGX Spark support added in Isaac Sim 5.1.0 ✓
   - Source: [Isaac Sim 5.1 GA Release Forum](https://forums.developer.nvidia.com/t/isaac-sim-5-1-ga-release/349634)

## ROS 2 Support (Verified)

- Isaac Sim 5.1 supports **ROS 2 Humble and ROS 2 Jazzy** ✓
- Ubuntu 22.04 → Automatically loads ROS 2 Humble internal libs
- Ubuntu 24.04 → Automatically loads ROS 2 Jazzy internal libs
- Supports Cyclone DDS middleware (Humble and Jazzy)
- Python 3.11 required for Isaac Sim
- Updated internal ROS 2 libraries include: common_interfaces, tf2_ros, sensor_msgs_py, simulation_interfaces v1.1.0
- Source: [ROS 2 Installation Docs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)
- Source: [ROS 2 Jazzy NVIDIA Projects](https://docs.ros.org/en/jazzy/Related-Projects/Nvidia-ROS2-Projects.html)

## Newton Physics Engine

- **Newton** is open-source physics engine built on NVIDIA Warp
- Codeveloped by NVIDIA, Google DeepMind, and Disney Research
- Managed by the Linux Foundation
- Apache 2.0 license
- Now available in Isaac Lab
- MuJoCo-Warp provides 70x acceleration for humanoid simulations
- Successfully deployed to real G1 robot
- Source: [Newton Physics NVIDIA Developer](https://developer.nvidia.com/newton-physics)
- Source: [Newton GitHub](https://github.com/newton-physics/newton)
- Source: [NVIDIA Blog - Newton Announcement](https://developer.nvidia.com/blog/announcing-newton-an-open-source-physics-engine-for-robotics-simulation/)

## Isaac Lab 2.3 Features (Verified)

- Whole-body control for humanoid robots
- Enhanced imitation learning
- Better locomotion capabilities
- Expanded teleoperation: Meta Quest VR, Manus gloves support
- Dexterous manipulation environments
- Automatic Domain Randomization (ADR)
- Population Based Training (PBT)
- Isaac Lab-Arena for policy evaluation (with Lightwheel)
- Unitree G1 teleoperation support
- Source: [NVIDIA Blog - Isaac Lab 2.3](https://developer.nvidia.com/blog/streamline-robot-learning-with-whole-body-control-and-enhanced-teleoperation-in-nvidia-isaac-lab-2-3/)

## Isaac Sim 5.1 New Features

- DGX Spark support
- ROS 2 Simulation Control extension
- Multi-arch container support
- Improved gripper physics (articulation collision contacts)
- RTX Sensor performance improvements
- Python 3.11 requirement
- Driver requirement: 580.65.06 or later on Linux
- Source: [Isaac Sim 5.1 Release Notes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)

## Breaking Changes in 5.1

- URDF importer: merge joints flag no longer supported (patch available)
- Source: [Isaac Sim 5.1 Release Notes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)

## Deprecations

- Windows 10 support ended October 14, 2025
- All deprecated extensions will be removed in Isaac Sim 6.0
- Isaac Gym superseded by Isaac Lab
- Source: [Isaac Sim Release Notes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)

## Upcoming: Isaac Sim 6.0

- Early developer preview released October 2025
- Focus on: higher-fidelity sensor simulation, simpler synthetic data workflows
- All deprecated extensions removed
- Source: [Isaac Sim 6.0 Forum Announcement](https://forums.developer.nvidia.com/t/isaac-sim-6-0-early-developer-release/356625)

## Documentation Links (Verified)

- Main docs: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- Isaac Lab: https://isaac-sim.github.io/IsaacLab/main/index.html
- Isaac Lab GitHub: https://github.com/isaac-sim/IsaacLab
- Isaac Sim GitHub: https://github.com/isaac-sim/IsaacSim
- Developer portal: https://developer.nvidia.com/isaac/sim

## Summary

The current entry is largely accurate for Isaac Sim 5.1.0. Minor observation:
- GPU minimum listed as RTX 4080 is slightly more conservative than official docs (RTX 3070+), but reasonable for practical use
- All version numbers, features, and capabilities are correctly stated
- ROS 2 Humble/Jazzy support confirmed
- Newton physics engine and open source status correctly mentioned
