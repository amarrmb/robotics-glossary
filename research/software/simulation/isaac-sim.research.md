# Isaac Sim Research Notes

**Research Date:** January 2026
**Current File:** src/content/docs/software/simulation/isaac-sim.mdx

## Current Version Information

### Isaac Sim
- **Latest Stable:** Isaac Sim 5.1.0 (released January 16, 2026)
- **Early Preview:** Isaac Sim 6.0 (early developer preview available)
- **General Availability:** Isaac Sim 5.0 announced at SIGGRAPH 2025
- **Isaac Sim is now open source** (extensions on GitHub; Omniverse Kit remains closed source)
- Source: [Isaac Sim Release Notes 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)
- Source: [NVIDIA Blog - Isaac Sim 5.0 GA](https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/)

### Isaac Lab
- **Latest:** Isaac Lab 2.3.0 (built on Isaac Sim 5.1)
- **Previous:** Isaac Lab 2.2 (released with Isaac Sim 5.0 at SIGGRAPH 2025)
- Source: [Isaac Lab Release Notes](https://isaac-sim.github.io/IsaacLab/main/source/refs/release_notes.html)
- Source: [Isaac Lab GitHub](https://github.com/isaac-sim/IsaacLab)

## Corrections Needed in Current Entry

### Version Number (CRITICAL)
- **Current entry says:** Isaac Sim 4.5 (2026)
- **Should be:** Isaac Sim 5.1.0 (current stable as of January 2026)
- Isaac Sim 4.5 is outdated; official docs redirect to 5.x

### System Requirements Updates

| Component | Entry Says | Current (5.1) |
|-----------|-----------|---------------|
| GPU Minimum | RTX 2070 | RTX 4080 |
| GPU Recommended | RTX 4080+ | RTX 5080 / RTX PRO 6000 Blackwell |
| VRAM Minimum | 8GB | 16GB |
| RAM Minimum | 32GB | 32GB (correct) |
| RAM Recommended | 64GB+ | 64GB (correct) |
| OS | Ubuntu 22.04/24.04 | Ubuntu 22.04/24.04 (correct) |

- Source: [Isaac Sim 5.1 Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)

### Docker Image Tag
- **Current entry says:** `nvcr.io/nvidia/isaac-sim:4.5.0`
- **Should be:** `nvcr.io/nvidia/isaac-sim:5.1.0` (or latest)

### Installation Path
- **Current entry says:** `~/.local/share/ov/pkg/isaac-sim-4.5.0/isaac-sim.sh`
- **Should be:** Updated to 5.1.0 path

## ROS 2 Support

- Isaac Sim 5.1 supports **ROS 2 Humble and ROS 2 Jazzy**
- Ubuntu 22.04 → Automatically loads ROS 2 Humble internal libs
- Ubuntu 24.04 → Automatically loads ROS 2 Jazzy internal libs
- Supports Cyclone DDS middleware (Humble and Jazzy)
- Python 3.11 required for Isaac Sim; internal ROS 2 libs compiled with Python 3.12
- Source: [ROS 2 Installation Docs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

## Physics Engine

- Uses **NVIDIA PhysX SDK** (PhysX 5)
- PhysX 5 features: FEM soft-body dynamics, PBD for liquids/inflatables
- Supports: rigid/articulated bodies, cloth, fluids, soft bodies
- Isaac Lab 2.2+ includes updated joint friction modeling via latest PhysX APIs
- Source: [Isaac Sim Physics Docs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/index.html)

## New Features in 5.x (not in current entry)

- **Newton Physics Engine** now available in Isaac Lab (codeveloped with Google DeepMind and Disney Research)
- **Isaac Lab-Arena** for policy evaluation
- **MobilityGen** for diverse physics and perception-based data generation
- **DGX Spark support** added in 5.1.0
- **Simulation Interfaces v1.1.0** for ROS 2
- **New robots:** Schunk grippers, Booster T1, G1 hand variants
- Source: [NVIDIA Newsroom](https://nvidianews.nvidia.com/news/nvidia-accelerates-robotics-research-and-development-with-new-open-models-and-simulation-libraries)

## Deprecations and Changes

- **Windows 10 support ended October 14, 2025** - dropped in future releases
- All deprecated extensions will be removed in Isaac Sim 6.0
- Isaac Gym superseded by Isaac Lab
- Source: [Isaac Sim Release Notes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)

## Documentation Links (verified working)

- Main docs: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- Isaac Lab: https://isaac-sim.github.io/IsaacLab/main/index.html
- Isaac Lab GitHub: https://github.com/isaac-sim/IsaacLab
- Isaac Sim GitHub: https://github.com/isaac-sim/IsaacSim
- Developer portal: https://developer.nvidia.com/isaac/sim

## Summary of Required Updates

1. **Version:** 4.5 → 5.1.0
2. **GPU minimum:** RTX 2070 → RTX 4080
3. **VRAM minimum:** 8GB → 16GB
4. **Docker tag:** 4.5.0 → 5.1.0
5. **Installation path:** Update version number
6. **Add:** Newton Physics Engine mention
7. **Add:** Open source status note
8. **Add:** Windows 10 deprecation note
9. **Update:** Isaac Lab version to 2.3.0
