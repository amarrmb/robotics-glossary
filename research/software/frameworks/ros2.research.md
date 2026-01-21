# ROS 2 Research Notes

**Research Date:** 2026-01-20

## Current Versions (as of January 2026)

### Active ROS 2 Distributions

| Distribution | Type | Release | EOL | Status |
|-------------|------|---------|-----|--------|
| **Jazzy Jalisco** | LTS | May 2024 | May 2029 | Current LTS - Recommended |
| **Kilted Kaiju** | Standard | May 2025 | Nov 2026 | Latest stable |
| **Humble Hawksbill** | LTS | May 2022 | May 2027 | Supported (approaching EOL) |
| **Rolling** | Dev | Continuous | N/A | Development branch |

- **Source:** https://docs.ros.org/en/rolling/Releases.html
- **Source:** https://endoflife.date/ros-2

### Upcoming Release
- **Lyrical Luth** - Expected May 2026 (LTS)
- **Source:** https://docs.ros.org/en/rolling/Releases/Release-Lyrical-Luth.html

## Corrections Needed in Current File

### Version Information (Lines 16-20)
**Current content states:**
- Humble Hawksbill (LTS until 2027) — Recommended for production
- Jazzy Jalisco (2024) — Latest features
- Rolling — Development branch

**Corrections:**
1. **Jazzy Jalisco** is now LTS (until May 2029) - should be recommended for production
2. **Kilted Kaiju** (May 2025, standard release until Nov 2026) - missing from list
3. **Humble Hawksbill** - still valid but approaching EOL (May 2027)
4. Recommendation should shift to Jazzy as primary LTS

### Key Changes in Kilted Kaiju (May 2025)
- First ROS 2 release to support **Eclipse Zenoh** as Tier 1 middleware
- Python performance improvements (~10x speedup with events executor)
- NV12 image format support in common interfaces
- Windows installation now uses Pixi/Conda by default
- 178 contributors, 53 testers
- **Source:** https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html

### Default Middleware Clarification (Line 91-92)
**Current content states:** CycloneDDS is default

**Correction:**
- **Fast DDS** is the default middleware for Humble, Jazzy, and Kilted
- CycloneDDS is available but not default
- Eclipse Zenoh is now Tier 1 in Kilted
- **Source:** https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Different-Middleware-Vendors.html

## NVIDIA Isaac ROS Updates

### Current Version
- **Isaac ROS 4.0** - Generally available
- Latest minor: 3.2 Update 9 (v3.2-9) on GitHub
- Supports JetPack 7.0 and Isaac Sim 5.1
- Supports Jetson AGX Thor
- **Source:** https://nvidia-isaac-ros.github.io/releases/index.html

### Key Features
- **PyNITROS** - Python API for CUDA with NITROS support (PyTorch tensors, images)
- **NITROS** (NVIDIA Isaac Transport for ROS) - Zero-copy GPU memory sharing
- Improved FoundationPose performance for pose estimation
- nvBlox improvements (~1cm voxel resolution)
- Nova Event Data Recorder for live data collection
- **Source:** https://nvidia-isaac-ros.github.io/concepts/nitros/index.html

### NITROS Description (Lines 265-267)
- Current description is accurate
- Could add: "Uses ROS 2 type adaptation and type negotiation features"
- **Source:** https://nvidia-isaac-ros.github.io/concepts/nitros/index.html

## Documentation Links to Verify

| Current Link | Status | Notes |
|-------------|--------|-------|
| https://docs.ros.org/en/humble/ | Valid | Consider updating to Jazzy |
| https://docs.ros.org/en/humble/Tutorials.html | Valid | Jazzy tutorials also available |
| https://navigation.ros.org/ | Valid | Nav2 documentation |
| https://discourse.ros.org/ | Valid | Community forum |
| https://nvidia-isaac-ros.github.io/ | Valid | Isaac ROS docs |

## Summary of Required Updates

1. **Version section needs update:**
   - Add Kilted Kaiju
   - Update Jazzy to show LTS status
   - Change production recommendation from Humble to Jazzy

2. **Architecture diagram correction:**
   - CycloneDDS shown as "(Default)" is incorrect
   - Fast DDS is actually the default
   - Should add Zenoh as option (new in Kilted)

3. **Minor updates:**
   - Learn More links could point to Jazzy instead of Humble
   - Isaac ROS section is accurate but could mention current version

## Sources

- ROS 2 Distributions: https://docs.ros.org/en/rolling/Releases.html
- ROS 2 Humble: https://docs.ros.org/en/humble/Releases.html
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Releases.html
- ROS 2 Kilted: https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html
- ROS 2 Lyrical: https://docs.ros.org/en/rolling/Releases/Release-Lyrical-Luth.html
- ROS 2 Middleware: https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Different-Middleware-Vendors.html
- REP 2000: https://www.ros.org/reps/rep-2000.html
- endoflife.date: https://endoflife.date/ros-2
- Isaac ROS: https://nvidia-isaac-ros.github.io/
- Isaac ROS Release Notes: https://nvidia-isaac-ros.github.io/releases/index.html
- Isaac ROS NITROS: https://nvidia-isaac-ros.github.io/concepts/nitros/index.html
- NVIDIA Developer: https://developer.nvidia.com/isaac/ros
