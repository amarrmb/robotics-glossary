# ROS 2 Research Notes

**Research Date:** 2026-01-21

## Current Versions (as of January 2026)

### Active ROS 2 Distributions

| Distribution | Type | Release | EOL | Status |
|-------------|------|---------|-----|--------|
| **Jazzy Jalisco** | LTS | May 2024 | May 2029 | Current LTS - Recommended |
| **Kilted Kaiju** | Standard | May 2025 | Nov 2026 | Latest stable (Patch 1: 2025-07-28) |
| **Humble Hawksbill** | LTS | May 2022 | May 2027 | Supported (legacy) |
| **Rolling Ridley** | Dev | June 2020 | Ongoing | Development branch |

- **Source:** https://docs.ros.org/en/rolling/Releases.html
- **Source:** https://endoflife.date/ros-2
- **Source:** https://www.ros.org/reps/rep-2000.html

### Upcoming Release
- **Lyrical Luth** - Expected May 2026 (LTS, 5-year support until 2031)
- Will be 12th ROS 2 release
- Even year = LTS release
- **Source:** https://docs.ros.org/en/rolling/Releases/Release-Lyrical-Luth.html

## Validation Against Current File

### Version Section (Lines 17-22) — CORRECT ✓
Current file states:
- Jazzy Jalisco (LTS until 2029) — Recommended for production ✓
- Kilted Kaiju (stable until Nov 2026) — Latest features ✓
- Humble Hawksbill (LTS until 2027) — Legacy support ✓
- Rolling — Development branch ✓

**No corrections needed** — File already updated to current state.

### ROS 1 vs ROS 2 Comparison Table (Lines 28-35) — CORRECT ✓
- Communication: DDS (industry standard) — Correct
- Real-time support — Correct
- DDS Security — Correct
- Multi-robot native support — Correct
- Multi-platform support — Correct
- Managed node lifecycle — Correct

### Architecture Diagram (Lines 91-95)
Current file shows:
```
│  │   FastDDS   │  │ CycloneDDS  │  │   Zenoh, Other Impls    │ │
│  │  (Default)  │  │             │  │                         │ │
```
**Status:** CORRECT ✓
- Fast DDS is indeed the default middleware
- CycloneDDS available as alternative
- Zenoh listed as option (now Tier 1 in Kilted)
- **Source:** https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Different-Middleware-Vendors.html

## DDS/Middleware Information

### Default Middleware
- **FastDDS (eProsima)** — Default RMW if available
- In Kilted Kaiju: `fastrtps` package renamed to `fastdds` (rmw names unchanged)
- **Source:** https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html

### Available Middleware Options
- **FastDDS** (eProsima) — Default
- **CycloneDDS** (Eclipse) — Lightweight, real-time focused
- **Zenoh** (rmw_zenoh_cpp) — New Tier 1 in Kilted, lighter than DDS
- **RTI Connext** — Commercial option
- **RTI Connext Micro** — Deprecated in Kilted, will be removed in future release
- **Source:** https://docs.ros.org/en/rolling/Concepts/Advanced/About-Middleware-Implementations.html

### Zenoh as Alternative (New in Kilted)
- First ROS 2 release with Zenoh as Tier 1 middleware
- rmw_zenoh available via binary packages
- Requires Zenoh router (zenohd) to be active
- Better for challenging network conditions
- Addresses DDS n² discovery traffic issue
- **Source:** https://github.com/ros2/rmw_zenoh

## NVIDIA Isaac ROS

### Current Version
- **Isaac ROS 3.2** (announced CES 2025)
- Latest: 3.2 Update 15 (v3.2-15) on GitHub
- Includes fixes for TF broadcasting, AprilTags, rosdep
- **Source:** https://nvidia-isaac-ros.github.io/releases/index.html

### Compatibility
- All Isaac ROS packages designed and tested for ROS 2 Jazzy
- Isaac Sim compatible with ROS 2 Humble and Jazzy
- **Source:** https://nvidia-isaac-ros.github.io/

### NITROS (NVIDIA Isaac Transport for ROS)
- Zero-copy GPU memory transfer between nodes
- Introduced with ROS 2 Humble (type adaptation/negotiation features)
- Performance benchmarks: 3x improvement on Xavier, 7x on Orin
- Requirement: NITROS nodes must run in same process for zero-copy benefit
- **Source:** https://nvidia-isaac-ros.github.io/concepts/nitros/index.html
- **Source:** https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/

### Isaac ROS Section Validation (Lines 261-270)
Current file states:
- "GPU-accelerated ROS 2 packages bring 10-100x speedups" — Reasonable claim for full pipelines
- "Zero-copy GPU memory sharing between ROS 2 nodes" — Correct (with same-process caveat)

**No major corrections needed** — Description is accurate.

## Kilted Kaiju Key Changes (May 2025)

- `fastrtps` package renamed to `fastdds`
- `ament_target_dependencies()` deprecated in favor of `target_link_libraries()`
- `rmw_connextddsmicro` deprecated, will be removed in future release
- Eclipse Zenoh now Tier 1 middleware
- **Source:** https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html

## Minor Suggestions (Optional Updates)

1. **Installation examples (Lines 112-131):** Uses `ros-humble-desktop`
   - Could add Jazzy option: `ros-jazzy-desktop`
   - Jazzy is now recommended LTS

2. **Key Packages table (Line 259):** Shows `gazebo_ros2`
   - Kilted recommends Gazebo Ionic
   - Consider noting Gazebo version compatibility

3. **Learn More links (Lines 299-303):** Point to Jazzy docs
   - Already correct — links to jazzy documentation

## Summary

**No major corrections required.** The current file (last_validated: 2026-01-20) is accurate:
- Version information is up-to-date
- DDS default (FastDDS) is correctly shown
- Core concepts and architecture are accurate
- NVIDIA Isaac ROS/NITROS claims are reasonable

## Sources

- ROS 2 Distributions: https://docs.ros.org/en/rolling/Releases.html
- REP 2000: https://www.ros.org/reps/rep-2000.html
- ROS 2 Kilted Kaiju: https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html
- ROS 2 Lyrical Luth: https://docs.ros.org/en/rolling/Releases/Release-Lyrical-Luth.html
- ROS 2 Middleware: https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Different-Middleware-Vendors.html
- ROS 2 endoflife: https://endoflife.date/ros-2
- Isaac ROS: https://nvidia-isaac-ros.github.io/
- Isaac ROS Release Notes: https://nvidia-isaac-ros.github.io/releases/index.html
- Isaac ROS NITROS: https://nvidia-isaac-ros.github.io/concepts/nitros/index.html
- NVIDIA Developer Isaac ROS: https://developer.nvidia.com/isaac/ros
- rmw_zenoh: https://github.com/ros2/rmw_zenoh
