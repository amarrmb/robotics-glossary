# Jetson Orin Research Notes

**Research Date:** 2026-01-21
**Current File:** src/content/docs/hardware/compute/jetson-orin.mdx

---

## Product Lifecycle - VERIFIED ✓

- **All Orin commercial modules extended to Q1 2032** (confirmed)
  - Jetson AGX Orin 64GB, 32GB
  - Jetson Orin NX 16GB, 8GB
  - Jetson Orin Nano 8GB, 4GB
- Jetson AGX Orin Industrial: available through **July 2033**

**File Status:** Correctly states LTS through January 2032 ✓

**Source:** [Jetson Product Lifecycle](https://developer.nvidia.com/embedded/lifecycle)

---

## AGX Orin Specifications

### AGX Orin 64GB - VERIFIED ✓
| Spec | File Value | Verified |
|------|------------|----------|
| AI Performance | 275 TOPS (INT8) | ✓ |
| GPU | 2048 CUDA cores, 64 Tensor cores | ✓ |
| CPU | 12-core Arm Cortex-A78AE | ✓ |
| Memory | 32GB or 64GB LPDDR5 | ✓ |
| Power | 15W - 60W | ✓ |

### AGX Orin 32GB - CORRECTION NEEDED
| Spec | Actual Value |
|------|--------------|
| AI Performance | **200 TOPS** (not 275) |
| GPU | **1792 CUDA cores, 56 Tensor cores** |
| CPU | **8-core Arm Cortex-A78AE** |
| Memory | 32GB LPDDR5 |
| Power | **15W - 40W** (not 60W) |

**Correction Needed:** File's AGX Orin table shows 64GB specs. Should clarify 32GB variant has different specs.

**Sources:**
- [NVIDIA Jetson AGX Orin Technical Brief](https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf)
- [NVIDIA Developer Blog - AGX Orin 32GB](https://developer.nvidia.com/blog/jetson-agx-orin-32gb-module-now-available)

---

## Orin NX Specifications - VERIFIED ✓

### Orin NX 16GB
| Spec | File Value | Verified |
|------|------------|----------|
| AI Performance | 100 TOPS (157 w/ Super Mode) | ✓ |
| GPU | 1024 CUDA cores | ✓ |
| CPU | 8-core Arm Cortex-A78AE | ✓ |
| Memory | 16GB LPDDR5 | ✓ |
| Power | 10W - 25W (40W Super Mode) | ✓ |

### Orin NX 8GB
| Spec | File Value | Verified |
|------|------------|----------|
| AI Performance | 70 TOPS (117 w/ Super Mode) | ✓ |
| GPU | 1024 CUDA cores | ✓ |
| CPU | 6-core | ✓ |
| Memory | 8GB LPDDR5 | ✓ |

**Source:** [NVIDIA Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

---

## Orin Nano Specifications - VERIFIED ✓

### Orin Nano 8GB
| Spec | File Value | Verified |
|------|------------|----------|
| AI Performance | 40 TOPS (67 w/ Super Mode) | ✓ |
| GPU | 1024 CUDA cores | ✓ |
| CPU | 6-core Arm Cortex-A78AE | ✓ |
| Memory | 8GB LPDDR5 | ✓ |
| Power | 7W - 15W (25W Super Mode) | ✓ |

### Orin Nano 4GB
| Spec | File Value | Verified |
|------|------------|----------|
| AI Performance | 20 TOPS (32 w/ Super Mode) | ✓ |
| GPU | 512 CUDA cores | ✓ |
| CPU | 6-core | ✓ |
| Memory | 4GB LPDDR5 | ✓ |

**Source:** [NVIDIA Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)

---

## JetPack Software Versions - VERIFIED ✓

### JetPack 6.2.1 (Current LTS)
| Component | File Value | Verified |
|-----------|------------|----------|
| Jetson Linux | 36.4.4 | ✓ |
| Linux Kernel | 5.15 | ✓ |
| Ubuntu | 22.04 | ✓ |
| CUDA | 12.6 | ✓ |
| TensorRT | 10.3 | ✓ |
| cuDNN | 9.3 | ✓ |
| VPI | 3.2 | ✓ |
| DLA | 3.1 | ✓ |

**Sources:**
- [JetPack 6.2.1 Release Notes](https://docs.nvidia.com/jetson/jetpack/release-notes/)
- [JetPack Downloads](https://developer.nvidia.com/embedded/jetpack/downloads)

### JetPack 7.x Status - VERIFIED ✓
- JetPack 7.0/7.1 released for **Jetson Thor** (T5000, T4000)
- Ubuntu: **24.04 LTS**
- Kernel: **6.8**
- CUDA: **13.0**
- **Orin support expected in JetPack 7.2** (Q1 2026)

**File Status:** Correctly notes JetPack 7.x primary target is Thor, Orin support coming later ✓

**Sources:**
- [JetPack 7.0 Forum Announcement](https://forums.developer.nvidia.com/t/jetpack-7-0-jetson-linux-38-2-for-nvidia-jetson-thor-is-now-live/343128)
- [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)

---

## Super Mode (JetPack 6.2) - VERIFIED ✓

### How It Works
- Software-only optimization (no hardware changes)
- Increases GPU/DLA/CPU clock frequencies
- Requires higher power budget

### Performance Gains
- Orin NX: GPU 765 MHz → 1173 MHz
- Orin Nano: GPU 635 MHz → 1020 MHz
- Up to **2x inference performance boost**
- Up to **70% increase in AI TOPS** for NX series
- Up to **67% increase** for Nano series

**File Status:** Super Mode TOPS values are correct ✓

**Source:** [JetPack 6.2 Super Mode Blog](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)

---

## Isaac ROS Compatibility - VERIFIED ✓

- Isaac ROS packages available for Jetson Orin series
- Compatible with **ROS 2 Humble**
- Key packages: nvblox, image_pipeline, AprilTags, NITROS
- jetson_stats package for monitoring
- Nova Carter reference AMR uses Isaac ROS + Nav2

**Sources:**
- [ROS 2 Humble - NVIDIA Projects](https://docs.ros.org/en/humble/Related-Projects/Nvidia-ROS2-Projects.html)
- [Isaac ROS Jetson](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html)
- [Isaac ROS Getting Started](https://nvidia-isaac-ros.github.io/getting_started/index.html)

---

## Thor Comparison Table - VERIFIED ✓

| Spec | File Value | Verified |
|------|------------|----------|
| AI Performance | 2,070 TFLOPS (FP4) | ✓ |
| Memory | 128GB LPDDR5X | ✓ |
| Dev Kit Price | $3,499 | ✓ |

**Source:** [NVIDIA Newsroom - Jetson Thor](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)

---

## New Products Note (2026)

### Jetson T4000 (announced CES 2026)
- AI Performance: **1200 FP4 TFLOPS**
- Memory: **64GB**
- Power: **40W - 70W**
- Over 4x AI compute vs AGX Orin
- Supported by JetPack 7.1
- Available through August 2035

**Note:** File mentions Thor; T4000/T5000 are Thor-based modules.

**Source:** [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)

---

## Summary of Corrections Needed

### Must Fix
1. **AGX Orin specs table:** Should clarify 32GB variant has different specs (200 TOPS, 8-core, 40W max) vs 64GB (275 TOPS, 12-core, 60W)

### Minor / Optional
1. JetPack 7.2 will bring Orin support - could be more specific than "planned for 2026"

---

## Verified Correct (No Changes Needed)

- LTS through January 2032 ✓
- JetPack 6.2.1 software versions (CUDA 12.6, TensorRT 10.3, cuDNN 9.3) ✓
- Super Mode feature description ✓
- Super Mode TOPS values for NX and Nano ✓
- All Orin NX specifications ✓
- All Orin Nano specifications ✓
- JetPack 7.x primary target being Thor ✓
- Thor comparison specs ✓
- Architecture diagram components ✓
- Power management modes ✓

---

## All Sources

1. [NVIDIA Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
2. [NVIDIA Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
3. [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
4. [JetPack 6.2.1 Release Notes](https://docs.nvidia.com/jetson/jetpack/release-notes/)
5. [JetPack Downloads](https://developer.nvidia.com/embedded/jetpack/downloads)
6. [JetPack 6.2 Super Mode Blog](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)
7. [Jetson Product Lifecycle](https://developer.nvidia.com/embedded/lifecycle)
8. [JetPack 7.0 Forum](https://forums.developer.nvidia.com/t/jetpack-7-0-jetson-linux-38-2-for-nvidia-jetson-thor-is-now-live/343128)
9. [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)
10. [NVIDIA AGX Orin Technical Brief](https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf)
11. [NVIDIA Developer Blog - AGX Orin 32GB](https://developer.nvidia.com/blog/jetson-agx-orin-32gb-module-now-available)
12. [ROS 2 Humble - NVIDIA Projects](https://docs.ros.org/en/humble/Related-Projects/Nvidia-ROS2-Projects.html)
13. [Isaac ROS Jetson](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html)
14. [Isaac ROS Getting Started](https://nvidia-isaac-ros.github.io/getting_started/index.html)
15. [NVIDIA Newsroom - Jetson Thor](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)
