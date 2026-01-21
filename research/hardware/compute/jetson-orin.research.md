# Jetson Orin Research Notes

**Research Date:** January 2026
**Current File:** src/content/docs/hardware/compute/jetson-orin.mdx

---

## Current Version Numbers (January 2026)

### JetPack SDK
- **JetPack 6.2.1** (L4T 36.4.4) - Latest production release for Orin (June 2025)
  - Linux Kernel 5.15, Ubuntu 22.04
  - CUDA 12.6, TensorRT 10.3, cuDNN 9.3, VPI 3.2
- **JetPack 7.x** - Primary target is Jetson Thor
  - CUDA 13.0, Linux Kernel 6.8, Ubuntu 24.04
  - Will support Orin series in 2026

**Source:** [JetsonHacks - JetPack 6.2.1](https://jetsonhacks.com/2025/06/30/jetpack-6-2-1-is-now-available/)
**Source:** [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack)

### Super Mode (JetPack 6.2 - January 2025)
Major feature not documented in current glossary entry:
- Unlocks higher GPU/CPU clocks via firmware update
- Enables significantly higher TOPS on NX and Nano modules
- Requires higher power budget

**Source:** [NVIDIA JetPack 6.2 Blog](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)

---

## Corrections Needed

### 1. JetPack Version Numbers (OUTDATED)
- **Current in glossary:** CUDA 12.2, TensorRT 8.6
- **Actual (JetPack 6.2.1):** CUDA 12.6, TensorRT 10.3, cuDNN 9.3
- **JetPack 7.0 in glossary:** CUDA 12.8, TensorRT 10.2
- **Actual:** CUDA 13.0 (aligns with SBSA)

### 2. Orin NX Specs - Super Mode Missing
| Spec | Glossary (Standard) | With Super Mode |
|------|---------------------|-----------------|
| 16GB TOPS | 100 TOPS | **157 TOPS** |
| 8GB TOPS | 70 TOPS | **117 TOPS** |
| Power | 10W - 25W | **10W - 40W** |
| GPU Clock | 765 MHz | **1173 MHz** |

**Source:** [Connect Tech - Super Mode](https://connecttech.com/jetson-orin-nx-nano-super-mode/)
**Source:** [Seeed Studio reComputer Super](https://www.seeedstudio.com/reComputer-Super-J4012-p-6443.html)

### 3. Orin Nano Specs - Super Mode Missing
| Spec | Glossary (Standard) | With Super Mode |
|------|---------------------|-----------------|
| 8GB TOPS | 40 TOPS | **67 TOPS** |
| 4GB TOPS | 20 TOPS | **32 TOPS** |
| Power | 7W - 15W | **7W - 25W** |
| GPU Clock | 625 MHz | **1050 MHz** |

**Source:** [NVIDIA Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)

### 4. Product Lifecycle (INCORRECT)
- **Glossary says:** LTS through 2030
- **Actual:** Extended to **January 2032** for all Orin modules
- Announced due to growing customer demand

**Source:** [NVIDIA Jetson Product Lifecycle](https://developer.nvidia.com/embedded/lifecycle)
**Source:** [NVIDIA Developer Forums - Lifecycle Extension](https://forums.developer.nvidia.com/t/product-lifecycle-extension-of-jetson-xavier-nx-and-jetson-orin-family/312586)

### 5. Jetson Thor Comparison (OUTDATED)
- **Glossary says:** 800+ TOPS, 2026+ deployment
- **Actual:**
  - 2,070 FP4 TFLOPS performance
  - Blackwell GPU with 2560 cores, 96 Tensor Cores
  - 128GB LPDDR5X memory
  - 14-core Arm Neoverse-V3AE CPU
  - **Released August 2025** (now available)
  - Developer kit: $3,499

**Source:** [NVIDIA Newsroom - Jetson Thor](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)

### 6. Description Timeline
- **Glossary says:** "2022-2025"
- **Should say:** Still in active production, supported through 2032

---

## Specification Verification

### AGX Orin - VERIFIED CORRECT
- 275 TOPS (INT8) - **CORRECT**
- 2048 CUDA cores, 64 Tensor cores - **CORRECT**
- 12-core Arm Cortex-A78AE - **CORRECT**
- 32GB or 64GB LPDDR5 - **CORRECT**
- 15W-60W configurable - **CORRECT**

**Source:** [NVIDIA Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

### AGX Orin Industrial
- 248 TOPS (not documented in glossary, may be worth adding)
- 15W-75W power range

---

## Product Status (January 2026)

| Module | Status | End of Life |
|--------|--------|-------------|
| AGX Orin 64GB | Active Production | January 2032 |
| AGX Orin 32GB | Active Production | January 2032 |
| Orin NX 16GB | Active Production | January 2032 |
| Orin NX 8GB | Active Production | January 2032 |
| Orin Nano 8GB | Active Production | January 2032 |
| Orin Nano 4GB | Active Production | January 2032 |
| Jetson Thor | Generally Available | - |

**Source:** [Jetson Product Lifecycle](https://developer.nvidia.com/embedded/lifecycle)

---

## Isaac ROS Support

- Isaac ROS 4.0.0 - Explicit Orin support unclear in docs (forum question raised Oct 2025)
- ROS 2 Humble/Jazzy compatible with JetPack 6.x
- Prebuilt Debian packages available for Jetson platforms
- Removed support for: JetPack 5.1.2, Ubuntu 20.04, Jetson Xavier
- jetson_stats package for monitoring available

**Source:** [NVIDIA Isaac ROS](https://developer.nvidia.com/isaac/ros)
**Source:** [Isaac ROS Jetson Docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html)
**Source:** [NVIDIA Forums - Isaac ROS 4.0 Orin Support](https://forums.developer.nvidia.com/t/does-isaac-ros-4-0-0-officially-support-jetson-orin/349564)

---

## All Sources

1. [NVIDIA Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
2. [NVIDIA Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
3. [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
4. [JetsonHacks - JetPack 6.2.1](https://jetsonhacks.com/2025/06/30/jetpack-6-2-1-is-now-available/)
5. [NVIDIA JetPack 6.2 Blog](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)
6. [Jetson Product Lifecycle](https://developer.nvidia.com/embedded/lifecycle)
7. [NVIDIA Forums - Lifecycle Extension](https://forums.developer.nvidia.com/t/product-lifecycle-extension-of-jetson-xavier-nx-and-jetson-orin-family/312586)
8. [NVIDIA Newsroom - Jetson Thor](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)
9. [Connect Tech - Super Mode](https://connecttech.com/jetson-orin-nx-nano-super-mode/)
10. [Seeed Studio reComputer Super](https://www.seeedstudio.com/reComputer-Super-J4012-p-6443.html)
11. [NVIDIA Isaac ROS](https://developer.nvidia.com/isaac/ros)
12. [Isaac ROS Jetson Docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html)
13. [NVIDIA Forums - Isaac ROS 4.0](https://forums.developer.nvidia.com/t/does-isaac-ros-4-0-0-officially-support-jetson-orin/349564)
