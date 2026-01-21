# Jetson Orin Research Notes

**Research Date:** January 2026
**Current File:** src/content/docs/hardware/compute/jetson-orin.mdx

---

## Current Version Numbers (January 2026)

### JetPack SDK
- **JetPack 6.2.1** (L4T 36.4.4) - Latest production release
- **JetPack 6.2** (L4T 36.4.3) - Includes CUDA 12.6, TensorRT 10.3, cuDNN 9.3
- **JetPack 7.0** - Now available (Linux Kernel 6.8, Ubuntu 24.04 LTS)
  - Primary target: Jetson Thor
  - Full support for Orin NX/Nano with Super Mode

**Source:** [JetPack SDK | NVIDIA Developer](https://developer.nvidia.com/embedded/jetpack-sdk-62)

### Orin Nano Super (NEW - Not in current file)
- **67 TOPS** AI performance (up from 40 TOPS with software upgrade)
- **102 GB/s** memory bandwidth (up from 68 GB/s)
- **25W max power** (up from 15W)
- GPU frequency: 1020 MHz (up from 635 MHz)
- Price: $249 for developer kit

**Source:** [Jetson Orin Nano Super Developer Kit | NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)

### Orin NX Super Mode (Updated specs)
- **16GB version:** Up to 157 TOPS (from 100 TOPS standard)
- **8GB version:** Up to 117 TOPS (from 70 TOPS standard)
- Power: 10W - 40W (was 10W - 25W in file)
- 70% performance improvement via software upgrade

**Source:** [Jetson AGX Orin for Next-Gen Robotics | NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

### AGX Orin (Verified)
- **64GB:** 275 TOPS - Correct in file
- **32GB:** 200 TOPS (file says 275 - needs correction)
- **Industrial:** 248 TOPS
- 12-core Arm Cortex-A78AE - Correct
- 2048 CUDA cores, 64 Tensor cores - Correct
- Power: 15W - 60W - Correct

**Source:** [NVIDIA Jetson AGX Orin Series Technical Brief](https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf)

---

## Corrections Needed

### 1. AGX Orin 32GB TOPS Rating
- **Current:** 275 TOPS (same as 64GB)
- **Actual:** 200 TOPS
- AGX Orin 32GB module delivers up to 200 TOPS with 15W-40W power

### 2. Orin NX Specs (Super Mode not documented)
- **Current 16GB:** 100 TOPS
- **Actual 16GB Super:** 157 TOPS
- **Current 8GB:** 70 TOPS
- **Actual 8GB Super:** 117 TOPS
- **Current power:** 10W - 25W
- **Actual power:** 10W - 40W

### 3. Orin Nano Specs (Super Mode not documented)
- **Current 8GB:** 40 TOPS
- **Actual 8GB Super:** 67 TOPS
- **Current power:** 7W - 15W
- **Actual power:** 7W - 25W

### 4. JetPack Version Numbers
- **Current:** JetPack 6.2 with CUDA 12.2, TensorRT 8.6
- **Actual:** JetPack 6.2 with CUDA 12.6, TensorRT 10.3, cuDNN 9.3
- **Current JetPack 7:** CUDA 12.8, TensorRT 10.2
- **Actual:** JetPack 7 is now available (not just future)

### 5. Product Lifecycle
- **Current:** LTS through 2030
- **Actual:** Extended to Q1 2032 (announced 2024)

### 6. Jetson Thor Specs
- **Current:** 800+ TOPS
- **Actual:** Up to 2070 FP4 TFLOPS
- Thor is now generally available (not 2026+ as stated)

### 7. Missing: Jetson T4000 Module
- New module offering up to 1200 FP4 TFLOPS
- 64 GB memory, 40W-70W power
- Over 4x AI compute vs AGX Orin

---

## Product Status (January 2026)

| Module | Status | Availability |
|--------|--------|--------------|
| AGX Orin 64GB | Active Production | Available |
| AGX Orin 32GB | Active Production | Available |
| AGX Orin Industrial | Active Production | Available |
| Orin NX 16GB | Active Production | Available |
| Orin NX 8GB | Active Production | Available |
| Orin Nano 8GB Super | Active Production | Available |
| Jetson Thor | Generally Available | Available |
| Jetson T4000 | Announced | Coming |

**Source:** [Jetson Product Lifecycle | NVIDIA Developer](https://developer.nvidia.com/embedded/lifecycle)

---

## Isaac ROS Compatibility

- All Isaac ROS packages compatible with ROS 2 Jazzy
- Works on Jetson Orin boards (Nano and up)
- Supports x86 with NVIDIA GPU and previous gen Xavier
- Isaac ROS Jetson provides jetson_stats integration for monitoring

**Source:** [Isaac ROS Jetson Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html)

---

## Sources

1. [Jetson AGX Orin for Next-Gen Robotics | NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
2. [Jetson Orin Nano Super Developer Kit | NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
3. [JetPack SDK | NVIDIA Developer](https://developer.nvidia.com/embedded/jetpack-sdk-62)
4. [JetPack Archive | NVIDIA Developer](https://developer.nvidia.com/embedded/jetpack-archive)
5. [Jetson Product Lifecycle | NVIDIA Developer](https://developer.nvidia.com/embedded/lifecycle)
6. [NVIDIA Jetson AGX Orin Series Technical Brief](https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf21/jetson-orin/nvidia-jetson-agx-orin-technical-brief.pdf)
7. [Isaac ROS Jetson Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html)
8. [Jetson Thor | NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/)
9. [NVIDIA Blackwell-Powered Jetson Thor Now Available | NVIDIA Newsroom](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)
10. [Product Lifecycle Extension Forum Post | NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/product-lifecycle-extension-of-jetson-xavier-nx-and-jetson-orin-family/312586)
