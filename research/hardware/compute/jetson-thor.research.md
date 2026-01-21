# Jetson Thor Research Notes

**Research Date:** January 20, 2026

## Current Status

- **General Availability:** August 25, 2025 (developer kits and production modules)
- **Developer Kit Price:** $3,499
- **T5000 Module Price:** $2,999 (1,000 units)
- **T4000 Module Price:** $1,999 (1,000 units) - available Q2 2026
- **JetPack 7.1** is now the latest release (as of January 2026)

## Corrections Needed

### AI Performance Specs (MAJOR CORRECTION)
- **Current file states:** 800 TOPS (INT8), 400 TFLOPS (FP8)
- **Actual specs:** 2,070 FP4 TFLOPS (sparse), up to 7.5x higher AI compute than Orin
- Initial GTC 2024 announcement mentioned 800 TFLOPS FP8, but production specs are significantly higher

### CPU Configuration (CORRECTION)
- **Current file states:** 16-core Arm Neoverse V2
- **Actual specs:** 14-core Arm Neoverse V3AE @ 2.6 GHz (T5000), 12-core (T4000)
- CPU performance is 3.1x higher than Orin

### GPU Specifications (CORRECTION)
- **Current file states:** 4096+ CUDA cores, 512 Tensor Cores
- **Actual specs:** 2,560 CUDA cores, 96 fifth-generation Tensor Cores (T5000)
- T4000 variant: 1,536 CUDA cores, 64 Tensor Cores

### Memory Bandwidth (CORRECTION)
- **Current file states:** 512 GB/s
- **Actual specs:** 273 GB/s (256-bit bus @ 4266 MHz LPDDR5X)

### Power Configuration (CORRECTION)
- **Current file states:** 50W - 150W
- **Actual specs:** 40W - 130W (configurable)

### JetPack Version (UPDATE)
- **Current file states:** JetPack 7.0 primary SDK
- **Update:** JetPack 7.1 now available (Jetson Linux 38.4)
- JetPack 7.0 uses Jetson Linux 38.2

### Software Stack Versions (CORRECTION)
- **Current file states:** CUDA 12.8, TensorRT 10.2
- **Actual specs:** CUDA 13.0, TensorRT 10.13, cuDNN 9.12
- Linux Kernel 6.8 LTS, Ubuntu 24.04 LTS

### Isaac ROS Version
- **Current file states:** Isaac ROS 4.0
- **Status:** Isaac ROS 4.0 is correct, now generally available for Thor
- Note: Isaac Perceptor and Nova packages not yet optimized for AGX Thor

## Module Variants

### Jetson T5000 (in Developer Kit)
- 2,070 FP4 TFLOPS
- 14-core Arm Neoverse-V3AE CPU
- 2,560 CUDA cores, 96 Tensor Cores (5th gen)
- 128 GB LPDDR5X memory
- 40-130W power range

### Jetson T4000 (Lower-cost variant)
- 1,200 FP4 TFLOPS
- 12-core Arm Neoverse-V3AE CPU
- 1,536 CUDA cores, 64 Tensor Cores
- 64 GB LPDDR5X memory
- 40-75W power range
- Available Q2 2026

## Developer Kit Contents
- Jetson T5000 module (128GB)
- Reference carrier board
- 1TB NVMe SSD
- WiFi 6E support
- 5 GbE networking
- QSFP28 socket
- HDMI 2.1 + DisplayPort connections

## Industry Adoption
Early adopters include:
- Agility Robotics
- Amazon Robotics
- Boston Dynamics (integrating into Atlas)
- Caterpillar
- Figure
- Hexagon
- Medtronic
- Meta

Evaluating:
- 1X
- John Deere
- OpenAI
- Physical Intelligence

## Key Features Verified
- Blackwell GPU architecture with 5th-gen Tensor Cores
- Transformer Engine with FP4/FP8 dynamic switching
- Multi-Instance GPU (MIG) support
- Safety Island with lockstep cores (ASIL-D capable)
- Holoscan Sensor Bridge integration
- Up to 20 camera connections
- 8K display output support

## Sources

- [NVIDIA Jetson Thor Official Page](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/)
- [NVIDIA Newsroom: Jetson Thor Now Available](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)
- [NVIDIA Developer Blog: Introducing Jetson Thor](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/)
- [JetPack SDK Downloads](https://developer.nvidia.com/embedded/jetpack/downloads)
- [JetPack 7.0 Forum Announcement](https://forums.developer.nvidia.com/t/jetpack-7-0-jetson-linux-38-2-for-nvidia-jetson-thor-is-now-live/343128)
- [RidgeRun: JetPack 7.0 Components](https://developer.ridgerun.com/wiki/index.php/NVIDIA_Jetson_AGX_Thor/JetPack_7.0/Getting_Started/Components)
- [Isaac ROS 4.0 Announcement](https://forums.developer.nvidia.com/t/nvidia-isaac-ros-4-0-for-thor-has-arrived/352109)
- [JetsonHacks: JetPack 7.1 and T4000](https://jetsonhacks.com/2026/01/12/jetpack-7-1-and-jetson-t4000-now-available/)
- [CNX Software: Jetson AGX Thor Developer Kit](https://www.cnx-software.com/2025/08/19/3499-nvidia-jetson-agx-thor-developer-kit-2070-tops-jetson-t5000-som-for-robotics-and-edge-ai/)
- [CNX Software: Jetson T4000](https://www.cnx-software.com/2026/01/06/nvidia-jetson-t4000-edge-ai-embedded-system-offers-5gbe-networking-four-poe-camera-ports-dio-can-bus-and-more/)
- [Seeed Studio: Jetson AGX Thor Developer Kit](https://www.seeedstudio.com/NVIDIA-Jetson-AGX-Thor-Developer-Kit-p-9965.html)
