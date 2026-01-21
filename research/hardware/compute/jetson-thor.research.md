# Jetson Thor Research Notes

**Research Date:** 2026-01-21
**Current File:** src/content/docs/hardware/compute/jetson-thor.mdx

---

## Product Status - VERIFIED

- **General Availability:** August 25, 2025 (developer kits and T5000 modules)
- **Developer Kit Price:** $3,499 - VERIFIED
- **T5000 Module Price:** $2,999 (1,000 units) - VERIFIED
- **T4000 Module Price:** $1,999 (1,000 units) - VERIFIED

**Sources:**
- [NVIDIA Newsroom: Jetson Thor Now Available](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)
- [CNX Software: Developer Kit](https://www.cnx-software.com/2025/08/19/3499-nvidia-jetson-agx-thor-developer-kit-2070-tops-jetson-t5000-som-for-robotics-and-edge-ai/)

---

## T4000 Availability - CORRECTION NEEDED

| Item | File States | Actual |
|------|-------------|--------|
| T4000 Availability | Q2 2026 | **Now available** (CES 2026) |

- T4000 announced at CES 2026 (January 2026)
- JetPack 7.1 adds T4000 support
- T4000 available through authorized NVIDIA distributors

**Sources:**
- [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)
- [JetsonHacks: JetPack 7.1 and T4000](https://jetsonhacks.com/2026/01/12/jetpack-7-1-and-jetson-t4000-now-available/)
- [CNX Software: Jetson T4000](https://www.cnx-software.com/2026/01/06/nvidia-jetson-t4000-edge-ai-embedded-system-offers-5gbe-networking-four-poe-camera-ports-dio-can-bus-and-more/)

---

## T5000 Specifications - VERIFIED

| Spec | File Value | Verified |
|------|------------|----------|
| AI Performance | 2,070 FP4 TFLOPS (sparse) | VERIFIED |
| GPU Architecture | Blackwell | VERIFIED |
| CUDA Cores | 2,560 | VERIFIED |
| Tensor Cores | 96 (5th gen) | VERIFIED |
| Transformer Engine | Yes (FP4/FP8) | VERIFIED |
| CPU | 14-core Arm Neoverse V3AE @ 2.6 GHz | VERIFIED |
| Memory | 128GB LPDDR5X unified | VERIFIED |
| Memory Bandwidth | 273 GB/s | VERIFIED |
| Power | 40W - 130W | VERIFIED |
| Process | 4nm | VERIFIED |

**Source:** [NVIDIA Developer Blog: Introducing Jetson Thor](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/)

---

## T4000 Specifications - CORRECTIONS NEEDED

| Spec | File Value | Actual Value |
|------|------------|--------------|
| AI Performance | 1,200 FP4 TFLOPS | **1,242 TFLOPS** (some sources say 1,200) |
| Power | 40W - 75W | **40W - 70W** |

**Note:** Multiple sources vary between 1,200 and 1,242 TFLOPS. 1,200 is acceptable approximation.

**Sources:**
- [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)
- [Forecr T4000 Benchmarks](https://www.forecr.io/blogs/hardware/nvidia-jetson-t4000-and-t5000-benchmarks)

---

## Power Specifications - MINOR CORRECTION

| Module | File States | Official Spec |
|--------|-------------|---------------|
| T5000 | 40W - 130W | **75W - 120W configurable, 130W max** |
| T4000 | 40W - 75W | **40W - 70W** |

**Source:** [NVIDIA Jetson Thor Official Page](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/)

---

## JetPack Software - VERIFIED

| Component | File Value | Verified |
|-----------|------------|----------|
| JetPack Version | 7.1 | VERIFIED |
| Jetson Linux | 38.4 | VERIFIED |
| Linux Kernel | 6.8 LTS | VERIFIED |
| Ubuntu | 24.04 LTS | VERIFIED |
| CUDA | 13.0 | VERIFIED |
| TensorRT | 10.13 | VERIFIED |
| cuDNN | 9.12 | VERIFIED |

**Sources:**
- [JetPack SDK Downloads](https://developer.nvidia.com/embedded/jetpack/downloads)
- [JetPack 7.0 Forum](https://forums.developer.nvidia.com/t/jetpack-7-0-jetson-linux-38-2-for-nvidia-jetson-thor-is-now-live/343128)

---

## Isaac ROS - VERIFIED

| Component | File Value | Verified |
|-----------|------------|----------|
| Isaac ROS Version | 4.0 | VERIFIED |
| ROS 2 Distribution | Jazzy | VERIFIED |
| JetPack Support | 7.0+ | VERIFIED |

**Known Limitations:**
- Isaac Perceptor and Nova packages not yet optimized for AGX Thor
- ZED SDK compatibility noted as updated

**Sources:**
- [Isaac ROS 4.0 Forum Announcement](https://forums.developer.nvidia.com/t/nvidia-isaac-ros-4-0-for-thor-has-arrived/352109)
- [Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)

---

## Development Workflow - MINOR UPDATE

| Item | File States | Note |
|------|-------------|------|
| JetPack compatibility | 7.0 | Should say **7.0+** or **7.1** |

File says "JetPack 7.0 compatibility" in Development Workflow section - could update to "JetPack 7.x" or "JetPack 7.1".

---

## Early Adopters - VERIFIED

Industry leaders using Jetson Thor:
- Agility Robotics
- Amazon Robotics
- Boston Dynamics
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

**Source:** [NVIDIA Newsroom](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)

---

## Summary of Corrections Needed

### Must Fix
1. **T4000 Availability:** Change "Q2 2026" to "Now available" or "January 2026" (CES 2026 announcement)
2. **T4000 Power:** Minor: 40W-75W should be 40W-70W

### Minor / Optional
1. **T5000 Power:** Could clarify "75W-120W configurable, 130W max" vs current "40W-130W"
2. **Development Workflow:** Update "JetPack 7.0" to "JetPack 7.1" or "JetPack 7.x"
3. **T4000 TFLOPS:** 1,200 vs 1,242 - current value acceptable

---

## Verified Correct (No Changes Needed)

- T5000 all specifications
- JetPack 7.1 software versions (CUDA 13.0, TensorRT 10.13, cuDNN 9.12)
- Architecture diagram (Blackwell, Neoverse V3AE, Tensor Cores)
- Transformer Engine feature description
- Safety Island (ASIL-D capable)
- Isaac ROS 4.0 integration
- Pricing ($3,499 dev kit, $2,999 T5000, $1,999 T4000)
- Memory specs (128GB T5000, 64GB T4000, 273 GB/s bandwidth)
- Target applications
- Orin comparison table

---

## All Sources

1. [NVIDIA Jetson Thor Official Page](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/)
2. [NVIDIA Newsroom: Jetson Thor Now Available](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)
3. [NVIDIA Developer Blog: Introducing Jetson Thor](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/)
4. [JetPack SDK Downloads](https://developer.nvidia.com/embedded/jetpack/downloads)
5. [JetPack 7.0 Forum Announcement](https://forums.developer.nvidia.com/t/jetpack-7-0-jetson-linux-38-2-for-nvidia-jetson-thor-is-now-live/343128)
6. [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)
7. [JetsonHacks: JetPack 7.1 and T4000](https://jetsonhacks.com/2026/01/12/jetpack-7-1-and-jetson-t4000-now-available/)
8. [Isaac ROS 4.0 Forum Announcement](https://forums.developer.nvidia.com/t/nvidia-isaac-ros-4-0-for-thor-has-arrived/352109)
9. [Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)
10. [CNX Software: Jetson AGX Thor Developer Kit](https://www.cnx-software.com/2025/08/19/3499-nvidia-jetson-agx-thor-developer-kit-2070-tops-jetson-t5000-som-for-robotics-and-edge-ai/)
11. [CNX Software: Jetson T4000](https://www.cnx-software.com/2026/01/06/nvidia-jetson-t4000-edge-ai-embedded-system-offers-5gbe-networking-four-poe-camera-ports-dio-can-bus-and-more/)
12. [Forecr T4000/T5000 Benchmarks](https://www.forecr.io/blogs/hardware/nvidia-jetson-t4000-and-t5000-benchmarks)
