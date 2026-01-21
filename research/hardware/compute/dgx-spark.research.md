# DGX Spark Research Notes

**Research Date:** 2026-01-21
**Entry File:** `src/content/docs/hardware/compute/dgx-spark.mdx`

---

## Validation Status: MOSTLY ACCURATE

The current glossary entry has been validated against official sources. All major specifications are correct.

---

## Verified Specifications (All Confirmed)

| Spec | Current Value | Status | Source |
|------|---------------|--------|--------|
| GPU | NVIDIA GB10 Grace Blackwell Superchip | ✓ Correct | [NVIDIA Official](https://www.nvidia.com/en-us/products/workstations/dgx-spark/) |
| AI Performance | 1,000 TOPS (INT8), 1 PFLOP (FP4 sparse) | ✓ Correct | [NVIDIA Official](https://www.nvidia.com/en-us/products/workstations/dgx-spark/) |
| CUDA Cores | 6,144 | ✓ Correct | [VideoCardz](https://videocardz.com/newz/nvidia-dgx-spark-features-6144-cuda-cores-just-as-many-as-rtx-5070) |
| Tensor Cores | 192 (5th gen) | ✓ Correct | [NVIDIA Hardware Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| RT Cores | 48 (4th gen) | ✓ Correct | [NVIDIA Hardware Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| Memory | 128GB unified LPDDR5X | ✓ Correct | [NVIDIA Official](https://www.nvidia.com/en-us/products/workstations/dgx-spark/) |
| Memory Bandwidth | 273 GB/s | ✓ Correct | [NVIDIA Hardware Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| CPU | 20-core ARM (10x Cortex-X925 + 10x Cortex-A725) | ✓ Correct | [WCCFTech](https://wccftech.com/nvidia-gb10-superchip-soc-3nm-20-arm-v9-2-cpu-cores-nvfp4-blackwell-gpu-lpddr5x-9400-memory-140w-tdp/) |
| Storage | 4TB NVMe (Founders), 1TB+ (partners) | ✓ Correct | [Micro Center](https://www.microcenter.com/product/699008/nvidia-dgx-spark) |
| Connectivity | 1x 10GbE, 2x QSFP (200Gbps), WiFi 7, BT 5.3, 4x USB 4.0 | ✓ Correct | [NVIDIA Hardware Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| Power | 240W PSU, 140W TDP | ✓ Correct | [NVIDIA Hardware Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| Dimensions | 150 × 150 × 50.5 mm (1.2 kg) | ✓ Correct | [NVIDIA Hardware Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |

---

## Pricing & Availability (Confirmed)

| Item | Current Value | Status | Source |
|------|---------------|--------|--------|
| Founders Edition | $3,999 (4TB) | ✓ Correct | [NVIDIA Marketplace](https://marketplace.nvidia.com/en-us/enterprise/personal-ai-supercomputers/dgx-spark/) |
| Partner versions | From $2,999 (1TB) | ✓ Correct | [Glukhov Price Analysis](https://www.glukhov.org/post/2025/10/nvidia-dgx-spark-prices/) |
| Availability date | October 15, 2025 | ✓ Correct | [NVIDIA Newsroom](https://nvidianews.nvidia.com/news/nvidia-dgx-spark-arrives-for-worlds-ai-developers) |

**Partner note:** Not all partner versions include ConnectX-7 200Gbps networking (e.g., some budget models only have 10GbE + WiFi 7).

---

## Software Stack (Verified & Updated)

### DGX OS
- **Base:** Ubuntu 24.04 LTS ✓
- **Kernel:** HWE kernel 6.14 ✓
- **Current version:** DGX OS 7.3.1
- **Driver:** 580.95.05
- **CUDA:** 13.0.2

Source: [DGX Spark Release Notes](https://docs.nvidia.com/dgx/dgx-spark/release-notes.html)

### PyTorch
- **Current claim:** PyTorch 2.9 ✓ Correct
- CUDA 13 requires torch v2.9 minimum
- PyTorch 2.8 does NOT support CUDA 13/sm_121

Sources:
- [PyTorch Forums](https://discuss.pytorch.org/t/nvidia-dgx-spark-support/223677)
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/effective-pytorch-and-cuda/348230)

### Isaac Sim
- **Current claim:** Isaac Sim 5.x (5.1.0) ✓ Correct
- aarch64 builds supported on DGX Spark
- Available as standalone package: isaac-sim-standalone-5.1.0-linux-aarch64.zip

Source: [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)

### Isaac Lab Limitations on DGX Spark (aarch64)
- **Current claim:** SkillGen, OpenXR, SKRL/JAX training not supported ✓ Correct
- Live-streaming not yet supported
- Some pip packages require building from source (flash-attention)
- Multi Node Training has limitations

Sources:
- [Isaac Lab Installation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)
- [NVIDIA Build Guide](https://build.nvidia.com/spark/isaac/overview)

### ROS 2
- **Current claim:** ros-jazzy-isaac-ros-dev-tools ✓ Correct
- Isaac ROS 4.0 fully supports ROS 2 Jazzy
- ROS 2 Jazzy on Ubuntu 24.04 is recommended

Sources:
- [ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy/Related-Projects/Nvidia-ROS2-Projects.html)
- [Isaac Sim ROS Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

---

## Manufacturing Details (Reference)

- **Process:** TSMC 3nm
- **Design:** Two dielets (S-die from MediaTek, G-die NVIDIA Blackwell)
- **Packaging:** Advanced 2.5D packaging
- **Interconnect:** NVLink C2C (600 GB/s bidirectional)
- **Collaboration:** MediaTek provides CPU die and memory controller

Sources:
- [ServeTheHome Hot Chips 2025](https://www.servethehome.com/nvidia-outlines-gb10-soc-architecture-at-hot-chips-2025/)
- [PC Gamer MediaTek Details](https://www.pcgamer.com/hardware/processors/further-details-of-nvidias-gb10-superchip-shine-light-on-the-complex-collaboration-with-mediatek/)

---

## CES 2026 Updates

- Software-only update (no hardware changes)
- Performance improved up to 2.5× since launch
- Better open-source framework support: PyTorch, vLLM, SGLang, llama.cpp, LlamaIndex
- TensorRT-LLM enhancements and decoding optimizations
- Two DGX Spark systems can be clustered for 256GB combined memory

Sources:
- [NVIDIA Developer Blog](https://developer.nvidia.com/blog/new-software-and-model-optimizations-supercharge-nvidia-dgx-spark/)
- [StorageReview CES 2026](https://www.storagereview.com/news/nvidia-dgx-spark-achieves-2-5x-performance-and-8x-video-speed-in-ces-2026-enterprise-update)

---

## Minor Suggestions for Future Updates

1. **Consider adding:** Mention of two-unit clustering capability (256GB combined)
2. **Consider adding:** Note about partner variations lacking ConnectX-7
3. **Consider updating:** Reference CES 2026 performance improvements (2.5×)

---

## Primary Sources

### Official NVIDIA Documentation
- [NVIDIA DGX Spark Official Page](https://www.nvidia.com/en-us/products/workstations/dgx-spark/)
- [DGX Spark Hardware Overview](https://docs.nvidia.com/dgx/dgx-spark/hardware.html)
- [DGX Spark Release Notes](https://docs.nvidia.com/dgx/dgx-spark/release-notes.html)
- [DGX Spark User Guide (Jan 2026)](https://docs.nvidia.com/dgx/dgx-spark/dgx-spark.pdf)
- [NVIDIA Marketplace](https://marketplace.nvidia.com/en-us/enterprise/personal-ai-supercomputers/dgx-spark/)
- [NVIDIA Developer Blog - CES 2026](https://developer.nvidia.com/blog/new-software-and-model-optimizations-supercharge-nvidia-dgx-spark/)

### Isaac Sim/Lab
- [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [Isaac Lab Installation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)
- [NVIDIA Build Guide - Isaac on Spark](https://build.nvidia.com/spark/isaac/overview)

### ROS Documentation
- [ROS 2 Jazzy NVIDIA Projects](https://docs.ros.org/en/jazzy/Related-Projects/Nvidia-ROS2-Projects.html)
- [Isaac Sim ROS Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

### Technical Analysis
- [VideoCardz - CUDA Core Count](https://videocardz.com/newz/nvidia-dgx-spark-features-6144-cuda-cores-just-as-many-as-rtx-5070)
- [WCCFTech - GB10 CPU Details](https://wccftech.com/nvidia-gb10-superchip-soc-3nm-20-arm-v9-2-cpu-cores-nvfp4-blackwell-gpu-lpddr5x-9400-memory-140w-tdp/)
- [LMSYS In-Depth Review](https://lmsys.org/blog/2025-10-13-nvidia-dgx-spark/)
- [ServeTheHome - GB10 Architecture](https://www.servethehome.com/nvidia-outlines-gb10-soc-architecture-at-hot-chips-2025/)
