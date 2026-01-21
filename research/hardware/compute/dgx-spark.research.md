# DGX Spark Research Notes

**Research Date:** January 2026
**Entry File:** `src/content/docs/hardware/compute/dgx-spark.mdx`

---

## Key Corrections Needed

### 1. GPU Name Error
- **Current:** "NVIDIA GB110 (Blackwell architecture)"
- **Correct:** "NVIDIA GB10 Grace Blackwell Superchip"
- **Source:** [NVIDIA Official](https://www.nvidia.com/en-us/products/workstations/dgx-spark/)

### 2. Price Incorrect
- **Current:** "starting at $3,000"
- **Correct:** $3,999 for Founders Edition (4TB); Partner versions from $2,999 (1TB)
- **Source:** [NVIDIA Marketplace](https://marketplace.nvidia.com/en-us/enterprise/personal-ai-supercomputers/dgx-spark/)

### 3. Availability Date Incorrect
- **Current:** "began shipping in May 2025"
- **Correct:** Commercially available October 15, 2025
- **Source:** [NVIDIA Newsroom](https://nvidianews.nvidia.com/news/nvidia-dgx-spark-arrives-for-worlds-ai-developers)

### 4. Memory Bandwidth Incorrect
- **Current:** "512 GB/s"
- **Correct:** 273 GB/s (raw), up to 301 GB/s aggregate via C2C NVLINK
- **Source:** [NVIDIA Hardware Overview](https://docs.nvidia.com/dgx/dgx-spark/hardware.html)

### 5. CPU Description Incomplete
- **Current:** "MediaTek ARM-based (Grace-class)"
- **Correct:** 20-core ARM (10x Cortex-X925 + 10x Cortex-A725), co-designed with MediaTek
- **Source:** [WCCFTech GB10 Details](https://wccftech.com/nvidia-gb10-superchip-soc-3nm-20-arm-v9-2-cpu-cores-nvfp4-blackwell-gpu-lpddr5x-9400-memory-140w-tdp/)

### 6. Power Specification Incorrect
- **Current:** "200W typical"
- **Correct:** 240W power supply, 140W TDP for GB10 SoC, ~170W typical under load
- **Source:** [NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/dgx-spark-power-clarification/349668)

### 7. Connectivity Incorrect
- **Current:** "2x 10GbE, WiFi 7, TB5"
- **Correct:** 1x 10GbE RJ-45, 2x QSFP (200 Gbps via ConnectX-7), WiFi 7, Bluetooth 5.3, 4x USB 4.0, 1x HDMI 2.1a
- **Note:** No Thunderbolt 5 mentioned in official specs; USB 4.0 ports instead
- **Source:** [NVIDIA Hardware Overview](https://docs.nvidia.com/dgx/dgx-spark/hardware.html)

### 8. Form Factor Description
- **Current:** "Mac Studio-sized desktop"
- **Correct:** 150 × 150 × 50.5 mm (5.9" × 5.9" × 1.98"), 1.2 kg, NUC-style
- **Source:** [Storage Review](https://www.storagereview.com/review/nvidia-dgx-spark-review-the-ai-appliance-bringing-datacenter-capabilities-to-desktops)

### 9. CUDA Cores Missing
- **Current:** "4096" in architecture diagram
- **Correct:** 6,144 CUDA cores
- **Source:** [VideoCardz](https://videocardz.com/newz/nvidia-dgx-spark-features-6144-cuda-cores-just-as-many-as-rtx-5070)

### 10. Tensor Cores Missing
- **Current:** "512" in architecture diagram
- **Correct:** 192 fifth-generation Tensor Cores
- **Source:** [IntuitionLabs Review](https://intuitionlabs.ai/articles/nvidia-dgx-spark-review)

### 11. RT Cores
- **Correct:** 48 fourth-generation RT cores
- **Source:** [IntuitionLabs Review](https://intuitionlabs.ai/articles/nvidia-dgx-spark-review)

### 12. Software Stack Updates
- **Current:** "CUDA 12.8" and "Ubuntu 24.04"
- **Correct:** CUDA 13 required (torch v2.9), DGX OS based on Ubuntu 24.04 with HWE kernel 6.14
- **Source:** [NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/can-i-downgrade-dgx-spark-to-cuda-12/348191)

### 13. Isaac Sim Version
- **Current:** "isaac-sim-4.0.0"
- **Correct:** Isaac Sim 5.x supported, requires driver 580.95.05
- **Note:** Some Isaac Lab features not supported on aarch64 (SkillGen, OpenXR, SKRL JAX)
- **Source:** [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)

---

## Verified Specifications

| Spec | Verified Value | Source |
|------|----------------|--------|
| GPU | NVIDIA GB10 Grace Blackwell Superchip | [NVIDIA](https://www.nvidia.com/en-us/products/workstations/dgx-spark/) |
| AI Performance | 1,000 TOPS (INT8), 1 PFLOP (FP4 sparse) | [NVIDIA](https://www.nvidia.com/en-us/products/workstations/dgx-spark/) |
| CUDA Cores | 6,144 | [VideoCardz](https://videocardz.com/newz/nvidia-dgx-spark-features-6144-cuda-cores-just-as-many-as-rtx-5070) |
| Tensor Cores | 192 (5th gen) | [IntuitionLabs](https://intuitionlabs.ai/articles/nvidia-dgx-spark-review) |
| RT Cores | 48 (4th gen) | [IntuitionLabs](https://intuitionlabs.ai/articles/nvidia-dgx-spark-review) |
| Memory | 128GB unified LPDDR5X | [NVIDIA](https://www.nvidia.com/en-us/products/workstations/dgx-spark/) |
| Memory Bandwidth | 273 GB/s (raw) | [LMSYS Review](https://lmsys.org/blog/2025-10-13-nvidia-dgx-spark/) |
| CPU | 20-core ARM (10x X925 + 10x A725) | [WCCFTech](https://wccftech.com/nvidia-gb10-superchip-soc-3nm-20-arm-v9-2-cpu-cores-nvfp4-blackwell-gpu-lpddr5x-9400-memory-140w-tdp/) |
| Storage | 4TB NVMe SSD (Founders), 1TB+ (partners) | [Micro Center](https://www.microcenter.com/product/699008/nvidia-dgx-spark) |
| Power | 240W PSU, 140W TDP | [NVIDIA Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| Dimensions | 150 × 150 × 50.5 mm | [NVIDIA Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| Weight | 1.2 kg (2.6 lbs) | [Storage Review](https://www.storagereview.com/review/nvidia-dgx-spark-review-the-ai-appliance-bringing-datacenter-capabilities-to-desktops) |
| Networking | 1x 10GbE, 2x QSFP (200Gbps), WiFi 7, BT 5.3 | [NVIDIA Docs](https://docs.nvidia.com/dgx/dgx-spark/hardware.html) |
| Price | $3,999 (Founders), $2,999+ (partners) | [NVIDIA Marketplace](https://marketplace.nvidia.com/en-us/enterprise/personal-ai-supercomputers/dgx-spark/) |
| Availability | October 15, 2025 | [NVIDIA Newsroom](https://nvidianews.nvidia.com/news/nvidia-dgx-spark-arrives-for-worlds-ai-developers) |

---

## Software Stack Notes

- **OS:** DGX OS based on Ubuntu 24.04 with HWE kernel 6.14
- **CUDA:** CUDA 13 (required for GB10)
- **PyTorch:** torch v2.9 required for CUDA 13 support
- **Architecture:** aarch64 (ARM64)
- **Container Runtime:** NVIDIA Container Toolkit preinstalled

### Isaac Sim Limitations on DGX Spark (aarch64)
- SkillGen not supported
- OpenXR/XR teleoperation not supported
- SKRL/JAX training not validated
- Livestream/Hub Workstation Cache not supported
- Cosmos Transfer1 not supported

---

## CES 2026 Updates
- Software-only update announced
- NVFP4 format support for up to 2.6x performance increase
- DGX Spark Playbooks for bundled workflows
- Local Nsight Copilot for CUDA assistance
- External AI accelerator mode
- **Source:** [HotHardware CES 2026](https://hothardware.com/news/nvidia-dgx-spark-performance-and-sdk-updates-ces2026)

---

## Primary Sources

- [NVIDIA DGX Spark Official Page](https://www.nvidia.com/en-us/products/workstations/dgx-spark/)
- [NVIDIA DGX Spark Marketplace](https://marketplace.nvidia.com/en-us/enterprise/personal-ai-supercomputers/dgx-spark/)
- [DGX Spark Hardware Overview](https://docs.nvidia.com/dgx/dgx-spark/hardware.html)
- [DGX Spark User Guide PDF (Jan 2026)](https://docs.nvidia.com/dgx/dgx-spark/dgx-spark.pdf)
- [DGX Spark Release Notes](https://docs.nvidia.com/dgx/dgx-spark/release-notes.html)
- [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [NVIDIA Newsroom - DGX Spark Launch](https://nvidianews.nvidia.com/news/nvidia-dgx-spark-arrives-for-worlds-ai-developers)
