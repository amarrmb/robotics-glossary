# Neural Networks for Robotics - Research Notes

**Research Date:** January 2026
**File:** src/content/docs/concepts/ai/neural-networks.mdx

---

## Current Version Numbers

### NVIDIA Software Stack
- **TensorRT:** 10.14.1 (latest)
  - Source: [NVIDIA TensorRT Docs](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/release-notes.html)
- **JetPack 6:** 6.2 (latest for Orin) - includes CUDA 12.6, TensorRT 10.3, cuDNN 9.3
  - Source: [JetPack Downloads](https://developer.nvidia.com/embedded/jetpack/downloads)
- **JetPack 7:** Available for Jetson Thor - Linux Kernel 6.8, Ubuntu 24.04 LTS
  - Source: [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- **Isaac ROS DNN Inference:** v4.0-0 (November 2025)
  - Source: [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference/releases)

### NVIDIA Hardware
- **Jetson Thor:** Now available (Blackwell GPU, 2560 cores, 96 Tensor Cores)
  - 2070 FP4 TFLOPS / 1035 FP8 TFLOPS
  - 128GB LPDDR5X memory, 273 GB/s bandwidth
  - 14-core Arm Neoverse-V3AE CPU
  - 7.5x more AI compute vs Jetson Orin
  - Power: 75W-120W (configurable)
  - Source: [NVIDIA Blog](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/)
- **Jetson AGX Orin 64GB:** Up to 275 TOPS, 15-60W
- **Jetson Orin NX:** Up to 100 TOPS
- **Jetson Orin Nano Super:** Up to 67 TOPS (with JetPack 6.2 Super Mode: 70% higher gen AI performance)
  - Source: [NVIDIA Jetson Benchmarks](https://developer.nvidia.com/embedded/jetson-benchmarks)

---

## Corrections Needed

### Inference Speed Table
- **Current entry:** "Jetson Thor | 5-15ms" - needs verification
- **Note:** Jetson Thor is now available; performance specs confirmed but specific ms latency benchmarks may vary by model

### Foundation Models Table - Updates Required

| Model | Current Entry | Update Needed |
|-------|---------------|---------------|
| GR00T | "Humanoid - Full-body control" | Update to "GR00T N1.6" - latest version with full body control, uses Cosmos Reason |
| RT-2 | "VLA - Manipulation from language" | Accurate, but note Google also released Gemini Robotics On-Device (2025) |
| PaLM-E | "VLM - Embodied reasoning" | Accurate - 562B params, combines PaLM + ViT-22B |
| Octo | "Policy - Generalizable manipulation" | Add note: Octo-Small (27M) / Octo-Base (93M params), also mention OpenVLA (Jan 2025) |

### Object Detection Models
- **YOLO:** Document now mentions "YOLO" generically
  - Latest: YOLO26 (January 2026) - native end-to-end, no NMS, 43% faster CPU inference
  - Previous: YOLOv12 (February 2025), YOLO11 (September 2024)
  - Source: [Ultralytics YOLO26 Docs](https://docs.ultralytics.com/models/yolo26/)

### Depth Estimation
- **DepthAnything:** Document mentions "DepthAnything"
  - Latest: Depth Anything 3 (DA3) - November 2025 - 44.3% better pose accuracy vs VGGT
  - Also: Video Depth Anything (CVPR 2025 Highlight)
  - Source: [Depth Anything 3 GitHub](https://github.com/ByteDance-Seed/Depth-Anything-3)

### Vision Transformers
- **DINO:** Document mentions "DINO"
  - Latest: DINOv3 (August 2025) - 7B params, 1.7B training images
  - +6 mIoU on ADE20K vs DINOv2
  - Source: [Meta AI DINOv2 Blog](https://ai.meta.com/blog/dino-v2-computer-vision-self-supervised-learning/)

---

## GR00T N1 Details (for Foundation Models section)

- **Full name:** NVIDIA Isaac GR00T N1
- **Latest version:** GR00T N1.6
- **Architecture:** Vision-Language-Action (VLA) with dual-system
  - System 2: Vision-language module for environment interpretation
  - System 1: Diffusion transformer for real-time motor actions
- **Brain:** Uses Cosmos Reason 2 for reasoning
- **Training:** 780,000 synthetic trajectories (6,500 hours equivalent) in 11 hours
- **Performance:** 40% improvement with synthetic + real data
- **Partners:** Agility Robotics, Boston Dynamics, 1X, NEURA Robotics
- Source: [NVIDIA GR00T N1 Blog](https://developer.nvidia.com/blog/accelerate-generalist-humanoid-robot-development-with-nvidia-isaac-gr00t-n1/)

---

## Additional Notes

### Gemini Robotics On-Device (Google, 2025)
- VLA foundation model for on-device inference
- Fine-tunable with as few as 50 demonstrations
- 60%+ success rate on new tasks with ~100 demos
- Source: [InfoQ - Google Gemini Robotics](https://www.infoq.com/news/2025/07/google-gemini-robotics/)

### OpenVLA (UC Berkeley, January 2025)
- First commercially-usable open-weight VLA model
- Apache 2.0 licensed
- Trained on RT-X dataset (527K trajectories)
- Matches Google RT-2 performance
- Source: [RoboCloud Guide](https://robocloud-dashboard.vercel.app/learn/blog/open-weight-robot-models)

### Transformer Engine on Jetson Thor
- Document mentions "Transformer Engine" for foundation models
- Confirmed: Jetson Thor supports VLA models like GR00T N1.5 and popular LLMs/VLMs
- Source: [NVIDIA Newsroom](https://nvidianews.nvidia.com/news/nvidia-blackwell-powered-jetson-thor-now-available-accelerating-the-age-of-general-robotics)

---

## Summary of Required Changes

1. **Foundation Models table:** Update GR00T to "GR00T N1.6", consider adding OpenVLA
2. **Object detection:** Could specify YOLO26 as latest or keep generic "YOLO"
3. **Depth estimation:** Could update to "DepthAnything V3" or keep generic
4. **Vision:** Could update DINO to DINOv3 or keep generic
5. **Hardware specs:** Jetson Thor specs confirmed accurate
6. **TensorRT command:** Still valid syntax

**Overall Assessment:** Content is accurate and well-structured. Updates are optional - either specify latest versions (which will age quickly) or keep generic references (more maintainable).
