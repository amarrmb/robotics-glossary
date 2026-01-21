# Neural Networks for Robotics - Research Notes

**Research Date:** 2026-01-21
**File:** src/content/docs/concepts/ai/neural-networks.mdx

---

## Current Version Numbers

### NVIDIA Software Stack
- **TensorRT:** 10.x (TensorRT 10.0 GA released, TensorRT 10.13 in JetPack 7.1)
  - **TensorRT-LLM:** v1.0 with 8x inference performance improvement
  - **TensorRT Edge-LLM:** New open-source C++ SDK for LLM/VLM inference on Jetson (CES 2026)
  - Source: [TensorRT SDK](https://developer.nvidia.com/tensorrt), [TensorRT Edge-LLM Blog](https://developer.nvidia.com/blog/accelerating-llm-and-vlm-inference-for-automotive-and-robotics-with-nvidia-tensorrt-edge-llm)
- **JetPack 7.1:** Latest for Jetson Thor (Jetson Linux 38.4)
  - Includes: CUDA 13, cuDNN 9.12, TensorRT 10.13
  - OS: Linux Kernel 6.8, Ubuntu 24.04 LTS
  - JetPack 7.2 coming with Jetson Orin support
  - Source: [JetPack Downloads](https://developer.nvidia.com/embedded/jetpack/downloads), [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)
- **Isaac ROS DNN Inference:** TensorRT and Triton nodes, NITROS-accelerated
  - Source: [Isaac ROS DNN Inference](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_inference/index.html)

### NVIDIA Hardware
- **Jetson Thor/T5000:** Up to 2070 FP4 TFLOPS, 128GB memory, 40-130W, Blackwell GPU
  - 14-core Arm Neoverse-V3AE CPU, MIG support
  - 7.5x more AI compute vs Orin, 3.5x better efficiency
  - Source: [Jetson Thor Blog](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/)
- **Jetson T4000:** Up to 1200 FP4 TFLOPS, 64GB memory, 40-70W (available CES 2026)
  - 4x AI compute vs AGX Orin
  - Source: [JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)
- **Jetson AGX Orin 64GB:** Up to 275 TOPS, 15-60W
- **Jetson Orin Nano Super:** Up to 67 TOPS, 7-25W, $249
  - Source: [Jetson Modules](https://developer.nvidia.com/embedded/jetson-modules)

---

## Foundation Models - Verification

### GR00T
- **Current in file:** GR00T N1.6 ✓ (correct)
- **Details:** World's first open foundation model for generalized humanoid robot reasoning
- **Architecture:** Dual-system VLA - Vision-Language Model + Diffusion Transformer
- GR00T N1.6 integrates NVIDIA Cosmos Reason for loco-manipulation and reasoning
- GR00T N1.5 trained using GR00T-Dreams synthetic data (36 hours vs 3 months manual)
- 40% performance boost with synthetic + real data
- Source: [GR00T N1 Blog](https://developer.nvidia.com/blog/accelerate-generalist-humanoid-robot-development-with-nvidia-isaac-gr00t-n1/), [GR00T N1.6 Blog](https://developer.nvidia.com/blog/building-generalist-humanoid-capabilities-with-nvidia-isaac-gr00t-n1-6-using-a-sim-to-real-workflow)

### OpenVLA
- **Parameters:** 7B (Llama 2 7B backbone)
- **Architecture:** SigLIP + DinoV2 visual encoder → projector → Llama 2 7B
- **Training:** 970k real-world robot demos from Open X-Embodiment
- Outperforms 55B RT-2-X by 16.5% on 29 evaluation tasks
- Apache 2.0 licensed (open-source)
- Source: [OpenVLA Paper](https://arxiv.org/abs/2406.09246), [OpenVLA Website](https://openvla.github.io/)

### RT-2
- Google DeepMind's VLA model (July 2023)
- Built on PaLI-X and PaLM-E foundations
- Co-fine-tuned with RT-1 robot data
- 62% success on novel scenarios vs RT-1's 32%
- Source: [RT-2 Blog](https://deepmind.google/blog/rt-2-new-model-translates-vision-and-language-into-action/)

### PaLM-E
- **Largest version:** 562B parameters
- Combines PaLM + ViT-22B
- Published March 2023
- Source: [PaLM-E Blog](https://research.google/blog/palm-e-an-embodied-multimodal-language-model/)

### Octo
- **Parameters:** Octo-Small (27M), Octo-Base (93M)
- Uses diffusion policy for continuous trajectories
- Trained on 800k robot episodes from Open X-Embodiment (25 datasets)
- Published RSS 2024 (May 2024)
- Source: [Octo Website](https://octo-models.github.io/), [Octo Paper](https://arxiv.org/abs/2405.12213)

---

## Corrections/Updates Needed

### Foundation Models Table (Line 143-149)
| Model | Current Entry | Status |
|-------|---------------|--------|
| RT-2 | "VLA - Manipulation from language" | ✓ Accurate |
| GR00T N1.6 | "VLA - Humanoid full-body control" | ✓ Accurate |
| OpenVLA | "VLA - Generalizable manipulation (open-weight)" | ✓ Accurate |
| PaLM-E | "VLM - Embodied reasoning" | ✓ Accurate |
| Octo | "Policy - Generalizable manipulation" | ✓ Accurate |

### Hardware Table (Lines 112-118)
- **Jetson Thor timing:** File says "5-15ms" - reasonable estimate
- **Note:** Thor now has T4000/T5000 module variants; could clarify

### Line 150
- "These run efficiently on Jetson Thor with Transformer Engine"
- **Status:** Thor uses Blackwell GPU with native Transformer Engine support ✓

### TensorRT Command (Line 129)
- `trtexec --onnx=model.onnx --saveEngine=model.trt --fp16`
- **Status:** Still valid syntax ✓

---

## Emerging/Optional Updates

### SmolVLA (Hugging Face)
- 450M parameters - compact VLA alternative
- Trained on LeRobot dataset
- Comparable performance to larger VLAs
- Could be added as emerging open-source option

### TensorRT Edge-LLM
- New SDK for on-device LLM/VLM inference (CES 2026)
- Partners: Bosch, ThunderSoft, MediaTek
- Relevant for robotics LLM deployment

---

## Summary

**Overall Assessment:** The current file is accurate and well-maintained.

**Verified as correct:**
- GR00T N1.6 version and description
- Foundation models table entries
- TensorRT optimization workflow
- Training paradigms section
- CNN/Transformer/RNN descriptions

**Minor potential updates:**
- Could distinguish Jetson Thor T4000/T5000 variants
- TensorRT Edge-LLM is new and relevant (CES 2026)
- SmolVLA as emerging compact alternative

**No critical corrections required.**
