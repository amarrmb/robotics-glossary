# Vision-Language-Action (VLA) Models - Research Notes

**Research Date:** 2026-01-21
**File to create:** src/content/docs/concepts/ai/vla.mdx
**Description:** VLA models for robot control - from RT-2 to GR00T

---

## Key Concepts to Cover

### What are VLA Models?
Vision-Language-Action (VLA) models are multimodal foundation models that integrate:
- **Vision**: Camera images/video of the robot's environment
- **Language**: Natural language instructions (e.g., "pick up the red cup")
- **Action**: Low-level robot control outputs (joint positions, motor commands)

VLAs directly map visual observations + language instructions → executable robot actions in a single neural network forward pass.

### Historical Context
- **July 2023**: RT-2 pioneered the VLA concept (Google DeepMind)
- **2024**: OpenVLA made VLAs accessible as open-source
- **2025**: Explosion of VLA models (GR00T N1, π0, Gemini Robotics, Helix, SmolVLA)
- **2026**: Mature ecosystem with edge deployment (Jetson Thor) and NVIDIA/Hugging Face integration

---

## Current Version Numbers (January 2026)

### NVIDIA GR00T
- **GR00T N1.6**: Latest version (CES 2026)
  - Integrates Cosmos Reason VLM (2B variant)
  - 2x larger DiT (32 layers vs 16 in N1.5)
  - Whole-body humanoid control with reasoning
  - Pre-trained on 10k+ hours of robot data
  - Source: [NVIDIA Isaac GR00T GitHub](https://github.com/NVIDIA/Isaac-GR00T)
- **GR00T N1.5**: March 2025 release
  - First open foundation model for humanoid robots
  - Source: [GR00T N1 Paper](https://arxiv.org/abs/2503.14734)

### OpenVLA
- **OpenVLA**: 7B parameters (June 2024, Stanford)
  - Architecture: SigLIP + DinoV2 → Projector → Llama 2 7B
  - Training: 970k real-world demos from Open X-Embodiment (70+ domains)
  - Outperforms RT-2-X (55B) by 16.5% with 7x fewer parameters
  - Apache 2.0 license
  - Source: [OpenVLA Paper](https://arxiv.org/abs/2406.09246), [GitHub](https://github.com/openvla/openvla)
- **OpenVLA-OFT**: March 2025 update
  - 25-50x faster inference
  - Multi-image input support
  - High-frequency bimanual control
- **OpenVLA-FAST**: January 2025
  - Up to 15x faster inference with FAST tokenizer
  - Source: [OpenVLA-FAST](https://openvla.github.io/)

### Google DeepMind
- **RT-2**: July 2023 (foundational VLA)
  - Built on PaLI-X and PaLM-E backbones
  - Actions as text tokens (256 bins)
  - Chain-of-thought reasoning for control
  - Source: [RT-2 Paper](https://arxiv.org/abs/2307.15818)
- **RT-2-X**: Trained on Open X-Embodiment dataset
  - 3x improvement on emergent skills vs RT-2
  - Source: [Open X-Embodiment](https://robotics-transformer-x.github.io/)
- **Gemini Robotics**: March 2025
  - Built on Gemini 2.0
  - Highly dexterous tasks (origami folding, card manipulation)
  - Source: [Gemini Robotics](https://deepmind.google/models/gemini-robotics/gemini-robotics/)
- **Gemini Robotics On-Device**: June 2025
  - First VLA available for fine-tuning
  - Adapts with 50-100 demonstrations
  - Source: [Gemini On-Device Blog](https://deepmind.google/discover/blog/gemini-robotics-on-device-brings-ai-to-local-robotic-devices/)
- **Gemini Robotics 1.5**: September 2025
  - Think-before-act reasoning
  - State-of-the-art spatial understanding
  - Source: [Gemini Robotics 1.5 Blog](https://deepmind.google/blog/gemini-robotics-15-brings-ai-agents-into-the-physical-world/)

### Physical Intelligence
- **π0 (pi-zero)**: October 2024
  - 3.3B parameters (PaliGemma backbone)
  - Flow matching for 50Hz continuous actions
  - Trained on 7 robot platforms, 68 tasks
  - Source: [π0 Paper](https://arxiv.org/html/2410.24164v1), [Physical Intelligence](https://www.physicalintelligence.company/blog/pi0)
- **π0-FAST**: 5x faster training, better generalization
- **π0.5**: September 2025
  - Open-world generalization to new environments
  - Source: [π0.5 Blog](https://www.physicalintelligence.company/blog/pi05)
- **openpi**: Open-source implementation with PyTorch
  - Source: [GitHub](https://github.com/Physical-Intelligence/openpi)

### Hugging Face LeRobot
- **SmolVLA**: June 2025
  - 450M parameters (compact)
  - Trained on LeRobot community data (10M frames, 487 datasets)
  - Matches larger VLAs (Octo, OpenVLA, π0) performance
  - Runs on consumer hardware
  - Source: [SmolVLA Blog](https://huggingface.co/blog/smolvla)
- **X-VLA**: 0.9B parameters
  - Soft-prompt mechanism for multi-embodiment
  - State-of-the-art across 6 sim platforms, 3 real robots
  - Source: [X-VLA Docs](https://huggingface.co/docs/lerobot/xvla)
- **LeRobot v0.4.0**: 2025
  - GR00T N1.5, π0.5 support
  - Dataset v3.0 (400GB+ datasets like OXE)
  - LIBERO benchmark (130+ tasks)
  - Source: [LeRobot v0.4.0](https://huggingface.co/blog/lerobot-release-v040)

### Figure AI
- **Helix**: February 2025
  - First VLA for full humanoid upper-body control
  - Dual-system architecture (VLM + visuomotor policy)
  - Trained on ~500 hours teleoperation data
  - Source: [Helix Blog](https://www.figure.ai/news/helix)

---

## VLA Architecture Patterns

### Single-Model Architecture (RT-2, OpenVLA, π0)
```
Image + Text → [Vision Encoder] → [LLM Backbone] → Action Tokens → Robot
                     ↓                    ↓
              Visual features      Tokenized actions
```
- Simple, lower latency
- Single forward pass
- Examples: RT-2, OpenVLA, π0

### Dual-System Architecture (GR00T N1, Helix)
```
┌─────────────────────────────────────────────────────────┐
│ System 2 (Slow): Vision-Language Model                   │
│   - Scene understanding                                  │
│   - Language comprehension                               │
│   - High-level reasoning                                 │
└───────────────────────┬─────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│ System 1 (Fast): Diffusion/Flow Action Generator        │
│   - Real-time motor control (~10ms latency)             │
│   - Smooth action trajectories                          │
│   - 24-50Hz control frequency                           │
└─────────────────────────────────────────────────────────┘
```
- Mirrors human cognition (fast reflexes + deliberate thinking)
- Better real-time performance
- Examples: GR00T N1.6, Helix

### Action Generation Methods

1. **Autoregressive Tokenization** (RT-2, OpenVLA)
   - Actions discretized to 256 bins
   - Predicted as text tokens
   - Simple but slow inference

2. **Flow Matching** (π0, GR00T N1)
   - Continuous action prediction
   - Non-autoregressive (fast)
   - 50Hz control possible

3. **Diffusion Policy** (GR00T N1, Helix)
   - Generates smooth trajectories
   - Handles multi-modal action distributions
   - ~10ms latency with optimization

4. **FAST Tokenization** (π0-FAST, OpenVLA-FAST)
   - Frequency-space Action Sequence Tokenization
   - DCT compression of action sequences
   - Up to 15x faster than standard tokenization

---

## NVIDIA + ROS2 Ecosystem Integration

### Isaac GR00T Stack
```
┌──────────────────────────────────────────────┐
│           Isaac GR00T N1.6                    │
│    (VLA Model + Cosmos Reason VLM)           │
└──────────────────────┬───────────────────────┘
                       ↓
┌──────────────────────────────────────────────┐
│        Isaac Lab / Isaac Lab-Arena           │
│   (Training, Evaluation, Sim-to-Real)        │
└──────────────────────┬───────────────────────┘
                       ↓
┌──────────────────────────────────────────────┐
│            Isaac Sim 5.1                      │
│   (Physics simulation, synthetic data)       │
└──────────────────────┬───────────────────────┘
                       ↓
┌──────────────────────────────────────────────┐
│      TensorRT / TensorRT Edge-LLM            │
│   (Optimized inference for deployment)       │
└──────────────────────┬───────────────────────┘
                       ↓
┌──────────────────────────────────────────────┐
│      Jetson Thor / T4000 / T5000             │
│   (Edge deployment hardware)                 │
└──────────────────────────────────────────────┘
```

### ROS2 Integration Approaches
- **openvla_ros2**: ROS 2 action server wrapper for VLA models
- **LeRobot**: PyTorch implementations with ROS-compatible interfaces
- **Isaac ROS**: NITROS-accelerated perception + VLA integration
- **Sensor messages**: Standard ROS 2 Image, JointState topics

### Hugging Face + NVIDIA Partnership
- Isaac GR00T integrated into LeRobot framework
- 2M robotics developers + 13M AI builders collaboration
- Reachy 2 humanoid fully interoperable with Jetson Thor
- Isaac Lab-Arena integrated into LeRobot EnvHub

---

## Deployment & Hardware

### NVIDIA Jetson Platforms for VLA

| Platform | AI Compute | Memory | Power | VLA Support |
|----------|-----------|--------|-------|-------------|
| Jetson Thor/T5000 | 2070 FP4 TFLOPS | 128GB | 40-130W | Full GR00T N1.6 |
| Jetson T4000 | 1200 FP4 TFLOPS | 64GB | 40-70W | Most VLAs |
| Jetson AGX Orin | 275 TOPS | 64GB | 15-60W | OpenVLA, SmolVLA |
| Jetson Orin Nano Super | 67 TOPS | 8GB | 7-25W | SmolVLA only |

### Inference Optimization
- **TensorRT Edge-LLM**: C++ SDK for LLM/VLM inference on Jetson
  - EAGLE-3 speculative decoding
  - NVFP4 quantization
  - Chunked prefill
- **FP4 Quantization**: 3.5x faster on Jetson Thor vs Orin
- **Real-time requirements**: 24Hz for VLA control loops

### Memory Requirements
| Model | Memory (Inference) |
|-------|-------------------|
| OpenVLA (7B) | 14-19 GB |
| SmolVLA (450M) | ~2 GB |
| GR00T N1.6 | ~8-16 GB |
| X-VLA (0.9B) | ~4 GB |

---

## Training Data & Datasets

### Open X-Embodiment Dataset
- 1M+ real robot trajectories
- 22 robot embodiments
- 527 skills, 160k+ tasks
- 60 datasets from 34 labs
- Foundation for RT-X, OpenVLA training
- Source: [Open X-Embodiment](https://robotics-transformer-x.github.io/)

### LeRobot Dataset Hub
- Community-contributed robot data
- RLDS format support
- 487+ datasets (SmolVLA training)
- Source: [LeRobot Hub](https://huggingface.co/lerobot)

### Synthetic Data Generation
- **Isaac Sim**: Physics-accurate synthetic data
- **GR00T-Dreams**: 36 hours vs 3 months manual collection
- **Domain Randomization**: Built into Isaac Lab
- Source: [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)

---

## Code Examples to Include

### Basic VLA Inference (OpenVLA style)
```python
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch

# Load model
processor = AutoProcessor.from_pretrained("openvla/openvla-7b")
model = AutoModelForVision2Seq.from_pretrained("openvla/openvla-7b")

# Prepare inputs
image = Image.open("camera_observation.jpg")
instruction = "Pick up the red block and place it on the blue plate"

inputs = processor(images=image, text=instruction, return_tensors="pt")

# Generate action
with torch.no_grad():
    action_tokens = model.generate(**inputs, max_new_tokens=256)

# Decode to robot actions (7-DoF: x, y, z, roll, pitch, yaw, gripper)
action = processor.decode(action_tokens)
```

### LeRobot π0 Usage
```python
from lerobot.common.policies.pi0.modeling_pi0 import Pi0Policy
from lerobot.common.policies.pi0.configuration_pi0 import Pi0Config

# Load pre-trained policy
config = Pi0Config()
policy = Pi0Policy(config)
policy.load_state_dict(torch.load("pi0_checkpoint.pt"))

# Run inference at 50Hz
while True:
    observation = get_camera_image()
    action = policy.select_action(observation)
    robot.execute(action)
```

### GR00T N1 with Isaac (conceptual)
```python
# Fine-tuning workflow
from gr00t.model import GR00TModel
from gr00t.data import RobotDataset

model = GR00TModel.from_pretrained("nvidia/gr00t-n1.6-base")
dataset = RobotDataset("path/to/your/demonstrations")

# Fine-tune on custom task
model.finetune(dataset, num_epochs=10)

# Export for Jetson deployment
model.export_tensorrt("gr00t_finetuned.trt")
```

---

## Diagrams Needed

1. **VLA Architecture Overview**
   - Input: Image + Language instruction
   - Processing: Vision encoder → LLM backbone
   - Output: Action tokens → Robot commands

2. **Dual-System Architecture (GR00T/Helix)**
   - System 2: VLM for understanding
   - System 1: Diffusion for real-time control

3. **NVIDIA Stack Diagram**
   - Isaac GR00T → Isaac Lab → Isaac Sim → TensorRT → Jetson

4. **VLA Evolution Timeline**
   - RT-2 (2023) → OpenVLA (2024) → GR00T/π0/Gemini (2025) → Current (2026)

5. **Action Generation Methods Comparison**
   - Autoregressive vs Flow Matching vs Diffusion

---

## Prerequisites & Related Concepts

### Required Background
- [Neural Networks](/robotics-glossary/concepts/ai/neural-networks/) — Foundation architectures
- [Reinforcement Learning](/robotics-glossary/concepts/ai/reinforcement-learning/) — Policy learning concepts
- [Transformers](/robotics-glossary/concepts/ai/neural-networks/#transformers) — Attention mechanism

### Related NVIDIA Tools
- [Isaac Lab](/robotics-glossary/software/nvidia/isaac-lab/) — Training environment
- [Isaac Sim](/robotics-glossary/software/simulation/isaac-sim/) — Simulation platform
- [TensorRT](/robotics-glossary/software/nvidia/tensorrt/) — Inference optimization
- [Jetson Thor](/robotics-glossary/hardware/compute/jetson-thor/) — Edge deployment

### Related ROS2 Concepts
- [MoveIt2](/robotics-glossary/software/ros2/moveit2/) — Classical manipulation alternative
- [Nav2](/robotics-glossary/software/ros2/nav2/) — Navigation stack

---

## Source URLs for Citations

### Primary Sources
- [GR00T N1 Paper](https://arxiv.org/abs/2503.14734) — NVIDIA Isaac GR00T N1 architecture
- [GR00T GitHub](https://github.com/NVIDIA/Isaac-GR00T) — Official repository
- [OpenVLA Paper](https://arxiv.org/abs/2406.09246) — OpenVLA architecture and training
- [OpenVLA GitHub](https://github.com/openvla/openvla) — Official repository
- [RT-2 Paper](https://arxiv.org/abs/2307.15818) — Original VLA concept
- [RT-2 Blog](https://deepmind.google/blog/rt-2-new-model-translates-vision-and-language-into-action/) — Google DeepMind announcement
- [π0 Paper](https://arxiv.org/html/2410.24164v1) — Physical Intelligence VLA
- [Gemini Robotics](https://deepmind.google/models/gemini-robotics/gemini-robotics/) — Google DeepMind robotics
- [SmolVLA Blog](https://huggingface.co/blog/smolvla) — Hugging Face compact VLA
- [LeRobot GitHub](https://github.com/huggingface/lerobot) — Hugging Face robotics framework
- [Helix Blog](https://www.figure.ai/news/helix) — Figure AI humanoid VLA
- [VLA Survey](https://vla-survey.github.io/) — Comprehensive VLA review paper

### NVIDIA Integration Sources
- [GR00T N1.6 CES Blog](https://nvidianews.nvidia.com/news/nvidia-releases-new-physical-ai-models-as-global-partners-unveil-next-generation-robots) — CES 2026 announcement
- [Isaac Lab-Arena LeRobot](https://huggingface.co/blog/nvidia/generalist-robotpolicy-eval-isaaclab-arena-lerobot) — Policy evaluation integration
- [TensorRT Edge-LLM](https://developer.nvidia.com/blog/accelerating-llm-and-vlm-inference-for-automotive-and-robotics-with-nvidia-tensorrt-edge-llm) — Edge inference SDK
- [Jetson Thor Blog](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/) — Edge hardware platform

### Wikipedia & Survey Sources
- [VLA Wikipedia](https://en.wikipedia.org/wiki/Vision-language-action_model) — General overview
- [Open X-Embodiment](https://robotics-transformer-x.github.io/) — Cross-embodiment dataset

---

## Summary

**Overall Assessment:** VLA models represent the current state-of-the-art for learning-based robot control, unifying perception, language understanding, and action generation in single end-to-end models.

**Key Takeaways for Glossary Entry:**
1. VLAs are the "brains" enabling general-purpose robots
2. Evolution: RT-2 (2023) → OpenVLA/GR00T/π0 (2024-2025) → Production-ready (2026)
3. Two main architectures: Single-model vs Dual-system
4. NVIDIA ecosystem is central: GR00T, Isaac Lab, TensorRT, Jetson
5. Hugging Face LeRobot democratizing access
6. Edge deployment maturing on Jetson Thor/T4000

**Badge Recommendation:** Deep Dive (comprehensive topic requiring understanding of neural networks, RL, and robotics)

**Word Count Target:** ~1500-2000 words (similar to reinforcement-learning.mdx)
