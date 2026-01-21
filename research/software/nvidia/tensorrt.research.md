# TensorRT Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/software/nvidia/tensorrt.mdx
**Description:** NVIDIA deep learning inference optimizer

---

## Overview

NVIDIA TensorRT is a high-performance deep learning inference SDK that optimizes trained neural networks for deployment on NVIDIA GPUs. It includes an inference compiler, runtime, and model optimizations that deliver low latency and high throughput for production applications.

## Current Version Information

- **Latest Version:** TensorRT 10.14.1 (as of January 2026)
- **TensorRT for RTX:** Version 1.3
- **TensorRT-LLM:** Separate product for LLM inference optimization
- **TensorRT Edge-LLM:** Open-source C++ SDK for edge platforms (Jetson Thor, DRIVE AGX Thor)

### Version 10.14.1 Features
- Support for NVIDIA GB300, DGX B300, and DGX Spark
- New IAttention API for fused attention kernels
- SerializationFlag::kINCLUDE_REFIT for maintaining refittable engines
- New APIs for Topk, NMS, NonZero output data type control (INT32/INT64)
- Engine statistics API (getEngineStat())

## Key Concepts to Cover

### 1. Core Optimization Techniques

#### Layer Fusion
- Vertical and horizontal kernel fusion
- Combines sequential operations (Conv + Bias + ReLU → single "CBR" kernel)
- Reduces memory bandwidth and kernel launch overhead
- Example: "ip1 + relu1" naming convention for fused layers

#### Kernel Auto-Tuning
- Benchmarks multiple kernel implementations during build
- Selects fastest kernel for specific tensor dimensions and target GPU
- Platform-specific optimization (engine must be built on deployment GPU)

#### Precision Calibration
- Supports: FP32, FP16, BF16, FP8, FP4, INT8, INT4, UINT8, BOOL
- FP8 deployment only on Ada/Hopper GPUs and newer
- INT8 with calibration for accuracy preservation
- Weight-only quantization (WoQ) for INT4

#### Dynamic Tensor Memory Management
- Memory reuse - allocates only for tensor's usage duration
- Reduces memory footprint and allocation overhead

### 2. Quantization Details

#### Explicit vs Implicit Quantization
- **Explicit (Q/DQ):** Predictable precision transitions via Q/DQ layers
- **Implicit:** TensorRT automatically selects INT8 where beneficial

#### Supported Schemes
- Symmetric quantization for INT8, FP8, INT4, FP4
- Weight-only quantization for INT4
- Calibration support for INT8, FP8, FP4 (weights and activations)

### 3. Workflow: PyTorch/ONNX → TensorRT

```
PyTorch Model → torch.onnx.export() → model.onnx → TensorRT Parser → TRT Engine
```

Key steps:
1. Train model in PyTorch
2. Export to ONNX with torch.onnx.export()
3. Optionally simplify with onnx-simplifier
4. Build TensorRT engine via trtexec or API
5. Deploy serialized engine

### 4. Torch-TensorRT Integration

```python
import torch_tensorrt
optimized_model = torch.compile(model, backend="tensorrt")
```

Features:
- JIT compilation via torch.compile backend
- Hybrid execution (TensorRT + standard PyTorch)
- Export workflow for C++ deployment
- Supports dynamic shapes

### 5. Command-Line Tools

#### trtexec
```bash
# Basic ONNX conversion
trtexec --onnx=model.onnx --saveEngine=model.trt

# With FP16 and dynamic shapes
trtexec --onnx=model.onnx \
    --fp16 \
    --minShapes=input:1x3x224x224 \
    --optShapes=input:8x3x224x224 \
    --maxShapes=input:16x3x224x224 \
    --saveEngine=model_fp16.trt

# Performance profiling
trtexec --loadEngine=model.trt --dumpProfile

# INT8 with calibration
trtexec --onnx=model.onnx --int8 --calib=calibration.cache
```

## ROS 2 / Robotics Integration

### Isaac ROS DNN Inference
- `isaac_ros_dnn_inference` package
- Provides TensorRT and Triton nodes
- NITROS-compatible for zero-copy communication
- Supports both Jetson and x86_64 with CUDA-capable GPU

### Key ROS 2 Packages
| Package | Purpose |
|---------|---------|
| `isaac_ros_tensor_rt` | TensorRT inference node |
| `isaac_ros_triton` | Triton inference server integration |
| `ros_deep_learning` | Community package for Jetson inference |

### TensorRT Edge-LLM (2026)
- Open-source C++ SDK for Jetson Thor platform
- Runs LLMs and VLMs on edge devices
- Part of JetPack 7.1
- Partners: Bosch, ThunderSoft, MediaTek

### Community ROS 2 Packages
- `ros2-depth-anything-v3-trt` - Monocular depth estimation
- `ros2-moge-trt` - MoGeV2 depth estimation
- `jetson-inference` - Hello AI World tutorials

## Jetson Deployment

### Supported Platforms
- Jetson Orin (AGX Orin, Orin NX, Orin Nano)
- Jetson Thor (JetPack 7.1)
- Jetson T4000 (2x performance over AGX Orin)

### Performance Examples
- YOLOv8 on Orin Nano: 47.56 FPS
- VILA-2.7B VLM on Orin Nano Super for visual queries
- 30x faster than Jetson Nano, 15% faster than AGX Xavier

## Prerequisites

### Hardware
- NVIDIA GPU with Compute Capability 7.5+ (Turing and newer)
- For FP8: Ada Lovelace or Hopper GPUs
- Minimum VRAM depends on model size

### Software
- CUDA Toolkit 12.x (12.9 for latest TensorRT)
- cuDNN 8.9.7 (optional, deprecated layers only)
- Python 3.8+ for Python API
- PyTorch 2.0+ for Torch-TensorRT

## Performance Characteristics

- Up to 40x faster inference vs CPU
- Up to 18x faster than TensorFlow on Volta GPUs
- FP16: ~2x memory reduction, significant speedup with Tensor Cores
- INT8: ~4x memory reduction on Turing+ GPUs
- Batch sizes of 32 multiples optimal for FP16/INT8 with Tensor Cores

## Code Examples to Include

### 1. Basic trtexec Conversion
```bash
trtexec --onnx=yolov8n.onnx --saveEngine=yolov8n.trt --fp16
```

### 2. Python API Engine Build
```python
import tensorrt as trt

logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
parser = trt.OnnxParser(network, logger)

with open("model.onnx", "rb") as f:
    parser.parse(f.read())

config = builder.create_builder_config()
config.set_flag(trt.BuilderFlag.FP16)

engine = builder.build_serialized_network(network, config)
with open("model.trt", "wb") as f:
    f.write(engine)
```

### 3. Torch-TensorRT Compilation
```python
import torch
import torch_tensorrt

model = YourModel().eval().cuda()
optimized = torch.compile(model, backend="tensorrt")
output = optimized(input_tensor)  # First call compiles, subsequent calls fast
```

### 4. ROS 2 Isaac TensorRT Node (conceptual)
```yaml
# Example launch configuration
tensor_rt_node:
  ros__parameters:
    model_file_path: "/models/detection.onnx"
    engine_file_path: "/models/detection.trt"
    input_tensor_names: ["input"]
    output_tensor_names: ["output"]
    input_binding_names: ["input"]
    output_binding_names: ["output"]
```

## Diagrams Needed

### 1. TensorRT Optimization Pipeline
```
┌─────────────────────────────────────────────────────────────────────┐
│                    TensorRT Optimization Pipeline                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────────┐  │
│  │ Trained  │───►│   ONNX   │───►│ TensorRT │───►│  Optimized   │  │
│  │  Model   │    │  Export  │    │  Builder │    │   Engine     │  │
│  │(PyTorch) │    │          │    │          │    │   (.trt)     │  │
│  └──────────┘    └──────────┘    └────┬─────┘    └──────────────┘  │
│                                       │                             │
│                          ┌────────────┼────────────┐                │
│                          │            │            │                │
│                     ┌────▼────┐ ┌─────▼────┐ ┌────▼─────┐          │
│                     │  Layer  │ │  Kernel  │ │Precision │          │
│                     │ Fusion  │ │Auto-Tune │ │ Calib.   │          │
│                     └─────────┘ └──────────┘ └──────────┘          │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2. ROS 2 Integration Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                   ROS 2 + TensorRT Pipeline                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────────────┐    │
│  │  Camera  │───►│   Image      │───►│  TensorRT Node      │    │
│  │  Driver  │    │   Resize     │    │  (isaac_ros_tensor) │    │
│  └──────────┘    └──────────────┘    └──────────┬──────────┘    │
│                                                  │               │
│                                           ┌──────▼──────┐        │
│                                           │ Detection/  │        │
│                                           │ Segmentation│        │
│                                           │   Results   │        │
│                                           └─────────────┘        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3. Precision Comparison
```
┌─────────────────────────────────────────────────────────────┐
│                    Precision Trade-offs                      │
├─────────────┬─────────────┬─────────────┬──────────────────┤
│  Precision  │   Memory    │   Speed     │    Accuracy      │
├─────────────┼─────────────┼─────────────┼──────────────────┤
│    FP32     │  Baseline   │  Baseline   │   Highest        │
│    FP16     │    ~50%     │   ~2x       │   Near FP32      │
│    INT8     │    ~25%     │   ~4x       │   Needs calib.   │
│    FP8      │    ~25%     │   ~4x       │   Better than    │
│             │             │             │   INT8 usually   │
└─────────────┴─────────────┴─────────────┴──────────────────┘
```

## Related Glossary Entries

- **Neural Networks** - prerequisite concept
- **Isaac ROS** - parent ecosystem
- **Jetson Orin** - primary deployment hardware
- **cuMotion** - uses TensorRT for ML inference in motion planning
- **nvblox** - uses DNNs accelerated by TensorRT for segmentation

## Source URLs for Citations

### Official NVIDIA Documentation
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/latest/index.html)
- [TensorRT Quick Start Guide](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/quick-start-guide.html)
- [TensorRT Best Practices](https://docs.nvidia.com/deeplearning/tensorrt/latest/performance/best-practices.html)
- [TensorRT Support Matrix](https://docs.nvidia.com/deeplearning/tensorrt/support-matrix/index.html)
- [TensorRT Prerequisites](https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/prerequisites.html)
- [TensorRT Release Notes](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/release-notes.html)

### NVIDIA Developer
- [TensorRT Developer Portal](https://developer.nvidia.com/tensorrt)
- [TensorRT-LLM](https://developer.nvidia.com/tensorrt-llm)
- [TensorRT Edge-LLM Blog](https://developer.nvidia.com/blog/accelerating-llm-and-vlm-inference-for-automotive-and-robotics-with-nvidia-tensorrt-edge-llm)
- [Jetson T4000 and JetPack 7.1 Blog](https://developer.nvidia.com/blog/accelerate-ai-inference-for-edge-and-robotics-with-nvidia-jetson-t4000-and-nvidia-jetpack-7-1)

### GitHub Repositories
- [NVIDIA TensorRT](https://github.com/NVIDIA/TensorRT)
- [Torch-TensorRT](https://github.com/pytorch/TensorRT)
- [NVIDIA Model Optimizer](https://github.com/NVIDIA/Model-Optimizer)
- [Isaac ROS DNN Inference](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference)
- [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning)
- [jetson-inference](https://github.com/dusty-nv/jetson-inference)

### Isaac ROS Documentation
- [Isaac ROS TensorRT and Triton](https://nvidia-isaac-ros.github.io/concepts/dnn_inference/tensorrt_and_triton_info.html)
- [ROS 2 NVIDIA Projects](https://docs.ros.org/en/humble/Related-Projects/Nvidia-ROS2-Projects.html)

### Torch-TensorRT
- [Torch-TensorRT Documentation](https://docs.pytorch.org/TensorRT/)
- [torch.compile Backend](https://docs.pytorch.org/TensorRT/user_guide/torch_compile.html)

### Technical Blogs and Tutorials
- [PyTorch to TensorRT Conversion (LearnOpenCV)](https://learnopencv.com/how-to-convert-a-model-from-pytorch-to-tensorrt-and-speed-up-inference/)
- [TensorRT Model Optimization (Medium)](https://medium.com/@zergtant/accelerating-model-inference-with-tensorrt-tips-and-best-practices-for-pytorch-users-7cd4c30c97bc)
- [ONNX Runtime TensorRT EP](https://onnxruntime.ai/docs/execution-providers/TensorRT-ExecutionProvider.html)

## Notes for Content Creation

### Badge/Level
- Suggest: **Practical** (like nvblox and cuMotion) - it's a hands-on tool for robotics deployment

### Tone
- Focus on robotics/ROS 2 integration use cases
- Emphasize Jetson deployment scenarios
- Include practical trtexec commands

### Structure (based on existing entries)
1. Frontmatter with title, description, last_validated, sidebar badge
2. Imports for Astro components
3. Level badge span
4. Bold intro paragraph
5. Aside note with key integration point
6. Prerequisites CardGrid
7. "Why X Matters" bullet list
8. Architecture diagram
9. Core Concepts sections
10. Code examples with Tabs
11. Key Parameters section
12. Performance comparison table
13. Related Terms CardGrid
14. Learn More links
15. Sources section

### Key Differentiators to Highlight
- Build-time optimization (not runtime)
- Platform-specific engine generation
- Integration with Isaac ROS ecosystem
- Edge deployment on Jetson platforms
