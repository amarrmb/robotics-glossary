# nvblox Research Notes

**Topic**: nvblox - NVIDIA Real-Time 3D Reconstruction
**Target file**: `src/content/docs/software/nvidia/nvblox.mdx`
**Research date**: 2026-01-21

---

## Key Concepts to Cover

### 1. What is nvblox?

- NVIDIA's GPU-accelerated library for real-time 3D scene reconstruction
- Builds volumetric representations from depth cameras and/or 3D LiDAR
- Outputs meshes, occupancy grids, and cost maps for robot navigation
- Part of the Isaac ROS ecosystem, tightly integrated with Nav2
- Open-source (Apache 2.0 license)
- Available via Python, C++, or ROS 2 interfaces
- Academic paper: "nvblox: GPU-Accelerated Incremental Signed Distance Field Mapping" (ICRA 2024)

### 2. Core Technical Concepts

#### TSDF (Truncated Signed Distance Function)
- Stores the signed distance to the closest surface at each voxel
- Primary representation for surface reconstruction
- Typical resolution: 5cm voxels (configurable via `voxel_size` parameter)
- Higher quality surface reconstructions than occupancy grids
- Incrementally updated as new sensor data arrives

#### ESDF (Euclidean Signed Distance Function)
- Full (non-truncated) distance field for path planning
- Provides immediate collision checking for robot positions
- GPU-accelerated incremental computation
- Default max distance: 10.0 meters (`esdf_integrator_max_distance_m`)
- Can be computed in 2D or 3D (`esdf_2d` parameter)

#### LayerCake Architecture
- Multiple co-registered volumetric layers in GPU memory:
  - TSDF layer (distance to surfaces)
  - ESDF layer (Euclidean distances for planning)
  - Color layer (RGB texture from cameras)
  - Occupancy layer (for dynamic objects)
  - Mesh layer (extracted surface)
  - Feature layer (deep learning features)
- All layers share the same voxel grid and alignment

### 3. Modes of Operation

#### Static Mode (Default)
- Assumes environment is unchanging
- Best for mapping fixed environments
- All sensor data integrated into single TSDF

#### People Reconstruction Mode
- Uses PeopleSemSegNet DNN for human segmentation
- Separates depth frames into people/non-people regions
- People integrated into separate occupancy layer
- Non-people regions integrated into TSDF
- Models: PeopleSemSegNet_Vanilla (desktop), PeopleSemSegNet_ShuffleSeg (Jetson)

#### Dynamic Reconstruction Mode
- General dynamic object detection without DNN
- Maintains high-confidence freespace layer
- Objects entering freespace flagged as dynamic
- Dynamic objects tracked in separate occupancy layer
- Works for any moving object class

### 4. Multi-Sensor Support

- Up to 4 simultaneous RealSense cameras (Jetson AGX Orin)
- 3D LiDAR integration
- Nvblox can integrate data from 3D lidar and up to 3 cameras simultaneously
- Configurable via `lidar` and `num_cameras` launch arguments

### 5. Nav2 Integration

#### Costmap Plugin
- `nvblox::NvbloxCostmapLayer` provides direct Nav2 integration
- Slices 3D reconstruction into 2D costmap for navigation
- Configurable height range via `min_height` and `max_height` parameters
- `slice_height` controls output slice height (default: 1.0m)

#### Pipeline
```
Depth Camera → nvblox → ESDF → 2D Costmap Slice → Nav2 → Path Planning
```

### 6. Architecture

#### System Components
```
Input Sensors            Processing              Output
─────────────           ──────────              ──────
RGB-D Camera ────┐      ┌──────────┐       ┌─────────────┐
                 ├─────►│  nvblox  │──────►│    Mesh     │
3D LiDAR ────────┤      │  Mapper  │──────►│    ESDF     │
                 │      └──────────┘──────►│  Costmap    │
Pose (SLAM/Odom)─┘            │            └─────────────┘
                              │
                       ┌──────▼──────┐
                       │  LayerCake  │
                       │  (GPU Mem)  │
                       └─────────────┘
```

#### ROS 2 Package Structure
| Package | Purpose |
|---------|---------|
| `isaac_ros_nvblox` | Core ROS 2 integration package |
| `nvblox_nav2` | Nav2 costmap layer plugin |
| `nvblox_rviz_plugin` | RViz mesh visualization |
| `nvblox_msgs` | Custom message definitions |
| `nvblox_examples_bringup` | Launch files for tutorials |

### 7. Performance

#### GPU Acceleration Benefits
- Up to 177x speedup in surface reconstruction vs CPU (voxblox)
- Up to 31x improvement in distance field computation
- Runs at <7ms per LiDAR scan on laptop (5cm resolution, 25m range)
- Real-time mesh updates streamed to RViz

#### Benchmarks from Paper
| Operation | nvblox (GPU) | voxblox (CPU) | Speedup |
|-----------|--------------|---------------|---------|
| TSDF Integration | Fast | Baseline | 177x |
| ESDF Computation | Fast | Baseline | 31x |

---

## Current Version Information

### Isaac ROS Releases
| Release | Tag | Notes |
|---------|-----|-------|
| Isaac ROS 3.2 | v3.2-15 (latest) | Fix for deserialization with dynamic object detection |
| Isaac ROS 3.1 | v0.31.0-dp | Developer Preview |
| Isaac ROS 3.0 | v3.0 | Infrastructure updates |

### ROS 2 Compatibility
- **ROS 2 Jazzy** (Ubuntu 24.04) - Recommended
- **ROS 2 Humble** (Ubuntu 22.04) - Supported for older Isaac ROS versions

### Platform Support
- **Jetson AGX Thor** with JetPack 7.x
- **Jetson AGX Orin, Orin NX, Orin Nano** with JetPack 6.x
- **x86_64** with NVIDIA discrete GPU

### nvblox Library Version
- nvblox_torch 0.0.8 (standalone library)
- Core library versioned with Isaac ROS releases

### Recent Updates (CES 2025 / Isaac ROS 3.2)
- Fine-grained ~1cm voxel resolution
- Bounded workspace support
- Multi-camera for workcell applications
- Dynamic mode support for Nova Carter/Nova Developer Kit
- Improved people detection for RealSense

---

## NVIDIA + ROS 2 Ecosystem Integration

### Isaac ROS Position
nvblox is a foundational component of Isaac ROS perception stack:
- **Visual SLAM (cuVSLAM)**: Provides pose for nvblox integration
- **cuMotion**: Uses nvblox ESDF for obstacle avoidance in manipulation
- **Isaac Perceptor**: Vision-based navigation using nvblox + cuVSLAM

### Nav2 Costmap Layer
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["nvblox_layer", "inflation_layer"]
      nvblox_layer:
        plugin: "nvblox::NvbloxCostmapLayer"
```

### cuMotion Integration
nvblox provides real-time ESDF for motion planning collision avoidance:
```
Depth Camera → nvblox (ESDF) → cuMotion Planner → Collision-Free Trajectory
```

### Isaac Sim Integration
- nvblox sample scene available in Isaac Sim 5.1
- Synthetic data generation for testing
- Hardware-in-the-loop simulation

---

## Code Examples to Include

### 1. Launch with RealSense Camera
```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py
```

### 2. Launch with Isaac Sim
```bash
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
```

### 3. Launch with People Segmentation
```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py \
  mode:=people_segmentation
```

### 4. Launch with Dynamic Reconstruction
```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py \
  mode:=people_detection
```

### 5. Multi-Camera Launch
```bash
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py \
  lidar:=True \
  num_cameras:=3
```

### 6. C++ Library Interface
```cpp
#include <nvblox/nvblox.h>

// Create mapper with 5cm voxels on GPU
const float voxel_size_m = 0.05;
const MemoryType memory_type = MemoryType::kDevice;
nvblox::Mapper mapper(voxel_size_m, memory_type);

// Integrate depth and color
mapper.integrateDepth(depth_image, T_L_C, camera);
mapper.integrateColor(color_image, T_L_C, camera);

// Generate ESDF and mesh
mapper.updateEsdf();
mapper.updateMesh();

// Save mesh to disk
nvblox::io::outputMeshLayerToPly(mapper.mesh_layer(), "output.ply");
```

### 7. Nav2 Costmap Configuration
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      plugins: ["nvblox_layer", "inflation_layer"]
      nvblox_layer:
        plugin: "nvblox::NvbloxCostmapLayer"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.55
```

### 8. Key ROS Parameters
```yaml
nvblox_node:
  ros__parameters:
    # Voxel resolution (default: 5cm)
    voxel_size: 0.05

    # ESDF settings
    esdf: true
    esdf_2d: true
    esdf_integrator_max_distance_m: 10.0

    # Height slice for costmap
    slice_height: 1.0
    min_height: 0.0
    max_height: 1.0

    # Integration rate
    max_tsdf_update_hz: 10.0
```

---

## Diagrams Needed

### 1. nvblox System Architecture
```
┌─────────────────────────────────────────────────────────────────────┐
│                        nvblox Pipeline                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Input                     Processing                    Output      │
│  ──────                    ──────────                    ──────      │
│                                                                      │
│  ┌──────────┐                                      ┌──────────────┐ │
│  │ RGB-D    │───┐     ┌──────────────────┐        │    Mesh      │ │
│  │ Camera   │   │     │     Mapper       │        │  (RViz)      │ │
│  └──────────┘   │     │  ┌────────────┐  │        └──────────────┘ │
│                 ├────►│  │   TSDF     │  │────────►                 │
│  ┌──────────┐   │     │  │ Integrator │  │        ┌──────────────┐ │
│  │  3D      │───┤     │  └────────────┘  │        │    ESDF      │ │
│  │ LiDAR    │   │     │  ┌────────────┐  │────────►│  (cuMotion)  │ │
│  └──────────┘   │     │  │   ESDF     │  │        └──────────────┘ │
│                 │     │  │ Integrator │  │                          │
│  ┌──────────┐   │     │  └────────────┘  │        ┌──────────────┐ │
│  │  Pose    │───┘     │  ┌────────────┐  │────────►│  2D Costmap  │ │
│  │ (SLAM)   │         │  │   Mesh     │  │        │   (Nav2)     │ │
│  └──────────┘         │  │ Generator  │  │        └──────────────┘ │
│                       │  └────────────┘  │                          │
│                       └──────────────────┘                          │
│                              │                                      │
│                       ┌──────▼──────┐                               │
│                       │  LayerCake  │                               │
│                       │  (GPU Mem)  │                               │
│                       │ TSDF|ESDF|  │                               │
│                       │ Color|Mesh  │                               │
│                       └─────────────┘                               │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2. TSDF/ESDF Concept
```
                  Surface
                     │
         ◄──────────│──────────►
         Negative   │   Positive
         (inside)   │   (outside)
                    │
    ─────┬─────┬────┼────┬─────┬─────  TSDF values
        -d   -d/2   0   d/2    d       (truncated at ±d)
                    │
    ─────┬─────┬────┼────┬─────┬─────  ESDF values
         │     │    │    │     │       (full distance)
        3d    2d    0    d    2d       (meters to surface)
```

### 3. Dynamic Reconstruction Pipeline
```
┌────────────────────────────────────────────────────────────────┐
│                Dynamic Reconstruction Mode                       │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌───────────┐    ┌───────────────┐    ┌──────────────────┐    │
│  │   Color   │───►│ PeopleSemSeg  │───►│ Segmentation     │    │
│  │   Image   │    │     DNN       │    │    Mask          │    │
│  └───────────┘    └───────────────┘    └────────┬─────────┘    │
│                                                  │              │
│  ┌───────────┐                        ┌─────────▼─────────┐    │
│  │   Depth   │───────────────────────►│   Depth Masker    │    │
│  │   Image   │                        └────────┬──────────┘    │
│  └───────────┘                                 │               │
│                               ┌────────────────┼────────────┐  │
│                               │                │            │  │
│                               ▼                ▼            │  │
│                    ┌──────────────┐   ┌──────────────┐      │  │
│                    │ Non-People   │   │   People     │      │  │
│                    │    Depth     │   │    Depth     │      │  │
│                    └──────┬───────┘   └──────┬───────┘      │  │
│                           │                  │              │  │
│                           ▼                  ▼              │  │
│                    ┌──────────────┐   ┌──────────────┐      │  │
│                    │    TSDF      │   │  Occupancy   │      │  │
│                    │   Layer      │   │   Layer      │      │  │
│                    │  (static)    │   │  (dynamic)   │      │  │
│                    └──────────────┘   └──────────────┘      │  │
│                                                              │  │
└──────────────────────────────────────────────────────────────┘  │
```

### 4. Nav2 Integration
```
                     ┌─────────────────────────────────────────┐
                     │              Nav2 Stack                  │
                     ├─────────────────────────────────────────┤
                     │                                          │
                     │  ┌─────────────┐    ┌─────────────┐     │
                     │  │   Planner   │◄───│   Costmap   │     │
                     │  │   Server    │    │  (2D slice) │     │
                     │  └─────────────┘    └──────▲──────┘     │
                     │                           │             │
                     │                   ┌───────┴───────┐     │
                     │                   │NvbloxCostmap  │     │
                     │                   │    Layer      │     │
                     │                   └───────▲───────┘     │
                     └───────────────────────────│─────────────┘
                                                 │
┌────────────────────────────────────────────────│─────────────┐
│                    nvblox                      │              │
│  ┌─────────────┐    ┌─────────────┐    ┌──────┴──────┐      │
│  │   Depth     │───►│   TSDF      │───►│    ESDF     │      │
│  │   Camera    │    │   Mapper    │    │   Slice     │      │
│  └─────────────┘    └─────────────┘    └─────────────┘      │
└──────────────────────────────────────────────────────────────┘
```

---

## Prerequisites / Related Concepts

1. **SLAM** - nvblox requires pose input (from cuVSLAM or other SLAM)
2. **Nav2** - Primary consumer of nvblox costmaps
3. **TF2** - Transform tree for sensor/robot frames
4. **Depth Cameras** - RealSense, stereo cameras
5. **LiDAR** - 3D point cloud sensors
6. **cuMotion** - Uses nvblox ESDF for manipulation
7. **Isaac ROS** - Overall GPU-accelerated ecosystem
8. **Coordinate Frames** - Understanding map/odom/base_link

---

## Source URLs for Citations

### Official Documentation
- [nvblox Main Documentation](https://nvidia-isaac.github.io/nvblox/)
- [Isaac ROS Nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)
- [Isaac ROS Nvblox Technical Details](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html)
- [ROS Parameters Reference](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html)
- [Nvblox Concepts](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html)

### GitHub Repositories
- [nvblox Core Library](https://github.com/nvidia-isaac/nvblox)
- [isaac_ros_nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)
- [Releases](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/releases)

### Academic Paper
- [nvblox: GPU-Accelerated Incremental Signed Distance Field Mapping (arXiv)](https://arxiv.org/abs/2311.00626)
- Authors: Alexander Millane, Helen Oleynikova, Emilie Wirbel, Remo Steiner, Vikram Ramasamy, David Tingdahl, Roland Siegwart
- Published: ICRA 2024, pages 2698-2705

### Tutorials
- [Isaac Sim Examples](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html)
- [RealSense Camera Examples](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_realsense.html)
- [Multi-RealSense Camera Examples](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_multi_realsense.html)

### Related Isaac ROS
- [Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)
- [CES 2025 Isaac ROS 3.2 Announcement](https://forums.developer.nvidia.com/t/ces-2025-isaac-ros-3-2-and-platform-updates/319021)

### Nav2 Integration
- [Nav2 Using Isaac Perceptor](https://docs.nav2.org/tutorials/docs/using_isaac_perceptor.html)
- [Nav2 Documentation](https://docs.nav2.org/)

### Related Projects
- [cuMotion with nvblox](https://curobo.org/get_started/2d_nvblox_demo.html)
- [Tinker-Twins nvblox fork](https://github.com/Tinker-Twins/NVIDIA-Isaac-ROS-Nvblox)

---

## Style Notes (from existing entries)

Based on review of cumotion.mdx, isaac-ros.mdx, nav2.mdx, and slam.mdx:

1. **Frontmatter**: title, description, last_validated, sidebar badge
2. **Imports**: Astro Starlight components (Aside, Card, CardGrid, LinkCard, Tabs, TabItem)
3. **Badge span**: `<span class="level-badge practical">Practical</span>` (Practical for tools)
4. **Opening**: Bold term definition, one-liner summary
5. **Aside notes**: For tips, cautions, version info
6. **Prerequisites section**: CardGrid with LinkCards
7. **"Why X Matters"** section: Bullet points highlighting key benefits
8. **Architecture section**: ASCII diagrams, package tables
9. **Code Examples**: Tabs for different scenarios, bash commands and YAML
10. **Related Terms**: CardGrid with LinkCards
11. **Learn More**: External tutorial links
12. **Sources**: Citation links at bottom with brief descriptions

---

## Notes for Content Creation

- **Badge type**: Practical (hands-on tool like cuMotion, Isaac ROS)
- **Emphasis**: GPU acceleration for real-time 3D reconstruction, Nav2 integration
- **Target audience**: ROS 2 developers building navigation systems
- **Related entries to link**:
  - `/robotics-glossary/software/ros2/nav2/`
  - `/robotics-glossary/software/isaac/isaac-ros/`
  - `/robotics-glossary/concepts/perception/slam/`
  - `/robotics-glossary/software/nvidia/cumotion/` (nvblox provides ESDF for cuMotion)
- **Version focus**: Isaac ROS 3.2 with ROS 2 Jazzy
- **Key differentiators**:
  - GPU-accelerated TSDF/ESDF computation (177x/31x speedup)
  - Real-time mesh visualization
  - Multiple operating modes (static, people, dynamic)
  - Multi-sensor support (cameras + LiDAR)
  - Direct Nav2 costmap integration

---

## Verification Checklist

- [x] nvblox is current term (actively maintained in Isaac ROS 3.2)
- [x] Version numbers verified (Isaac ROS 3.2, nvblox_torch 0.0.8)
- [x] Platform support confirmed (Jetson Orin/Thor, x86_64 with GPU)
- [x] ROS 2 compatibility confirmed (Jazzy recommended)
- [x] Performance claims sourced from paper (177x, 31x speedups)
- [x] TSDF/ESDF concepts explained
- [x] Nav2 integration documented
- [x] cuMotion integration mentioned
- [x] Multiple code examples identified
- [x] Academic citation included (ICRA 2024)
