# cuMotion Research Notes

**Topic**: cuMotion - NVIDIA GPU-accelerated Motion Planning
**Target file**: `src/content/docs/software/nvidia/cumotion.mdx`
**Research date**: 2026-01-21

---

## Key Concepts to Cover

### 1. What is cuMotion?

- NVIDIA's GPU-accelerated motion planning package for robotic manipulators
- Part of the Isaac ROS ecosystem
- Production-ready motion generation for robot arms
- Available as a MoveIt 2 plugin for easy integration
- Uses cuRobo as its backend (CUDA Accelerated Robot Library)
- Developer Preview introduced in Isaac 3.0, now mature in Isaac ROS 3.2/4.0

### 2. Core Capabilities

#### Motion Generation
- Produces collision-free, optimal-time, minimal-jerk trajectories
- Leverages massive GPU parallelism to evaluate thousands of trajectories simultaneously
- Plans in tens of milliseconds on discrete GPU (RTX 6000 Ada) or fraction of a second on Jetson
- Smooth trajectories that minimize jerk and accelerations

#### Robot Segmentation
- Filters robot geometry from depth image streams
- Uses cuMotion's kinematics and geometry processing
- Enables reconstruction of obstacles without spurious robot contributions
- Runs at ~3000 Hz on RTX 4090 for 480x480 images

#### Obstacle Avoidance
- Supports multiple obstacle representations:
  - Cuboids (primitive shapes)
  - Meshes (complex geometry)
  - Signed Distance Fields (SDF) from depth cameras via nvblox
  - Any combination of the three
- Real-time environment reconstruction from depth streams

### 3. Architecture

#### Package Structure
| Package | Purpose |
|---------|---------|
| `isaac_ros_cumotion` | Core planner node + robot segmentation node |
| `isaac_ros_cumotion_examples` | Demo launch files and examples |
| `isaac_ros_cumotion_moveit` | MoveIt 2 plugin exposing cuMotion as external planner |

#### Backend: cuRobo
cuRobo provides the algorithmic foundation:
- Forward and inverse kinematics (CUDA-accelerated)
- Collision checking (robot vs world)
- Numerical optimization (gradient descent, L-BFGS, MPPI)
- Geometric planning
- Trajectory optimization
- Implemented in PyTorch for custom cost terms

#### Motion Planning Pipeline
```
1. Collision-free IK solution generation
2. Seed generation for trajectory optimization
3. Parallel trajectory optimization across many seeds
4. Timestep re-optimization for smooth, time-optimal result
```

### 4. Integration with nvblox

nvblox provides real-time 3D reconstruction:
- GPU-accelerated Signed Distance Field (SDF) library
- Produces Euclidean Signed Distance Fields (ESDF) from depth streams
- Up to 177x speedup in surface reconstruction vs CPU methods
- Up to 31x improvement in distance field computation
- Enables efficient obstacle-aware planning in cuMotion

### 5. Robot Support

#### Officially Tested Robots
- **Universal Robots**: UR5e, UR10e (validated in tutorials)
- **Franka Emika**: Panda, FR3
- **Kinova**: Gen 3, Jaco 2 models (available in Isaac Sim)

#### Custom Robot Support
Requires:
- URDF file (standard robot description)
- XRDF file (Extended Robot Description Format - cuMotion-specific)

### 6. XRDF (Extended Robot Description Format)

XRDF supplements URDF with cuMotion-specific data:
- **Semantic labeling**: C-space configuration, tool frames
- **Limits**: Acceleration and jerk limits for smooth motion
- **Collision spheres**: Efficient geometry representation
- **Self-collision masking**: Pairs to skip during checking

Key components:
```yaml
format: xrdf
format_version: 1.0

modifiers:
  # Frame additions, joint modifications

cspace:
  joint_names: [joint_1, joint_2, ...]
  acceleration_limits: [...]
  jerk_limits: [...]

tool_frames:
  - end_effector_frame

geometry:
  collision_spheres:
    # Sphere definitions for robot links
```

Tools for creating XRDF:
- Visual Robot Description Editor in Isaac Sim 4.0+
- Manual YAML editing
- Legacy cuRobo format still accepted but deprecated

---

## Current Version Information

### Isaac ROS cuMotion Versions
| Release | Tag | Notes |
|---------|-----|-------|
| Isaac ROS 3.2 | v3.2, v3.2-1, v3.2-2, v3.2-3 | ROS 2 Humble, Jazzy compatible |
| Isaac ROS 4.0 | v4.0, v4.0-1 | ROS 2 Jazzy, Thor support |

### ROS 2 Compatibility
- **Jazzy** (Ubuntu 24.04) - Recommended
- **Humble** (Ubuntu 22.04) - Supported

### Isaac ROS 3.2 Updates
- v3.2-3: Fixed version incompatibility issue preventing cuMotion from being selected in MoveIt 2.5.8+
- Improved pick-and-place reference workflow robustness

### GPU Requirements
- **NVIDIA GPU with Volta architecture or newer** (Compute Capability 7.0+)
- Minimum 4 GB VRAM
- Supported platforms:
  - Jetson AGX Thor (JetPack 7.x)
  - Jetson AGX Orin, Orin NX, Orin Nano (JetPack 6.x)
  - x86_64 with discrete NVIDIA GPU

### Performance Benchmarks
| Metric | Performance |
|--------|-------------|
| Planning time (RTX 6000 Ada) | ~tens of milliseconds |
| Planning time (Jetson Orin) | <100ms for UR10 |
| Speedup vs CPU planners | 60x average |
| Speedup vs TrajOpt | 87x on desktop, 35x on Orin MAXN |
| Robot segmentation (4090) | ~3000 Hz (480x480 images) |

---

## NVIDIA + ROS 2 Ecosystem Integration

### MoveIt 2 Plugin
cuMotion integrates seamlessly as a MoveIt 2 planner:
```yaml
# In moveit config
planning_plugin: isaac_ros_cumotion_moveit/CuMotionPlannerPlugin
```

Benefits over standard OMPL planners:
- GPU parallelism for faster planning
- Better solutions in cluttered environments
- Smooth, time-optimal trajectories by default

### Isaac Manipulator
Higher-level workflows using cuMotion:
- Pose-to-pose motion
- Object-following
- Pick-and-place with perception

### Deployment Options
1. **Standalone MoveIt**: RViz visualization without robot
2. **Isaac Sim**: Simulated robot testing
3. **Physical robot**: Production deployment

---

## Code Examples to Include

### 1. Installation (Binary)
```bash
# Jazzy
sudo apt-get install -y ros-jazzy-isaac-ros-cumotion ros-jazzy-isaac-ros-cumotion-moveit

# Humble
sudo apt-get install -y ros-humble-isaac-ros-cumotion ros-humble-isaac-ros-cumotion-moveit
```

### 2. Installation (Source)
```bash
# Clone repository
cd ${ISAAC_ROS_WS}/src
git clone --recurse-submodules -b release-3.2 \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git

# Build
cd ${ISAAC_ROS_WS}
colcon build --symlink-install --packages-up-to isaac_ros_cumotion
source install/setup.bash
```

### 3. Launch cuMotion Planner Node
```bash
# Launch cuMotion planner with robot configuration
ros2 launch isaac_ros_cumotion cumotion_planner.launch.py \
  robot:=ur10e \
  urdf_path:=/path/to/ur10e.urdf \
  xrdf_path:=/path/to/ur10e.xrdf
```

### 4. MoveIt 2 Configuration
```yaml
# moveit_controllers.yaml
cumotion:
  planning_plugin: isaac_ros_cumotion_moveit/CuMotionPlannerPlugin
  request_adapters:
    - default_planner_request_adapters/AddTimeOptimalParameterization
```

### 5. Robot Segmentation Launch
```bash
# Launch robot segmentation node
ros2 launch isaac_ros_cumotion robot_segmentation.launch.py \
  robot:=ur10e \
  camera_topic:=/camera/depth/image_rect_raw
```

---

## Diagrams Needed

### 1. cuMotion Architecture
```
┌──────────────────────────────────────────────────────────────────┐
│                        MoveIt 2 Application                       │
│                    (RViz / Python API / C++)                      │
└─────────────────────────────┬────────────────────────────────────┘
                              │
┌─────────────────────────────▼────────────────────────────────────┐
│                  cuMotion MoveIt Plugin                           │
│              (isaac_ros_cumotion_moveit)                          │
└─────────────────────────────┬────────────────────────────────────┘
                              │
┌─────────────────────────────▼────────────────────────────────────┐
│                    cuMotion Planner Node                          │
│                    (isaac_ros_cumotion)                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐   │
│  │  Trajectory     │  │   Collision     │  │    Robot        │   │
│  │  Optimization   │  │   Checking      │  │  Segmentation   │   │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘   │
└─────────────────────────────┬────────────────────────────────────┘
                              │
┌─────────────────────────────▼────────────────────────────────────┐
│                         cuRobo Backend                            │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐            │
│  │   IK     │ │Collision │ │Trajectory│ │   GPU    │            │
│  │ Solver   │ │ Checker  │ │Optimizer │ │ Kernels  │            │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘            │
│                              │                                    │
│  ┌───────────────────────────▼──────────────────────────────┐    │
│  │                  CUDA / PyTorch                           │    │
│  └───────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────┘
```

### 2. Obstacle Avoidance Pipeline
```
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│  Depth Camera │───►│    nvblox     │───►│   cuMotion    │
│   Stream      │    │  (SDF Build)  │    │   Planner     │
└───────────────┘    └───────────────┘    └───────────────┘
                            │                     │
                     ┌──────▼──────┐       ┌──────▼──────┐
                     │   ESDF      │       │  Collision- │
                     │   Voxel     │       │  Free Path  │
                     │   Grid      │       └─────────────┘
                     └─────────────┘
```

### 3. Motion Planning Flow
```
Goal Pose ──► cuMotion Planner ──► Collision-Free Trajectory ──► Controller
                   │                        │
            ┌──────▼──────┐          ┌──────▼──────┐
            │   cuRobo    │          │  Smooth,    │
            │   Backend   │          │  Optimal-   │
            │  (GPU)      │          │  Time Path  │
            └─────────────┘          └─────────────┘
```

---

## Prerequisites / Related Concepts

1. **MoveIt 2** - Motion planning framework (cuMotion integrates as plugin)
2. **Motion Planning** - Concepts of path planning and trajectory optimization
3. **Kinematics** - Forward/inverse kinematics understanding
4. **Isaac ROS** - Overall GPU-accelerated ROS 2 ecosystem
5. **nvblox** - Real-time 3D reconstruction for obstacle avoidance
6. **URDF/XRDF** - Robot description formats
7. **ROS 2** - Middleware framework (Jazzy/Humble)
8. **CUDA** - GPU programming model (background understanding)

---

## Source URLs for Citations

### Official Documentation
- [Isaac ROS cuMotion Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html)
- [isaac_ros_cumotion Package](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html)
- [isaac_ros_cumotion_moveit Package](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html)
- [XRDF Documentation](https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html)
- [Isaac Manipulator](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_manipulator/index.html)

### GitHub Repositories
- [isaac_ros_cumotion GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion)
- [cuRobo GitHub](https://github.com/NVlabs/curobo)
- [Releases/Tags](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/releases)

### cuRobo Backend
- [cuRobo Official Site](https://curobo.org/)
- [cuRobo Installation](https://curobo.org/get_started/1_install_instructions.html)
- [cuRobo Technical Report](https://curobo.org/reports/curobo_report.pdf)
- [NVIDIA Research Publication](https://research.nvidia.com/publication/2023-05_curobo-parallelized-collision-free-robot-motion-generation)
- [NVIDIA Developer Blog - cuRobo](https://developer.nvidia.com/blog/cuda-accelerated-robot-motion-generation-in-milliseconds-with-curobo/)

### Tutorials
- [cuMotion MoveIt Plugin with Isaac Sim](https://nvidia-isaac-ros.github.io/concepts/manipulation/cumotion_moveit/tutorial_isaac_sim.html)
- [cuMotion with Perception](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_manipulator/tutorials/tutorial_e2e.html)
- [Robot Segmentation Example](https://curobo.org/advanced_examples/4_robot_segmentation.html)

### Isaac Sim Integration
- [cuRobo and cuMotion in Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_curobo.html)
- [XRDF Editor in Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/manipulators_robot_description_editor.html)

### nvblox Integration
- [nvblox Paper (arXiv)](https://arxiv.org/abs/2311.00626)
- [Isaac ROS nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)

### Release Notes
- [Isaac ROS Release Notes](https://nvidia-isaac-ros.github.io/releases/index.html)
- [CES 2025 Isaac ROS 3.2 Announcement](https://forums.developer.nvidia.com/t/ces-2025-isaac-ros-3-2-and-platform-updates/319021)

### Third-Party Resources
- [Black Coffee Robotics - cuRobo and ROS2](https://www.blackcoffeerobotics.com/blog/curobo-nvidia-and-ros2-for-motion-planning)
- [PickNik - MoveIt Pro with NVIDIA GPU Acceleration](https://picknik.ai/news-releases/MoveIt-Pro-Enhanced-with-NVIDIA-GPU-Acceleration.html)
- [Industrial Robot Motion Planning with GPUs (arXiv 2508.04146)](https://arxiv.org/html/2508.04146v2)

---

## Style Notes (from existing entries)

Based on review of isaac-ros.mdx, moveit2.mdx, and motion-planning.mdx:

1. **Frontmatter**: title, description, last_validated, sidebar badge
2. **Imports**: Astro Starlight components (Aside, Card, CardGrid, LinkCard, Tabs, TabItem, Steps)
3. **Badge span**: `<span class="level-badge practical">Practical</span>` (Practical for tools)
4. **Opening**: Bold term definition, one-liner summary
5. **Aside notes**: Version info, tips, cautions
6. **Prerequisites section**: CardGrid with LinkCards
7. **"Why X Matters"** section: Bullet points or comparison tables
8. **Architecture section**: ASCII diagrams, package tables
9. **Code Examples**: Tabs for different scenarios, bash commands
10. **Performance section**: Comparison tables
11. **Related Terms**: CardGrid with LinkCards
12. **Learn More**: External tutorial links
13. **Sources**: Citation links at bottom with brief descriptions

---

## Notes for Content Creation

- **Badge type**: Practical (hands-on tool like moveit2, isaac-ros)
- **Emphasis**: GPU acceleration benefits, MoveIt 2 integration, nvblox for perception
- **Target audience**: ROS 2 developers adding GPU-accelerated motion planning
- **Related entries to link**:
  - `/robotics-glossary/software/ros2/moveit2/`
  - `/robotics-glossary/software/isaac/isaac-ros/`
  - `/robotics-glossary/concepts/control/motion-planning/`
  - `/robotics-glossary/concepts/fundamentals/kinematics/`
- **Version focus**: Isaac ROS 3.2/4.0 with ROS 2 Jazzy
- **Key differentiators from standard MoveIt planners**:
  - GPU parallelism (1000s of trajectories)
  - Time-optimal, smooth trajectories by default
  - Better performance in cluttered environments
  - nvblox integration for real-time obstacle avoidance

---

## Verification Checklist

- [x] cuMotion is current term (not deprecated)
- [x] Version numbers verified (Isaac ROS 3.2.x, 4.0.x)
- [x] GPU requirements confirmed (Volta+, 4GB VRAM)
- [x] ROS 2 compatibility confirmed (Jazzy, Humble)
- [x] Performance claims sourced from official docs
- [x] XRDF format documented
- [x] nvblox integration described
- [x] Multiple code examples identified
