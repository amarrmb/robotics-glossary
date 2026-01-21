# Isaac Lab Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/software/nvidia/isaac-lab.mdx
**Status:** Research complete

---

## Summary

NVIDIA Isaac Lab is an open-source, GPU-accelerated framework for robot learning built on Isaac Sim. It's the successor to Isaac Gym and provides a unified platform for reinforcement learning, imitation learning, and motion planning research. Isaac Lab enables massive parallelization (4096+ environments) on a single GPU, making it practical to train sophisticated robot policies without supercomputer access.

---

## Key Concepts to Cover

### What is Isaac Lab?
- Open-source modular framework for robot learning
- Built on Isaac Sim (Omniverse)
- Successor to Isaac Gym (now deprecated)
- Unifies IsaacGymEnvs, OmniIsaacGymEnvs, and Orbit frameworks
- Licensed BSD-3-Clause (isaaclab_mimic extension: Apache 2.0)

### Why Isaac Lab Matters
- GPU-parallel training: 4096+ environments on single GPU
- Training speed: ~90,000 FPS for locomotion tasks (RTX A6000)
- Photorealistic rendering via RTX
- Full sensor simulation (cameras, LiDAR, IMU, contact)
- Domain randomization for sim-to-real transfer
- ROS 2 integration through Isaac Sim bridge

### Core Capabilities
1. **Reinforcement Learning**
   - Manager-based workflow (modular, structured)
   - Direct workflow (performance-critical, similar to Isaac Gym)
   - Multi-agent RL support (MAPPO, IPPO via SKRL)

2. **Imitation Learning**
   - Isaac Lab Mimic extension
   - Automatic trajectory generation from demonstrations
   - MimicGen-based data augmentation

3. **Motion Planning**
   - cuRobo integration
   - Inverse kinematics solvers

4. **Teleoperation**
   - Keyboard, spacemouse, VR headset support
   - Dexterous retargeting for multi-finger hands

---

## Version Information

### Current Version (January 2026)
- **Isaac Lab:** 2.3.1 (December 4, 2025)
- **Isaac Sim Compatibility:**
  - v2.3.X: Isaac Sim 4.5 / 5.0 / 5.1
  - v2.2.X: Isaac Sim 4.5 / 5.0
  - v2.1.X / v2.0.X: Isaac Sim 4.5

### Python Requirements
- Isaac Sim 5.X: Python 3.11
- Isaac Sim 4.X: Python 3.10

### System Requirements
- RAM: 32GB minimum (64GB+ recommended)
- VRAM: 16GB minimum
- GPU: NVIDIA with RT Cores (Volta+ architecture, Compute Capability 7.0+)
- Note: A100/H100 without RT cores NOT supported
- OS: Ubuntu 22.04 or 24.04

### Key Release History
- **v2.3** (Nov 2025): DexSuite for dexterous manipulation, ADR, PBT, SkillGen/Mimic integration
- **v2.2** (Sept 2025): Isaac Sim 5.0 compatibility, expanded physics features
- **v2.0** (Jan 2025, CES): Isaac Lab Mimic, AMP tasks for humanoids, tiled rendering 1.2x boost

---

## Supported RL Frameworks

| Framework | Notes |
|-----------|-------|
| RSL-RL | JIT/ONNX policy export |
| SKRL | PyTorch + JAX, AMP support, multi-agent |
| RL-Games | Vectorized training |
| Stable-Baselines3 | Extensive docs, numpy-based |

**Performance benchmark:** Training Isaac-Humanoid-v0, 65.5M steps on RTX 4090 varies by library.

---

## Supported Robots (16+)

### Quadrupeds
- Anybotics: ANYmal-B, ANYmal-C, ANYmal-D
- Unitree: A1, Go1, Go2
- Boston Dynamics: Spot

### Humanoids
- Unitree H1, G1
- Other standard humanoid models

### Manipulators
- Franka Emika Panda
- Kuka arms
- Universal Robots

### Hands
- Allegro Hand (with Kuka arm for DexSuite)
- Unitree three-finger hand
- Inspire five-finger hand

---

## Pre-Built Environments (30+)

### Locomotion
- Flat terrain velocity tracking
- Rough terrain locomotion
- Humanoid walking/running

### Manipulation
- Lift cube (various control modes: joint pos, IK abs, IK rel)
- Stack cube
- Push/reach tasks

### Contact-Rich (Factory Tasks)
- Peg insertion
- Gear meshing
- Nut-bolt fastening

### Dexterous (DexSuite v2.3+)
- `Isaac-Dexsuite-Kuka-Allegro-Lift-v0`
- `Isaac-Dexsuite-Kuka-Allegro-Reorientation-v0`

### AMP Tasks (v2.0+)
- Humanoid walking
- Humanoid running
- Humanoid dancing

---

## Code Examples to Include

### Basic Training Command
```bash
# Train quadruped locomotion with RSL-RL
python source/standalone/workflows/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-D-v0 \
    --num_envs 4096 \
    --headless
```

### Environment Creation (Manager-Based)
```python
from omni.isaac.lab.envs import ManagerBasedRLEnv

# Create parallel environments
env = ManagerBasedRLEnv(
    cfg=MyRobotEnvCfg,
    num_envs=4096,
)

# Training loop
obs = env.reset()
for _ in range(1000000):
    actions = policy(obs)
    obs, rewards, dones, infos = env.step(actions)
```

### Configuration Class
```python
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.envs import DirectRLEnvCfg

@configclass
class MyRobotEnvCfg(DirectRLEnvCfg):
    # Scene configuration
    scene: MySceneCfg = MySceneCfg(num_envs=4096)

    # Observation and action spaces
    observation_space = 48
    action_space = 12

    # Episode settings
    episode_length_s = 20.0
```

### Policy Export
```python
# Export trained policy for deployment
from isaaclab_rl.rsl_rl import export_policy_as_jit
export_policy_as_jit(agent, path="policy.pt")
```

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        Isaac Lab                                 │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌──────────────────┐  ┌───────────────┐ │
│  │ RL Frameworks    │  │ Imitation        │  │ Motion        │ │
│  │ RSL-RL, SKRL     │  │ Learning         │  │ Planning      │ │
│  │ RL-Games, SB3    │  │ (Mimic)          │  │ (cuRobo)      │ │
│  └────────┬─────────┘  └────────┬─────────┘  └───────┬───────┘ │
│           │                     │                     │         │
│  ┌────────▼─────────────────────▼─────────────────────▼───────┐ │
│  │              Environment API (Manager/Direct)               │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │ │
│  │  │ Robots      │  │ Sensors     │  │ Domain              │ │ │
│  │  │ Actuators   │  │ Cameras,IMU │  │ Randomization       │ │ │
│  │  │ Controllers │  │ Contact,Ray │  │ ADR, PBT            │ │ │
│  │  └─────────────┘  └─────────────┘  └─────────────────────┘ │ │
│  └───────────────────────────┬─────────────────────────────────┘ │
├──────────────────────────────┼──────────────────────────────────┤
│                         Isaac Sim                                │
│  ┌─────────────┐  ┌─────────────┐  ┌────────────────────────┐   │
│  │ PhysX 5     │  │ RTX Render  │  │ USD Scene Graph        │   │
│  │ (GPU)       │  │ (Tiled)     │  │ (OpenUSD)              │   │
│  └─────────────┘  └─────────────┘  └────────────────────────┘   │
├──────────────────────────────────────────────────────────────────┤
│                    NVIDIA Omniverse / GPU                        │
└──────────────────────────────────────────────────────────────────┘
```

---

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| State-based training | 1.6M+ FPS | Multi-GPU setup |
| Vision-based training | 60K+ FPS | Multi-GPU setup |
| Spot locomotion | ~90,000 FPS | RTX A6000, RSL-RL |
| Parallel environments | 4096+ | Single GPU |
| Tiled rendering boost | 1.2x | v2.0 improvement |

---

## Isaac Gym vs Isaac Lab (Migration)

| Aspect | Isaac Gym | Isaac Lab |
|--------|-----------|-----------|
| Status | Deprecated | Active |
| Config format | YAML | Python configclass |
| Scene creation | Manual loop | Cloner API (automatic) |
| Joint ordering | Depth-first | Breadth-first |
| Angle units | Radians | Degrees (UI) / Radians (API) |
| Rendering | Basic | RTX photorealistic |
| Soft body physics | No | Yes (PhysX 5) |
| ROS support | No | Yes (ROS 2 bridge) |

---

## ROS 2 Integration

- Isaac Sim provides ROS 2 bridge
- Supported: ROS 2 Jazzy (recommended), ROS 2 Humble
- Action graphs for sensor publishing
- MoveIt2 integration via isaac_moveit package
- Same ROS 2 code runs in sim and on real hardware

---

## Related Glossary Entries

- [Isaac Sim](/software/simulation/isaac-sim/) - Parent simulation platform
- [Isaac ROS](/software/isaac/isaac-ros/) - ROS 2 deployment packages
- [Reinforcement Learning](/concepts/ai/reinforcement-learning/) - Core training paradigm
- [cuMotion](/software/nvidia/cumotion/) - GPU motion planning
- [Jetson Orin](/hardware/compute/jetson-orin/) - Edge deployment

---

## Source URLs for Citations

### Primary Documentation
- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/main/index.html)
- [Isaac Lab GitHub](https://github.com/isaac-sim/IsaacLab)
- [NVIDIA Isaac Lab Developer Page](https://developer.nvidia.com/isaac/lab)

### Release Information
- [Isaac Lab Releases](https://github.com/isaac-sim/IsaacLab/releases/)
- [Isaac Lab 2.3 GA Discussion](https://github.com/isaac-sim/IsaacLab/discussions/3898)
- [Isaac Sim 5.0 and Lab 2.2 Blog](https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/)

### Technical Resources
- [Isaac Lab arXiv Paper](https://arxiv.org/html/2511.04831v1) - "A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning"
- [RL Frameworks Comparison](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_frameworks.html)
- [Available Environments](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html)

### Migration Guides
- [Migrating from IsaacGymEnvs](https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html)
- [Migrating from OmniIsaacGymEnvs](https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_omniisaacgymenvs.html)

### Blog Posts
- [Isaac Lab 2.3 Teleoperation Blog](https://developer.nvidia.com/blog/streamline-robot-learning-with-whole-body-control-and-enhanced-teleoperation-in-nvidia-isaac-lab-2-3/)
- [Spot Training Blog](https://developer.nvidia.com/blog/closing-the-sim-to-real-gap-training-spot-quadruped-locomotion-with-nvidia-isaac-lab)
- [Isaac Lab-Arena Blog](https://developer.nvidia.com/blog/simplify-generalist-robot-policy-evaluation-in-simulation-with-nvidia-isaac-lab-arena/)

### Installation
- [Local Installation Guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)
- [Pip Installation Guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html)

---

## Diagrams Needed

1. **Architecture Overview** - Show layers from RL frameworks → Isaac Lab → Isaac Sim → GPU
2. **Training Loop** - Agent-environment interaction with parallel envs
3. **Sim-to-Real Pipeline** - Train in Isaac Lab → Export policy → Deploy on Jetson

---

## Notes for Content Creation

- Badge: **Practical** (hands-on tool for robot learning)
- Emphasize GPU parallelization as key differentiator
- Show training command early (practical focus)
- Compare with Isaac Gym briefly (migration context)
- Link heavily to Isaac Sim entry (dependency)
- Include performance numbers (compelling for practitioners)
- Domain randomization enables sim-to-real transfer
