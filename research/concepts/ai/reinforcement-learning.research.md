# Reinforcement Learning for Robotics - Research Notes

**Research Date:** 2026-01-21
**File:** src/content/docs/concepts/ai/reinforcement-learning.mdx

---

## Current Version Numbers

### NVIDIA Isaac Lab
- **Isaac Lab:** v2.3.0 (November 2025) - latest GA release
  - Built on Isaac Sim 5.1
  - Supports DGX Spark with ARM multi-architecture Docker images
  - Source: [Isaac Lab 2.3 Release](https://github.com/isaac-sim/IsaacLab/discussions/3898)
- **Isaac Lab 2.2:** Released at SIGGRAPH 2025 (GA)
  - Full compatibility with Isaac Sim 5.0
  - Backwards compatible with Isaac Sim 4.5
  - Source: [Isaac Sim 5.0 / Lab 2.2 Announcement](https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/)

### NVIDIA Isaac Sim
- **Isaac Sim:** v5.1 (current with Isaac Lab 2.3)
- **Isaac Sim 5.0:** Open-sourced at SIGGRAPH 2025
  - Neural reconstruction and rendering
  - New Robot Import Wizard
  - MobilityGen extension for data generation
  - Source: [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)

### Supported RL Libraries in Isaac Lab
- **RSL-RL** - ETH Robotics Systems Lab library
- **RL-Games** - NVIDIA's GPU-optimized RL library
- **Stable-Baselines3** - Popular Python RL library
- **SKRL** - Modular RL library (supports PyTorch and JAX)
- **Ray/RLlib** - Distributed RL framework
- Source: [Isaac Lab RL Frameworks Comparison](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_frameworks.html)

### Physics Engines
- **PhysX** - Primary physics engine
- **NVIDIA Warp** - Differentiable physics
- **MuJoCo** - Also supported
- **Newton** - New GPU-accelerated, differentiable engine built on Warp
- Source: [Isaac Lab Documentation](https://developer.nvidia.com/isaac/lab)

---

## Verification of Current Content

### Code Example (Lines 121-133)
```python
from omni.isaac.lab.envs import ManagerBasedRLEnv
```
- **Status:** API verified - `omni.isaac.lab.envs` is correct module path
- `ManagerBasedRLEnv` is the current RL environment class
- `num_envs=4096` - accurate, can run 4096+ parallel envs
- Source: [Isaac Lab Envs API](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.html)

### rl_games Import (Lines 130-132)
```python
from rl_games.algos_torch import PPOAgent
```
- **Status:** Needs verification - rl_games API may differ
- Isaac Lab typically uses wrappers, not direct rl_games imports
- Consider updating to show wrapper-based approach
- Source: [Isaac Lab RL Training Tutorial](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/run_rl_training.html)

### Algorithm Table (Lines 87-93)
| Algorithm | Type | Best For | Status |
|-----------|------|----------|--------|
| PPO | Policy gradient | General purpose, stable | Correct |
| SAC | Actor-critic | Sample efficient, continuous control | Correct |
| TD3 | Actor-critic | Continuous control, less hyperparameter sensitive | Correct |
| DrQ-v2 | Image-based | Learning from pixels | Correct |

- **Research notes:**
  - PPO remains most reliable starting point (confirmed in NVIDIA docs)
  - SAC shows best sample efficiency (32 episodes to converge in some tasks)
  - TD3 achieves highest cumulative rewards but needs longer training
  - DrQ-v2 still foundational for visual RL (2021 paper, still baseline)
- Source: [PPO/SAC/TD3 Comparison](https://www.scirp.org/journal/paperinformation?paperid=131938), [DrQ-v2 GitHub](https://github.com/facebookresearch/drqv2)

### Sim-to-Real Section (Lines 96-116)
- **Domain randomization:** Still primary technique
- **Best practices (2025-2026):**
  - Combine system identification (SysID) for measurable params
  - Use domain randomization for uncertain/sensitive params
  - Include action delays in training
  - Use reward penalties for smooth control (avoid bang-bang)
- Source: [Sim-to-Real Review (Jan 2026)](https://www.sciencedirect.com/science/article/abs/pii/S0921889025004245)

### Isaac Lab Training Performance
- ~90,000 FPS training throughput with RTX A6000
- 4096+ parallel environments on single GPU
- Spot locomotion task training example available
- Source: [Isaac Lab Performance](https://developer.nvidia.com/blog/closing-the-sim-to-real-gap-training-spot-quadruped-locomotion-with-nvidia-isaac-lab/)

---

## New Features (Isaac Lab 2.3)

### Dexterous Manipulation
- **DexSuite:** New dexterous manipulation environments (Kuka arm + Allegro hand)
- **ADR:** Automatic Domain Randomization support
- **PBT:** Population-Based Training support
- Source: [Isaac Lab 2.3 Blog](https://developer.nvidia.com/blog/streamline-robot-learning-with-whole-body-control-and-enhanced-teleoperation-in-nvidia-isaac-lab-2-3/)

### SkillGen Workflow
- GPU-accelerated motion planning with cuRobo integration
- Collision-free manipulation demonstrations
- Combined with human input for adaptive demos
- Source: [Isaac Lab 2.3 Blog](https://developer.nvidia.com/blog/streamline-robot-learning-with-whole-body-control-and-enhanced-teleoperation-in-nvidia-isaac-lab-2-3/)

### Supported Robots (Quadrupeds)
- Anybotics ANYmal-B, ANYmal-C, ANYmal-D
- Unitree A1, Go1, Go2
- Boston Dynamics Spot
- Source: [Isaac Lab Robot Support](https://developer.nvidia.com/isaac/lab)

---

## Corrections/Updates Needed

### Line 118-133: Isaac Lab Code Example
- **Issue:** `from rl_games.algos_torch import PPOAgent` is not typical usage
- **Recommendation:** Update to show standard Isaac Lab training workflow:
  ```bash
  python source/standalone/workflows/rsl_rl/train.py --task Isaac-Velocity-Flat-Anymal-D-v0
  ```
- Or use the isaaclab_rl wrapper pattern

### Line 188: State of the Art (2026)
- **Current:** "Isaac Lab — NVIDIA's framework for GPU-accelerated robot learning"
- **Status:** Correct, could add version (2.3)
- **Add:** ADR, PBT, DexSuite as new capabilities

### Line 191-192: Missing Recent Developments
- **Consider adding:**
  - Isaac Lab-Arena for policy evaluation
  - SkillGen for demonstration generation
  - COMPASS pipeline for navigation
  - Source: [Isaac Lab-Arena Blog](https://developer.nvidia.com/blog/simplify-generalist-robot-policy-evaluation-in-simulation-with-nvidia-isaac-lab-arena/)

### Line 136-138: Isaac Lab Benefits
- "4096+ environments in parallel" - Correct
- "Photorealistic rendering" - Correct (RTX-based)
- "Domain randomization built-in" - Correct, now with ADR support

---

## ROS 2 Integration

### Current Integration Methods
- **CRISP Framework (Sept 2025):** ROS2-compatible RL pipeline for manipulators
  - Gymnasium interfaces
  - Validated on Franka FR3, Kuka IIWA14, Kinova Gen3
  - Source: [CRISP Paper](https://arxiv.org/html/2509.06819)
- **Isaac Sim → Gazebo → ROS 2:** Sim-to-real workflow documented
  - Source: [Sim-to-Real Transfer Paper](https://arxiv.org/abs/2501.02902)
- **ROS2Learn:** Framework for DRL with Gazebo (older, less active)
  - Source: [ROS2Learn GitHub](https://github.com/AcutronicRobotics/ros2learn)

---

## Summary

**Overall Assessment:** Content is accurate and well-structured.

**Verified as correct:**
- RL framework diagram and key components
- Algorithm table (PPO, SAC, TD3, DrQ-v2)
- Sim-to-real transfer concepts
- Domain randomization explanation
- Reward engineering examples
- Practical tips section

**Recommended updates:**
1. Update Isaac Lab code example to use current wrapper/CLI patterns
2. Add version numbers (Isaac Lab 2.3, Isaac Sim 5.1)
3. Mention ADR and PBT as new training techniques
4. Add Isaac Lab-Arena for policy evaluation
5. Could add Newton physics engine as emerging option

**No critical corrections required.**
