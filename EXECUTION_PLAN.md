# Robotics Glossary Execution Plan
## "The Ralph Wiggum Way" - One Thing at a Time

**Goal**: Build thought leadership in NVIDIA + ROS2 robotics. Zero to Hero journey.
**Philosophy**: Not a docs clone. Helpful summaries. Amazing content > more content.
**Validation Date**: January 20, 2026

---

## Phase 0: Audit & Fix Existing Content (Do This First)

### Files to Validate (19 content files)

Each file needs:
- [ ] Content accuracy check against official Jan 2026 sources
- [ ] Version numbers updated (JetPack, Isaac ROS, ROS 2 distros)
- [ ] Broken link check
- [ ] Add `last_validated: 2026-01-20` to frontmatter
- [ ] Add Sources section with proper citations

#### Priority 1: NVIDIA Stack (Our Differentiator)
| File | Status | Key Validation Points |
|------|--------|----------------------|
| `hardware/compute/jetson-orin.mdx` | ✅ | JetPack 6.x LTS status, verify Thor migration guide exists |
| `hardware/compute/jetson-thor.mdx` | ✅ | Verify specs (Thor was announced but check GA status) |
| `hardware/compute/dgx-spark.mdx` | ✅ | Announced GTC 2025, verify availability/specs |
| `software/isaac/isaac-ros.mdx` | ✅ | Isaac ROS 3.2 → check if 4.0 is out, NITROS changes |
| `software/simulation/isaac-sim.mdx` | ✅ | Isaac Sim 4.x status, Omniverse integration |

#### Priority 2: ROS2 Ecosystem
| File | Status | Key Validation Points |
|------|--------|----------------------|
| `software/frameworks/ros2.mdx` | ✅ | Humble EOL date, Jazzy status, Rolling changes |

#### Priority 3: Sensors & Fundamentals
| File | Status | Key Validation Points |
|------|--------|----------------------|
| `hardware/sensors/lidar.mdx` | ✅ | Current sensor landscape |
| `hardware/sensors/cameras.mdx` | ✅ | Depth camera options, NVIDIA support |
| `hardware/sensors/imu.mdx` | ✅ | Standard reference |
| `concepts/fundamentals/kinematics.mdx` | ✅ | Timeless, light review |
| `concepts/fundamentals/degrees-of-freedom.mdx` | ✅ | Timeless, light review |
| `concepts/perception/slam.mdx` | ✅ | Isaac ROS SLAM packages |
| `concepts/control/motion-planning.mdx` | ✅ | cuMotion integration |
| `concepts/control/pid.mdx` | ✅ | Timeless, light review |
| `concepts/ai/neural-networks.mdx` | ✅ | General, light review |
| `concepts/ai/reinforcement-learning.mdx` | ✅ | Robotics RL landscape |

---

## Phase 1: Content Enhancement (After Audit)

### Citation Standard

Every entry should have a **Sources** section at the bottom:

```markdown
## Sources

- [Official Name](url) — What we referenced from here
- [Paper/Article Title](url) — Specific claim this supports
```

**Rules**:
1. NVIDIA docs → cite directly with version
2. ROS docs → cite with distro
3. Academic concepts → cite authoritative textbook or paper
4. Performance claims → must have source or mark as "typical" / "approximate"

### Content Quality Checklist

For each entry, verify:

- [ ] **One clear definition** in first paragraph (no jargon soup)
- [ ] **Why it matters** for robotics (not just what it is)
- [ ] **Practical example** or code that actually works
- [ ] **Honest trade-offs** (not marketing copy)
- [ ] **Connected to the journey** (prerequisites + next steps)
- [ ] **Sources cited** for any factual claims

---

## Phase 2: Missing Critical Content (Zero to Hero Path)

### The Learning Journey

```
[Beginner] → [Foundation] → [Integration] → [Production]
```

### New Entries Needed (Prioritized)

#### Must Have (Core Journey)
| Entry | Why Essential | Priority |
|-------|--------------|----------|
| `concepts/fundamentals/transforms.mdx` | Foundation for all robotics math | P0 |
| `concepts/fundamentals/coordinate-frames.mdx` | Can't understand TF without this | P0 |
| `software/ros2/tf2.mdx` | Core ROS2 concept, links to fundamentals | P0 |
| `software/ros2/nav2.mdx` | Navigation is core use case | P0 |
| `software/ros2/moveit2.mdx` | Manipulation is core use case | P0 |
| `software/nvidia/cumotion.mdx` | NVIDIA's MoveIt alternative | P1 |
| `software/nvidia/nvblox.mdx` | 3D reconstruction, core for nav | P1 |
| `software/ros2/urdf-xacro.mdx` | Robot description, foundation | P1 |

#### Should Have (Complete the Story)
| Entry | Why Important | Priority |
|-------|--------------|----------|
| `concepts/perception/visual-odometry.mdx` | Bridges SLAM and cameras | P2 |
| `concepts/perception/sensor-fusion.mdx` | Real robots need this | P2 |
| `software/nvidia/isaac-lab.mdx` | Sim-to-real training | P2 |
| `software/nvidia/tensorrt.mdx` | Inference optimization | P2 |
| `hardware/sensors/depth-cameras.mdx` | Specific to stereo/ToF | P2 |

#### Differentiating Content (Thought Leadership)
| Entry | Why Important | Priority |
|-------|--------------|----------|
| `concepts/ai/vla.mdx` | VLA models (RT-2, GR00T, OpenVLA) - cutting edge robotics AI | P1 |
| `software/embedded/yocto.mdx` | Production Jetson deployments use Yocto, not Ubuntu | P1 |
| `software/embedded/nixos.mdx` | Reproducible robotics deployments - emerging best practice | P2 |

---

## Phase 3: Zero to Hero Learning Paths

### Path 1: "I Have a Jetson, Now What?"
```
jetson-orin → jetpack-setup → ros2-install → isaac-ros → first-robot
```

### Path 2: "ROS2 Developer → NVIDIA Acceleration"
```
ros2 → nav2 → isaac-ros → nitros → performance-tuning
```

### Path 3: "Robotics Beginner → First AMR"
```
degrees-of-freedom → coordinate-frames → transforms → tf2 → urdf → nav2
```

### Path 4: "AI-First Robotics (VLA/Foundation Models)"
```
neural-networks → reinforcement-learning → vla → isaac-lab → groot
```

### Path 5: "Production Jetson Deployment"
```
jetson-orin → yocto → isaac-ros → tensorrt → deployment-best-practices
```

**Implementation**: Add `learning_paths` frontmatter and create `/paths/` pages later.

---

## Execution Checklist (Ralph Wiggum Style)

### Week 1: Audit Existing NVIDIA Content
- [ ] Day 1-2: Validate `jetson-orin.mdx` - check JetPack 6/7 status
- [ ] Day 2-3: Validate `jetson-thor.mdx` - verify Thor GA status
- [ ] Day 3-4: Validate `isaac-ros.mdx` - check Isaac ROS version
- [ ] Day 4-5: Validate `isaac-sim.mdx` - check Isaac Sim 4.x
- [ ] Day 5: Validate `dgx-spark.mdx` - availability status

### Week 2: Audit ROS2 & Sensors
- [ ] Day 1: Validate `ros2.mdx` - distro status
- [ ] Day 2: Validate `slam.mdx` - Isaac SLAM packages
- [ ] Day 3: Validate sensor files (lidar, cameras, imu)
- [ ] Day 4-5: Validate fundamentals & concepts

### New Entries Created
| Entry | Status | Created | Notes |
|-------|--------|---------|-------|
| `concepts/fundamentals/transforms.mdx` | ✅ Complete | 2026-01-21 | P0 foundation for robotics math |
| `concepts/fundamentals/coordinate-frames.mdx` | ✅ Complete | 2026-01-21 | P0 prerequisite for TF2 |
| `software/ros2/tf2.mdx` | ✅ Complete | 2026-01-21 | P0 ROS 2 transform library |
| `software/ros2/nav2.mdx` | ✅ Complete | 2026-01-21 | P0 ROS 2 navigation stack |
| `software/ros2/moveit2.mdx` | ✅ Complete | 2026-01-21 | P0 ROS 2 motion planning framework |
| `software/ros2/urdf-xacro.mdx` | ✅ Complete | 2026-01-21 | P1 Robot description format |
| `software/nvidia/cumotion.mdx` | ✅ Complete | 2026-01-21 | P1 GPU-accelerated motion planning |
| `software/nvidia/nvblox.mdx` | ✅ Complete | 2026-01-21 | P1 Real-time 3D reconstruction |
| `software/nvidia/tensorrt.mdx` | ✅ Complete | 2026-01-21 | P2 Inference optimization |
| `software/nvidia/isaac-lab.mdx` | ✅ Complete | 2026-01-21 | P2 Sim-to-real training |
| `concepts/perception/visual-odometry.mdx` | ✅ Complete | 2026-01-21 | P2 Bridges SLAM and cameras |
| `concepts/perception/sensor-fusion.mdx` | ✅ Complete | 2026-01-21 | P2 Multi-sensor state estimation |
| `hardware/sensors/depth-cameras.mdx` | ✅ Complete | 2026-01-21 | P2 Stereo, ToF, structured light depth sensing |

### Week 3-4: New Critical Content
- [x] Write `transforms.mdx` and `coordinate-frames.mdx`
- [x] Write `tf2.mdx`
- [x] Write `nav2.mdx`
- [x] Write `moveit2.mdx`

### Week 5-6: NVIDIA Deep Dives
- [x] Write `cumotion.mdx`
- [x] Write `nvblox.mdx`
- [x] Write `tensorrt.mdx`

---

## Quality Gates

Before any content goes live:

1. **Accuracy**: Claims verified against official source dated 2025+
2. **Freshness**: No outdated version numbers or deprecated APIs
3. **Citations**: Sources section present with working links
4. **Clarity**: Definition understandable without prior knowledge
5. **Utility**: Reader can take action after reading
6. **Connection**: Links to prerequisites and next steps

---

## What We Are NOT Doing

- ❌ Copying NVIDIA docs verbatim
- ❌ Writing 5000-word essays when 500 words suffice
- ❌ Covering every NVIDIA product (focus: robotics)
- ❌ Generic AI/ML content (focus: robotics applications)
- ❌ Historical deep-dives (focus: what works today)

---

## Success Looks Like

- Engineer finds us searching "isaac ros tutorial"
- Reads 3 entries, understands how pieces connect
- Bookmarks for reference
- Shares with team
- Eventually contributes back

---

## Frontmatter Template (Updated)

```yaml
---
title: Term Name
description: One-line for search/social
last_validated: 2026-01-20
nvidia_version: "3.2"  # if applicable
ros_distro: humble     # if applicable
sidebar:
  badge:
    text: Practical
    variant: success
---
```

---

---

## How to Run (Scripts)

### Option 1: Single File
```bash
# Generate prompt for one file, copy to Claude Code
./scripts/claude-prompt.sh hardware/compute/jetson-orin.mdx
```

### Option 2: Interactive Loop
```bash
# Process all files with prompts, track progress
./scripts/run-all.sh
```

### Option 3: Direct to Claude Code
```bash
# Pipe prompt directly to claude (if installed)
./scripts/claude-prompt.sh hardware/compute/jetson-orin.mdx | claude
```

### Check Progress
```bash
# See what's done and what's remaining
./scripts/ralph-wiggum.sh --status
```

### Reset Progress
```bash
rm .ralph-progress
```

### Files Created
```
scripts/
├── ralph-wiggum.sh    # Full interactive workflow
├── claude-prompt.sh   # Generate prompt for one file
└── run-all.sh         # Batch processor

research/              # Research notes go here
├── hardware/
├── software/
└── concepts/
```

---

*"I'm helping!" - Ralph Wiggum*

*One file. One validation. One commit. Repeat.*
