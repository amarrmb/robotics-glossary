# Unified Robotics Learning Resource — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Transform the robotics-glossary into a single journey-based learning destination that covers robotics concepts (reference), DeviceNexus engineering stories (guides), and links them together with subtle "In Practice" callouts.

**Architecture:** Three-layer content model — Concepts (reference, definition-first), Guides (first-person engineering narratives), and "In Practice" callouts (one-sentence bridge at the bottom of relevant Concept pages). The learning path follows Stage 1→5 (Fundamentals → Perception → Control → Learning → AI). New `concepts/learning/` subtree fills the primary gap (how robots learn from demonstration).

**Tech Stack:** Astro + Starlight, MDX, astro.config.mjs sidebar config. No new dependencies. All content is MDX files. Run `npm run dev` to preview at `http://localhost:4321/robotics-glossary/`.

---

## Task 1: Add Guides Section to Sidebar + Create Index

**Files:**
- Modify: `astro.config.mjs`
- Create: `src/content/docs/guides/index.mdx`

**Step 1: Add Guides section to sidebar in astro.config.mjs**

In `astro.config.mjs`, insert a new sidebar group after `Software` and before `Contributing`:

```js
{
  label: 'Guides',
  collapsed: false,
  badge: { text: 'DeviceNexus', variant: 'caution' },
  items: [
    { label: 'Overview', slug: 'guides/index' },
    { label: 'Teaching a Robot to Pick Up a Candy Bar', slug: 'guides/act-training-lessons' },
    { label: '3 Numbers That Tell You How to Optimize', slug: 'guides/latency-floor-model' },
  ],
},
```

Also add a new `concepts/learning` autogenerate group inside the `Concepts` section, after `Control & Planning` and before `AI & Learning`:

```js
{
  label: 'Learning from Demonstration',
  autogenerate: { directory: 'concepts/learning' },
},
```

**Step 2: Create Guides index page**

Create `src/content/docs/guides/index.mdx`:

```mdx
---
title: Guides
description: Engineering stories from building DeviceNexus — how we applied robotics concepts to real problems
---

import { LinkCard, CardGrid } from '@astrojs/starlight/components';

These are first-person accounts of building with physical AI — the failures, the fixes, and what we learned. Each guide links back to the glossary concepts it applies.

They're companion pieces to the reference entries in [Concepts](/robotics-glossary/concepts/fundamentals/kinematics/). Read the concept for theory; read the guide for how it plays out in practice.

<CardGrid>
  <LinkCard
    title="Teaching a Robot to Pick Up a Candy Bar"
    description="56 episodes, 3 training runs, one candy bar. What we learned training an SO-101 arm with ACT."
    href="/robotics-glossary/guides/act-training-lessons/"
  />
  <LinkCard
    title="3 Numbers That Tell You How to Optimize"
    description="Before you try quantization: compute R1, R2, R3 first. The latency floor model from 30 hours of failed optimization."
    href="/robotics-glossary/guides/latency-floor-model/"
  />
</CardGrid>
```

**Step 3: Preview and verify sidebar renders correctly**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/
# Verify: "Guides" section visible in sidebar with DeviceNexus badge
# Verify: "Learning from Demonstration" group visible under Concepts
# Verify: /guides/ index page renders without errors
```

**Step 4: Commit**

```bash
git add astro.config.mjs src/content/docs/guides/index.mdx
git commit -m "feat(nav): add Guides section + Learning from Demonstration to sidebar"
```

---

## Task 2: Move Blog Posts into Guides

**Files:**
- Create: `src/content/docs/guides/act-training-lessons.mdx` (adapted from devicenexus-docs)
- Create: `src/content/docs/guides/latency-floor-model.mdx` (adapted from devicenexus-docs)

**Step 1: Copy and adapt the ACT training lessons post**

Copy content from `/home/amar/baskd/devicenexus-docs/src/content/docs/blog/act-training-lessons.mdx` into `src/content/docs/guides/act-training-lessons.mdx`.

Add Starlight frontmatter fields and a footer CTA. The frontmatter should become:

```yaml
---
title: "Teaching a Robot to Pick Up a Candy Bar"
description: "Everything we learned training an SO-101 arm with ACT — the bugs, the failures, and the data collection protocol that finally worked."
sidebar:
  badge:
    text: Guide
    variant: caution
last_updated: 2026-02-07
---
```

Add at the very top after frontmatter (before the italic intro line):

```mdx
import { Aside, LinkCard, CardGrid } from '@astrojs/starlight/components';
```

Add at the very bottom of the file:

```mdx
---

## Key Concepts in This Guide

<CardGrid>
  <LinkCard title="Imitation Learning" description="The paradigm behind training robots from demonstrations" href="/robotics-glossary/concepts/learning/imitation-learning/" />
  <LinkCard title="Teleoperation" description="How leader/follower arms work for data collection" href="/robotics-glossary/concepts/learning/teleoperation/" />
  <LinkCard title="Robot Training Dataset" description="What episodes are and why quality matters" href="/robotics-glossary/concepts/learning/robot-training-dataset/" />
  <LinkCard title="VLA Models" description="The model architecture (ACT) used in this guide" href="/robotics-glossary/concepts/ai/vla-models/" />
</CardGrid>

<Aside title="Built with DeviceNexus">
  The teach-train-deploy loop described here is what DeviceNexus automates. [Learn more at devicenexus.ai](https://devicenexus.ai)
</Aside>
```

**Step 2: Copy and adapt the latency floor model post**

Copy content from `/home/amar/baskd/devicenexus-docs/src/content/docs/blog/latency-floor-model.mdx` into `src/content/docs/guides/latency-floor-model.mdx`.

Frontmatter:

```yaml
---
title: "3 Numbers That Tell You Exactly How to Optimize Your AI Model"
description: "Before you spend a week trying FP4 quantization, compute these first."
sidebar:
  badge:
    text: Guide
    variant: caution
last_updated: 2026-02-01
---
```

Add at the very bottom:

```mdx
---

## Key Concepts in This Guide

<CardGrid>
  <LinkCard title="Inference Latency" description="R1, R2, R3 — the three latency floors" href="/robotics-glossary/concepts/ai/inference-latency/" />
  <LinkCard title="Jetson Thor" description="The hardware used in this optimization" href="/robotics-glossary/hardware/compute/jetson-thor/" />
  <LinkCard title="TensorRT" description="NVIDIA's inference optimization toolkit" href="/robotics-glossary/software/nvidia/tensorrt/" />
</CardGrid>

<Aside title="Built with DeviceNexus">
  Deploying optimized models to robot fleets is what DeviceNexus handles. [Learn more at devicenexus.ai](https://devicenexus.ai)
</Aside>
```

**Step 3: Verify both pages render**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/guides/act-training-lessons/
# Verify: renders without MDX errors, bottom CardGrid shows
# Open http://localhost:4321/robotics-glossary/guides/latency-floor-model/
# Verify: renders without MDX errors
```

**Step 4: Commit**

```bash
git add src/content/docs/guides/
git commit -m "feat(guides): add ACT training lessons + latency floor model guides"
```

---

## Task 3: Add "Imitation Learning" Entry

**Files:**
- Create: `src/content/docs/concepts/learning/imitation-learning.mdx`

This is the highest-priority missing entry. It explains THE paradigm shift in modern robotics.

**Step 1: Create the entry**

```mdx
---
title: Imitation Learning
description: Teaching robots by recording human demonstrations, not writing rules
sidebar:
  badge:
    text: Conceptual
    variant: note
last_validated: 2026-03-02
---

import { Aside, Card, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge conceptual">Conceptual</span>

**Imitation Learning** (also called *Learning from Demonstration* or *Behavioral Cloning*) is the approach of teaching a robot a task by recording human demonstrations and training a model to replicate them. Instead of writing rules ("if object is at position X, move joint 3 by Y degrees"), you show the robot the task repeatedly and let a neural network learn the mapping from observations to actions.

<Aside>
The key insight: humans are good at doing tasks but bad at writing down how. Imitation learning sidesteps the need to articulate the rules by learning directly from examples.
</Aside>

## How It Works

```
1. Human demonstrates the task
   └── Teleoperation: human controls robot through leader arm
   └── Recording: camera images + joint positions captured at 30fps

2. Demonstrations become a dataset
   └── Each run = one "episode"
   └── 50-200 episodes needed for reliable manipulation

3. A policy model trains on the dataset
   └── Input: camera image + current joint positions
   └── Output: next joint positions (actions)
   └── Loss: minimize difference between predicted and recorded actions

4. Trained policy runs on the robot autonomously
   └── Robot observes → model predicts → joints execute → repeat at 30fps
```

## Imitation Learning vs. Alternatives

| Approach | How it works | Robotics reality |
|----------|-------------|-----------------|
| **Imitation Learning** | Train on human demos | Practical for manipulation today |
| **Reinforcement Learning** | Learn from reward signal | Sample inefficient; simulation required |
| **Hand-coded rules** | Write explicit logic | Brittle; fails on any variation |
| **Classical planning** | Search through state space | Works for structured environments, not unstructured manipulation |

Reinforcement Learning (RL) gets more academic attention, but for physical manipulation tasks (picking, placing, assembly), imitation learning currently works better in practice — fewer samples needed, no simulation required, no reward engineering.

## The Data Collection Problem

The quality of demonstrations determines the quality of the policy. Common failure modes:

- **Too few episodes** — Model memorizes specific trajectories, can't generalize. Minimum 50 for simple pick-place.
- **No spatial diversity** — Objects only placed at one position during demos → model only works at that position.
- **Idle frames** — Robot sitting still at start/end of recording → model learns "wait before acting."
- **Inconsistent technique** — Demonstrations that vary too much in *how* the task is done confuse the model.

## Modern Architectures

**ACT (Action Chunking with Transformers)**
Predicts a *chunk* of N future actions at once, then re-observes. Avoids jitter from frame-by-frame prediction.

**Diffusion Policy**
Treats action prediction as denoising. More expressive; slower inference.

**VLA Models (Vision-Language-Action)**
Large models (Octo, π₀, GR00T) pretrained on diverse robot data, fine-tuned on your demos. Better generalization with fewer demonstrations.

## Related Terms

<CardGrid>
  <LinkCard title="Teleoperation" description="How you physically collect the demonstrations" href="/robotics-glossary/concepts/learning/teleoperation/" />
  <LinkCard title="Robot Training Dataset" description="What the recorded episodes look like, and why quality matters" href="/robotics-glossary/concepts/learning/robot-training-dataset/" />
  <LinkCard title="VLA Models" description="Large pretrained models fine-tuned via imitation learning" href="/robotics-glossary/concepts/ai/vla-models/" />
  <LinkCard title="Action Chunking" description="Predicting sequences of actions, not single actions" href="/robotics-glossary/concepts/learning/action-chunking/" />
</CardGrid>

<Aside title="In Practice">
  We used imitation learning to train a 6-DOF robot arm to pick up a candy bar — 56 demonstrations, 100K training steps. Every failure mode above appeared at least once. [Read the full account →](/robotics-glossary/guides/act-training-lessons/)
</Aside>

## Sources

- [ACT Paper — Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware](https://arxiv.org/abs/2304.13705) — Original ACT paper (Zhao et al., 2023)
- [LeRobot](https://github.com/huggingface/lerobot) — HuggingFace's open-source imitation learning library for robotics
- [Diffusion Policy](https://diffusion-policy.cs.columbia.edu/) — Diffusion-based behavioral cloning (Chi et al., 2023)
```

**Step 2: Preview**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/learning/imitation-learning/
# Verify: page renders, badge shows, Related Terms CardGrid renders
# Verify: "In Practice" callout shows with link to guide
```

**Step 3: Commit**

```bash
git add src/content/docs/concepts/learning/imitation-learning.mdx
git commit -m "feat(learning): add Imitation Learning entry"
```

---

## Task 4: Add "Teleoperation" Entry

**Files:**
- Create: `src/content/docs/concepts/learning/teleoperation.mdx`

**Step 1: Create the entry**

```mdx
---
title: Teleoperation
description: Controlling a robot remotely to record training demonstrations
sidebar:
  badge:
    text: Practical
    variant: success
last_validated: 2026-03-02
---

import { Aside, Card, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge practical">Practical</span>

**Teleoperation** is the control of a robot at a distance — either through a remote interface or, in the context of imitation learning, through a physically coupled *leader* device that the human manipulates while the *follower* robot mirrors the movements in real time.

In modern robot learning, teleoperation serves one purpose: **collecting high-quality training demonstrations**.

## Leader-Follower Setup

The most common approach for robot arm data collection uses two identical arms:

```
Human grips and moves → Leader Arm
                              │
                         (joint angles mirrored in real time)
                              │
                              ▼
                        Follower Arm → records data
```

The leader arm is unloaded (no task to perform) — the human moves it naturally. The follower arm, holding the object, mirrors every joint position exactly. Camera images from the follower's perspective + joint positions are recorded to a dataset file.

**Why physically coupled instead of joystick?**
Joint-to-joint mirroring preserves the natural dynamics of the human's hand. Joystick control adds a translation layer that introduces latency and reduces demonstration quality.

## What Gets Recorded

At 30fps during teleoperation, each frame captures:

| Channel | Data | Size |
|---------|------|------|
| `observation.state` | Joint positions (6 floats for 6-DOF arm) | 6 × float32 |
| `action` | Target joint positions to execute | 6 × float32 |
| `observation.images.overhead` | Camera frame (640×480 RGB) | ~900KB |
| `observation.images.side` | Second camera frame (optional but recommended) | ~900KB |

One 10-second demonstration at 30fps = 300 frames = ~540MB uncompressed video + ~14KB joint data.

## Data Collection Rules

These rules are hard-won. Breaking them degrades model quality:

1. **Watch the camera feed, not the robot** — the model sees what the camera sees. Your demonstrations must match that viewpoint.
2. **Start moving before you press record** — idle frames at the start train the model to wait.
3. **Stop recording immediately when done** — idle frames at the end train the model to hold still when it shouldn't.
4. **Vary object position across spatial zones** — at least 6 zones across the workspace. Models overfit to a single position.
5. **Complete the full task on every episode** — partial demonstrations (reach without grasp) are worse than no demonstration.

## Hardware Options

| Approach | Hardware | Cost | Notes |
|----------|----------|------|-------|
| **Leader-follower arms** | SO-101, SO-ARM100, ALOHA | $300–$20K | Best demonstration quality |
| **VR controllers** | Meta Quest, Valve Index | $300–$1K | Good for bimanual; harder calibration |
| **Joystick / SpaceMouse** | 3Dconnexion | $150–$400 | Easy setup; lower demo quality |
| **Kinesthetic teaching** | (hand-guide the arm directly) | No extra hardware | Robot must support gravity comp |

## Related Terms

<CardGrid>
  <LinkCard title="Imitation Learning" description="The training paradigm that uses teleop demonstrations" href="/robotics-glossary/concepts/learning/imitation-learning/" />
  <LinkCard title="Robot Training Dataset" description="What the recordings become after collection" href="/robotics-glossary/concepts/learning/robot-training-dataset/" />
  <LinkCard title="End Effector" description="The gripper at the end of the arm doing the work" href="/robotics-glossary/concepts/fundamentals/end-effector/" />
  <LinkCard title="Kinematics" description="How joint positions map to arm position in space" href="/robotics-glossary/concepts/fundamentals/kinematics/" />
</CardGrid>

<Aside title="In Practice">
  We teleoped 56 demonstrations using SO-101 leader/follower arms connected via USB to a Jetson Orin. Idle frames at start/end were the single biggest data quality issue — cost us one entire training run. [Full story →](/robotics-glossary/guides/act-training-lessons/)
</Aside>

## Sources

- [LeRobot Teleoperation Guide](https://github.com/huggingface/lerobot?tab=readme-ov-file#teleoperate-your-robot) — practical setup for leader/follower arms
- [ACT Paper](https://arxiv.org/abs/2304.13705) — original leader-follower data collection methodology
- [ALOHA Hardware](https://www.trossenrobotics.com/aloha.aspx) — low-cost bimanual teleoperation platform
```

**Step 2: Preview and commit**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/learning/teleoperation/
# Verify: renders correctly, "In Practice" callout appears

git add src/content/docs/concepts/learning/teleoperation.mdx
git commit -m "feat(learning): add Teleoperation entry"
```

---

## Task 5: Add "Robot Training Dataset" Entry

**Files:**
- Create: `src/content/docs/concepts/learning/robot-training-dataset.mdx`

**Step 1: Create the entry**

```mdx
---
title: Robot Training Dataset
description: The collection of recorded demonstrations used to train a robot policy
sidebar:
  badge:
    text: Practical
    variant: success
last_validated: 2026-03-02
---

import { Aside, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge practical">Practical</span>

A **robot training dataset** is the collection of recorded teleoperation demonstrations used to train an imitation learning policy. It's the bridge between human expertise (how to do the task) and robot capability (the trained model).

Quality matters enormously. A small, high-quality dataset consistently outperforms a large, noisy one.

## Structure: Episodes

A dataset is organized into **episodes** — individual recordings of one complete task execution.

```
dataset/
  data/
    chunk-000/
      episode_000000.parquet   ← 300 rows (10s at 30fps)
      episode_000001.parquet
      ...
  videos/
    chunk-000/
      observation.images.overhead_episode_000000.mp4
      observation.images.side_episode_000000.mp4
      ...
  meta/
    info.json      ← dataset schema, camera configs
    stats.json     ← per-column mean/std (used for normalization)
    episodes/
      chunk-000/episode_index.parquet
```

Each row in an episode file is one timestep (at 30fps = 33ms). Columns include `observation.state`, `action`, `episode_index`, `frame_index`, `timestamp`.

## Episode Quality Criteria

Not all episodes are equal. Each episode should pass:

| Check | Why it matters |
|-------|---------------|
| **Complete task** | Partial demos (reach without grasp) teach incomplete behavior |
| **No idle frames at start** | Models learn to "wait" if they see the arm stationary at the beginning |
| **No idle frames at end** | Models learn to "stop" early if the arm sits still after completion |
| **Consistent technique** | High variance in *how* the task is done dilutes the training signal |
| **Full gripper cycle** | For pick tasks: the dataset must contain open → close → open sequences |

## Spatial Diversity

A dataset with 50 episodes all at the same object position trains a model that only works at that position. Divide the workspace into zones and collect roughly equal coverage:

```
         ┌───────────────────────┐
         │   Z1   │  Z2   │  Z3  │  ← far
         ├────────┼───────┼──────┤
         │   Z4   │  Z5   │  Z6  │  ← near
         └───────────────────────┘
         left     center   right
```

Target: ~8 episodes per zone for 50 total. Also vary object orientation (±45°) within zones.

## How Many Episodes?

| Task complexity | Minimum | Reliable |
|-----------------|---------|---------|
| Simple pick-place, fixed position | 20 | 50 |
| Pick-place with spatial diversity | 50 | 150+ |
| Complex manipulation (multi-step) | 100 | 300+ |
| Bimanual tasks | 200 | 500+ |

These are rough guides. The true measure is evaluation success rate on held-out positions.

<Aside type="caution">
30% of our first dataset was incomplete episodes (robot reached but didn't grasp). Those episodes weren't just unhelpful — they actively harmed training by teaching the model to not close the gripper.
</Aside>

## Dataset Formats

| Format | Used by | Notes |
|--------|---------|-------|
| **LeRobot v3** | LeRobot, ACT, Diffusion Policy | Parquet + MP4 video, HuggingFace-native |
| **RLDS** | RT-2, open-source VLAs | TensorFlow Datasets format |
| **MCAP** | ROS 2, Foxglove | For raw sensor recordings before conversion |
| **HDF5** | ACT (original paper) | Legacy; most frameworks have moved to Parquet |

## Related Terms

<CardGrid>
  <LinkCard title="Imitation Learning" description="The training paradigm that uses these datasets" href="/robotics-glossary/concepts/learning/imitation-learning/" />
  <LinkCard title="Teleoperation" description="How the recordings are captured" href="/robotics-glossary/concepts/learning/teleoperation/" />
  <LinkCard title="Model Checkpoint" description="What you get after training on the dataset" href="/robotics-glossary/concepts/learning/model-checkpoint/" />
  <LinkCard title="Sim-to-Real" description="Validating the trained policy before deploying to real hardware" href="/robotics-glossary/concepts/learning/sim-to-real/" />
</CardGrid>

<Aside title="In Practice">
  We collected 56 episodes over 4 batches, validating quality after each batch of 10. Found 9/30 incomplete episodes in our first attempt — all discarded and re-recorded. The validation script caught issues automated tools would miss. [Full story →](/robotics-glossary/guides/act-training-lessons/)
</Aside>

## Sources

- [LeRobot Dataset Format](https://github.com/huggingface/lerobot/blob/main/examples/2_evaluate_pretrained_policy.md) — v3 dataset structure and validation
- [ACT Paper](https://arxiv.org/abs/2304.13705) — data collection methodology and episode count recommendations
```

**Step 2: Preview and commit**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/learning/robot-training-dataset/
# Verify: episode structure renders, tables look correct

git add src/content/docs/concepts/learning/robot-training-dataset.mdx
git commit -m "feat(learning): add Robot Training Dataset entry"
```

---

## Task 6: Add "Action Chunking" Entry

**Files:**
- Create: `src/content/docs/concepts/learning/action-chunking.mdx`

**Step 1: Create the entry**

```mdx
---
title: Action Chunking
description: Predicting sequences of future actions instead of one action at a time
sidebar:
  badge:
    text: Conceptual
    variant: note
last_validated: 2026-03-02
---

import { Aside, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge conceptual">Conceptual</span>

**Action Chunking** is the practice of having a robot policy predict a *sequence* of N future actions at once (a "chunk"), execute them open-loop, then re-observe and predict the next chunk. It's the core innovation in ACT (Action Chunking with Transformers).

## The Problem It Solves

A naive policy predicts one action per observation:

```
observe → predict action₁ → execute → observe → predict action₂ → execute → ...
```

This causes **compounding jitter**: every small prediction error gets corrected at the next frame, creating constant micro-corrections that look jerky and can destabilize grasps.

Action chunking predicts N actions at once:

```
observe → predict [a₁, a₂, ..., a₅₀] → execute all 50 → observe → predict next chunk
```

The robot commits to a smooth trajectory for the duration of the chunk, then re-evaluates. This produces more natural, human-like motion.

## Chunk Size Trade-offs

| Chunk size | Effect |
|------------|--------|
| **Too small (1-5)** | Jittery, constant correction |
| **Good (25-100)** | Smooth motion, re-observes often enough to correct |
| **Too large (200+)** | Smooth but can't correct for unexpected obstacles |

For a 30fps robot, chunk size 50 = 1.67 seconds of committed action before re-observing. Most ACT implementations use 50–100.

## Temporal Ensemble

A refinement: instead of hard-switching between chunks, maintain a sliding window of recent predictions and take a weighted average. This further smooths chunk boundaries.

```
chunk 1:  [a₁  a₂  a₃  a₄  a₅]
chunk 2:         [a₃' a₄' a₅' a₆ a₇]   ← overlapping predictions
             ↓
average:         [(a₃+a₃')/2  ...]      ← blended action
```

## In ACT

ACT uses a chunk size equal to `n_action_steps` (no temporal ensemble in the basic configuration). The policy is a CVAE (Conditional Variational Autoencoder) with a Transformer decoder that outputs all N actions simultaneously:

```
Input:  image + joint state + latent z
Output: [Δjoint₁, Δjoint₂, ..., Δjoint_N]  (the full chunk)
```

## Related Terms

<CardGrid>
  <LinkCard title="Imitation Learning" description="The training paradigm that uses action chunking" href="/robotics-glossary/concepts/learning/imitation-learning/" />
  <LinkCard title="VLA Models" description="Larger models that also use chunked action prediction" href="/robotics-glossary/concepts/ai/vla-models/" />
  <LinkCard title="Robot Training Dataset" description="The data that trains the chunking policy" href="/robotics-glossary/concepts/learning/robot-training-dataset/" />
</CardGrid>

## Sources

- [ACT Paper — Learning Fine-Grained Bimanual Manipulation](https://arxiv.org/abs/2304.13705) — Zhao et al., 2023. Introduces action chunking + temporal ensemble.
- [LeRobot ACT implementation](https://github.com/huggingface/lerobot/tree/main/lerobot/common/policies/act) — chunk_size and n_action_steps config
```

**Step 2: Preview and commit**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/learning/action-chunking/
# Verify: renders correctly

git add src/content/docs/concepts/learning/action-chunking.mdx
git commit -m "feat(learning): add Action Chunking entry"
```

---

## Task 7: Add "End Effector" Entry (Tier 3 — Fundamentals Gap)

**Files:**
- Create: `src/content/docs/concepts/fundamentals/end-effector.mdx`

**Step 1: Create the entry**

```mdx
---
title: End Effector
description: The tool at the tip of a robot arm that interacts with the world
sidebar:
  badge:
    text: Conceptual
    variant: note
last_validated: 2026-03-02
---

import { Aside, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge conceptual">Conceptual</span>

The **end effector** is the device at the end of a robot arm that physically interacts with the environment — the gripper, welding torch, suction cup, spray nozzle, or sensor mounted at the tip of the kinematic chain.

In robotics, the arm gets the end effector *to a position*. The end effector *does the work*.

## Types

| Type | Mechanism | Common use |
|------|-----------|-----------|
| **Parallel gripper** | Two fingers driven by a single actuator | General-purpose manipulation |
| **Multi-finger hand** | 3–5 actuated fingers | Dexterous manipulation |
| **Suction cup** | Vacuum pump | Flat objects, boxes (warehousing) |
| **Magnetic** | Electromagnet | Metal parts (automotive) |
| **Tool changer** | Mounts different tools | Flexible manufacturing |
| **Force-torque sensor** | Measures contact forces | Assembly, grinding |
| **Camera** | Mounted for close inspection | Quality control |

## The Gripper Coordinate Frame

The end effector has its own coordinate frame, called the **tool frame** (also: TCP — Tool Center Point). Motion planning and inverse kinematics work in terms of where the tool frame should be in space, not the arm itself.

```
World frame (fixed)
    │
    ▼
Base frame (robot base)
    │
    ▼
Joint frames (one per joint)
    │
    ▼
Flange frame (last joint)
    │
    ▼
Tool frame = End Effector position
```

When you command a robot to "move to position (0.5, 0.3, 0.2)", you're commanding the tool frame to that position.

## Relevance to Robot Learning

In imitation learning datasets, `observation.state` contains the joint positions of the arm. The end effector state (open/closed for a gripper, grip force) is typically an additional channel — often just one float for a parallel gripper (0.0 = closed, 1.0 = fully open).

The gripper cycle — open → move to object → close → lift — is the critical success criterion for a pick task. Missing this cycle in a training episode is an invalid demonstration.

## Related Terms

<CardGrid>
  <LinkCard title="Kinematics" description="How joint angles determine end effector position" href="/robotics-glossary/concepts/fundamentals/kinematics/" />
  <LinkCard title="Joints and Links" description="The mechanical chain the end effector sits at the tip of" href="/robotics-glossary/concepts/fundamentals/joints-and-links/" />
  <LinkCard title="Teleoperation" description="Human control that drives the end effector through demonstrations" href="/robotics-glossary/concepts/learning/teleoperation/" />
  <LinkCard title="Degrees of Freedom" description="How many independent ways the end effector can move" href="/robotics-glossary/concepts/fundamentals/degrees-of-freedom/" />
</CardGrid>

## Sources

- [Robotics: Modelling, Planning and Control](https://link.springer.com/book/10.1007/978-1-84628-642-1) — Siciliano et al. — standard reference for manipulator kinematics and end effector frames
- [ROS 2 TF2 — Tool Frames](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) — how tool frames are represented in ROS 2
```

**Step 2: Preview and commit**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/fundamentals/end-effector/
# Verify: renders, links to kinematics and joints-and-links work

git add src/content/docs/concepts/fundamentals/end-effector.mdx
git commit -m "feat(fundamentals): add End Effector entry"
```

---

## Task 8: Add "In Practice" Callouts to Existing Pages

**Files:**
- Modify: `src/content/docs/concepts/ai/vla-models.mdx`
- Modify: `src/content/docs/concepts/perception/slam.mdx`
- Modify: `src/content/docs/concepts/control/motion-planning.mdx`

These pages already exist and cover topics with real DeviceNexus application. Add a single `<Aside title="In Practice">` callout at the bottom (before the Sources section) linking to the relevant guide.

**Step 1: Add callout to vla-models.mdx**

Read the file first, then find the line just before `## Sources`. Insert:

```mdx
<Aside title="In Practice">
  We trained an ACT policy (a VLA-family model) on a 6-DOF robot arm using 56 demonstrations. The limiting factor wasn't the model — it was data quality. [Full training account →](/robotics-glossary/guides/act-training-lessons/)
</Aside>
```

Also ensure `Aside` is in the import line at the top.

**Step 2: Add callout to motion-planning.mdx**

Find the line before `## Sources`. Insert:

```mdx
<Aside title="In Practice">
  For AI-trained robot policies (ACT, VLA), motion planning is replaced by end-to-end learned action prediction — the model directly outputs joint positions from camera observations. Classical motion planning is still used for structured tasks and safety constraints.
</Aside>
```

**Step 3: Verify callouts render**

```bash
npm run dev
# Check /concepts/ai/vla-models/ — callout visible above Sources
# Check /concepts/control/motion-planning/ — callout visible above Sources
```

**Step 4: Commit**

```bash
git add src/content/docs/concepts/ai/vla-models.mdx \
        src/content/docs/concepts/control/motion-planning.mdx
git commit -m "feat: add In Practice callouts to VLA Models and Motion Planning"
```

---

## Task 9: Add Tier 2 Entry — "Sim-to-Real"

**Files:**
- Create: `src/content/docs/concepts/learning/sim-to-real.mdx`

**Step 1: Create the entry**

```mdx
---
title: Sim-to-Real Transfer
description: Training or validating robot policies in simulation before deploying to physical hardware
sidebar:
  badge:
    text: Conceptual
    variant: note
last_validated: 2026-03-02
---

import { Aside, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge conceptual">Conceptual</span>

**Sim-to-Real transfer** is the practice of training or validating a robot policy in a physics simulation, then deploying it to physical hardware. The core challenge: simulations are never perfectly accurate, so policies trained in simulation often fail when transferred to the real world — this gap is called the **sim-to-real gap**.

## Why Simulate?

- **Speed**: Run 1000 episodes in simulation in the time a robot can do 10 in the real world
- **Safety**: Test failure modes without risking hardware or people
- **Reset**: Simulation resets instantly; real robots need a human to reset the scene
- **Validation gate**: Before deploying a new model checkpoint to a real robot, run it in sim first

## The Sim-to-Real Gap

Simulations get many things wrong:

| Gap | Example |
|-----|---------|
| **Visual realism** | Perfect lighting, no shadows, missing reflections |
| **Contact physics** | Object sliding on table vs. resting with friction |
| **Actuator dynamics** | Servo lag, backlash, temperature-dependent behavior |
| **Sensor noise** | Camera exposure, focus blur, IMU drift |

The gap is smaller for proprioceptive tasks (joint control) and larger for contact-rich manipulation.

## Approaches

**Domain Randomization** — vary simulation parameters (lighting, object mass, friction) so the policy learns to handle a distribution of environments, not just one.

**Domain Adaptation** — train a separate model to map simulation observations to real-world observations (image style transfer).

**Sim as Validation Gate** — don't train in sim; use sim to validate a checkpoint before deploying to real hardware. Lower ambition, higher practical value.

## Tools

| Tool | Use |
|------|-----|
| **Isaac Sim** | NVIDIA's photorealistic robot simulator. USD-based scenes. |
| **Isaac Lab** | RL training framework built on Isaac Sim (successor to Isaac Gym) |
| **MuJoCo** | Fast physics simulator; widely used in academic RL |
| **Gazebo** | ROS-native simulator; lower visual fidelity |
| **PyBullet** | Lightweight Python-native physics simulator |

## Related Terms

<CardGrid>
  <LinkCard title="Robot Training Dataset" description="The data that trains the policy being validated" href="/robotics-glossary/concepts/learning/robot-training-dataset/" />
  <LinkCard title="Model Checkpoint" description="The artifact being validated before deployment" href="/robotics-glossary/concepts/learning/model-checkpoint/" />
  <LinkCard title="Isaac Sim" description="NVIDIA's physics simulation platform" href="/robotics-glossary/software/simulation/isaac-sim/" />
  <LinkCard title="Imitation Learning" description="Training paradigm that often uses sim-to-real validation" href="/robotics-glossary/concepts/learning/imitation-learning/" />
</CardGrid>

## Sources

- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) — NVIDIA's RL + imitation learning training framework on Isaac Sim
- [OpenAI Domain Randomization](https://openai.com/research/learning-dexterous-in-hand-manipulation) — original domain randomization paper (Dactyl hand)
- [RoboAgent sim-to-real](https://robopen.github.io/) — practical sim-to-real for manipulation
```

**Step 2: Preview and commit**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/learning/sim-to-real/
# Verify: renders correctly

git add src/content/docs/concepts/learning/sim-to-real.mdx
git commit -m "feat(learning): add Sim-to-Real Transfer entry"
```

---

## Task 10: Add Tier 2 Entry — "Model Checkpoint"

**Files:**
- Create: `src/content/docs/concepts/learning/model-checkpoint.mdx`

**Step 1: Create the entry**

```mdx
---
title: Model Checkpoint
description: A saved snapshot of a neural network's weights at a point during training
sidebar:
  badge:
    text: Practical
    variant: success
last_validated: 2026-03-02
---

import { Aside, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge practical">Practical</span>

A **model checkpoint** is a saved snapshot of a neural network's weights (parameters) at a specific point during training. For robot policies, a checkpoint is the artifact you deploy to the robot — it encodes everything the model learned from the training dataset.

## What's in a Checkpoint

```
checkpoints/
  100000/
    pretrained_model/
      config.json          ← model architecture + hyperparameters
      model.safetensors    ← weight values (~200MB for ACT)
      stats.json           ← dataset normalization statistics (mean/std per channel)
```

`stats.json` is critical: the model was trained on normalized inputs (zero mean, unit variance). At inference, raw joint positions and camera pixels must be normalized by the *same* statistics from the training dataset. A checkpoint deployed without its matching stats.json will behave incorrectly.

## Training Curve and Checkpoint Selection

During training, loss is logged every N steps. Multiple checkpoints are saved:

```
Step 10K:  loss 0.180  ← early, underfitting
Step 50K:  loss 0.072
Step 80K:  loss 0.058
Step 100K: loss 0.049  ← typically the best for ACT
Step 120K: loss 0.051  ← can overfit on small datasets
```

The checkpoint with the lowest validation loss is the best candidate, but evaluation on the real robot is the ground truth. A checkpoint at step 80K may generalize better than 100K if the dataset is small.

## Checkpoint Lifecycle

```
Training completes → Checkpoint saved
         ↓
Sim validation (optional) — run in Isaac Sim, check success rate
         ↓
Deploy to robot via OTA update
         ↓
Physical evaluation — success rate on real task
         ↓
If regression: rollback to previous checkpoint
```

## Related Terms

<CardGrid>
  <LinkCard title="Robot Training Dataset" description="The data the checkpoint was trained on" href="/robotics-glossary/concepts/learning/robot-training-dataset/" />
  <LinkCard title="Sim-to-Real" description="Validating a checkpoint in simulation before physical deploy" href="/robotics-glossary/concepts/learning/sim-to-real/" />
  <LinkCard title="OTA Model Update" description="How checkpoints get pushed to robot fleets" href="/robotics-glossary/concepts/learning/ota-model-update/" />
  <LinkCard title="Imitation Learning" description="The training process that produces checkpoints" href="/robotics-glossary/concepts/learning/imitation-learning/" />
</CardGrid>

## Sources

- [LeRobot checkpoint format](https://github.com/huggingface/lerobot) — config.json + model.safetensors + stats.json structure
- [HuggingFace safetensors](https://github.com/huggingface/safetensors) — safe, fast weight serialization format
```

**Step 2: Preview and commit**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/learning/model-checkpoint/
# Verify: renders correctly

git add src/content/docs/concepts/learning/model-checkpoint.mdx
git commit -m "feat(learning): add Model Checkpoint entry"
```

---

## Task 11: Add Tier 2 Entry — "OTA Model Update"

**Files:**
- Create: `src/content/docs/concepts/learning/ota-model-update.mdx`

**Step 1: Create the entry**

```mdx
---
title: OTA Model Update
description: Pushing new model weights to deployed robots over-the-air without manual intervention
sidebar:
  badge:
    text: Practical
    variant: success
last_validated: 2026-03-02
---

import { Aside, CardGrid, LinkCard } from '@astrojs/starlight/components';

<span class="level-badge practical">Practical</span>

**OTA (Over-the-Air) model update** is the process of pushing new neural network weights to a deployed robot remotely — without physical access or manual file transfer. It's what makes continuous improvement practical at fleet scale: train a better model, push it to every robot, and the fleet improves overnight.

## Why It Matters

Without OTA updates, improving a deployed robot requires:
1. Training new checkpoint on a workstation
2. Copying checkpoint to a USB drive (or SSH transfer)
3. SSHing into the robot
4. Stopping the running policy
5. Replacing the checkpoint directory
6. Restarting the policy

Multiply this by 10 robots, and it becomes a full day's work. OTA reduces this to: training completes → push to fleet → robots hot-swap the model automatically.

## Hot-Swapping

A well-designed OTA system supports **hot-swap**: the robot loads the new model weights while it's running, without stopping the control loop. The sequence:

```
Training service pushes new checkpoint (model.safetensors + stats.json)
    ↓
Robot receives MODEL_UPDATE message via WebSocket
    ↓
Robot downloads and verifies checkpoint (SHA256 hash check)
    ↓
Robot loads new weights into memory (thread-safe)
    ↓
Control loop uses new model on next inference cycle
    ↓
Old weights garbage collected
```

The transition happens between inference cycles — typically at a chunk boundary — so there's no interruption to robot motion.

## Rollback

OTA systems should support rollback: if the new model performs worse, revert to the previous checkpoint. This requires:
- Keeping the N most recent checkpoints on the robot
- A success metric that can trigger automated rollback
- Remote rollback command capability

## Fleet Deployment

For multi-robot fleets, OTA enables staged rollouts:

```
Training completes → Deploy to 1 robot (canary)
                          ↓
                  24h evaluation → success rate acceptable?
                          ↓
                  Yes → Deploy to full fleet
                  No  → Rollback canary, investigate
```

<Aside title="In Practice">
  DeviceNexus uses this pattern — the training service pushes a new checkpoint to Aether (the connectivity hub), which broadcasts a MODEL_UPDATE message. Each robot-agent downloads and hot-swaps the model without stopping its control loop. [Learn more at devicenexus.ai](https://devicenexus.ai)
</Aside>

## Related Terms

<CardGrid>
  <LinkCard title="Model Checkpoint" description="The artifact being pushed" href="/robotics-glossary/concepts/learning/model-checkpoint/" />
  <LinkCard title="Imitation Learning" description="The training process that creates the new model" href="/robotics-glossary/concepts/learning/imitation-learning/" />
  <LinkCard title="Sim-to-Real" description="Validation step before OTA deployment" href="/robotics-glossary/concepts/learning/sim-to-real/" />
</CardGrid>

## Sources

- [AWS IoT Greengrass OTA](https://docs.aws.amazon.com/greengrass/v2/developerguide/update-greengrass-core-v2.html) — reference implementation for edge OTA
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/) — NVIDIA's robot software distribution pipeline
```

**Step 2: Preview and commit**

```bash
npm run dev
# Open http://localhost:4321/robotics-glossary/concepts/learning/ota-model-update/
# Verify: renders correctly

git add src/content/docs/concepts/learning/ota-model-update.mdx
git commit -m "feat(learning): add OTA Model Update entry"
```

---

## Task 12: Final Check — Cross-links and Navigation

**Step 1: Verify all internal links resolve**

```bash
npm run build 2>&1 | grep -i "warn\|error\|broken\|404"
# Expect: 0 broken link warnings
```

**Step 2: Check sidebar renders all new sections**

```bash
npm run dev
# Verify sidebar shows:
# - "Learning from Demonstration" group with 6 entries (imitation-learning, teleoperation,
#   robot-training-dataset, action-chunking, sim-to-real, model-checkpoint, ota-model-update)
# - "Guides" group with 2 entries + index
# - End Effector appears under Fundamentals (autogenerated)
```

**Step 3: Verify the learner journey — click through Stage 1→5**

Navigate the full path:
1. `/` → Start Here → How to Use
2. Fundamentals: Joints → Kinematics → Degrees of Freedom → End Effector
3. Learning: Imitation Learning → Teleoperation → Dataset → Action Chunking
4. Guides: ACT Training Lessons (check "Key Concepts" CardGrid links back to glossary)

**Step 4: Final commit**

```bash
git add -A
git commit -m "feat: complete unified learning resource — Guides section + Learning subtree"
git push
```
