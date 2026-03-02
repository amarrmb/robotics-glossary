# Design: Unified Robotics Learning Resource

**Date**: 2026-03-02
**Status**: Approved

## Problem

The robotics documentation landscape is fragmented. The robotics-glossary covers theory well but has a critical gap — it explains *how robots move* but not *how robots learn*. DeviceNexus has blog content (ACT training lessons, latency floor model) that fills this gap experientially, but it lives in a separate site and hasn't shipped.

The goal: one site where someone going from 0→1 in robotics can learn progressively, see concepts applied in practice (DeviceNexus's work), and discover the product naturally through genuine educational content.

## Design

### Structure: Single Project

The robotics-glossary Astro/Starlight site is the unified destination. Blog posts from `devicenexus-docs` move in as a top-level "Guides" section.

```
robotics-glossary/
├── Getting Started
│   └── How to Use
├── Concepts (reference layer — definitions + theory)
│   ├── Fundamentals    → Stage 1: what is a robot
│   ├── Perception      → Stage 2: how does it sense the world
│   ├── Control         → Stage 3: how does it move
│   ├── Learning        → Stage 4: how does it learn  ← primary gap to fill
│   └── AI & Models     → Stage 5: production AI + inference
├── Hardware            → Jetson, DGX Spark, sensors
├── Software            → ROS 2, Isaac, Nav2
└── Guides              → DeviceNexus perspective pieces (story layer)
    ├── Teaching a Robot to Pick Up a Candy Bar   ← from devicenexus-docs
    ├── 3 Numbers That Tell You How to Optimize   ← from devicenexus-docs
    └── (future: deployment, fleet management)
```

The Concepts section = general knowledge (citable, non-opinionated).
The Guides section = DeviceNexus voice (first-person, opinionated, links to product).

### Content Strategy: Journey-Based Learning

Sidebar order in Concepts follows the learner journey. A new reader progresses linearly: Fundamentals → Perception → Control → Learning → AI & Models.

**The learning gap (Stage 4) requires 4 new entries:**

| Entry | What it answers |
|-------|----------------|
| Imitation Learning | Why modern robots learn from human demonstrations, not code |
| Teleoperation | How you physically teach a robot (leader/follower arms, data collection) |
| Robot Training Dataset | What an "episode" is, why quality matters, spatial coverage |
| Action Chunking | The key insight behind ACT — why you predict sequences, not single actions |

**Supporting new entries (Tier 2):**
- Sim-to-Real (validates before deployment)
- Model Checkpoint (what you deploy, how rollback works)
- OTA Model Update (why fleet robots need remote model updates)
- Inference Latency (R1/R2/R3 framework from the blog)

**Tier 3 — perception gap:**
- Computer Vision (feature detection, depth estimation basics)
- Point Cloud (primary 3D data type, referenced everywhere but no page)
- End Effector (fundamental manipulation concept, strangely absent)

### Subtle Pitch: "In Practice" Callout

On relevant glossary entries only, a small callout at the bottom links to the corresponding guide. The callout reads like educational context, not marketing.

**Pattern:**
```mdx
<Aside title="In Practice">
  DeviceNexus built a system for exactly this — recording leader arm
  demonstrations, converting them to training datasets, and retraining
  automatically when the robot fails. [See how it worked →](/robotics-glossary/guides/act-training-lessons/)
</Aside>
```

**Which entries get this callout:**

| Glossary Entry | Pitch angle | Links to |
|---------------|-------------|----------|
| Imitation Learning | TEACH tab + intervention loop | ACT training guide |
| Teleoperation | Leader/follower, MCAP recording | ACT training guide |
| Robot Training Dataset | Episode quality, spatial diversity | ACT training guide |
| Inference Latency | R1/R2/R3 model, real Jetson numbers | Latency floor guide |
| OTA Model Update | Aether fleet deployment | (future guide) |
| Sim-to-Real | Validation gate before deploy | (future guide) |

Entries NOT getting this callout: Kinematics, SLAM, PID, SLAM, Joint Types, etc. — pure reference content stays clean.

### Linking Model

**Glossary → Guides**: Every "In Practice" callout links to the relevant guide.

**Guides → Glossary**: Each blog post's first use of a technical term links to its glossary entry. "We collected 56 *episodes*" → `/concepts/learning/robot-training-dataset/`. This turns blog posts into teaching tools.

**Guides → DeviceNexus site**: Blog posts end with a footer CTA to devicenexus.ai. Not inline — only in the tail.

**Reader paths:**

```
Path A (researcher discovering robotics):
  Searches "imitation learning" → Glossary entry →
  "In Practice" callout → ACT guide →
  Footer CTA → devicenexus.ai

Path B (practitioner reading the blog):
  Reads ACT guide (shared on HN) →
  Clicks "episode" term → Glossary →
  Discovers other entries → bookmarks site
```

## Implementation Plan (high level)

1. **Add "Guides" section** to Starlight sidebar config + create the section index
2. **Move blog posts** from `devicenexus-docs` into `guides/` — adapt frontmatter to Starlight format
3. **Add 4 Tier 1 Learning entries** (imitation learning, teleoperation, dataset/episode, action chunking)
4. **Add "In Practice" callouts** to relevant existing pages (cross-link to guides)
5. **Add Tier 2 entries** (sim-to-real, checkpoint, OTA update, inference latency)
6. **Add Tier 3 entries** (end effector, computer vision, point cloud)
7. **Update navigation** — ensure sidebar reflects the Stage 1→5 journey order

## Non-Goals

- Not merging the Astro build configs or themes — the robotics-glossary Starlight theme is kept as-is
- Not publishing DeviceNexus product documentation inside the glossary — Guides are engineering stories, not docs
- Not adding pricing, signup CTAs, or marketing copy inline in glossary entries
