# Robotics Glossary — Project Structure & Design Plan

## Vision

**The canonical, community-maintained reference for robotics concepts and the NVIDIA ecosystem.**

Like [Modal's GPU Glossary](https://modal.com/gpu-glossary), but for robotics — covering hardware, software, perception, control, and the complete NVIDIA stack (Jetson, Isaac, Omniverse). A hyperlinked knowledge graph that connects concepts across abstraction layers.

---

## What Makes Modal's GPU Glossary Great

1. **Hyperlinked knowledge graph** — Every term links to related concepts, enabling exploration
2. **Keyboard navigation** — Arrow keys to move between pages, feels like a terminal
3. **Clean, focused design** — Minimal chrome, content-first
4. **Spans the full stack** — From hardware to compiler flags to performance tuning
5. **Linear + non-linear reading** — Can read sequentially or jump around
6. **Open source content** — CC-BY license, community contributions welcome

## Where We Can Be Better

| Modal GPU Glossary | Our Robotics Glossary |
|--------------------|----------------------|
| Single domain (GPUs/CUDA) | Multi-domain (hardware, software, perception, control, NVIDIA stack) |
| Text-heavy entries | Rich media: diagrams, architecture visuals, code snippets |
| No filtering/search | Faceted search + tag filtering |
| Static table of contents | Interactive concept map / knowledge graph visualization |
| No "learning paths" | Curated journeys for different roles (embedded eng, ML eng, roboticist) |
| Limited cross-referencing | Explicit "prerequisite" and "builds on" relationships |
| No version tracking | Version badges for Isaac/Jetson releases |
| Desktop-first | Mobile-first responsive |

---

## Tech Stack Recommendation

### **Astro + Starlight** (Recommended)

```
Why Astro:
✓ Blazing fast (ships zero JS by default)
✓ Starlight theme built for documentation
✓ MDX support for interactive components
✓ Built-in search (Pagefind)
✓ Easy GitHub Pages deployment
✓ Component islands for interactivity where needed
✓ Great DX with hot reload
```

**Alternative: Docusaurus** — More mature, React-based, but heavier

---

## Content Architecture

### Category Taxonomy

```
robotics-glossary/
├── fundamentals/           # Core robotics concepts
│   ├── kinematics/
│   ├── dynamics/
│   ├── control-theory/
│   └── coordinate-systems/
│
├── hardware/               # Physical components
│   ├── compute/            # Jetson, edge devices
│   ├── sensors/            # LiDAR, cameras, IMU, encoders
│   ├── actuators/          # Motors, grippers, servos
│   └── platforms/          # AMRs, manipulators, humanoids
│
├── perception/             # Sensing and understanding
│   ├── computer-vision/
│   ├── depth-sensing/
│   ├── sensor-fusion/
│   └── slam/
│
├── planning-control/       # Decision and execution
│   ├── motion-planning/
│   ├── path-planning/
│   ├── manipulation/
│   └── navigation/
│
├── software/               # Frameworks and tools
│   ├── ros/                # ROS/ROS2 ecosystem
│   ├── middleware/
│   ├── simulation/
│   └── deployment/
│
├── nvidia-stack/           # NVIDIA ecosystem (our differentiator)
│   ├── jetson/             # Hardware platform
│   │   ├── orin/
│   │   ├── xavier/
│   │   └── thor/
│   ├── isaac/              # Robotics software
│   │   ├── isaac-ros/
│   │   ├── isaac-sim/
│   │   ├── isaac-lab/
│   │   └── isaac-manipulator/
│   ├── omniverse/          # Simulation platform
│   ├── cuda/               # GPU computing (link to Modal for deep dives)
│   └── tensorrt/           # Inference optimization
│
├── ai-ml/                  # Learning systems
│   ├── foundation-models/  # GR00T, COMPASS, etc.
│   ├── reinforcement-learning/
│   ├── imitation-learning/
│   └── vla-models/         # Vision-Language-Action
│
└── operations/             # Production concerns
    ├── fleet-management/
    ├── ota-updates/
    ├── safety/
    └── compliance/
```

### Entry Schema (Frontmatter)

```yaml
---
# Required
title: "Isaac ROS"
slug: "isaac-ros"
category: "nvidia-stack/isaac"
summary: "GPU-accelerated ROS 2 packages for robotics perception and navigation"

# Relationships (for knowledge graph)
prerequisites:
  - ros2
  - cuda
  - jetson
related:
  - isaac-sim
  - tensorrt
  - nitros
builds_on:
  - computer-vision
  - depth-sensing
enables:
  - autonomous-navigation
  - manipulation

# Metadata
tags:
  - nvidia
  - ros
  - perception
  - gpu-accelerated
difficulty: intermediate  # beginner | intermediate | advanced
nvidia_version: "3.2"     # Version tracking
last_updated: 2025-01-20

# Learning paths this belongs to
learning_paths:
  - getting-started-jetson
  - perception-engineer
  - autonomous-navigation
---
```

---

## Site Structure

```
/                           # Landing page with category overview
├── /glossary/              # All terms A-Z
├── /category/{name}/       # Category landing pages
├── /term/{slug}/           # Individual term pages
├── /paths/                 # Curated learning journeys
│   ├── /getting-started/
│   ├── /perception-engineer/
│   ├── /navigation-engineer/
│   └── /ml-roboticist/
├── /graph/                 # Interactive knowledge graph
├── /search/                # Full-text search
└── /contribute/            # Contribution guide
```

---

## Page Templates

### 1. Term Page Layout

```
┌─────────────────────────────────────────────────────────────────┐
│ [Breadcrumb: nvidia-stack > isaac > isaac-ros]                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Isaac ROS                                              [v3.2]  │
│  ═══════════                                                    │
│                                                                 │
│  GPU-accelerated ROS 2 packages for robotics perception         │
│  and navigation, optimized for NVIDIA Jetson and GPUs.          │
│                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐                      │
│  │ Prerequisites   │  │ Related         │                      │
│  │ • ROS 2         │  │ • Isaac SIM     │                      │
│  │ • CUDA          │  │ • TensorRT      │                      │
│  │ • Jetson        │  │ • NITROS        │                      │
│  └─────────────────┘  └─────────────────┘                      │
│                                                                 │
│  ## Overview                                                    │
│  [Main content with diagrams, code examples...]                 │
│                                                                 │
│  ## Key Components                                              │
│  - isaac_ros_visual_slam                                        │
│  - isaac_ros_apriltag                                           │
│  - isaac_ros_dnn_inference                                      │
│                                                                 │
│  ## Architecture                                                │
│  [Diagram showing NITROS zero-copy pipeline]                    │
│                                                                 │
│  ## Code Example                                                │
│  ```python                                                      │
│  # Launch Isaac ROS Visual SLAM                                 │
│  ...                                                            │
│  ```                                                            │
│                                                                 │
│  ## Learn More                                                  │
│  - [Official Docs](...)                                         │
│  - [GitHub](...)                                                │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│  ← Previous: Isaac SIM          Next: Isaac Lab →               │
└─────────────────────────────────────────────────────────────────┘
```

### 2. Category Page Layout

```
┌─────────────────────────────────────────────────────────────────┐
│  NVIDIA Stack                                                   │
│  ════════════                                                   │
│                                                                 │
│  The complete NVIDIA robotics ecosystem: from Jetson hardware   │
│  to Isaac software to Omniverse simulation.                     │
│                                                                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  [Visual: Stack diagram showing layers]                    │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                 │
│  ## Subcategories                                               │
│                                                                 │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐               │
│  │   Jetson    │ │    Isaac    │ │  Omniverse  │               │
│  │  Hardware   │ │  Software   │ │ Simulation  │               │
│  │  12 terms   │ │  24 terms   │ │   8 terms   │               │
│  └─────────────┘ └─────────────┘ └─────────────┘               │
│                                                                 │
│  ## All Terms in This Category                                  │
│  [Filterable list with tags]                                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 3. Knowledge Graph Page

```
┌─────────────────────────────────────────────────────────────────┐
│  Knowledge Graph                                    [Filter ▼]  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│              ┌──────────┐                                       │
│         ┌────│  Jetson  │────┐                                  │
│         │    └──────────┘    │                                  │
│         ▼                    ▼                                  │
│   ┌──────────┐         ┌──────────┐                             │
│   │   CUDA   │─────────│Isaac ROS │                             │
│   └──────────┘         └──────────┘                             │
│         │                    │                                  │
│         ▼                    ▼                                  │
│   ┌──────────┐         ┌──────────┐                             │
│   │ TensorRT │─────────│   SLAM   │                             │
│   └──────────┘         └──────────┘                             │
│                                                                 │
│  [Interactive D3.js/Vis.js force-directed graph]                │
│  Click nodes to navigate, drag to explore                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Design System

### Color Palette

```css
/* Primary - NVIDIA Green inspired but distinct */
--color-primary: #76B900;        /* NVIDIA Green */
--color-primary-dark: #5A8F00;

/* Accent - Robotics Blue */
--color-accent: #0066CC;
--color-accent-light: #3399FF;

/* Category Colors */
--color-hardware: #FF6B35;       /* Orange */
--color-software: #4ECDC4;       /* Teal */
--color-nvidia: #76B900;         /* Green */
--color-ai-ml: #9B59B6;          /* Purple */
--color-perception: #3498DB;     /* Blue */
--color-control: #E74C3C;        /* Red */

/* Surfaces */
--color-bg: #0D1117;             /* Dark mode default */
--color-bg-light: #FFFFFF;
--color-surface: #161B22;
--color-surface-light: #F6F8FA;

/* Text */
--color-text: #E6EDF3;
--color-text-muted: #8B949E;
```

### Typography

```css
/* Headings - Technical, modern */
--font-display: 'Inter', 'SF Pro Display', system-ui;

/* Body - Readable */
--font-body: 'Inter', 'SF Pro Text', system-ui;

/* Code - Monospace */
--font-mono: 'JetBrains Mono', 'Fira Code', monospace;
```

### Key Interactions

1. **Keyboard Navigation**
   - `←` / `→` : Previous/Next term
   - `/` : Focus search
   - `g` then `h` : Go home
   - `g` then `g` : Go to graph

2. **Hover States**
   - Term links show preview tooltip with summary
   - Category badges highlight related terms

3. **Mobile Gestures**
   - Swipe left/right for navigation
   - Pull down to search

---

## Directory Structure (Implementation)

```
robotics-glossary/
├── .github/
│   ├── workflows/
│   │   └── deploy.yml          # GitHub Pages deployment
│   ├── ISSUE_TEMPLATE/
│   │   ├── new-term.yml
│   │   └── correction.yml
│   └── CONTRIBUTING.md
│
├── src/
│   ├── content/
│   │   ├── terms/              # All glossary entries (MDX)
│   │   │   ├── fundamentals/
│   │   │   ├── hardware/
│   │   │   ├── perception/
│   │   │   ├── planning-control/
│   │   │   ├── software/
│   │   │   ├── nvidia-stack/
│   │   │   ├── ai-ml/
│   │   │   └── operations/
│   │   ├── paths/              # Learning journeys
│   │   └── config.ts           # Content collections config
│   │
│   ├── components/
│   │   ├── TermCard.astro
│   │   ├── CategoryGrid.astro
│   │   ├── RelatedTerms.astro
│   │   ├── KnowledgeGraph.tsx  # React island for interactivity
│   │   ├── SearchModal.astro
│   │   ├── VersionBadge.astro
│   │   └── CodeBlock.astro
│   │
│   ├── layouts/
│   │   ├── BaseLayout.astro
│   │   ├── TermLayout.astro
│   │   └── CategoryLayout.astro
│   │
│   ├── pages/
│   │   ├── index.astro         # Landing page
│   │   ├── glossary/
│   │   │   └── [...slug].astro
│   │   ├── category/
│   │   │   └── [...slug].astro
│   │   ├── paths/
│   │   │   └── [...slug].astro
│   │   ├── graph.astro
│   │   └── search.astro
│   │
│   └── styles/
│       ├── global.css
│       └── variables.css
│
├── public/
│   ├── images/
│   │   ├── diagrams/
│   │   └── icons/
│   └── robots.txt
│
├── scripts/
│   ├── generate-graph.ts       # Build knowledge graph JSON
│   ├── validate-links.ts       # Check for broken links
│   └── generate-search.ts      # Build search index
│
├── astro.config.mjs
├── package.json
├── tsconfig.json
└── README.md
```

---

## Initial Content Roadmap

### Phase 1: Foundation (Week 1-2)
**~30 core terms to establish structure**

```
Fundamentals (5):
- degrees-of-freedom
- kinematics
- dynamics
- coordinate-frames
- transforms

Hardware (5):
- jetson-orin
- lidar
- depth-camera
- imu
- encoder

NVIDIA Stack (10):
- jetson
- isaac-ros
- isaac-sim
- isaac-lab
- omniverse
- cuda (link to Modal)
- tensorrt
- nitros
- cumotion
- nvblox

Software (5):
- ros2
- urdf
- gazebo
- moveit
- nav2

Perception (5):
- slam
- visual-odometry
- point-cloud
- semantic-segmentation
- object-detection
```

### Phase 2: Depth (Week 3-4)
**~50 additional terms with richer content**

- Deep dives on Jetson variants (Xavier, Orin, Thor)
- Isaac ROS package breakdown
- Complete sensor taxonomy
- Control theory fundamentals

### Phase 3: Community (Ongoing)
- Open for contributions
- Bi-weekly new term additions
- Version updates for NVIDIA releases

---

## GitHub Pages Deployment

```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm
      - run: npm ci
      - run: npm run build
      - uses: actions/upload-pages-artifact@v3
        with:
          path: dist/

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/deploy-pages@v4
        id: deployment
```

---

## Success Metrics

1. **Discoverability**: First page of Google for "robotics glossary", "NVIDIA Isaac glossary"
2. **Engagement**: >3 min average session, >3 pages per session
3. **Community**: >100 GitHub stars in first month, >10 contributors
4. **Completeness**: >200 terms within 6 months
5. **Authority**: Linked from NVIDIA docs, ROS wiki, robotics courses

---

## Next Steps

1. [ ] Initialize Astro project with Starlight
2. [ ] Set up GitHub repo structure
3. [ ] Create base components and layouts
4. [ ] Write first 10 terms as templates
5. [ ] Deploy to GitHub Pages
6. [ ] Create contribution guidelines
7. [ ] Announce on robotics communities

---

*This plan aims to create the definitive robotics glossary — better than Modal's GPU Glossary through richer media, better navigation, learning paths, and comprehensive NVIDIA ecosystem coverage.*
