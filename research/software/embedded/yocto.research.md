# Yocto for Robotics Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/software/embedded/yocto.mdx
**Description:** Custom Linux distributions for Jetson and embedded robotics

---

## Overview

The Yocto Project is a Linux Foundation collaborative open source project that produces tools and processes enabling the creation of custom Linux distributions for embedded and IoT software. It is hardware-architecture independent and enables highly customizable, repeatable builds. For robotics, Yocto allows building production-grade operating systems optimized for specific robot hardware with only necessary components included.

## Current Version Information

### Yocto Project Releases (January 2026)

| Release | Codename | Status | Support Until |
|---------|----------|--------|---------------|
| 5.0 | Scarthgap | **LTS** (Current) | April 2028 |
| 5.1 | Styhead | Stable | ~October 2026 |
| 4.0 | Kirkstone | LTS | April 2026 |
| 3.1 | Dunfell | LTS (EOL) | April 2024 |

### Scarthgap 5.0 Key Components
- Linux kernel 6.6
- GCC 13.2
- glibc 2.39
- LLVM 18.1
- Over 300 recipe upgrades from previous release

**Note:** Major releases occur every ~6 months (April and October). LTS releases every 2 years with 4-year support.

### meta-tegra for Jetson (January 2026)
- Current support: **L4T R36.4.4** / **JetPack 6.2.1**
- CUDA 12.6 support (requires gcc 10 toolchain for CUDA compilation)
- Supported branch: `scarthgap` (aligns with Yocto 5.0)

### meta-ros for ROS 2 (January 2026)
- Supported ROS 2 distros: **Jazzy (LTS)**, Humble (LTS), Iron, Rolling
- Recommended combination: Scarthgap (Yocto 5.0) + Jazzy (ROS 2)
- Alternative: Kirkstone (Yocto 4.0) + Humble (ROS 2)

## Key Concepts to Cover

### 1. Core Yocto Components

#### Poky Reference Distribution
- Reference implementation containing OpenEmbedded build system
- Includes BitBake build engine and core metadata
- Template for creating custom distributions
- NOT a distribution itself — a toolkit for building distributions

#### BitBake Build Engine
- Task scheduler and execution engine
- Parses recipes (.bb files) and configuration
- Handles dependency resolution and parallel builds
- Shared between Yocto Project and OpenEmbedded

#### OpenEmbedded-Core (OE-Core)
- Base layer containing core recipes
- Essential packages for building minimal Linux images
- Hardware-agnostic base metadata

### 2. Layer Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Yocto Layer Stack (Robotics)                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │              Your Custom Layer (meta-myrobot)                  │ │
│  │         - Robot-specific recipes and configurations            │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                               │                                      │
│  ┌─────────────────┐  ┌───────┴───────┐  ┌─────────────────────┐   │
│  │   meta-ros2     │  │ meta-tegra    │  │ meta-tegra-community│   │
│  │   (ROS 2)       │  │ (Jetson BSP)  │  │ (extras)            │   │
│  └─────────────────┘  └───────────────┘  └─────────────────────┘   │
│                               │                                      │
│  ┌────────────────────────────┴────────────────────────────────┐   │
│  │                   openembedded-core (OE-Core)                │   │
│  │              - Base recipes, classes, tools                  │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                               │                                      │
│  ┌────────────────────────────┴────────────────────────────────┐   │
│  │                       Poky                                   │   │
│  │              - Reference distribution                        │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

#### Layer Naming Convention
- `meta-*` prefix for all layers
- BSP layers: `meta-tegra`, `meta-raspberrypi`, `meta-intel`
- Software layers: `meta-ros`, `meta-qt6`
- Distro layers: `meta-poky`

#### Layer Configuration (layer.conf)
- BBFILE_PRIORITY: Determines override precedence (higher wins)
- LAYERDEPENDS: Specifies required layers
- LAYERSERIES_COMPAT: Compatible Yocto releases

### 3. Configuration Files

#### bblayers.conf
```bash
# Example bblayers.conf for Jetson robotics build
BBLAYERS ?= " \
  /path/to/poky/meta \
  /path/to/poky/meta-poky \
  /path/to/meta-openembedded/meta-oe \
  /path/to/meta-openembedded/meta-python \
  /path/to/meta-tegra \
  /path/to/meta-ros/meta-ros2 \
  /path/to/meta-ros/meta-ros2-jazzy \
  /path/to/meta-myrobot \
"
```

#### local.conf
```bash
# Example local.conf for Jetson Orin robotics
MACHINE = "jetson-agx-orin-devkit"
DISTRO = "poky"

# Enable ROS 2 Jazzy
ROS_DISTRO = "jazzy"

# Parallelism
BB_NUMBER_THREADS = "8"
PARALLEL_MAKE = "-j 8"

# Image features
IMAGE_INSTALL:append = " ros-jazzy-ros-base ros-jazzy-nav2-bringup"

# Package formats
PACKAGE_CLASSES = "package_deb"

# Accept NVIDIA licenses for CUDA components
LICENSE_FLAGS_ACCEPTED = "commercial_nvidia"
```

### 4. NVIDIA Jetson Integration (meta-tegra)

#### Supported Platforms (January 2026)
- Jetson AGX Orin (64GB, 32GB)
- Jetson AGX Orin Industrial
- Jetson Orin NX (16GB, 8GB)
- Jetson Orin Nano (8GB, 4GB)

#### Machine Names
```
jetson-agx-orin-devkit
jetson-orin-nano-devkit-nvme
jetson-orin-nx-xavier-nx-devkit
```

#### CUDA Integration Notes
- NVIDIA provides CUDA as Ubuntu packages only (no source)
- meta-tegra deconstructs packages for Yocto integration
- Requires gcc 10 toolchain (CUDA 12.6 max compatible compiler)
- cuda.bbclass handles nvcc compilation flags

#### tegra-demo-distro
- Reference distribution demonstrating meta-tegra usage
- Includes setup scripts and working examples
- Supports A/B rootfs updates via swupdate
- Good starting point for Jetson-based robots

### 5. ROS 2 Integration (meta-ros)

#### Branch/Release Mapping
| Yocto Release | ROS 2 Distro | Support Status |
|---------------|--------------|----------------|
| Scarthgap (5.0) | Jazzy | Through April 2028 |
| Kirkstone (4.0) | Humble | Through April 2026 |
| Styhead (5.1) | Jazzy, Rolling | Current development |

#### Adding ROS 2 Packages to Image
```bash
# In local.conf or image recipe
IMAGE_INSTALL:append = " \
    ros-jazzy-ros-base \
    ros-jazzy-rclcpp \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
"
```

#### Key ROS 2 Recipes Available
- ros-jazzy-ros-base: Core ROS 2 runtime
- ros-jazzy-nav2-*: Navigation stack
- ros-jazzy-moveit2: Motion planning
- ros-jazzy-isaac-ros-*: NVIDIA Isaac ROS packages

### 6. Build Automation with kas

kas is a Python-based tool by Siemens that simplifies Yocto builds through YAML configuration.

#### Example kas Configuration (jetson-robot.yml)
```yaml
header:
  version: 14
  includes:
    - repo: meta-tegra-community
      file: kas/machine/jetson-agx-orin-devkit.yml

machine: jetson-agx-orin-devkit
distro: poky

repos:
  poky:
    url: "https://git.yoctoproject.org/poky"
    refspec: scarthgap
    layers:
      meta:
      meta-poky:

  meta-openembedded:
    url: "https://git.openembedded.org/meta-openembedded"
    refspec: scarthgap
    layers:
      meta-oe:
      meta-python:
      meta-networking:

  meta-tegra:
    url: "https://github.com/OE4T/meta-tegra.git"
    refspec: scarthgap

  meta-ros:
    url: "https://github.com/ros/meta-ros.git"
    refspec: scarthgap
    layers:
      meta-ros2:
      meta-ros2-jazzy:

local_conf_header:
  robotics: |
    ROS_DISTRO = "jazzy"
    IMAGE_INSTALL:append = " ros-jazzy-ros-base"
    LICENSE_FLAGS_ACCEPTED = "commercial_nvidia"
```

#### kas Commands
```bash
# Build image
kas build jetson-robot.yml

# Enter shell for manual bitbake commands
kas shell jetson-robot.yml

# Checkout repos only (no build)
kas checkout jetson-robot.yml
```

### 7. Real-Time Linux Support (PREEMPT_RT)

#### Enabling PREEMPT_RT Kernel
```bash
# In local.conf
PREFERRED_PROVIDER_virtual/kernel = "linux-yocto-rt"
# Or for existing kernel:
LINUX_KERNEL_TYPE = "preempt-rt"
```

#### Why Real-Time Matters for Robotics
- Deterministic control loop timing
- Consistent sensor polling intervals
- Predictable actuator response
- Required for safety-critical applications

#### Testing Real-Time Performance
- Use `cyclictest` tool for latency measurements
- Disable power management features for consistency
- Consider disabling graphics drivers if not needed

### 8. Development Workflow with devtool

#### Key devtool Commands
```bash
# Add new recipe for custom ROS package
devtool add myrobot-pkg /path/to/source

# Modify existing recipe for debugging
devtool modify ros-jazzy-nav2-bringup

# Upgrade recipe to new version
devtool upgrade ros-jazzy-slam-toolbox

# Finish and incorporate changes
devtool finish myrobot-pkg meta-myrobot
```

#### IDE Integration
```bash
# Generate VSCode configuration
devtool ide-sdk ros-jazzy-rclcpp

# Shared SDK mode for cross-compilation
devtool ide-sdk --mode=shared
```

### 9. Extensible SDK (eSDK)

The eSDK provides cross-development toolchain tailored to your image:

```bash
# Generate SDK installer
bitbake core-image-minimal -c populate_sdk_ext

# Install SDK
./poky-glibc-x86_64-core-image-minimal-aarch64-jetson-agx-orin-toolchain-ext-5.0.sh

# Source environment
source /opt/poky/5.0/environment-setup-aarch64-poky-linux
```

## Code Examples to Include

### 1. Minimal Jetson Robotics Build Setup
```bash
# Clone required repositories
git clone -b scarthgap https://git.yoctoproject.org/poky
git clone -b scarthgap https://github.com/OE4T/meta-tegra.git
git clone -b scarthgap https://github.com/ros/meta-ros.git
git clone -b scarthgap https://git.openembedded.org/meta-openembedded

# Initialize build environment
source poky/oe-init-build-env build

# Configure bblayers.conf and local.conf (see examples above)

# Build image
bitbake core-image-minimal
```

### 2. Custom Robot Layer Creation
```bash
# Create layer structure
bitbake-layers create-layer meta-myrobot
bitbake-layers add-layer meta-myrobot

# Layer structure:
# meta-myrobot/
# ├── conf/
# │   └── layer.conf
# ├── recipes-robot/
# │   └── myrobot-bringup/
# │       └── myrobot-bringup_1.0.bb
# └── recipes-images/
#     └── myrobot-image.bb
```

### 3. Robot Image Recipe
```bitbake
# recipes-images/myrobot-image.bb
SUMMARY = "Custom robot image with ROS 2 and navigation"
LICENSE = "MIT"

inherit core-image

IMAGE_FEATURES += "ssh-server-openssh"

IMAGE_INSTALL = " \
    packagegroup-core-boot \
    ${CORE_IMAGE_EXTRA_INSTALL} \
    ros-jazzy-ros-base \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    myrobot-bringup \
"

# Increase rootfs size for ROS packages
IMAGE_ROOTFS_EXTRA_SPACE = "2097152"
```

### 4. Custom ROS Package Recipe
```bitbake
# recipes-robot/myrobot-bringup/myrobot-bringup_1.0.bb
SUMMARY = "Robot bringup launch files and configuration"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://LICENSE;md5=..."

inherit ros_ament_cmake

DEPENDS = " \
    rclcpp \
    nav2-bringup \
    robot-state-publisher \
"

SRC_URI = "git://github.com/myorg/myrobot-bringup.git;branch=main;protocol=https"
SRCREV = "${AUTOREV}"
S = "${WORKDIR}/git"
```

## Diagrams Needed

### 1. Yocto Build Flow
```
┌─────────────────────────────────────────────────────────────────────┐
│                       Yocto Build Process                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐                                                    │
│  │   Recipes   │──────┐                                             │
│  │   (.bb)     │      │                                             │
│  └─────────────┘      │                                             │
│                       ▼                                              │
│  ┌─────────────┐  ┌──────────┐  ┌───────────┐  ┌───────────────┐   │
│  │   Config    │─►│ BitBake  │─►│  Fetch/   │─►│   Compile/    │   │
│  │ (local.conf)│  │ Parser   │  │  Unpack   │  │   Package     │   │
│  └─────────────┘  └──────────┘  └───────────┘  └───────┬───────┘   │
│                                                         │            │
│  ┌─────────────┐                               ┌───────▼───────┐   │
│  │   Layers    │                               │  Image Gen    │   │
│  │(bblayers.conf)                              │ (.wic/.tar)   │   │
│  └─────────────┘                               └───────────────┘   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2. Robotics Layer Integration
```
┌────────────────────────────────────────────────────────────────┐
│           Yocto for Robotics: Layer Integration                 │
├────────────────────────────────────────────────────────────────┤
│                                                                 │
│    Application Layer                                            │
│    ┌──────────────────────────────────────────────────────┐    │
│    │ meta-myrobot (your custom robot packages)            │    │
│    └──────────────────────────────────────────────────────┘    │
│                            │                                    │
│    Middleware Layer                                             │
│    ┌─────────────────┐    │    ┌─────────────────────────┐    │
│    │ meta-ros2-jazzy │◄───┼───►│ meta-tegra-community    │    │
│    │ (ROS 2 runtime) │    │    │ (extra Jetson packages) │    │
│    └─────────────────┘    │    └─────────────────────────┘    │
│                            │                                    │
│    BSP Layer                                                    │
│    ┌──────────────────────────────────────────────────────┐    │
│    │ meta-tegra (Jetson Linux, CUDA, TensorRT)            │    │
│    └──────────────────────────────────────────────────────┘    │
│                            │                                    │
│    Core Layer                                                   │
│    ┌──────────────────────────────────────────────────────┐    │
│    │ openembedded-core / poky                             │    │
│    └──────────────────────────────────────────────────────┘    │
│                                                                 │
└────────────────────────────────────────────────────────────────┘
```

### 3. Jetson Deployment Flow
```
┌─────────────────────────────────────────────────────────────────────┐
│                 Yocto → Jetson Deployment Flow                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │
│  │   bitbake   │───►│   .wic      │───►│   Flash     │             │
│  │   build     │    │   image     │    │   Tool      │             │
│  └─────────────┘    └─────────────┘    └──────┬──────┘             │
│                                                │                     │
│                                                ▼                     │
│  Recovery Mode USB ──────────────────► ┌─────────────────┐         │
│                                         │  Jetson Orin   │         │
│                                         │  (eMMC/NVMe)   │         │
│                                         └─────────────────┘         │
│                                                                      │
│  Alternative: A/B Updates via swupdate (OTA)                        │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## Prerequisites / Related Terms

### Must-Know Prerequisites
- **Linux basics**: Command line, package management, filesystem structure
- **Embedded systems concepts**: Cross-compilation, bootloaders, device trees
- **ROS 2**: If building ROS-based robot images

### Related Glossary Entries
- **Jetson Orin** - Primary deployment target for robotics
- **ROS 2** - Robot Operating System integrated via meta-ros
- **Isaac ROS** - NVIDIA's GPU-accelerated ROS packages
- **TensorRT** - Inference runtime included in Jetson images

## Source URLs for Citations

### Official Yocto Project
- [Yocto Project Homepage](https://www.yoctoproject.org/)
- [Yocto Project Documentation](https://docs.yoctoproject.org/)
- [Releases Schedule](https://wiki.yoctoproject.org/wiki/Releases)
- [Creating Custom Distributions](https://docs.yoctoproject.org/dev-manual/custom-distribution.html)
- [Understanding Layers](https://docs.yoctoproject.org/dev-manual/layers.html)
- [Scarthgap 5.0 Release Notes](https://docs.yoctoproject.org/migration-guides/release-notes-5.0.html)
- [devtool Reference](https://docs.yoctoproject.org/ref-manual/devtool-reference.html)
- [Extensible SDK Manual](https://docs.yoctoproject.org/sdk-manual/extensible.html)

### meta-tegra (Jetson Support)
- [meta-tegra GitHub](https://github.com/OE4T/meta-tegra)
- [tegra-demo-distro GitHub](https://github.com/OE4T/tegra-demo-distro)
- [OE4T Organization](https://github.com/oe4t)
- [OpenEmbedded Layer Index - meta-tegra](https://layers.openembedded.org/layerindex/branch/scarthgap/layer/meta-tegra/)
- [RidgeRun Yocto for Jetson Guide](https://developer.ridgerun.com/wiki/index.php/Yocto_Support_for_NVIDIA_Jetson_Platforms)

### meta-ros (ROS 2 Support)
- [meta-ros GitHub](https://github.com/ros/meta-ros)
- [meta-ros Release Plan](https://github.com/ros/meta-ros/issues/1195)
- [ROS Discourse - meta-ros Discussion](https://discourse.openrobotics.org/t/ros2-yocto-meta-layer/9643)

### kas Build Tool
- [kas GitHub](https://github.com/siemens/kas)
- [kas Documentation](https://kas.readthedocs.io/)
- [Setting Up Yocto with kas](https://embeddeduse.com/2022/06/24/setting-up-yocto-projects-with-kas/)

### Real-Time Linux
- [Yocto PREEMPT_RT Guide](https://docs.yoctoproject.org/2.3/kernel-dev/kernel-dev.html)
- [Building RT Image with Yocto (FOSDEM)](https://archive.fosdem.org/2018/schedule/event/rt_linux_with_yocto/attachments/slides/2684/export/events/attachments/rt_linux_with_yocto/slides/2684/Yocto_RT.pdf)

### Tutorials and Guides
- [Yocto for RobotOps Tutorial](https://www.robotsops.com/comprehensive-tutorial-yocto-project-in-robotops/)
- [Yocto Project Embedded Development Guide](https://theembeddedkit.io/yocto/)
- [Witekio: Yocto for NVIDIA Jetson](https://witekio.com/blog/yocto-for-nvidia-jetson/)
- [Acceleration Robotics: Yocto + ROS 2](https://news.accelerationrobotics.com/yocto-ros2-hardware-acceleration/)

## Notes for Content Creation

### Badge/Level
- Suggest: **Advanced** - Yocto has a steep learning curve and targets production deployments

### Tone
- Emphasize practical robotics use cases (not general embedded)
- Focus on Jetson + ROS 2 combination as primary example
- Include working code snippets that can be adapted

### Key Points to Highlight
1. **Why Yocto over stock JetPack**: Reproducibility, minimal footprint, custom kernel options
2. **Layer architecture**: Understanding layers is fundamental
3. **meta-tegra + meta-ros combination**: The robotics-specific stack
4. **kas for automation**: Modern workflow improvement
5. **Real-time support**: Important for robot control

### Structure (based on existing entries)
1. Frontmatter with title, description, last_validated, sidebar badge (Advanced)
2. Imports for Astro components
3. Level badge span
4. Bold intro paragraph
5. Aside note about complexity/learning curve
6. Prerequisites CardGrid
7. "Why Yocto for Robotics" bullet list
8. Layer architecture diagram
9. Core Concepts sections (Layers, Configuration, meta-tegra, meta-ros)
10. Code examples with Tabs (kas config, bitbake commands, recipes)
11. Jetson Deployment section
12. Real-Time Linux section
13. Related Terms CardGrid
14. Learn More links
15. Sources section

### Potential Asides
- Warning about build times (initial build can take hours)
- Note about NVIDIA license acceptance for CUDA
- Tip about using kas for reproducibility
- Caution about engine-specific dependencies (like TensorRT builds)
