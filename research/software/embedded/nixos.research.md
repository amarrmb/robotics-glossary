# NixOS for Robotics Research Notes

**Research Date:** 2026-01-21
**Target File:** src/content/docs/software/embedded/nixos.mdx
**Description:** Reproducible, declarative OS for robotics deployments

---

## Overview

NixOS is a Linux distribution built around the Nix package manager that takes a fully declarative approach to system configuration and package management. For robotics, NixOS enables reproducible, immutable system deployments where configurations can be version-controlled and reliably deployed across fleets of robots with identical behavior.

### Core Philosophy

- **Declarative Configuration**: The entire system is defined in configuration files (Nix expressions)
- **Reproducibility**: Same inputs produce identical outputs across machines and time
- **Immutability**: Packages in `/nix/store` cannot be modified after build
- **Atomic Updates**: System changes are atomic with instant rollback capability
- **Isolation**: Packages built in sandboxes with explicit dependencies

## Current Version Information

### NixOS Releases (January 2026)

| Release | Status | Support | Key Features |
|---------|--------|---------|--------------|
| 25.05 | **Stable** | Current | Latest packages, CUDA 12.8 |
| 24.11 | Stable | Until May 2026 | Production recommended |
| 24.05 | Oldstable | Until Nov 2025 | LTS-like stability |
| Unstable | Rolling | Continuous | Bleeding edge |

### Nix Package Manager Version
- Current stable: **Nix 2.24.x**
- Flakes feature: Experimental but widely adopted
- Deterministic builds via flake.lock

### Key Components for Robotics

| Component | Version/Status | Purpose |
|-----------|----------------|---------|
| nix-ros-overlay | Active (2025) | ROS/ROS 2 packages for Nix |
| jetpack-nixos | JetPack 5/6/7 | NVIDIA Jetson support |
| nixpkgs CUDA | CUDA 12.8 | GPU compute support |

## Key Concepts to Cover

### 1. Nix vs NixOS

#### Nix (Package Manager)
- Can run on any Linux distribution, macOS
- Provides isolated, reproducible package management
- Creates development environments with `nix-shell` or `nix develop`

#### NixOS (Operating System)
- Full Linux distribution using Nix for system configuration
- Entire OS defined in `/etc/nixos/configuration.nix`
- System-wide reproducibility and atomic updates

### 2. Nix Flakes

Flakes are the modern way to define Nix projects with:
- **flake.nix**: Project definition with inputs and outputs
- **flake.lock**: Pinned dependency versions for reproducibility
- Standardized project structure
- Better composability between projects

#### Example flake.nix for ROS 2
```nix
{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "ros2-dev";
          packages = [
            pkgs.colcon
            (with pkgs.rosPackages.jazzy; buildEnv {
              paths = [ ros-core nav2-bringup ];
            })
          ];
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
```

### 3. nix-ros-overlay

Primary project for running ROS/ROS 2 on NixOS, maintained by lopsided98.

#### Current Status (January 2026)
- **ROS 1 & Gazebo Classic**: Removed from master/develop after EOL (Jan/May 2025)
- **ROS 1 Legacy**: Available in `ros1-25.05` branch until end of 2025
- **ROS 2 Support**: Jazzy, Humble (via Kirkstone), Rolling
- **Platforms**: x86_64-linux, aarch64-linux

#### Supported ROS 2 Distributions
| ROS 2 Distro | nix-ros-overlay Branch | Status |
|--------------|------------------------|--------|
| Jazzy | master/develop | Primary support |
| Humble | develop | Supported |
| Rolling | develop | Development |

#### Quick Start
```bash
# With flakes enabled
nix develop github:lopsided98/nix-ros-overlay/master#example-ros2-desktop-jazzy

# Test ROS 2
ros2 launch demo_nodes_cpp talker_listener_launch.xml
```

#### Binary Cache
Prebuilt packages available on Cachix:
```bash
cachix use ros
# Or manually add to nix.conf:
# substituters = https://ros.cachix.org
# trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=
```

### 4. NVIDIA Jetson Support (jetpack-nixos)

Maintained by Anduril, enables NixOS on NVIDIA Jetson devices.

#### Supported Versions
| JetPack | Status | Notes |
|---------|--------|-------|
| JetPack 7 (Thor AGX) | Supported | CUDA 13 pending nixpkgs 25.11 |
| JetPack 6 | Supported | Full CUDA support |
| JetPack 5 | Supported | Full CUDA support |

#### Supported Devices
- Jetson AGX Orin (64GB/32GB)
- Jetson Orin NX (16GB/8GB)
- Jetson Orin Nano (8GB/4GB)
- **NOT Supported**: Jetson Nano, TX2, TX1 (dropped in JetPack 5)

#### Key Configuration
```nix
# configuration.nix for Jetson Orin
{ config, pkgs, ... }:

{
  imports = [
    (builtins.fetchTarball {
      url = "https://github.com/anduril/jetpack-nixos/archive/master.tar.gz";
    })
  ];

  hardware.nvidia-jetpack = {
    enable = true;
    som = "orin-agx";
    carrierBoard = "devkit";
    # Optional: disable automatic CUDA configuration
    # configureCuda = false;
  };

  # Mandatory for Jetson
  config.cudaCapabilities = [ "8.7" ];  # Orin capability
}
```

#### CUDA Notes
- nixpkgs auto-configured for CUDA when using jetpack-nixos modules
- `config.cudaCapabilities` is **mandatory** for Jetsons (not in defaults)
- CUDA 13 support for JetPack 7 awaiting nixpkgs 25.11

### 5. NVIDIA CUDA on NixOS

#### Enabling CUDA
```nix
# In configuration.nix
{
  services.xserver.videoDrivers = [ "nvidia" ];

  # Driver 560+: choose open or proprietary
  hardware.nvidia.open = true;  # Turing+ (RTX 20, GTX 16 series onwards)

  # Enable unfree packages for CUDA
  nixpkgs.config.allowUnfree = true;

  # Or for specific CUDA packages
  nixpkgs.config.cudaSupport = true;
}
```

#### CUDA Version (January 2026)
- Default in nixpkgs unstable: CUDA 12.8
- Compatibility: PyTorch, Numba handle version differences gracefully

### 6. Cross-Compilation for ARM/Embedded

NixOS supports cross-compilation via `pkgsCross`:

#### Available Targets for Robotics
```bash
# Cross-compile hello for aarch64
nix-build '<nixpkgs>' -A pkgsCross.aarch64-multiplatform.hello

# Common targets:
# pkgsCross.aarch64-multiplatform     - General ARM64
# pkgsCross.aarch64-multiplatform-musl - ARM64 with musl libc
# pkgsCross.aarch64-embedded          - ARM64 bare-metal/minimal
# pkgsCross.armv7l-hf-multiplatform   - ARM32 (Raspberry Pi)
```

#### Cross-Compilation Dev Shell
```nix
let pkgs = import <nixpkgs> {
  crossSystem = { config = "aarch64-unknown-linux-gnu"; };
};
in pkgs.callPackage (
  { mkShell, pkg-config, zlib }: mkShell {
    nativeBuildInputs = [ pkg-config ];  # Build tools
    buildInputs = [ zlib ];              # Target libraries
  }
) {}
```

#### binfmt Emulation
```nix
# configuration.nix - run aarch64 binaries on x86_64
{ boot.binfmt.emulatedSystems = [ "aarch64-linux" ]; }
```

### 7. Fleet Management

Tools for managing multiple NixOS robots:

#### Bento (KISS Approach)
- Simple sftp-based configuration distribution
- Each client polls for changes and runs `nixos-rebuild`
- Good for small fleets

#### Fleet (Cluster Tool)
- Manages multiple hosts at once
- Secure secrets storage with git encryption
- Automatic secret regeneration

#### Colmena / morph / deploy-rs
- Modern deployment tools for NixOS fleets
- Support for parallel deployments
- Rolling updates with health checks

### 8. Comparison: NixOS vs Docker for Robotics

| Aspect | NixOS | Docker |
|--------|-------|--------|
| **Reproducibility** | Build-time deterministic | Runtime reproducible |
| **Isolation** | Nix store paths | Container namespaces |
| **Overhead** | Native performance | Container runtime overhead |
| **Portability** | NixOS required for full benefits | Any container runtime |
| **Configuration** | Declarative (Nix language) | Dockerfile + compose |
| **Rollback** | Instant (generations) | Requires image management |
| **Learning Curve** | Steep (Nix language) | Moderate (familiar concepts) |
| **Embedded Fit** | Excellent (no runtime) | Good (needs container runtime) |

#### Best Practice: Use Both
- Nix for reproducible Docker image builds
- Docker for deployment portability
- Nix provides deterministic inputs, Docker provides runtime isolation

### 9. Why NixOS for Robotics?

#### Reproducibility
- Identical system configuration across entire robot fleet
- Version-controlled infrastructure (GitOps)
- "Works on my machine" eliminated

#### Immutability & Rollback
- Atomic system updates
- Instant rollback to previous generations
- Safe deployment of updates to production robots

#### Dependency Management
- No dependency conflicts between packages
- Multiple versions can coexist
- Isolated development environments

#### Declarative Configuration
- Entire robot system defined in code
- Easy auditing and compliance
- Self-documenting infrastructure

#### Production Benefits
- Minimal attack surface (only needed packages)
- Reproducible security patches
- Offline deployments via binary caches

## Code Examples to Include

### 1. Basic NixOS Configuration for ROS 2 Robot
```nix
# /etc/nixos/configuration.nix
{ config, pkgs, ... }:

{
  imports = [ ./hardware-configuration.nix ];

  # Enable flakes
  nix.settings.experimental-features = [ "nix-command" "flakes" ];

  # System packages
  environment.systemPackages = with pkgs; [
    git vim htop
    # ROS 2 packages via overlay
  ];

  # Networking for ROS 2 DDS
  networking = {
    hostName = "robot-01";
    firewall.allowedUDPPorts = [ 7400 7401 ]; # DDS discovery
  };

  # Real-time kernel for control loops
  boot.kernelPackages = pkgs.linuxPackages_rt;

  system.stateVersion = "24.11";
}
```

### 2. Robot Development Shell (flake.nix)
```nix
{
  description = "Robot development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
  };

  outputs = { self, nixpkgs, nix-ros-overlay }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ nix-ros-overlay.overlays.default ];
      };
    in {
      devShells.${system}.default = pkgs.mkShell {
        name = "robot-dev";

        buildInputs = with pkgs; [
          colcon
          (rosPackages.jazzy.buildEnv {
            paths = with rosPackages.jazzy; [
              ros-core
              nav2-bringup
              slam-toolbox
              rviz2
            ];
          })
        ];

        shellHook = ''
          echo "ROS 2 Jazzy development environment"
          echo "Run: ros2 launch nav2_bringup navigation_launch.py"
        '';
      };
    };

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
```

### 3. Jetson Orin NixOS Configuration
```nix
# flake.nix for Jetson Orin robot
{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    jetpack-nixos.url = "github:anduril/jetpack-nixos";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
  };

  outputs = { self, nixpkgs, jetpack-nixos, nix-ros-overlay }:
    let
      system = "aarch64-linux";
    in {
      nixosConfigurations.robot-orin = nixpkgs.lib.nixosSystem {
        inherit system;
        modules = [
          jetpack-nixos.nixosModules.default
          ({ config, pkgs, ... }: {
            nixpkgs.overlays = [ nix-ros-overlay.overlays.default ];

            hardware.nvidia-jetpack = {
              enable = true;
              som = "orin-agx";
              carrierBoard = "devkit";
            };

            # CUDA capability for Orin
            config.cudaCapabilities = [ "8.7" ];

            environment.systemPackages = with pkgs; [
              (rosPackages.jazzy.buildEnv {
                paths = with rosPackages.jazzy; [
                  ros-core
                  # isaac-ros packages when available
                ];
              })
            ];

            system.stateVersion = "24.11";
          })
        ];
      };
    };
}
```

## Diagrams Needed

### 1. NixOS Architecture Overview
```
┌─────────────────────────────────────────────────────────────────┐
│                    NixOS System Architecture                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              /etc/nixos/configuration.nix                │   │
│  │              (Declarative System Definition)             │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    nixos-rebuild                         │   │
│  │         (Build & Activate System Configuration)          │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                   │
│            ┌─────────────────┼─────────────────┐                │
│            ▼                 ▼                 ▼                │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐      │
│  │ Generation 1 │  │ Generation 2 │  │ Generation N     │      │
│  │ (rollback)   │  │ (previous)   │  │ (current/active) │      │
│  └──────────────┘  └──────────────┘  └──────────────────┘      │
│                              │                                   │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                     /nix/store                           │   │
│  │  (Immutable Package Store - Content-Addressed)           │   │
│  │  /nix/store/abc123-ros-jazzy-nav2/                      │   │
│  │  /nix/store/def456-python3-3.11/                        │   │
│  │  /nix/store/ghi789-linux-6.6-rt/                        │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2. NixOS for Robotics Stack
```
┌─────────────────────────────────────────────────────────────────┐
│              NixOS Robotics Software Stack                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Application Layer                                               │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ Your Robot Application (ROS 2 packages, custom nodes)   │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                   │
│  Middleware Layer                                                │
│  ┌──────────────────┐  ┌──────────────────────────────────┐    │
│  │  nix-ros-overlay │  │  NVIDIA Packages (via jetpack)   │    │
│  │  (ROS 2 Jazzy)   │  │  CUDA, TensorRT, cuDNN           │    │
│  └──────────────────┘  └──────────────────────────────────┘    │
│                              │                                   │
│  Platform Layer                                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ jetpack-nixos (Jetson) / nixpkgs (x86_64)               │   │
│  │ Kernel, drivers, system libraries                        │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                   │
│  Infrastructure Layer                                            │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    NixOS Base                            │   │
│  │  Nix package manager, systemd, networking                │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 3. Fleet Deployment Flow
```
┌─────────────────────────────────────────────────────────────────┐
│              NixOS Fleet Deployment Flow                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐                                               │
│  │  Git Repo    │◄── Developer pushes configuration changes     │
│  │  (flake.nix) │                                               │
│  └──────┬───────┘                                               │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────┐    ┌────────────────┐                        │
│  │   CI/CD      │───►│  Binary Cache  │                        │
│  │  (GitHub     │    │  (Cachix/S3)   │                        │
│  │   Actions)   │    └───────┬────────┘                        │
│  └──────────────┘            │                                   │
│                              │                                   │
│         ┌────────────────────┼────────────────────┐             │
│         ▼                    ▼                    ▼             │
│  ┌────────────┐      ┌────────────┐      ┌────────────┐        │
│  │  Robot 1   │      │  Robot 2   │      │  Robot N   │        │
│  │  (pull &   │      │  (pull &   │      │  (pull &   │        │
│  │  rebuild)  │      │  rebuild)  │      │  rebuild)  │        │
│  └────────────┘      └────────────┘      └────────────┘        │
│                                                                  │
│  Each robot: nixos-rebuild switch --flake github:org/repo       │
│  Rollback:   nixos-rebuild switch --rollback                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Prerequisites / Related Terms

### Must-Know Prerequisites
- **Linux basics**: Command line, package management, filesystems
- **Version control (Git)**: NixOS configurations are typically version-controlled
- **Functional programming concepts**: Helpful for understanding Nix language
- **ROS 2**: If using nix-ros-overlay for robotics

### Related Glossary Entries
- **Yocto** - Alternative embedded Linux approach (build-time vs runtime reproducibility)
- **ROS 2** - Robot Operating System, primary middleware for nix-ros-overlay
- **Jetson Orin** - Primary NVIDIA hardware target for jetpack-nixos
- **Isaac ROS** - NVIDIA's GPU-accelerated ROS packages
- **Docker** - Complementary containerization technology

### Comparison with Yocto

| Aspect | NixOS | Yocto |
|--------|-------|-------|
| Approach | Declarative OS + Package Manager | Build system for custom distros |
| Reproducibility | Runtime & build-time | Build-time |
| Learning curve | Steep (Nix language) | Very steep (BitBake, layers) |
| Build time | Fast (binary cache) | Slow (hours for initial build) |
| Flexibility | High (any package combination) | Very high (BSP customization) |
| Production use | Growing adoption | Industry standard |
| Best for | Fleet management, development | Custom embedded images |

## Source URLs for Citations

### Official NixOS/Nix
- [NixOS Homepage](https://nixos.org/) - Official project page
- [NixOS Manual](https://nixos.org/manual/nixos/stable/) - System documentation
- [Nix Manual](https://nixos.org/manual/nix/stable/) - Package manager documentation
- [NixOS Wiki - Flakes](https://wiki.nixos.org/wiki/Flakes) - Flakes documentation
- [NixOS Reproducible Builds](https://reproducible.nixos.org/) - Reproducibility tracking
- [Nixpkgs](https://github.com/NixOS/nixpkgs) - Package repository

### ROS Integration
- [nix-ros-overlay GitHub](https://github.com/lopsided98/nix-ros-overlay) - Primary ROS/ROS 2 overlay
- [Clearpath nix-ros-base](https://github.com/clearpathrobotics/nix-ros-base) - Alternative ROS packaging
- [ROSCon 2022 Talk - Better ROS Builds with Nix](http://download.ros.org/downloads/roscon/2022/Better%20ROS%20Builds%20with%20Nix.pdf) - Clearpath presentation
- [ROS Discourse - Nix Discussion](https://discourse.openrobotics.org/t/build-systems-package-management-and-nix/41488) - Community discussion
- [Stop Fighting Your ROS 2 Environment](https://index.0x77.dev/blog/ros-devenv) - Practical tutorial

### NVIDIA/Jetson Support
- [jetpack-nixos GitHub](https://github.com/anduril/jetpack-nixos) - Anduril's Jetson NixOS module
- [NixOS Wiki - CUDA](https://wiki.nixos.org/wiki/CUDA) - CUDA configuration guide
- [NixOS Wiki - NVIDIA](https://wiki.nixos.org/wiki/Nvidia) - NVIDIA driver configuration
- [NVIDIA Jetson Embedded](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/) - Hardware platform

### Cross-Compilation
- [nix.dev - Cross Compilation](https://nix.dev/tutorials/cross-compilation.html) - Official tutorial
- [NixOS Wiki - Cross Compiling](https://nixos.wiki/wiki/Cross_Compiling) - Community guide
- [NixOS on ARM](https://nixos.wiki/wiki/NixOS_on_ARM) - ARM platform support

### Fleet Management
- [Bento GitHub](https://github.com/rapenne-s/bento) - KISS fleet management
- [Fleet GitHub](https://github.com/CertainLach/fleet) - Cluster configuration tool
- [Managing NixOS Fleet (Blog Series)](https://dataswamp.org/~solene/2022-09-02-managing-a-fleet-of-nixos-part1.html) - Design choices
- [Botnix GitHub](https://github.com/nervosys/Botnix) - Robotics-focused NixOS fork

### Comparisons & Best Practices
- [Nix vs Docker (DevZero)](https://www.devzero.io/blog/nix-vs-docker) - Detailed comparison
- [Will Nix Overtake Docker? (Replit)](https://blog.replit.com/nix-vs-docker) - Industry perspective
- [Nix and Containers (Flox)](https://flox.dev/blog/nix-and-containers-why-not-both/) - Using both together
- [Nix Flakes Explained (Determinate Systems)](https://determinate.systems/blog/nix-flakes-explained/) - Flakes overview
- [Why NixOS Replacing Ubuntu in DevOps 2025](https://markaicode.com/nixos-replacing-ubuntu-devops-2025/) - Adoption trends

### Tutorials & Learning
- [zero-to-nix](https://zero-to-nix.com/) - Beginner-friendly introduction
- [NixOS & Flakes Book](https://nixos-and-flakes.thiscute.world/) - Comprehensive guide
- [Declarative Dev Environments with devenv](https://www.blog.brightcoding.dev/2025/09/28/declarative-development-environments-with-nix-and-devenv-zero-fuss-100-reproducible-set-ups/) - devenv tutorial
- [Michael Stapelberg - Declarative NixOS Installation](https://michael.stapelberg.ch/posts/2025-06-01-nixos-installation-declarative/) - Installation guide

## Notes for Content Creation

### Badge/Level
- Suggest: **Advanced** - Similar to Yocto, steep learning curve but powerful
- Alternative: **Practical** if focusing on development environments rather than full OS

### Tone
- Emphasize practical robotics use cases
- Focus on ROS 2 integration via nix-ros-overlay
- Include Jetson deployment examples
- Compare/contrast with Yocto (same category, different approach)

### Key Points to Highlight
1. **Reproducibility** - The primary value proposition for robotics fleets
2. **Atomic updates & rollback** - Critical for production robot deployments
3. **nix-ros-overlay** - The bridge to ROS 2 ecosystem
4. **jetpack-nixos** - NVIDIA Jetson support for edge AI robots
5. **Fleet management** - Managing multiple robots with version-controlled configs
6. **Development environments** - Nix shells for consistent dev experience

### Structure (based on existing entries)
1. Frontmatter with title, description, last_validated, sidebar badge (Advanced)
2. Imports for Astro components
3. Level badge span
4. Bold intro paragraph
5. Aside note about learning curve
6. Prerequisites CardGrid (Linux, Git, ROS 2)
7. "Why NixOS for Robotics" bullet list
8. NixOS vs Nix distinction
9. Architecture diagram
10. nix-ros-overlay section with flake examples
11. Jetson support section (jetpack-nixos)
12. Cross-compilation section
13. Comparison with Yocto table
14. Fleet management overview
15. Related Terms CardGrid
16. Learn More links
17. Sources section

### Potential Asides
- Caution about steep learning curve (like Yocto)
- Note about flakes being "experimental" but widely used
- Tip about using Cachix for faster builds
- Note about ROS 1 EOL and legacy support

### Differentiation from Yocto Entry
- Yocto: Build-time customization, BSP focus, production images
- NixOS: Runtime declarative config, fleet management, development environments
- Both: Reproducibility, embedded Linux, Jetson support
- Complementary use: Yocto for custom BSP, NixOS for fleet deployment
