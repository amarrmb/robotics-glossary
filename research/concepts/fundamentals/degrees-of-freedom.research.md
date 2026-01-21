# Degrees of Freedom (DOF) - Research Notes

**Date:** 2026-01-21 (updated)
**Previous:** 2026-01-20

## Current Software Versions

### NVIDIA Isaac
- **Isaac Sim 5.1.0**: Current latest version (docs available)
- **Isaac Sim 5.0**: General availability released at SIGGRAPH 2025
- **Isaac Lab 2.2**: Released alongside Isaac Sim 5.0
- **Newton Physics Engine**: Now available in Isaac Lab (announced Jan 2026)
- **Isaac GR00T N1.6**: Open reasoning VLA model for robot skills
- Source: [NVIDIA Isaac Sim Release Notes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)
- Source: [NVIDIA Robotics Announcement](https://nvidianews.nvidia.com/news/nvidia-accelerates-robotics-research-and-development-with-new-open-models-and-simulation-libraries)

### ROS 2
- **Kilted Kaiju**: Latest release (May 23, 2025), supported until November 2026
- **Jazzy Jalisco**: LTS release, Patch Release 6 (Aug 2025), supported until May 2029
- **Lyrical Luth**: Upcoming May 2026 release
- **Rolling Ridley**: Continuous development distribution
- Source: [ROS 2 Distributions](https://docs.ros.org/en/jazzy/Releases.html)
- Source: [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/Releases/Release-Kilted-Kaiju.html)

## Content Validation

### Rigid Body 6 DOF - VERIFIED
- 3 translation (X, Y, Z) + 3 rotation (roll, pitch, yaw) = 6 DOF
- Source: [Modern Robotics - Northwestern](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-2-degrees-of-freedom-of-a-robot/)

### Robot Configuration Table - VERIFIED
- 6-DOF Industrial arm: "typical example is an industrial robot with six revolute joints"
- 7-DOF Collaborative arm: Redundant with 1 extra DOF for flexibility
- Source: [ROS 2 Joint Kinematics](https://docs.ros.org/en/rolling/p/hardware_interface/doc/joints_userdoc.html)

### Human Hand 27 DOF - VERIFIED
- 4 DOF × 4 fingers = 16 DOF (3 flexion/extension + 1 abduction/adduction each)
- Thumb = 5 DOF (more complex joint structure)
- Palm/wrist = 6 DOF (3 translation + 3 rotation)
- Total = 27 DOF
- Source: [ResearchGate - Human Hand DOF](https://www.researchgate.net/figure/Finger-flexion-extension-Scene-The-human-hand-has-27-degrees-of-freedom-DOF-7-4-in_fig1_355898836)
- Source: [Northwestern Mechatronics - DOF of Human Arm](https://hades.mech.northwestern.edu/index.php/DOF_of_the_Human_Arm)

### Grübler's Formula - VERIFIED (minor notation note)
Current entry shows planar formula:
```
DOF = 3(n-1) - 2j₁ - j₂
```

General form (Chebychev-Grübler-Kutzbach criterion):
```
DOF = m(N-1-J) + Σfᵢ
```
Where:
- m = 3 for planar, 6 for spatial
- N = number of links (including ground)
- J = number of joints
- fᵢ = freedoms provided by joint i

The planar formula in the entry is correct. The mention to "replace 3 with 6" for spatial is accurate.
- Source: [Modern Robotics - Northwestern](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-2-degrees-of-freedom-of-a-robot/)
- Source: [Wikipedia - Chebychev-Grübler-Kutzbach](https://en.wikipedia.org/wiki/Chebychev–Grübler–Kutzbach_criterion)

### Joint Types - Additional Context from ROS
| Joint Type | DOF | Description |
|------------|-----|-------------|
| Revolute | 1 | Hinge with position limits |
| Continuous | 1 | Hinge without limits |
| Prismatic | 1 | Sliding joint |
| Universal | 2 | Two perpendicular axes |
| Cylindrical | 2 | Rotation + translation |
| Spherical | 3 | Ball-and-socket |
- Source: [ROS 2 Joint Kinematics](https://docs.ros.org/en/rolling/p/hardware_interface/doc/joints_userdoc.html)

### Configuration Space - VERIFIED
- C-space dimension equals DOF
- "if a robot has three joints, then its configuration space has three dimensions"
- Source: [Mecharithm - Configuration Space](https://mecharithm.com/learning/lesson/configuration-space-topology-representation-robot-5)

### Kinematics Types from ROS Documentation
- **Serial kinematics**: Joints independent, DOF = number of joints
- **Parallel/closed-loop kinematics**: Joints coupled, DOF < number of joints
- URDF supports serial chains; closed-loop via mimic joints or custom plugins
- Source: [ROS 2 Joint Kinematics](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/joints_userdoc.html)

## Corrections Needed

**None identified** - The current entry is accurate.

Minor suggestions for enhancement (optional):
1. Grübler's formula could note the assumption of independent constraints
2. The formula has limitations with redundant/dependent constraints

## Sources Verification

| Source in Entry | Status | Notes |
|-----------------|--------|-------|
| Modern Robotics - Northwestern | VALID | URL works, content verified |
| ROS 2 Joint Kinematics | VALID | URL works, content current |
| Mecharithm - Configuration Space | VALID | URL works, content verified |

## NVIDIA Isaac DOF-Related Features
- Joint types: revolute, prismatic (actuated and unactuated)
- Physics via PhysX: joint friction, actuation, rigid/soft body dynamics
- IK controllers support null-space posture regularization for humanoid DOFs
- Source: [NVIDIA Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/rig_mobile_robot.html)
