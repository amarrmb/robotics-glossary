# Degrees of Freedom (DOF) - Research Notes

**Date:** 2026-01-20

## Current Software Versions

### NVIDIA Isaac
- **Isaac Sim 5.0**: General availability released at SIGGRAPH 2025
- **Isaac Sim 5.1.0**: Current latest with ROS 2 Humble and Jazzy library updates
- **Isaac Lab 2.2**: Released alongside Isaac Sim 5.0
- Source: [NVIDIA Isaac Sim Release Notes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html)

### ROS 2
- **Jazzy Jalisco**: LTS release, supported until May 2029 (Patch Release 6 - Aug 2025)
- **Kilted Kaiju**: Standard release (May 2025), supported until November 2026
- **Lyrical Luth**: Upcoming May 2026 release
- Source: [ROS 2 Distributions](https://docs.ros.org/en/jazzy/Releases.html)

## Content Validation

### Rigid Body 6 DOF - VERIFIED
- 3 translation (X, Y, Z) + 3 rotation (roll, pitch, yaw) = 6 DOF
- Confirmed by NVIDIA: "six degrees of freedom (DOF) pose describes a rigid body in 3D space, detailing the translational and rotational state"
- Source: [NVIDIA Developer - Deep Object Pose Estimation](https://forums.developer.nvidia.com/t/generate-synthetic-data-for-deep-object-pose-estimation-training-with-nvidia-isaac-ros/279416)

### Robot Configuration Table - VERIFIED
- 6-DOF Industrial arm: "typical example is an industrial robot with six revolute joints"
- 7-DOF Collaborative arm: MoveIt IKFast tested with 6DOF and 7DOF robot arm manipulators
- Source: [ROS Joint Kinematics docs.ros.org](https://docs.ros.org/en/rolling/p/hardware_interface/doc/joints_userdoc.html)
- Source: [MoveIt IKFast Tutorial](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html)

### Human Hand 27 DOF - VERIFIED (with caveats)
- The figure of 27 DOF is commonly cited but varies by model:
  - 4 DOF × 4 fingers = 16 DOF
  - Thumb = 5 DOF
  - Wrist rotation/translation = 6 DOF
  - Total = 27 DOF
- Some models use 21 DOF (fingers only) or up to 30 DOF
- Current entry uses 27, which is the most commonly cited value
- Source: [Northwestern Mechatronics - DOF of Human Arm](https://hades.mech.northwestern.edu/index.php/DOF_of_the_Human_Arm)
- Source: [ResearchGate - Finger flexion/extension](https://www.researchgate.net/figure/Finger-flexion-extension-Scene-The-human-hand-has-27-degrees-of-freedom-DOF-7-4-in_fig1_355898836)

### Grubler's Formula - NEEDS CORRECTION
Current entry shows planar formula only:
```
DOF = 3(n-1) - 2j₁ - j₂
```

More accurate general form (Chebychev-Grubler-Kutzbach criterion):
```
DOF = m(N-1-J) + Σfi
```
Where:
- m = 3 for planar, 6 for spatial
- N = number of links (including ground)
- J = number of joints
- fi = freedoms provided by joint i

The current simplified planar formula is valid but the notation could be clearer.
- Source: [Modern Robotics - Northwestern](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-2-degrees-of-freedom-of-a-robot/)
- Source: [Wikipedia - Chebychev-Grubler-Kutzbach criterion](https://en.wikipedia.org/wiki/Chebychev–Grübler–Kutzbach_criterion)

### Configuration Space - VERIFIED
- "The dimension of the Configuration Space is equal to the number of degrees of freedom of the Robot"
- "if a robot has three joints, then its configuration space has three dimensions"
- Source: [Modern Robotics - C-Space Topology](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-3-1-configuration-space-topology/)
- Source: [Mecharithm - Configuration Space](https://mecharithm.com/learning/lesson/configuration-space-topology-representation-robot-5)

### Joint Types in ROS - Additional Context
- Revolute: hinge joint with position limits (1 DOF)
- Continuous: hinge without position limits (1 DOF)
- Prismatic: sliding joint along axis (1 DOF)
- Cylindrical: 2 DOF
- Universal: 2 DOF
- Spherical: 3 DOF
- Source: [ROS 2 Joint Kinematics](https://docs.ros.org/en/rolling/p/hardware_interface/doc/joints_userdoc.html)

## Corrections Needed

1. **Grubler's Formula**: The planar-specific formula is correct but could benefit from:
   - Clarifying this is the planar case (m=3)
   - Adding note that spatial mechanisms use m=6
   - The formula has limitations with dependent constraints

2. **Minor clarification**: The entry correctly states the formula but variable naming conventions vary in literature (some use 'n' for links, others use 'N')

## No Corrections Needed

- Rigid body 6 DOF description
- Robot configuration examples (2D mobile base, quadcopter, industrial/collaborative arms)
- Redundancy explanation
- Constraints reducing DOF
- Configuration space vs workspace description

## Additional Notes

- NVIDIA Isaac SDK supports various robot platforms including 6 DoF Cobots
- ROS 2 ros2_control distinguishes between serial kinematics (independent joints, DOF = joints) and parallel kinematics (coupled joints, DOF < joints)
- MoveIt IKFast generator tool currently does not work with >7 DOF arms
