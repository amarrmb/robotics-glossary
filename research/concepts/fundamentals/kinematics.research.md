# Kinematics Research Notes

**Research Date:** 2026-01-21
**Entry File:** src/content/docs/concepts/fundamentals/kinematics.mdx

## Current Version Numbers

### NVIDIA Isaac Sim
- **Latest version:** 5.1.0 (current stable), 6.0 Early Developer Release available
- **Current file references:** 5.1.0 - CORRECT
- Isaac Sim 5.0 released at SIGGRAPH 2025
- Note: All deprecated extensions will be removed in 6.0

Sources:
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/release_notes.html
- https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/

### ROS 2
- **Jazzy Jalisco:** Released May 23, 2024 - still supported but NOT the latest
- **Kilted:** Current latest ROS 2 distribution
- **Lyrical Luth:** Expected May 2026
- **Current file references:** Jazzy as "current LTS" - NEEDS UPDATE (Kilted is now latest)

Sources:
- https://docs.ros.org/en/jazzy/Releases.html
- https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html

### MoveIt 2
- Multiple IK solvers available: KDL (default), TRAC-IK, IKFast, pick_ik, LMA
- MoveIt recommends ROS 2 Jazzy on Ubuntu 24.04 for new users
- **Current file link:** `moveit.picknik.ai/jazzy/doc/concepts/kinematics.html` - Returns 404
- **Correct link:** `moveit.picknik.ai/main/doc/concepts/kinematics.html` (Rolling docs) or use Humble

Sources:
- https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html
- https://moveit.picknik.ai/main/doc/concepts/kinematics.html
- https://moveit.picknik.ai/humble/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html

## Corrections Needed

### 1. MoveIt Link (Line 147)
- **Current:** `https://moveit.picknik.ai/jazzy/doc/concepts/kinematics.html`
- **Issue:** Returns 404 - no Jazzy-specific MoveIt docs at this path
- **Suggested fix:** Use `https://moveit.picknik.ai/main/doc/concepts/kinematics.html` or Humble docs

### 2. ROS 2 Status (Line 153)
- **Current:** Implies Jazzy is "current LTS"
- **Issue:** Kilted is now the latest ROS 2 distribution; Jazzy is supported but older
- **Suggested fix:** Clarify Jazzy is "supported" rather than "current"

### 3. Isaac Sim Link (Line 151)
- **Current:** `https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/concepts/kinematics_solver.html`
- **Status:** VERIFIED WORKING - Version 5.1.0 is correct

### 4. Stanford CS223A (Line 154)
- **Current:** References "Winter 2025"
- **Status:** Course was offered Winter 2025 (Jan-Mar 2025) with Prof. Khatib
- **Suggested fix:** Update to reflect 2024-2025 academic year or remove specific quarter

## Verified Sources (No Changes Needed)

- Stanford SEE CS223A link: https://see.stanford.edu/Course/CS223A - WORKING
- Modern Robotics textbook: https://hades.mech.northwestern.edu/index.php/Modern_Robotics - WORKING
- Modern Robotics PDF: https://hades.mech.northwestern.edu/images/7/7f/MR.pdf - WORKING
- Book updated December 2019 - matches current file reference

## Technical Content Review

- Forward/Inverse kinematics explanations are accurate
- DH parameters table is correct
- Python code example is mathematically correct
- Joint type DOF descriptions are accurate

## Summary of Required Updates

| Item | Priority | Issue |
|------|----------|-------|
| MoveIt link | High | 404 error |
| ROS 2 Jazzy status | Medium | Jazzy no longer "current" |
| Stanford CS223A date | Low | Minor date clarification |
