# Kinematics Research Notes

**Research Date:** January 2026

## Current Entry Assessment

The kinematics.mdx entry covers fundamental concepts accurately:
- Forward vs Inverse Kinematics distinction is correct
- Kinematic chain explanation is accurate
- DH parameters table is correct
- Python code example computes correctly

## Version Numbers and Current Status

### NVIDIA Isaac Sim
- **Current Version:** Isaac Sim 5.1.0 (latest recommended)
- **Previous Major:** Isaac Sim 5.0 (released at SIGGRAPH 2025, now open source)
- **Key Feature:** LulaKinematicsSolver implements KinematicsSolver interface for FK/IK
- **Note:** Isaac Sim 5.0+ is now open source on GitHub
- **Deprecation:** All deprecated extensions will be removed in Isaac Sim 6.0
- **Source:** https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/concepts/kinematics_solver.html

### ROS 2 / MoveIt 2
- **ROS 2 LTS:** Jazzy Jalisco (released May 23, 2024, supported until May 2029)
- **Supported Distributions:** Jazzy (LTS), Humble (LTS), Rolling
- **Next Release:** Kilted Kaiju (May 2025)
- **MoveIt 2 Documentation:** https://moveit.picknik.ai/main/doc/concepts/kinematics.html
- **Source:** https://docs.ros.org/en/jazzy/Releases.html

### MoveIt 2 IK Solvers (Current)
- **KDL** (default) - Numerical jacobian-based, works with serial chains only
- **TRAC-IK** - Combines KDL extension + SQP optimization, more robust
- **IKFast** - Analytic solver, extremely fast (microseconds), supports up to 7 DOF
- **pick_ik** - Modern solver by PickNik, supports custom cost functions, global/local modes
- **LMA** - Levenberg-Marquardt kinematics plugin
- **Cached IK** - Persistent cache to speed up other IK solvers
- **Source:** https://moveit.picknik.ai/humble/doc/concepts/kinematics.html

## External Links Verification

### Current Entry Links
| Link | Status | Notes |
|------|--------|-------|
| Stanford CS223A (see.stanford.edu) | VALID | Course still active, offered Winter 2025 |
| Modern Robotics (Northwestern) | VALID | Free textbook still available, latest version Dec 2019 |
| MoveIt Kinematics (moveit.picknik.ai) | VALID | URL format correct, still active |

### Stanford CS223A
- Course actively taught at Stanford (Winter 2025: 98 students enrolled)
- Instructor: Professor Oussama Khatib
- Online version available through Stanford Online ($4,725, 3 units)
- **Source:** https://see.stanford.edu/Course/CS223A

### Modern Robotics Textbook
- Authors: Kevin M. Lynch (Northwestern), Frank C. Park (Seoul National)
- Published: Cambridge University Press, 2017
- Current: Updated first edition (December 2019 preprint, "version 2")
- Free PDF available at: https://hades.mech.northwestern.edu/index.php/Modern_Robotics
- Coursera specialization also available
- **Source:** https://hades.mech.northwestern.edu/index.php/Modern_Robotics

## Corrections Needed

1. **MoveIt Link Update (Optional):** Current link points to `/main/` (Rolling). Could add Jazzy-specific link for LTS users:
   - Current: `https://moveit.picknik.ai/main/doc/concepts/kinematics.html`
   - Alternative for LTS: `https://moveit.picknik.ai/jazzy/doc/concepts/kinematics.html`

2. **No major corrections required** - The entry content is technically accurate and current.

## Potential Enhancements (Not Required)

- Could mention modern IK solvers (TRAC-IK, pick_ik) as alternatives to analytical methods
- Could reference Isaac Sim's LulaKinematicsSolver for simulation contexts
- Could add link to MoveIt 2 documentation for Jazzy (current LTS)

## Summary

The kinematics entry is **accurate and well-written**. All external links are valid. No factual corrections needed. The conceptual explanations of FK, IK, kinematic chains, and DH parameters are correct. The Python code example is mathematically accurate.
