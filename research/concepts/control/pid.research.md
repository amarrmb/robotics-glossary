# PID Control - Research Notes

**Research Date:** January 2026
**Entry:** src/content/docs/concepts/control/pid.mdx

---

## Current Entry Review

The current entry covers:
- PID equation and three terms (P, I, D)
- Ziegler-Nichols tuning method
- Code example with anti-windup
- Common issues (integral windup, derivative noise, setpoint kick)
- Applications in robotics
- Limitations and alternatives

---

## ROS 2 PID Controller (ros2_control)

### Current Versions (January 2026)
- **ros2_control** documentation available for: Rolling, Humble, Jazzy, Iron, Kilted
- **control_toolbox** version: 5.8.1 (Rolling), 5.3.2 (Kilted), 4.6.0 (Jazzy)
- Plugin: `pid_controller/PidController`

### Key Features
- Uses `PidROS` implementation from `control_toolbox` package
- Supports direct topic control or chained controllers
- Supports second-order PID control with derivative of reference

### Anti-Windup Strategies (Important Update)
The `control_toolbox::Pid` class offers three anti-windup strategies:
1. **NONE**: No anti-windup; integral accumulates without correction
2. **BACK_CALCULATION**: Adjusts integral based on difference between unsaturated/saturated outputs using `tracking_time_constant`
3. **CONDITIONAL_INTEGRATION**: Freezes integration when saturated or when error drives further saturation

### Deprecation Notice (Jazzy to Kilted Migration)
- Legacy `antiwindup` boolean parameter deprecated
- Legacy `i_clamp_max/i_clamp_min` deprecated in favor of `antiwindup_strategy` parameter
- PID state publisher topic moved to `<controller_name>` namespace (initially off)

### Sources
- [PID Controller - ROS2_Control Rolling Jan 2026](https://control.ros.org/rolling/doc/ros2_controllers/pid_controller/doc/userdoc.html)
- [PID Controller - ROS2_Control Jazzy Jan 2026](https://control.ros.org/jazzy/doc/ros2_controllers/pid_controller/doc/userdoc.html)
- [control_toolbox documentation](https://control.ros.org/master/doc/control_toolbox/doc/index.html)
- [AntiWindupStrategy Struct - Jazzy 4.6.0](https://docs.ros.org/en/ros2_packages/jazzy/api/control_toolbox/generated/structcontrol__toolbox_1_1AntiWindupStrategy.html)
- [Migration Guide: Jazzy to Kilted](https://control.ros.org/master/doc/ros2_controllers/doc/migration.html)

---

## NVIDIA Isaac Sim

### Current Versions (January 2026)
- **Isaac Sim 5.1.0**: Current stable release
- **Isaac Sim 5.0**: Released at SIGGRAPH 2025 (General Availability)
- **Isaac Sim 6.0**: Early Developer Release (build from source only, GA pending)
- **Isaac Sim 4.5**: Previous version (compatible with Isaac Lab 2.0)

### PID Controller in Isaac
- `use_pid_controller` parameter enables PID for trajectory tracking
- PID parameters available for: forward error, lateral error, heading error
- Articulation controller handles low-level joint position/velocity/effort control

### Key Resources
- `DifferentialController` class for mobile robots
- `WheelBasePoseController` for wheeled platforms
- Controllers inherit from `BaseController` interface

### Sources
- [Controller Extension - NVIDIA Docs](https://docs.nvidia.com/isaac/doc/extensions/controller/doc/index.html)
- [Adding a Controller Tutorial - Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/core_api_tutorials/tutorial_core_adding_controller.html)
- [Articulation Controller - Isaac Sim 6.0](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_simulation/articulation_controller.html)
- [Isaac Sim GA Announcement](https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/)

---

## Ziegler-Nichols Tuning Method

### Current Entry Review - Potential Correction Needed

The current entry uses `Ki` and `Kd` directly in the table. Standard Ziegler-Nichols notation uses time constants `Ti` and `Td`, then converts to gains.

**Standard Ziegler-Nichols Table (Time Constants):**

| Control Type | Kp      | Ti      | Td     |
|--------------|---------|---------|--------|
| P            | 0.5·Ku  | ∞       | 0      |
| PI           | 0.45·Ku | Pu/1.2  | 0      |
| PID          | 0.6·Ku  | Pu/2    | Pu/8   |

**Conversion to Gains:**
- Ki = Kp / Ti (or Ki = Kp · (1/Ti))
- Kd = Kp · Td

**For PID:**
- Kp = 0.6·Ku
- Ki = Kp / (Pu/2) = 2·Kp/Pu (matches current entry)
- Kd = Kp · (Pu/8) = Kp·Pu/8 (current entry shows Kp·Tu/8 - correct, using Tu notation)

### Verification
The current entry's table appears **correct** when using direct gain notation:
- PI: Ki = 1.2·Kp/Tu is equivalent to Ki = Kp/(Tu/1.2)
- PID: Ki = 2·Kp/Tu, Kd = Kp·Tu/8

### Notes
- Method published in 1942 by Ziegler and Nichols
- Aims for ~25% overshoot (aggressive tuning)
- Good starting point but often needs refinement for industrial use

### Sources
- [Ziegler-Nichols Method - Wikipedia](https://en.wikipedia.org/wiki/Ziegler–Nichols_method)
- [PID Tuning via Classical Methods - Engineering LibreTexts](https://eng.libretexts.org/Bookshelves/Industrial_and_Systems_Engineering/Chemical_Process_Dynamics_and_Controls_(Woolf)/09:_Proportional-Integral-Derivative_(PID)_Control/9.03:_PID_Tuning_via_Classical_Methods)
- [Ziegler-Nichols Tuning Rules - MStarLabs](https://www.mstarlabs.com/control/znrule.html)

---

## Anti-Windup Techniques (General)

### Back-Calculation
- Uses difference between unsaturated and saturated output as feedback
- Requires tuning parameter: Kb (back-calculation gain) or Tt (tracking time constant)
- Good for processes with frequent or prolonged saturation

### Conditional Integration (Clamping)
- Disables integral when saturated or when error drives further saturation
- No parameter to tune
- Use for most applications

### Current Entry Review
- Code example includes simple clamping: `self.integral = max(-100, min(100, self.integral))`
- This is a valid anti-windup approach but doesn't distinguish between strategies

### Sources
- [PID Anti-Windup Techniques - Erdos Miller](https://info.erdosmiller.com/blog/pid-anti-windup-techniques)
- [Anti-Windup Control Using PID Controller - MathWorks](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)
- [Integral Windup - Wikipedia](https://en.wikipedia.org/wiki/Integral_windup)

---

## Corrections/Updates Needed

1. **No major factual errors found** - The entry is technically accurate

2. **Consider adding:**
   - Brief mention of modern anti-windup strategies (back-calculation vs clamping)
   - Reference to ros2_control PID controller for ROS users
   - Note that Ziegler-Nichols yields aggressive tuning (~25% overshoot)

3. **Minor clarification:**
   - The Ziegler-Nichols table could note that Ti/Tu notation varies in literature
   - Code example's anti-windup is "clamping" style (valid but simple)

4. **Version-specific notes for ROS 2:**
   - Jazzy to Kilted migration deprecated old antiwindup parameters
   - New `antiwindup_strategy` parameter with NONE/BACK_CALCULATION/CONDITIONAL_INTEGRATION options
