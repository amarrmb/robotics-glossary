# PID Control - Research Notes

**Research Date:** 2026-01-21
**Entry:** src/content/docs/concepts/control/pid.mdx

---

## Current Entry Review

The current entry covers:
- PID equation and three terms (P, I, D)
- Ziegler-Nichols tuning method with lookup table
- Python code example with anti-windup (clamping)
- Common issues (integral windup, derivative noise, setpoint kick)
- Applications in robotics
- Limitations and alternatives (MPC, LQR, adaptive, neural network)

---

## ROS 2 PID Controller (ros2_control)

### Current Versions (January 2026)
- **pid_controller**: 6.2.0 (Rolling), 2.52.0 (Humble)
- **control_toolbox**: 6.1.0 (Rolling), 5.8.1 (Rolling older), 5.6.0 (Kilted), 4.6.0 (Jazzy)
- Plugin: `pid_controller/PidController`

### ROS 2 Distribution Status
- **Rolling**: Development branch (January 2026)
- **Kilted Kaiju**: Released May 2025, supported until November 2026
- **Jazzy Jalisco**: LTS, released May 2024, supported until May 2029
- **Humble Hawksbill**: LTS
- **Lyrical Luth**: Planned for May 2026

### Key Features
- Uses `PidROS` implementation from `control_toolbox` package
- Supports direct topic control or chained controllers
- Supports second-order PID control with derivative of reference
- Feed-forward gain for enhanced dynamic response
- Angle wraparound for continuous joints (normalizes errors to -π to π)

### Anti-Windup Strategies (Important)
The `control_toolbox::Pid` class offers three anti-windup strategies via `antiwindup_strategy` parameter:

1. **NONE** (default): No anti-windup; integral accumulates without correction
2. **BACK_CALCULATION**: Adjusts integral based on difference between unsaturated/saturated outputs using `tracking_time_constant`
3. **CONDITIONAL_INTEGRATION**: Freezes integration when saturated or when error drives further saturation

Note: LEGACY antiwindup strategy is deprecated and will be removed.

### Key Parameters
| Parameter | Type | Default |
|-----------|------|---------|
| `gains.<dof>.p` | double | 0.0 |
| `gains.<dof>.i` | double | 0.0 |
| `gains.<dof>.d` | double | 0.0 |
| `gains.<dof>.antiwindup_strategy` | string | "none" |
| `gains.<dof>.tracking_time_constant` | double | 0.0 |
| `gains.<dof>.i_clamp_max/min` | double | ±infinity |
| `gains.<dof>.u_clamp_max/min` | double | ±infinity |
| `gains.<dof>.feedforward_gain` | double | 0.0 |
| `gains.<dof>.angle_wraparound` | bool | false |

### Sources
- [PID Controller - ROS2_Control Rolling Jan 2026](https://control.ros.org/rolling/doc/ros2_controllers/pid_controller/doc/userdoc.html)
- [PID Controller - ROS2_Control Humble Jan 2026](https://control.ros.org/humble/doc/ros2_controllers/pid_controller/doc/userdoc.html)
- [pid_controller: Rolling 6.2.0](https://docs.ros.org/en/rolling/p/pid_controller/)
- [pid_controller: Humble 2.52.0](https://docs.ros.org/en/humble/p/pid_controller/doc/userdoc.html)
- [control_toolbox documentation](https://control.ros.org/master/doc/control_toolbox/doc/index.html)
- [PidROS Class - Rolling 5.8.1](https://docs.ros.org/en/rolling/p/control_toolbox/generated/classcontrol__toolbox_1_1PidROS.html)
- [Kilted Kaiju Release](https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html)

---

## NVIDIA Isaac

### Current Versions (January 2026)
- **Isaac ROS 3.2**: Announced at CES 2025
- **Isaac Sim 5.1.0**: Current stable release
- **Isaac Sim 4.5.0**: Previous version

### PID in Isaac
- No dedicated PID controller package in Isaac ROS
- Isaac ROS focuses on perception, navigation, motion planning (cuMotion)
- Use standard ros2_control PID packages with Isaac ROS
- Isaac Sim `DifferentialController` and `ArticulationController` use internal PID for joint control

### Sources
- [Isaac ROS - NVIDIA Developer](https://developer.nvidia.com/isaac/ros)
- [CES 2025 - Isaac ROS 3.2 Updates](https://forums.developer.nvidia.com/t/ces-2025-isaac-ros-3-2-and-platform-updates/319021)
- [ROS 2 Installation - Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

---

## Ziegler-Nichols Tuning Method

### Current Entry Verification
The entry's Ziegler-Nichols table is **correct**:

| Control Type | Kp | Ki | Kd |
|--------------|----|----|-----|
| P | 0.5·Ku | 0 | 0 |
| PI | 0.45·Ku | 1.2·Kp/Tu | 0 |
| PID | 0.6·Ku | 2·Kp/Tu | Kp·Tu/8 |

### Design Criteria
- Method aims for **~25% overshoot** (quarter amplitude decay)
- Aggressive tuning - good for disturbance rejection
- Often requires refinement for production use
- Entry correctly notes: "yields aggressive response with ~25% overshoot"

### Sources
- [Ziegler-Nichols Method - Wikipedia](https://en.wikipedia.org/wiki/Ziegler–Nichols_method)
- [PID Tuning via Classical Methods - Engineering LibreTexts](https://eng.libretexts.org/Bookshelves/Industrial_and_Systems_Engineering/Chemical_Process_Dynamics_and_Controls_(Woolf)/09:_Proportional-Integral-Derivative_(PID)_Control/9.03:_PID_Tuning_via_Classical_Methods)

---

## Anti-Windup Techniques

### MathWorks Documentation (Verified)
The linked MathWorks page covers three approaches:
1. **Back-Calculation**: Unwinds integrator using feedback when saturated
2. **Integrator Clamping**: Conditional integration
3. **Tracking Mode**: External back-calculation for complex scenarios

Entry link valid: https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html

### Current Entry Review
- Code example uses simple clamping: `self.integral = max(-100, min(100, self.integral))`
- Entry describes three strategies: clamping, back-calculation, conditional integration
- All correctly explained

### Sources
- [Anti-Windup Control Using PID Controller - MathWorks](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)

---

## Corrections/Updates Needed

### No Major Errors Found
The entry is technically accurate and well-structured.

### Minor Updates to Consider

1. **ROS 2 Source Link Update**
   - Current link: `https://control.ros.org/rolling/doc/ros2_controllers/pid_controller/doc/userdoc.html`
   - Link is still valid - no change needed

2. **Version Information**
   - Could add version numbers to sources section for clarity
   - pid_controller: Rolling 6.2.0, Humble 2.52.0

3. **Anti-Windup Strategies**
   - Entry covers clamping, back-calculation, conditional integration
   - Could note that ros2_control uses `antiwindup_strategy` parameter with same options

4. **No Deprecations Affecting Entry**
   - ROS 2 deprecations are internal implementation details
   - Entry's general PID concepts remain accurate

### Validation Status
- Ziegler-Nichols table: **Correct**
- PID equation: **Correct**
- Anti-windup explanations: **Correct**
- Sources: **All valid and current**
