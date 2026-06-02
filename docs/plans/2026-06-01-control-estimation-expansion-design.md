# Control, Estimation & Dynamics Foundations — Glossary Expansion

**Date:** 2026-06-01
**Status:** Design (awaiting review)
**Author:** Content review pass

## Problem

The glossary's `concepts/control/` category currently holds only **PID Control** and
**Motion Planning**. A content review on 2026-06-01 found a coherent gap along the
*control & estimation* spine:

- **State estimation is never named.** The idea of "recover the true state from noisy,
  partial sensors" is used implicitly by `perception/sensor-fusion` but never defined.
- **Filters are name-dropped, not explained.** Kalman / EKF / UKF / particle filters
  appear inside `sensor-fusion` *in the fusion context only* — there is no entry that
  explains what the filter itself does.
- **System identification is absent** — building a dynamics model from data is missing.
- **PID's "Limitations" section promises pages that don't exist.** It tells the reader to
  "consider LQR / MPC / adaptive control" with no links, because no such entries exist.

A wider grep (`as-title:0` for every probed term) surfaced a **systemic pattern**: many
terms are referenced across many files but have no home entry. The glossary is meant to be
a navigation layer; today a lot of links lead nowhere. This spec fixes the highest-value
slice of that — the control/estimation spine plus the load-bearing dynamics foundations
those entries must link to — and records the rest as tracked future work.

## Goals

1. Add the control/estimation entries that complete PID's neighborhood.
2. Add the **Tier-1 foundations** (Dynamics, Jacobian, Force/Impedance Control) that the
   control entries depend on — without them, the new pages would themselves dangle.
3. Hold the existing quality bar: accessible-first prose, concrete intuition before math,
   a worked numeric or runnable code example per entry. No verbose filler.
4. Fix the two dangling cross-reference sites (PID, Sensor Fusion) so nothing new dangles.

## Non-Goals (YAGNI)

- No standalone **Particle Filter** entry — covered well enough by `sensor-fusion` + a
  cross-link; revisit when SLAM coverage deepens.
- No **adaptive control** entry yet.
- No separate **Observability/Controllability** page — folded into State-Space Representation.
- Tiers 2–4 (see Future Work) are *recorded*, not built here.

## Style Contract (every new entry)

Matches current in-repo conventions (which are slightly ahead of `style-guide.md`):

- Frontmatter: `title`, `description`, `last_validated: 2026-06-01`, and a sidebar badge.
- Opens with `<span class="level-badge {conceptual|practical|deepdive}">…</span>`.
- First sentence: **bold term** + plain-English definition + why it matters for robotics.
- One `<Aside type="tip">` near the top carrying the key intuition or common misconception.
- `## Prerequisites` `CardGrid` of `LinkCard`s where it aids the reader.
- An ASCII diagram for the core mental model.
- **A worked numeric example or runnable code block** — the quality bar; no hand-waving.
- `## Related Terms` `CardGrid`.
- `## Sources` with real, dated, linkable references.
- Prose length 250–500 words. Every symbol named in words before it appears in an equation.

## Entries to Add (10)

### Foundations — `src/content/docs/concepts/fundamentals/`

| File | Title | Badge | Angle |
|------|-------|-------|-------|
| `dynamics.mdx` | Robot Dynamics | Conceptual | Forces, torques, mass, inertia, equations of motion. The counterpart to the existing Kinematics entry — kinematics is *where*, dynamics is *what force gets it there*. Forward vs inverse dynamics. |
| `jacobian.mdx` | Jacobian | Deep Dive | The matrix bridging joint-space and task-space velocities/forces. Underpins IK, velocity control, static force mapping. **Singularities folded in** (loss of rank → loss of a motion direction). |

### Control & Estimation — `src/content/docs/concepts/control/`

| File | Title | Badge | Angle |
|------|-------|-------|-------|
| `state-space.mdx` | State-Space Representation | Conceptual | Describe a system as a state vector evolving via A/B/C/D matrices. Controllability & observability intuition. The shared language for Kalman, LQR, MPC. |
| `state-estimation.mdx` | State Estimation | Conceptual | "Recover the true state from noisy, partial measurements." Observers, the predict→correct loop. Umbrella that routes to specific filters. |
| `kalman-filter.mdx` | Kalman Filter | Deep Dive | Predict→update cycle; covariance as "how much to trust each source." EKF/UKF for nonlinear systems. Cross-links to sensor-fusion for the multi-sensor application. |
| `complementary-filter.mdx` | Complementary Filter | Practical | The lightweight "poor man's Kalman" for attitude — high-pass the gyro, low-pass the accel. Short runnable code. |
| `system-identification.mdx` | System Identification | Practical | Fit a dynamics model from input/output data. Grey-box vs black-box. Feeds MPC and sim-to-real. |
| `lqr.mdx` | LQR | Deep Dive | Optimal control for linear systems via Q/R cost weights. The principled step up from hand-tuned PID. Fills a PID dangling link. |
| `mpc.mdx` | Model Predictive Control (MPC) | Deep Dive | Receding-horizon optimization that respects constraints; needs a model (State-Space / System ID). Fills the other PID dangling link. |
| `force-control.mdx` | Force / Impedance Control | Deep Dive | The contact-control half PID/LQR/MPC omit — how a robot regulates *force*, not just position. Impedance vs admittance, compliance, why it matters for manipulation and human safety. |

**Badge legend:** `conceptual` (idea-level), `practical` (hands-on, code-forward),
`deepdive` (math-heavy / advanced) — the three classes that exist in `custom.css`.

## Back-Fixes (so nothing new dangles)

1. **`concepts/control/pid.mdx`** — in the "Limitations" section, turn the LQR / MPC / adaptive
   bullets into live links to the new `lqr.mdx`, `mpc.mdx` (adaptive stays plain text — not built).
2. **`concepts/perception/sensor-fusion.mdx`** — in the EKF/UKF/Particle tabs, add a
   "see **Kalman Filter** for how the filter itself works" link to `kalman-filter.mdx`.
   Add a reciprocal "applied to multiple sensors → **Sensor Fusion**" link from `kalman-filter.mdx`.

## Sidebar / Wiring

No `astro.config.mjs` change needed — the "Control & Planning" and "Fundamentals" sidebar
groups use `autogenerate: { directory: ... }`, so new `.mdx` files appear automatically.

## Dependency / Build Order

```
Dynamics ─┐
Jacobian ─┤
          ├─► State-Space ─► State Estimation ─► Kalman ─► Complementary
          │                 └─► System ID
          └─────────────────────► LQR
                                  MPC          (need State-Space + Dynamics + System ID)
                                  Force Control (needs Dynamics + Jacobian)
```

Suggested authoring sequence: Dynamics, Jacobian → State-Space → State Estimation → Kalman →
Complementary → System ID → LQR → MPC → Force Control → back-fixes.

## Verification (per entry, before marking done)

- `npm run build` passes (Astro/Starlight validates frontmatter + MDX + internal links).
- Every internal `LinkCard`/link href resolves to a real page (no 404s in the built `dist/`).
- The entry contains a worked example that is internally consistent (numbers/code check out).
- Reading test: a reader who knows PID can follow the new entry without leaving the page for
  an undefined prerequisite — or the prerequisite is linked.

## Future Work (tracked, not built here)

Captured from the same 2026-06-01 review. Each is a "referenced-but-undefined" term.

**Tier 2 — referenced everywhere, defined nowhere**
- Actuators & Encoders (new `hardware/actuators/` category) — referenced in 8–10 files.
- Calibration — camera + hand-eye (Sensor Fusion explicitly requires it, links nothing).
- Domain Randomization (6 files; core sim-to-real technique).
- Quaternions / rotation representations (gimbal lock; sits next to Transforms).

**Tier 3 — modern robot-learning currency**
- Diffusion Policy (SOTA imitation learning; platform trains ACT/GR00T/Pi0).
- Pose Estimation (6-DoF) (manipulation grasping output).
- Behavior Cloning (verify Imitation Learning coverage first; expand or add).

**Tier 4 — software / systems**
- MuJoCo (and maybe Gazebo) — only Isaac Sim has an entry, yet robot-agent ships a MuJoCo adapter.
- DDS (ROS 2 transport, 7 files).
- Safety / E-stop / functional safety (8 files, no category).
- Occupancy Grid / Costmap; Trajectory Optimization.
- Object Detection / Segmentation (verify Computer Vision coverage first).
