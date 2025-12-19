# Work Logbook - ROB6323 Go2 Project

This file tracks all modifications made to the baseline repository for the Go2 quadruped locomotion project.

---

## 2025-12-18 - Initial Setup

### Changes Made

#### 1. Contact Sensor Registration
**File:** `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py`
- **Line 71:** Added `self.scene.sensors["contact_sensor"] = self._contact_sensor`
- **Purpose:** Properly register the contact sensor with the scene for force data tracking
- **Context:** Required for implementing contact-based rewards and terminations

#### 2. Base Height Termination Threshold
**File:** `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py`
- **Line 84:** Added `base_height_min = 0.05`
- **Purpose:** Lowered termination threshold from 0.20m to 0.05m to allow robot to crouch lower during learning
- **Context:** Part of early termination criteria to speed up training and encourage elevated base

---

## 2025-12-18 - Tutorial Implementation (Parts 1-6)

### Part 1: Action Rate Penalties
**File:** `rob6323_go2_env.py`
- **Lines 58-64:** Added `last_actions` buffer (shape: num_envs × action_dim × 3) for action history tracking
- **Lines 153-157, 163-164:** Implemented first and second derivative action rate penalties
- **Purpose:** Penalize jerky motions to encourage smooth actuator commands
- **Config:** `action_rate_reward_scale = -0.1`

### Part 2: Low-Level PD Controller
**File:** `rob6323_go2_env.py`
- **Lines 38-42:** Added PD control parameters (Kp, Kd, torque_limits)
- **Lines 97-103:** Updated `_pre_physics_step()` to compute desired joint positions
- **Lines 105-120:** Implemented manual torque-level PD control in `_apply_action()`
- **Purpose:** Full control over gains and torque limits instead of using implicit controller

**File:** `rob6323_go2_env_cfg.py`
- **Lines 19, 36-39:** Added PD gains (Kp=20.0, Kd=0.5, torque_limits=100.0)
- **Lines 70-76:** Disabled implicit actuator (stiffness=0, damping=0)

### Part 3: Early Termination (Base Height Min)
**File:** `rob6323_go2_env.py`
- **Lines 183-186:** Added base height check in `_get_dones()`
- **Purpose:** Terminate episodes early if robot collapses (base < 5cm)
- **Already configured:** `base_height_min = 0.05` in cfg

### Part 4: Raibert Heuristic (Gait Shaping)
**File:** `rob6323_go2_env_cfg.py`
- **Line 29:** Updated observation space from 48 to 52 (added 4 clock inputs)
- **Lines 32-34:** Added reward scales for Raibert, feet clearance, contact tracking

**File:** `rob6323_go2_env.py`
- **Lines 67-77:** Added foot body indices (`_feet_ids`) for all 4 feet
- **Lines 83-88:** Added gait state variables (gait_indices, clock_inputs, desired_contact_states)
- **Lines 135:** Added clock inputs to observations
- **Lines 159-160:** Call gait update and Raibert reward calculation
- **Lines 277-283:** Added `foot_positions_w` property
- **Lines 284-340:** Implemented `_step_contact_targets()` for gait phase tracking
- **Lines 344-377:** Implemented `_reward_raibert_heuristic()` for foot placement guidance
- **Purpose:** Use Raibert heuristic as "teacher" to encourage proper stepping patterns

### Part 5: Refined Reward Function

#### Part 5.1: Configuration
**File:** `rob6323_go2_env_cfg.py`
- **Lines 101-105:** Added reward scales for orientation, lin_vel_z, dof_vel, ang_vel_xy

#### Part 5.2: Implementation
**File:** `rob6323_go2_env.py`
- **Lines 55-58:** Added logging keys: "orient", "lin_vel_z", "dof_vel", "ang_vel_xy"
- **Lines 167-176:** Implemented 4 stability reward terms:
  - `rew_orient`: Penalize body tilt (projected gravity XY)
  - `rew_lin_vel_z`: Penalize vertical bouncing
  - `rew_dof_vel`: Penalize fast joint velocities
  - `rew_ang_vel_xy`: Penalize roll/pitch rates
- **Lines 183-186:** Integrated into rewards dictionary
- **Purpose:** Shape natural, stable locomotion behavior

### Part 6: Advanced Foot Interaction

#### Part 6.1: Configuration
**Already present in cfg (lines 32-34):**
- `feet_clearance_reward_scale = -30.0`
- `tracking_contacts_shaped_force_reward_scale = 4.0`

#### Part 6.2: Sensor Indices
**File:** `rob6323_go2_env.py`
- **Lines 68-83:** Separated foot indices for robot (positions) vs sensor (forces)
  - `_feet_ids`: For accessing `robot.data.body_pos_w`
  - `_feet_ids_sensor`: For accessing `_contact_sensor.data.net_forces_w`
- **Purpose:** Contact sensor has different internal indexing than robot articulation

#### Part 6.3: Foot Rewards Implementation
**File:** `rob6323_go2_env.py`
- **Lines 59-60:** Added logging keys: "feet_clearance", "tracking_contacts_shaped_force"
- **Lines 178-183:** Implemented foot clearance reward
  - Calculates dynamic target height (8cm max + 2cm offset) based on gait phase
  - Penalizes deviation only during swing phase
- **Lines 185-192:** Implemented contact force tracking reward
  - Penalizes ground contact forces during swing using exponential decay
  - Uses proper sensor indices for force data
- **Lines 203-204:** Integrated into rewards dictionary
- **Purpose:** Encourage proper foot lifting during swing and grounding during stance

---

## Debugging Fixes

### Fix 1: Missing numpy import (Job 134821, 134834)
**Error:** `NameError: name 'np' is not defined` at line 365
**File:** `rob6323_go2_env.py`
**Fix:** Added `import numpy as np` at line 11
**Context:** Raibert heuristic used `np.pi` for clock input calculation

### Fix 2: Tensor shape mismatch (Job 134834, 134838)
**Error:** `RuntimeError: stack expects each tensor to be equal size, but got [4096] at entry 0 and [4096, 1] at entry 10`
**File:** `rob6323_go2_env.py`
**Fix:** Added `.squeeze()` to line 215 in collision penalty calculation
**Context:** `torch.max(..., dim=1)[0]` returned shape `[4096, 1]` instead of `[4096]`, causing mismatch when stacking rewards

---

## Summary of Implementation

**Tutorial Completion:** Parts 1-6 fully implemented
**Files Modified:**
- `rob6323_go2_env.py` - Environment logic and reward calculations
- `rob6323_go2_env_cfg.py` - Configuration parameters and reward scales

**Total Reward Terms:** 12
1. Linear velocity tracking (XY)
2. Angular velocity tracking (Yaw)
3. Action rate penalties (smoothness)
4. Raibert heuristic (foot placement)
5. Orientation (body tilt)
6. Linear velocity Z (bouncing)
7. Joint velocities (fast motion)
8. Angular velocity XY (roll/pitch)
9. Feet clearance (swing height)
10. Contact force tracking (stance grounding)

---

## Notes
- Following tutorial at: `tutorial/tutorial.md`
- Only modifying: `rob6323_go2_env.py` and `rob6323_go2_env_cfg.py`
- NOT modifying: `rsl_rl_ppo_cfg.py` (PPO hyperparameters off-limits per project instructions)

---

## Next Steps
- [x] Implement action rate penalties (Part 1 of tutorial)
- [x] Add low-level PD controller (Part 2 of tutorial)
- [x] Implement early termination (Part 3 of tutorial)
- [x] Implement Raibert Heuristic rewards (Part 4 of tutorial)
- [x] Add refined reward terms (Part 5 of tutorial)
- [x] Implement advanced foot interaction rewards (Part 6 of tutorial)
- [x] Test training on Greene cluster (Run_01 completed)
- [ ] Analyze TensorBoard metrics and tune reward scales if needed
- [ ] Consider domain randomization for sim-to-real transfer

---

## Training Runs

### Run_01: Baseline (Job ID: 134652)
**Date:** 2025-12-18 09:39:23
**Status:** ✅ Completed
**Duration:** ~500 iterations

**Configuration:**
- **Environment:** Baseline configuration (before tutorial implementation)
- **Observation Space:** 48 (no clock inputs yet)
- **Action Space:** 12
- **Num Envs:** 4096
- **Episode Length:** 20s
- **Reward Terms:** Only velocity tracking (lin_vel, yaw_rate)
  - `lin_vel_reward_scale = 1.0`
  - `yaw_rate_reward_scale = 0.5`
- **Actuator:** Implicit PD controller (stiffness=25.0, damping=0.5)
- **Termination:** Base contact, upside-down only

**PPO Hyperparameters:**
- Learning rate: 0.0003 (adaptive schedule)
- Gamma: 0.99
- Lambda (GAE): 0.95
- Mini-batches: 4
- Learning epochs: 5
- Steps per env: 32
- Policy network: [256, 128, 64] (ELU activation)
- Value network: [256, 128, 64] (ELU activation)

**Results:**
- Checkpoints saved: model_0.pt through model_499.pt (every 50 iterations)
- Final model: `model_499.pt`
- Exported policy: `policy.pt`, `policy.onnx`
- Evaluation video: `videos/play/rl-video-step-0.mp4`
- TensorBoard logs: `events.out.tfevents.1766068784.b-31-1.37.0`

**Notes:**
- Used baseline implementation with minimal reward shaping
- No action smoothing, posture stabilization, or foot interaction rewards
- Serves as performance baseline for comparison with enhanced implementations

**Logs Location:** `.logs/134652/rsl_rl/go2_flat_direct/2025-12-18_09-39-23/`

---

### Run_02: Full Implementation (Job ID: 134838)
**Date:** 2025-12-18 18:04:49
**Status:** ❌ Failed (Reward Imbalance)
**Duration:** 500 iterations

**Configuration:**
- **Environment:** Full tutorial implementation (Parts 1-6) + collision penalties
- **Observation Space:** 52 (added 4 clock inputs for gait)
- **Action Space:** 12
- **Num Envs:** 4096
- **Episode Length:** 20s
- **Reward Terms:** 11 terms
  - Velocity tracking (2 terms)
  - Action smoothing (1 term)
  - Posture stabilization (4 terms: orient, lin_vel_z, dof_vel, ang_vel_xy)
  - Gait shaping (1 term: Raibert heuristic)
  - Foot interaction (2 terms: clearance, contact force tracking)
  - Collision penalty (1 term: base contact penalty)
- **Actuator:** Manual PD controller (Kp=20.0, Kd=0.5, torque_limits=100.0)
- **Termination:** Base contact, upside-down, base height < 0.05m

**Implementation Details:**
- **File:** `rob6323_go2_env_cfg.py`
  - Line 111: `base_collision_penalty_scale = -1.0`
- **File:** `rob6323_go2_env.py`
  - Line 11: Added `import numpy as np` (Fix 1)
  - Line 61: Added "base_collision_penalty" logging
  - Line 215: Added `.squeeze()` to collision penalty (Fix 2)
  - Lines 213-216: Collision penalty calculation
    - Penalizes base contact forces above 0.5N threshold
    - Uses clamped linear penalty scaled by contact force magnitude

**Results:**
- **Mean Reward:** -252.64 (vs Run_01: +29.52)
- **Velocity Tracking Performance:**
  - Lin vel XY: 0.10 (vs Run_01: 0.98) → **90% degradation**
  - Ang vel Z: 0.09 (vs Run_01: 0.49) → **81% degradation**
- **Penalty Breakdown:**
  - Contact force tracking: -6.37 (largest penalty)
  - Raibert heuristic: -3.83
  - Action rate: -1.70
  - Feet clearance: -0.55
  - Orient: -0.11
  - Others: minimal

**Analysis:**
- **Primary Issue:** Reward scale imbalance
  - Tracking rewards: ~1.5 total positive potential
  - Penalties: ~12.5 total negative accumulated
  - Ratio: 8:1 penalties overwhelming tracking
- **Robot Behavior:** Learned "do nothing" policy
  - Standing still minimizes penalties
  - Fails primary objective (velocity tracking)
  - Episodes reach full length (999 steps) without crashes
- **Root Cause:** Penalty scales (especially contact_force: 4.0, Raibert: -10.0, feet_clearance: -30.0) too aggressive relative to tracking rewards

**Logs Location:** `.logs/Run_02/rsl_rl/go2_flat_direct/2025-12-18_18-04-49/`

---

### Run_03: Rebalanced Rewards (Job ID: 134880)
**Date:** 2025-12-18 19:06:03
**Status:** ❌ Failed (Insufficient Rebalancing)
**Duration:** 500 iterations

**Objective:** Fix reward imbalance from Run_02 by reducing the 2 largest penalty scales

**Changes from Run_02:**
- **File:** `rob6323_go2_env_cfg.py`
  - Line 32: `raibert_heuristic_reward_scale = -1.0` (was -10.0, 10x reduction)
  - Line 34: `tracking_contacts_shaped_force_reward_scale = 0.4` (was 4.0, 10x reduction)

**Unchanged from Run_02:**
- `feet_clearance_reward_scale = -30.0`
- `action_rate_reward_scale = -0.1`
- All posture stability penalties (orient, lin_vel_z, dof_vel, ang_vel_xy)
- Collision penalty, tracking rewards, PD controller, observation space

**Results:**
- **Mean Reward:** -62.34 (improved from Run_02: -252.64, but still negative)
- **Velocity Tracking Performance:**
  - Lin vel XY: 0.11 (vs Run_01: 0.98) → **89% degradation** (marginal improvement from Run_02's 90%)
  - Ang vel Z: 0.14 (vs Run_01: 0.49) → **73% degradation** (improved from Run_02's 81%)

**Analysis:**
- **Primary Issue:** 10x penalty reduction wasn't enough
  - Robot still learned conservative "do nothing" policy
  - Penalties still discouraging locomotion despite reduction
  - Tracking rewards (lin_vel: 1.0, yaw: 0.5) provide insufficient gradient
- **Root Cause:** Need stronger **positive incentives** to move, not just smaller penalties
  - Current positive potential: ~1.5 total
  - Remaining penalties still significant (clearance: -30.0, contact: 0.4, etc.)
- **Conclusion:** Reducing penalties alone is insufficient - must boost tracking rewards

**Logs Location:** `.logs/Run_03/rsl_rl/go2_flat_direct/2025-12-18_19-06-03/`

---

### Run_04: Boosted Tracking Rewards (Job ID: TBD)
**Date:** TBD
**Status:** 🔄 Preparing
**Expected Duration:** ~30 minutes

**Objective:** Fix reward imbalance by boosting tracking rewards instead of further reducing penalties

**Strategy:** Provide stronger positive gradient toward velocity tracking

**Changes from Run_03:**
- **File:** `rob6323_go2_env_cfg.py`
  - Line 98: `lin_vel_reward_scale = 3.0` (was 1.0, 3x boost)
  - Line 99: `yaw_rate_reward_scale = 1.5` (was 0.5, 3x boost)

**Unchanged from Run_03:**
- All penalty scales (Raibert: -1.0, contact_force: 0.4, clearance: -30.0, etc.)
- All posture stability penalties
- PD controller, observation space, termination criteria

**Expected Impact:**
- New positive reward potential: ~4.5 (was ~1.5)
- Stronger gradient toward velocity tracking objectives
- Penalties remain meaningful but don't dominate
- Robot should pursue locomotion while experiencing smoothness penalties
- Motion should be functional + smoother than Run_01 baseline
