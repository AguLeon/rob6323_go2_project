# Quadruped Locomotion Training Report: Unitree Go2 Robot

**Author:** Agustin Leon and Asrita Bobba
**Course:** ROB-GY 6323 - Reinforcement Learning for Robotics
**Robot Platform:** Unitree Go2 Quadruped
**Framework:** Isaac Lab + RSL-RL (PPO Algorithm)
**Compute:** NYU Greene HPC Cluster

---

## Project Overview

This report documents our work developing a locomotion policy for the Unitree Go2 quadruped robot using reinforcement learning. We started with the Isaac Lab tutorial baseline and experimented with reward tuning, penalty balancing, domain randomization, and joint friction modeling. The goal is to eventually transfer the trained policy to the physical robot.

---

## Part 1: Following the Tutorial

We started by implementing the Isaac Lab quadruped locomotion tutorial. Key components:

**Control Architecture:**
- Manual PD controller with explicit torque computation: `τ = Kp(q_des - q) - Kd·q̇`, clamped to ±100 N·m
- Actuator implicit control disabled (stiffness=0, damping=0) for full torque authority
- Control frequency: 50 Hz (simulation dt=1/200s, decimation=4)

**Observation & Action Spaces:**
- 52-dimensional observations: base state (linear/angular velocities, projected gravity), joint positions/velocities, actions, and 4 gait clock signals (one per foot)
- 12-dimensional actions: joint position offsets (scaled by 0.25 from policy output)

**Reward Structure:**
- Tracking rewards: linear velocity XY and yaw rate following commanded velocities
- Gait shaping: Raibert heuristic foot placement, shaped contact forces, feet clearance during swing
- Regularization: action rate penalties (first & second derivatives), orientation, joint velocities, foot slip
- Contact sensors (3-step history, 5ms update) track foot forces and air time

**Termination Criteria:**
- Base height < 0.05m, upside-down orientation, or torso-ground contact force > 1.0 N

The baseline configuration used conservative reward scales (lin_vel=1.0, yaw=0.5) with standard penalty values for action smoothness (-0.1), contact force shaping (0.4), and feet clearance (-30.0).

**Run_01** was the tutorial baseline. Training on 4096 parallel environments for 500 iterations gave mean reward of -18.86 and linear velocity of 0.983 m/s. The robot walked upright with no crashes, but the negative mean reward showed penalties dominated the signal.

---

## Part 2: Experiments and Locomotion Improvement

### Baseline Optimization: Runs 02-04

**Run_02** tested whether increasing tracking rewards would improve velocity. We increased linear velocity reward by 50% (1.0→1.5) and yaw rate proportionally (0.5→0.75). Mean reward improved to -16.73, linear velocity increased to 1.231 m/s, and the robot still didn't crash. This worked better than trying to reduce penalties.

**Run_03** doubled the baseline tracking rewards (lin_vel=2.0, yaw=1.0). Mean reward improved to -14.52, linear velocity reached 1.541 m/s, still no crashes. The pattern was consistent.

**Run_04** tripled the baseline tracking rewards (lin_vel=3.0, yaw=1.5). Linear velocity reached 2.701 m/s (nearly 3x baseline) and mean reward improved to -11.28. Episode length stayed at maximum (999 steps). Looking at penalties: contact force shaping was largest at -1.517, then action rate (-0.906), Raibert heuristic (-0.262), and feet clearance (-0.201). This became our baseline for later experiments.

### Penalty Exploration: Runs 05-09

After Run_04, we tried reducing penalties to get the mean reward positive while keeping locomotion quality.

**Run_05** cut the largest penalty (contact force shaping) from 0.4 to 0.2. This failed completely - the robots barely moved. Why? The contact force penalty penalizes foot-ground forces during swing phase: `penalty = -(1 - desired_contact_states) * (1 - exp(-foot_force^2/100))`. Halving it meant weaker incentive to lift feet, so the policy learned to drag feet instead of walking properly. This taught us that contact force penalty at 0.4 is essential and can't be reduced.

**Run_06** tried smaller penalties instead: action rate from -0.1 to -0.05 and feet clearance from -30.0 to -15.0. Training worked but performance barely changed from Run_04 (lin_vel~2.70), so these penalties weren't the bottleneck.

**Run_07** combined stricter termination (base_height_min from 0.05m to 0.20m) with Run_05's reduced contact force penalty. This also failed with poor performance like Run_05. Lesson: harsh termination plus weak gait penalties doesn't work.

**Run_08** went back to Run_04's penalties and increased tracking rewards by 33% (lin_vel=4.0, yaw=2.0). Surprisingly, performance got worse: linear velocity dropped to 1.99 m/s (27% worse than Run_04). Not sure why - maybe training instability at this intermediate reward scale.

**Run_09** tested just reducing action rate penalty to -0.05 with Run_04's tracking rewards (3.0/1.5). Linear velocity matched Run_04 at 2.69 m/s, confirming action rate penalty wasn't the issue. Run_04's setup was still best for the 3.0/1.5 scale.

### Higher Tracking Rewards: Run_10

The instructor mentioned good performance should reach `track_lin_vel_xy_exp` around 48. Run_04 got 2.701, way below that. So we tried much higher tracking rewards.

**Run_10** massively increased tracking rewards: linear velocity to 16.0 (5.3x from Run_04) and yaw rate to 8.0. Kept all Run_04 penalties the same. Results were much better: linear velocity reached 15.56 m/s with no crashes. That's about 32% of the instructor's target. The tracking reward potential went from ~6/step in Run_04 to ~32/step.

However, watching the training revealed robot-to-robot collisions (4096 environments spaced 4.0m apart). The collision metric (`base_contact`) showed 0.000 despite visible collisions. Looking at the code: the metric only tracks forces on the robot's torso: `base_contact_force_magnitude = torch.norm(net_contact_forces[:, :, self._base_id])`. Leg-to-robot or foot-to-robot collisions between neighboring environments aren't captured. So the metric measures torso-ground contact for termination, not robot-robot interference during training.

### Domain Randomization: Runs 11a-11b

Run_10 performed well in simulation, but the real robot will face different surfaces (carpet vs. concrete), payload changes, actuator wear, and bumps. Domain randomization trains on varied physics parameters to handle this.

**Run_11a** tried aggressive randomization from scratch with wide friction ranges (0.5-1.25) and standard command ranges (±1.0 m/s). This failed: mean reward -61.10, track_lin_vel_xy_exp 0.78 (97% worse than Run_04's 2.70). Too hard to learn from scratch.

**Run_11b (Job ID: 135552)** used a more conservative approach:
- Narrower friction: [0.8, 1.2] for static/dynamic friction, [0.0, 0.05] for restitution
- Reduced command ranges: ±0.6 m/s instead of ±1.0 m/s to make learning easier
- 100 buckets for material properties

This worked much better: track_lin_vel_xy_exp recovered to 2.14 (175% improvement over Run_11a), though still 21% below Run_04's 2.70. Mean reward was more negative (-30.96 vs. -11.28), probably from foot slip penalties and harder dynamics. But training was stable with no crashes.

### Summary of Experimental Runs

| Run | Base Run | Variable Changes | Values Changed | Job ID |
|-----|----------|-----------------|----------------|--------|
| 01 | Tutorial | Baseline | `lin_vel_reward_scale=1.0`<br>`yaw_rate_reward_scale=0.5` | 134852 |
| 02 | Run_01 | Tracking rewards (+50%) | `lin_vel_reward_scale`: 1.0→**1.5**<br>`yaw_rate_reward_scale`: 0.5→**0.75** | 134868 |
| 03 | Run_01 | Tracking rewards (+100%) | `lin_vel_reward_scale`: 1.0→**2.0**<br>`yaw_rate_reward_scale`: 0.5→**1.0** | 134881 |
| 04 | Run_01 | Tracking rewards (+200%) | `lin_vel_reward_scale`: 1.0→**3.0**<br>`yaw_rate_reward_scale`: 0.5→**1.5** | 134916 |
| 05 | Run_04 | Contact force penalty (-50%) | `tracking_contacts_shaped_force_reward_scale`: 0.4→**0.2** | 135062 |
| 06 | Run_04 | Action + clearance penalties | `action_rate_reward_scale`: -0.1→**-0.05**<br>`feet_clearance_reward_scale`: -30.0→**-15.0** | 135327 |
| 07 | Run_05 | Contact force + termination | `tracking_contacts_shaped_force_reward_scale`: 0.4→**0.2**<br>`base_height_min`: 0.05→**0.20** | 135037 |
| 08 | Run_04 | Tracking rewards (+33%) | `lin_vel_reward_scale`: 3.0→**4.0**<br>`yaw_rate_reward_scale`: 1.5→**2.0** | 135358 |
| 09 | Run_04 | Action rate penalty (-50%) | `action_rate_reward_scale`: -0.1→**-0.05** | 135367 |
| 10 | Run_04 | Tracking rewards (+433%) | `lin_vel_reward_scale`: 3.0→**16.0**<br>`yaw_rate_reward_scale`: 1.5→**8.0** | 135378 |
| 11a | Run_04 | Domain randomization (failed) | Wide friction ranges: (0.5-1.25)<br>Standard command ranges ±1.0 m/s | 135457 |
| 11b | Run_04 | Friction DR | Narrow friction: (0.8-1.2)<br>Reduced commands: ±0.6 m/s | 135552 |
| 12 | Run_11b | Joint friction + high rewards | Joint friction: stiction `U(0.0, 2.5)`, viscous `U(0.0, 0.3)` (1 scalar/env)<br>High tracking rewards: `lin_vel_reward_scale=16.0`, `yaw_rate_reward_scale=8.0` | 135705 |

---

## Part 3: Bonus Implementation - Joint Friction

Real actuators have stiction and viscous damping that the basic PD controller doesn't account for. To better match real hardware, we implemented a joint friction model that subtracts friction torques from the PD control signal.

### Joint Friction Model

The torque applied to each joint is modified as:
```
τ = τ_PD - (τ_stiction + τ_viscous)
τ_stiction = k_stiction · tanh(q̇ / v₀)
τ_viscous = k_viscous · q̇
```

Where:
- `τ_PD = Kp(q_des - q) - Kd·q̇` is the standard PD control torque
- `k_stiction` is the stiction coefficient (models static friction at low velocities)
- `k_viscous` is the viscous damping coefficient (models friction proportional to velocity)
- `v₀ = 0.1 rad/s` is the velocity scale for the tanh transition

### Implementation Details

Friction coefficients are sampled randomly per environment at reset:
- k_stiction ∼ U(0.0, 2.5) N·m
- k_viscous ∼ U(0.0, 0.3) N·m·s

One coefficient per environment applies to all 12 joints, giving different friction levels across the 4096 parallel environments. This randomization helps the policy become robust to actuator variations.

The implementation is in `rob6323_go2_env.py`:
- Lines 45-47: Storage for friction coefficients
- Lines 154-158: Application in `_apply_action()` method
- Lines 342-355: Sampling at reset

### Run_12: Testing Joint Friction

**Run_12** (Job ID: 135705, running) combines:
- Joint friction enabled with randomization
- Run_11b's domain randomization (narrow friction ranges 0.8-1.2, reduced command ranges ±0.6 m/s)
- Run_10's high tracking rewards (lin_vel=16.0, yaw=8.0)

This tests whether we can achieve both high performance (from strong tracking rewards) and robustness (from domain randomization + actuator friction) simultaneously. The combination of external dynamics randomization (surface friction) and actuator randomization (joint friction) should improve sim-to-real transfer.

In the available TensorBoard logs (final iteration), Run_12 reached mean reward 369.17 with `track_lin_vel_xy_exp=15.77` and `track_ang_vel_z_exp=7.85` (episode length 999). Tracking errors were low (`lin_vel_error=0.045`, `ang_vel_error=0.051`). These results are close to Run_10’s tracking performance, but Run_12 also includes randomized joint friction, so it is a more difficult training setup. This is still a simulation result, so rollout inspection and hardware testing are needed to confirm robustness.

---

## Part 4: Final Comments

### Key Lessons

**Reward Tuning:** Increasing tracking rewards from 1.0 to 16.0 for linear velocity improved performance from 0.98 to 15.56 m/s without crashes. Key lessons: (1) tracking rewards scale well when penalties are balanced correctly, (2) contact force shaping penalty at 0.4 is critical for proper gait and can't be reduced without breaking walking, (3) reward scaling isn't always linear - Run_08 performed worse at 4.0/2.0 than Run_04 at 3.0/1.5.

**Domain Randomization:** Aggressive randomization from scratch doesn't work (Run_11a failed at 0.78 m/s). A more conservative approach worked better: Run_11b used narrower friction ranges (0.8-1.2) plus reduced command ranges (±0.6 m/s) to get 2.14 m/s with stable training.

**Joint Friction:** Added stiction and viscous damping to model real actuator friction on top of Run_11b's randomization. Combined with Run_10's higher tracking rewards (16.0/8.0) in Run_12 to test if we can achieve both high performance and robustness.

### Next Steps

The progression from optimizing rewards (Run_10) to domain randomization (Run_11b) to actuator modeling (Run_12) should help with transferring to the real robot, though we won't know until we actually test it. Future work would involve deploying the trained policy on the physical Go2 robot and iterating based on real-world performance.
