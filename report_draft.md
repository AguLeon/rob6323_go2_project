# Quadruped Locomotion Training Report: Unitree Go2 Robot

**Author:** Agustin Leon and Asrita Bobba
**Course:** ROB-GY 6323 - Reinforcement Learning for Robotics
**Robot Platform:** Unitree Go2 Quadruped
**Framework:** Isaac Lab + RSL-RL (PPO Algorithm)
**Compute:** NYU Greene HPC Cluster

---

## Project Overview

This report documents our work developing a locomotion policy for the Unitree Go2 quadruped robot using reinforcement learning. We started with the Isaac Lab tutorial baseline and experimented with reward tuning, penalty balancing, and domain randomization. The goal is to eventually transfer the trained policy to the physical robot.

## Initial Implementation: Following the Tutorial

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

## Baseline Optimization: Runs 01-04

**Run_01** was the tutorial baseline with default reward scales. Training on 4096 parallel environments for 500 iterations gave mean reward of -18.86 and linear velocity of 0.983 m/s. The robot walked upright with no crashes, but the negative mean reward showed penalties dominated the signal.

**Run_02** tested whether increasing tracking rewards would improve velocity. We increased linear velocity reward by 50% (1.0→1.5) and yaw rate proportionally (0.5→0.75). Mean reward improved to -16.73, linear velocity increased to 1.231 m/s, and the robot still didn't crash. This worked better than trying to reduce penalties.

**Run_03** doubled the baseline tracking rewards (lin_vel=2.0, yaw=1.0). Mean reward improved to -14.52, linear velocity reached 1.541 m/s, still no crashes. The pattern was consistent.

**Run_04** tripled the baseline tracking rewards (lin_vel=3.0, yaw=1.5). Linear velocity reached 2.701 m/s (nearly 3x baseline) and mean reward improved to -11.28. Episode length stayed at maximum (999 steps). Looking at penalties: contact force shaping was largest at -1.517, then action rate (-0.906), Raibert heuristic (-0.262), and feet clearance (-0.201). This became our baseline for later experiments.

## Penalty Exploration: Runs 05-09

After Run_04, we tried reducing penalties to get the mean reward positive while keeping locomotion quality.

**Run_05** cut the largest penalty (contact force shaping) from 0.4 to 0.2. This failed completely - the robots barely moved. Why? The contact force penalty penalizes foot-ground forces during swing phase: `penalty = -(1 - desired_contact_states) * (1 - exp(-foot_force^2/100))`. Halving it meant weaker incentive to lift feet, so the policy learned to drag feet instead of walking properly. This taught us that contact force penalty at 0.4 is essential and can't be reduced.

**Run_06** tried smaller penalties instead: action rate from -0.1 to -0.05 and feet clearance from -30.0 to -15.0. Training worked but performance barely changed from Run_04 (lin_vel~2.70), so these penalties weren't the bottleneck.

**Run_07** combined stricter termination (base_height_min from 0.05m to 0.20m) with Run_05's reduced contact force penalty. This also failed with poor performance like Run_05. Lesson: harsh termination plus weak gait penalties doesn't work.

**Run_08** went back to Run_04's penalties and increased tracking rewards by 33% (lin_vel=4.0, yaw=2.0). Surprisingly, performance got worse: linear velocity dropped to 1.99 m/s (27% worse than Run_04). Not sure why - maybe training instability at this intermediate reward scale.

**Run_09** tested just reducing action rate penalty to -0.05 with Run_04's tracking rewards (3.0/1.5). Linear velocity matched Run_04 at 2.69 m/s, confirming action rate penalty wasn't the issue. Run_04's setup was still best for the 3.0/1.5 scale.

## Run 10: Higher Tracking Rewards

The instructor mentioned good performance should reach `track_lin_vel_xy_exp` around 48. Run_04 got 2.701, way below that. So we tried much higher tracking rewards.

**Run_10** massively increased tracking rewards: linear velocity to 16.0 (5.3x from Run_04) and yaw rate to 8.0. Kept all Run_04 penalties the same. Results were much better: linear velocity reached 15.56 m/s with no crashes. That's about 32% of the instructor's target. The tracking reward potential went from ~6/step in Run_04 to ~32/step.

However, watching the training revealed robot-to-robot collisions (4096 environments spaced 4.0m apart). The collision metric (`base_contact`) showed 0.000 despite visible collisions. Looking at the code: the metric only tracks forces on the robot's torso: `base_contact_force_magnitude = torch.norm(net_contact_forces[:, :, self._base_id])`. Leg-to-robot or foot-to-robot collisions between neighboring environments aren't captured. So the metric measures torso-ground contact for termination, not robot-robot interference during training.

## Domain Randomization: Runs 11a-11c

Run_10 performed well in simulation, but the real robot will face different surfaces (carpet vs. concrete), payload changes, actuator wear, and bumps. Domain randomization trains on varied physics parameters to handle this.

**Run_11a** tried aggressive randomization from scratch with wide friction ranges (0.5-1.25) and standard command ranges (±1.0 m/s). This failed: mean reward -61.10, track_lin_vel_xy_exp 0.78 (97% worse than Run_04's 2.70). Too hard to learn from scratch.

**Run_11b - Stage 1 DR (Job ID: 135552)** used a staged approach:
- Narrower friction: [0.8, 1.2] for static/dynamic friction, [0.0, 0.05] for restitution
- Reduced command ranges: ±0.6 m/s instead of ±1.0 m/s to make learning easier
- 100 buckets for material properties

This worked much better: track_lin_vel_xy_exp recovered to 2.14 (175% improvement over Run_11a), though still 21% below Run_04's 2.70. Mean reward was more negative (-30.96 vs. -11.28), probably from foot slip penalties and harder dynamics. But training was stable with no crashes.

**Run_11c - Stage 2 DR (Job ID: 135580, completed)** expanded randomization:
- Wider friction: [0.5, 1.25] for static/dynamic friction
- Base mass randomization: ±1-3 kg payload variation
- Kept reduced command ranges from Stage 1

The two-stage approach (narrow→wide) helps the policy adapt gradually instead of failing immediately.

## Joint Friction Model: Run_12

**Run_12** (Job ID: 135624, running) adds a joint friction model on top of Run_11c. Real actuators have stiction and viscous damping that the basic PD controller doesn't account for.

**Joint Friction Model:**
The torque applied to each joint subtracts friction from PD control:
```
τ = τ_PD - (τ_stiction + τ_viscous)
τ_stiction = k_stiction · tanh(q̇ / v₀)
τ_viscous = k_viscous · q̇
```

Friction coefficients are sampled randomly per environment at reset:
- k_stiction ∼ U(0.0, 2.5) N·m
- k_viscous ∼ U(0.0, 0.3) N·m·s
- v₀ = 0.1 rad/s (stiction velocity scale)

One coefficient per environment applies to all 12 joints, giving different friction across the 4096 parallel environments.

**Changes from Run_11c:**
- Tracking rewards increased to Run_10 levels: lin_vel=16.0, yaw=8.0
- Joint friction enabled with randomization
- Keeps Run_11c's Stage 2 DR (wide friction + mass randomization)
- Collision penalty stays at -1.0

This combines randomization of external dynamics (surface friction, mass) with actuator randomization (joint friction) for better sim-to-real transfer.

## Summary of Key Results

| Run | Base Run | Variable Changes | Values Changed | Lin Vel | Job ID |
|-----|----------|-----------------|----------------|---------|--------|
| 01 | Tutorial | Baseline | `lin_vel_reward_scale=1.0`<br>`yaw_rate_reward_scale=0.5`<br>`tracking_contacts_shaped_force_reward_scale=0.4`<br>`action_rate_reward_scale=-0.1`<br>`feet_clearance_reward_scale=-30.0`<br>`base_height_min=0.05` | 0.983 | 134852 |
| 02 | Run_01 | Tracking rewards (+50%) | `lin_vel_reward_scale`: 1.0→**1.5**<br>`yaw_rate_reward_scale`: 0.5→**0.75** | 1.231 | 134868 |
| 03 | Run_01 | Tracking rewards (+100%) | `lin_vel_reward_scale`: 1.0→**2.0**<br>`yaw_rate_reward_scale`: 0.5→**1.0** | 1.541 | 134881 |
| 04 | Run_01 | Tracking rewards (+200%) | `lin_vel_reward_scale`: 1.0→**3.0**<br>`yaw_rate_reward_scale`: 0.5→**1.5** | 2.701 | 134916 |
| 05 | Run_04 | Contact force penalty (-50%) | `tracking_contacts_shaped_force_reward_scale`: 0.4→**0.2** | Failed | 135062 |
| 06 | Run_04 | Action + clearance penalties | `action_rate_reward_scale`: -0.1→**-0.05**<br>`feet_clearance_reward_scale`: -30.0→**-15.0** | ~2.70 | 135327 |
| 07 | Run_05 | Contact force + termination | `tracking_contacts_shaped_force_reward_scale`: 0.4→**0.2**<br>`base_height_min`: 0.05→**0.20** | Failed | 135037 |
| 08 | Run_04 | Tracking rewards (+33%) | `lin_vel_reward_scale`: 3.0→**4.0**<br>`yaw_rate_reward_scale`: 1.5→**2.0** | 1.99 | 135358 |
| 09 | Run_04 | Action rate penalty (-50%) | `action_rate_reward_scale`: -0.1→**-0.05** | 2.69 | 135367 |
| 10 | Run_04 | Tracking rewards (+433%) | `lin_vel_reward_scale`: 3.0→**16.0**<br>`yaw_rate_reward_scale`: 1.5→**8.0** | 15.56 | 135378 |
| 11a | Run_04 | Domain randomization | Wide friction ranges: (0.5-1.25)<br>Standard command ranges ±1.0 m/s | 0.78 | 135457 |
| 11b | Run_04 | Friction DR (stage 1) | Narrow friction: (0.8-1.2)<br>Reduced commands: ±0.6 m/s | 2.14 | 135552 |
| 11c | Run_04 | Friction + mass DR (stage 2) | Wide friction: (0.5-1.25)<br>Mass randomization ±1-3 kg<br>Reduced commands: ±0.6 m/s | TBD | 135580 |
| 12 | Run_11c | DR + joint friction | Tracking rewards: 16.0/8.0<br>Joint friction: stiction (0.0-2.5), viscous (0.0-0.3)<br>Maintains Run_11c DR | Running | 135624 |

## Conclusions

We made progress through three phases:

**Phase 1 - Reward Tuning (Runs 01-10):** Increasing tracking rewards from 1.0 to 16.0 for linear velocity improved performance from 0.98 to 15.56 m/s without crashes. Key lessons: (1) tracking rewards scale well when penalties are balanced correctly, (2) contact force shaping penalty at 0.4 is critical for proper gait and can't be reduced without breaking walking, (3) reward scaling isn't always linear - Run_08 performed worse at 4.0/2.0 than Run_04 at 3.0/1.5.

**Phase 2 - Domain Randomization (Runs 11a-11c):** We learned that aggressive randomization from scratch doesn't work (Run_11a failed at 0.78 m/s). A staged approach worked better: Run_11b used narrower friction ranges plus reduced command ranges to get 2.14 m/s. Run_11c then expanded to wider friction and added mass randomization. The gradual approach prevents training from collapsing.

**Phase 3 - Joint Friction (Run_12):** Added stiction and viscous damping to model real actuator friction on top of Run_11c's randomization. Combined with Run_10's higher tracking rewards (16.0/8.0) to see if we can get both high performance and robustness.

The progression from optimizing rewards (Run_10) to staged randomization (Run_11b→11c) to actuator modeling (Run_12) should help with transferring to the real robot, though we won't know until we actually test it.
