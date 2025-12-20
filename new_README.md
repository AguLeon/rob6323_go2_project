# ROB6323 Go2 Project - Code Modifications (Run_11b selected; includes Run_12 plan)

This document summarizes the **functional changes** we implemented on top of the starter repository to improve locomotion learning and prepare for domain randomization. It focuses on **new reward/penalty terms and environment functionality**, not on per-run hyperparameter sweeps.

**Selected run:** **Run_11b** (Stage 1 domain randomization with an easier curriculum).

## Run summary table

The table below summarizes the main configuration changes explored across runs and the resulting linear velocity performance.

| Run | Base Run | Variable Changes | Values Changed | Lin Vel | Job ID |
|-----|----------|-----------------|----------------|---------|--------|
| 01 | Tutorial | Baseline | `lin_vel_reward_scale=1.0`<br>`yaw_rate_reward_scale=0.5`<br>`tracking_contacts_shaped_force_reward_scale=0.4`<br>`action_rate_reward_scale=-0.1`<br>`feet_clearance_reward_scale=-30.0`<br>`base_height_min=0.05` | 0.983 | 134852 |
| 02 | Run_01 | Tracking rewards (+50%) | `lin_vel_reward_scale`: 1.0->**1.5**<br>`yaw_rate_reward_scale`: 0.5->**0.75** | 1.231 | 134868 |
| 03 | Run_01 | Tracking rewards (+100%) | `lin_vel_reward_scale`: 1.0->**2.0**<br>`yaw_rate_reward_scale`: 0.5->**1.0** | 1.541 | 134881 |
| 04 | Run_01 | Tracking rewards (+200%) | `lin_vel_reward_scale`: 1.0->**3.0**<br>`yaw_rate_reward_scale`: 0.5->**1.5** | 2.701 | 134916 |
| 05 | Run_04 | Contact force penalty (-50%) | `tracking_contacts_shaped_force_reward_scale`: 0.4->**0.2** | Failed | 135062 |
| 06 | Run_04 | Action + clearance penalties | `action_rate_reward_scale`: -0.1->**-0.05**<br>`feet_clearance_reward_scale`: -30.0->**-15.0** | ~2.70 | 135327 |
| 07 | Run_05 | Contact force + termination | `tracking_contacts_shaped_force_reward_scale`: 0.4->**0.2**<br>`base_height_min`: 0.05->**0.20** | Failed | 135037 |
| 08 | Run_04 | Tracking rewards (+33%) | `lin_vel_reward_scale`: 3.0->**4.0**<br>`yaw_rate_reward_scale`: 1.5->**2.0** | 1.99 | 135358 |
| 09 | Run_04 | Action rate penalty (-50%) | `action_rate_reward_scale`: -0.1->**-0.05** | 2.69 | 135367 |
| 10 | Run_04 | Tracking rewards (+433%) | `lin_vel_reward_scale`: 3.0->**16.0**<br>`yaw_rate_reward_scale`: 1.5->**8.0** | 15.56 | 135378 |
| 11a | Run_04 | Domain randomization | Added `EventCfg` with:<br>`static_friction_range=(0.5, 1.25)`<br>`dynamic_friction_range=(0.5, 1.25)`<br>`restitution_range=(0.0, 0.1)` | 0.78 | 135457 |
| 11b | Run_04 | Friction DR (stage 1) | Train from scratch; enable friction randomization with narrow range (e.g., 0.8-1.2) + reduce command ranges to about +/-0.6 m/s | 2.14 | 135552 |
| 12 | Run_11b | DR + joint friction + high rewards | `lin_vel_reward_scale`: 3.0->**16.0**<br>`yaw_rate_reward_scale`: 1.5->**8.0**<br>Keep collision penalty: `base_collision_penalty_scale=-1.0`<br>Add joint friction: `stiction_range=(0.0, 2.5)`, `viscous_range=(0.0, 0.3)` (1 scalar/env) | Running | 135667 |

## Where the changes live

- Environment: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py`
- Environment config: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py`
- Gym registration: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/__init__.py`
- Training entry (CLI forwarding utility): `scripts/rsl_rl/train.py`

## Run_11b - Selected parameter values

These are the key parameters used for **Run_11b** in `Rob6323Go2EnvCfg` (friction DR Stage 1 enabled by default):

- **Selected Gym task ID:** `Template-Rob6323-Go2-Direct-v0`
- **Simulation / scene**
  - `dt=1/200`, `decimation=4`, `episode_length_s=20.0`
  - `num_envs=4096`, `env_spacing=4.0`
  - Physics materials: `friction_combine_mode="multiply"`, `restitution_combine_mode="multiply"`
- **Spaces**
  - `action_space=12`, `action_scale=0.25`
  - `observation_space=52` (includes 4 gait clock inputs)
- **Controller**
  - `Kp=20.0`, `Kd=0.5`, `torque_limits=100.0`
- **Tracking rewards (Run_04-level, used for DR stability)**
  - `lin_vel_reward_scale=3.0`, `yaw_rate_reward_scale=1.5`
- **Penalties / shaping terms**
  - `action_rate_reward_scale=-0.1`
  - `raibert_heuristic_reward_scale=-1.0`
  - `feet_clearance_reward_scale=-30.0`
  - `tracking_contacts_shaped_force_reward_scale=0.4`
  - `orient_reward_scale=-5.0`
  - `lin_vel_z_reward_scale=-0.02`
  - `dof_vel_reward_scale=-0.0001`
  - `ang_vel_xy_reward_scale=-0.001`
  - `foot_slip_reward_scale=-0.1`
  - `base_collision_penalty_scale=-1.0`
- **Terminations**
  - `base_height_min=0.05`
  - Torso contact termination when base contact force magnitude exceeds `1.0`
  - Upside-down termination when `projected_gravity_b[:, 2] > 0`
- **Command sampling**
  - Normal: `command_lin_vel_x_range=(-1.0, 1.0)`, `command_lin_vel_y_range=(-1.0, 1.0)`, `command_yaw_rate_range=(-1.0, 1.0)`
  - Under DR (enabled via `use_dr_command_ranges=True`): `command_lin_vel_x_range_dr=(-0.6, 0.6)`, `command_lin_vel_y_range_dr=(-0.6, 0.6)`, `command_yaw_rate_range_dr=(-1.0, 1.0)`
- **Domain randomization (Stage 1 events at reset)**
  - `static_friction_range=(0.8, 1.2)`
  - `dynamic_friction_range=(0.8, 1.2)`
  - `restitution_range=(0.0, 0.05)`
  - `num_buckets=100`
- **Contact sensor**
  - `history_length=3`, `update_period=0.005`, `track_air_time=True`

## Contact sensing + logging additions

- Registered the contact sensor with the scene so contact forces and contact history can be used for rewards/terminations.
- Added per-episode logging of reward components and performance metrics (e.g., velocity tracking errors, base height, pitch/roll) for easier TensorBoard debugging.

## Part 1 - Action smoothness (action-rate penalties)

To reduce "jerky" high-frequency control, we added action-rate penalties using an action history buffer:

- **First derivative:** penalize `||a_t - a_{t-1}||^2`
- **Second derivative:** penalize `||a_t - 2 a_{t-1} + a_{t-2}||^2`

Config knob: `Rob6323Go2EnvCfg.action_rate_reward_scale`.

## Part 2 - Manual low-level PD control (explicit torque computation)

We switched from implicit actuator control to a manually computed PD controller:

- Disabled implicit stiffness/damping in the actuator config (stiffness=0, damping=0)
- Computed torques from joint position error + joint velocity feedback
- Clamped torques by a configurable limit

Config knobs: `Rob6323Go2EnvCfg.Kp`, `Rob6323Go2EnvCfg.Kd`, `Rob6323Go2EnvCfg.torque_limits`.

## Part 3 - Early termination for failed episodes

We added/strengthened early-termination conditions so obviously failed rollouts end quickly:

- Base contact force threshold (torso contact)
- Upside-down detection (gravity projection sign)
- Base height minimum threshold (`base_height_min`)

Config knob: `Rob6323Go2EnvCfg.base_height_min`.

## Part 4 - Gait shaping (Raibert heuristic + gait clock inputs)

We implemented a gait-shaping term based on a Raibert-style foot placement heuristic and added gait phase features:

- Tracks a gait phase index and produces **4 clock inputs** (one per foot)
- Computes a Raibert foot-placement error and adds it as a penalty term
- Produces a smooth "desired contact state" signal per foot used by foot-related reward terms

Config knobs: `Rob6323Go2EnvCfg.observation_space` (52 includes 4 clock inputs), `Rob6323Go2EnvCfg.raibert_heuristic_reward_scale`.

## Part 5 - Reward refinement (posture and motion regularization)

We added several standard stabilization penalties to improve natural, stable locomotion:

- Base orientation penalty (projected gravity XY magnitude)
- Vertical linear velocity penalty (`v_z`)
- Joint velocity penalty
- Base angular velocity XY penalty

Config knobs:
- `orient_reward_scale`
- `lin_vel_z_reward_scale`
- `dof_vel_reward_scale`
- `ang_vel_xy_reward_scale`

## Part 6 - Foot interaction rewards/penalties

We added foot-level shaping using the contact sensor and the gait's desired contact state:

- **Feet clearance penalty:** penalize insufficient swing-foot height
- **Shaped contact-force term:** shapes contact force based on desired contact timing
- **Foot slip penalty:** penalize foot velocity when the foot is "supposed to be in contact"
- **Base collision penalty:** penalize torso contact force above a small threshold (separate from termination logic)

Config knobs:
- `feet_clearance_reward_scale`
- `tracking_contacts_shaped_force_reward_scale`
- `foot_slip_reward_scale`
- `base_collision_penalty_scale`

## Run_11b - Domain randomization

We implemented domain randomization using Isaac Lab's `EventManager` interface:

- **Run_11b:** randomize rigid-body material properties at reset using narrower friction ranges (0.8-1.2) to avoid performance collapse.
- **DR-aware command sampling:** when `events` are enabled, reduce commanded linear velocity ranges to ±0.6 m/s (easier curriculum under DR).

## Part 7 - Joint friction torque model (used in Run_12 plan)

To better match real hardware, we added an optional joint friction model that subtracts stiction + viscous torques from the PD torques:

- `tau = tau_PD - (tau_stiction + tau_viscous)`
- `tau_stiction = k_stiction * tanh(dq / v0)` with `k_stiction ~ U(0.0, 2.5)` sampled per-environment at reset
- `tau_viscous = k_viscous * dq` with `k_viscous ~ U(0.0, 0.3)` sampled per-environment at reset

Config knobs: `enable_joint_friction`, `randomize_joint_friction`, `stiction_range`, `viscous_range`, `stiction_velocity_scale`.

Relevant config:
- `Rob6323Go2EnvCfg.events` (Run_11b uses friction randomization)
- `Rob6323Go2EnvCfg.use_dr_command_ranges` and `*_range_dr` values
- `Rob6323Go2EventCfgStage1` (narrow friction ranges for stable training)

## Training utility (no `train.slurm` edits)

To support selecting different Gym task IDs without editing Slurm scripts, `scripts/rsl_rl/train.py` appends any args provided via the `ISAAC_ARGS` environment variable into `sys.argv` before parsing. This enables `./train.sh --task=...`-style overrides while keeping `train.slurm` untouched.
