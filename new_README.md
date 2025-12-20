# ROB6323 Go2 Project - Code Modifications (through Run_11b)

This document summarizes the **functional changes** we implemented on top of the starter repository to improve locomotion learning and prepare for domain randomization. It focuses on **new reward/penalty terms and environment functionality**, not on per-run hyperparameter sweeps.

## Where the changes live

- Environment: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py`
- Environment config: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py`
- Gym registration: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/__init__.py`
- Training entry (CLI forwarding utility): `scripts/rsl_rl/train.py`

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

## Run_11b - Domain randomization (staged, friction first)

We implemented domain randomization using Isaac Lab's `EventManager` interface:

- **Stage 1 (Run_11b):** randomize rigid-body material properties at reset using narrower friction ranges to avoid performance collapse.
- **DR-aware command sampling:** when `events` are enabled, optionally reduce commanded linear velocity ranges (easier curriculum under DR).

Relevant config:
- `Rob6323Go2EnvCfg.events` (Run_11b defaults to Stage 1)
- `Rob6323Go2EnvCfg.use_dr_command_ranges` and `*_range_dr` values
- `Rob6323Go2EventCfgStage1` / `Rob6323Go2EventCfgStage2` (Stage 2 adds wider friction; optional mass randomization is defined for follow-up runs)

## Training utility (no `train.slurm` edits)

To support selecting different Gym task IDs without editing Slurm scripts, `scripts/rsl_rl/train.py` appends any args provided via the `ISAAC_ARGS` environment variable into `sys.argv` before parsing. This enables `./train.sh --task=...`-style overrides while keeping `train.slurm` untouched.

## Optional: additional task registration for follow-up runs

- Added a separate Gym ID for a Stage 2 DR configuration:
  - `Template-Rob6323-Go2-Direct-Run11c-v0`
  - Uses `Rob6323Go2EnvCfgRun11c` (Stage 2 DR config)
