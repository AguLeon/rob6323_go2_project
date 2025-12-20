# ROB6323 Go2 Project — Isaac Lab

----
Note: **Logs for all runs available here:** [Google Drive folder](https://drive.google.com/drive/folders/1iKTXRSzleZ_O_0G2LWdkNHcRCAVvaGXI?usp=sharing)
----

This repository is the starter code for the NYU Reinforcement Learning and Optimal Control project in which students train a Unitree Go2 walking policy in Isaac Lab starting from a minimal baseline and improve it via reward shaping and robustness strategies. Please read this README fully before starting and follow the exact workflow and naming rules below to ensure your runs integrate correctly with the cluster scripts and grading pipeline.

## Repository policy

- Fork this repository and do not change the repository name in your fork.  
- Your fork must be named rob6323_go2_project so cluster scripts and paths work without modification.

### Prerequisites

- **GitHub Account:** You must have a GitHub account to fork this repository and manage your code. If you do not have one, [sign up here](https://github.com/join).

### Links
1.  **Project Webpage:** [https://machines-in-motion.github.io/RL_class_go2_project/](https://machines-in-motion.github.io/RL_class_go2_project/)
2.  **Project Tutorial:** [https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md](https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md)

## Connect to Greene

- Connect to the NYU Greene HPC via SSH; if you are off-campus or not on NYU Wi‑Fi, you must connect through the NYU VPN before SSHing to Greene.  
- The official instructions include example SSH config snippets and commands for greene.hpc.nyu.edu and dtn.hpc.nyu.edu as well as VPN and gateway options: https://sites.google.com/nyu.edu/nyu-hpc/accessing-hpc?authuser=0#h.7t97br4zzvip.

## Clone in $HOME

After logging into Greene, `cd` into your home directory (`cd $HOME`). You must clone your fork into `$HOME` only (not scratch or archive). This ensures subsequent scripts and paths resolve correctly on the cluster. Since this is a private repository, you need to authenticate with GitHub. You have two options:

### Option A: Via VS Code (Recommended)
The easiest way to avoid managing keys manually is to configure **VS Code Remote SSH**. If set up correctly, VS Code forwards your local credentials to the cluster.
- Follow the [NYU HPC VS Code guide](https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code) to set up the connection.

> **Tip:** Once connected to Greene in VS Code, you can clone directly without using the terminal:
> 1. **Sign in to GitHub:** Click the "Accounts" icon (user profile picture) in the bottom-left sidebar. If you aren't signed in, click **"Sign in with GitHub"** and follow the browser prompts to authorize VS Code.
> 2. **Clone the Repo:** Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`), type **Git: Clone**, and select it.
> 3. **Select Destination:** When prompted, select your home directory (`/home/<netid>/`) as the clone location.
>
> For more details, see the [VS Code Version Control Documentation](https://code.visualstudio.com/docs/sourcecontrol/intro-to-git#_clone-a-repository-locally).

### Option B: Manual SSH Key Setup
If you prefer using a standard terminal, you must generate a unique SSH key on the Greene cluster and add it to your GitHub account:
1. **Generate a key:** Run the `ssh-keygen` command on Greene (follow the official [GitHub documentation on generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key)).
2. **Add the key to GitHub:** Copy the output of your public key (e.g., `cat ~/.ssh/id_ed25519.pub`) and add it to your account settings (follow the [GitHub documentation on adding a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)).

### Execute the Clone
Once authenticated, run the following commands. Replace `<your-git-ssh-url>` with the SSH URL of your fork (e.g., `git@github.com:YOUR_USERNAME/rob6323_go2_project.git`).
```
cd $HOME
git clone <your-git-ssh-url> rob6323_go2_project
```
*Note: You must ensure the target directory is named exactly `rob6323_go2_project`. This ensures subsequent scripts and paths resolve correctly on the cluster.*
## Install environment

- Enter the project directory and run the installer to set up required dependencies and cluster-side tooling.  
```
cd $HOME/rob6323_go2_project
./install.sh
```
Do not skip this step, as it configures the environment expected by the training and evaluation scripts. It will launch a job in burst to set up things and clone the IsaacLab repo inside your greene storage. You must wait until the job in burst is complete before launching your first training. To check the progress of the job, you can run `ssh burst "squeue -u $USER"`, and the job should disappear from there once it's completed. It takes around **30 minutes** to complete. 
You should see something similar to the screenshot below (captured from Greene):

![Example burst squeue output](docs/img/burst_squeue_example.png)

In this output, the **ST** (state) column indicates the job status:
- `PD` = pending in the queue (waiting for resources).
- `CF` = instance is being configured.
- `R`  = job is running.

On burst, it is common for an instance to fail to configure; in that case, the provided scripts automatically relaunch the job when this happens, so you usually only need to wait until the job finishes successfully and no longer appears in `squeue`.

## What to edit

- In this project you'll only have to modify the two files below, which define the Isaac Lab task and its configuration (including PPO hyperparameters).  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py
PPO hyperparameters are defined in source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/agents/rsl_rl_ppo_cfg.py, but you shouldn't need to modify them.

## How to edit

- Option A (recommended): Use VS Code Remote SSH from your laptop to edit files on Greene; follow the NYU HPC VS Code guide and connect to a compute node as instructed (VPN required off‑campus) (https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code). If you set it correctly, it makes the login process easier, among other things, e.g., cloning a private repo.
- Option B: Edit directly on Greene using a terminal editor such as nano.  
```
nano source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py
```
- Option C: Develop locally on your machine, push to your fork, then pull changes on Greene within your $HOME/rob6323_go2_project clone.

> **Tip:** Don't forget to regularly push your work to github

## Launch training

- From $HOME/rob6323_go2_project on Greene, submit a training job via the provided script.  
```
cd "$HOME/rob6323_go2_project"
./train.sh
```
- Check job status with SLURM using squeue on the burst head node as shown below.  
```
ssh burst "squeue -u $USER"
```
Be aware that jobs can be canceled and requeued by the scheduler or underlying provider policies when higher-priority work preempts your resources, which is normal behavior on shared clusters using preemptible partitions.

## Where to find results

- When a job completes, logs are written under logs in your project clone on Greene in the structure logs/[job_id]/rsl_rl/go2_flat_direct/[date_time]/.  
- Inside each run directory you will find a TensorBoard events file (events.out.tfevents...), neural network checkpoints (model_[epoch].pt), YAML files with the exact PPO and environment parameters, and a rollout video under videos/play/ that showcases the trained policy.  

## Download logs to your computer

Use `rsync` to copy results from the cluster to your local machine. It is faster and can resume interrupted transfers. Run this on your machine (NOT on Greene):

```
rsync -avzP -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' <netid>@dtn.hpc.nyu.edu:/home/<netid>/rob6323_go2_project/logs ./
```

*Explanation of flags:*
- `-a`: Archive mode (preserves permissions, times, and recursive).
- `-v`: Verbose output.
- `-z`: Compresses data during transfer (faster over network).
- `-P`: Shows progress bar and allows resuming partial transfers.

## Visualize with TensorBoard

You can inspect training metrics (reward curves, loss values, episode lengths) using TensorBoard. This requires installing it on your local machine.

1.  **Install TensorBoard:**
    On your local computer (do NOT run this on Greene), install the package:
    ```
    pip install tensorboard
    ```

2.  **Launch the Server:**
    Navigate to the folder where you downloaded your logs and start the server:
    ```
    # Assuming you are in the directory containing the 'logs' folder
    tensorboard --logdir ./logs
    ```

3.  **View Metrics:**
    Open your browser to the URL shown (usually `http://localhost:6006/`).

## Debugging on Burst

Burst storage is accessible only from a job running on burst, not from the burst login node. The provided scripts do not automatically synchronize error logs back to your home directory on Greene. However, you will need access to these logs to debug failed jobs. These error logs differ from the logs in the previous section.

The suggested way to inspect these logs is via the Open OnDemand web interface:

1.  Navigate to [https://ood-burst-001.hpc.nyu.edu](https://ood-burst-001.hpc.nyu.edu).
2.  Select **Files** > **Home Directory** from the top menu.
3.  You will see a list of files, including your `.err` log files.
4.  Click on any `.err` file to view its content directly in the browser.

> **Important:** Do not modify anything inside the `rob6323_go2_project` folder on burst storage. This directory is managed by the job scripts, and manual changes may cause synchronization issues or job failures.

## Project scope reminder

- The assignment expects you to go beyond velocity tracking by adding principled reward terms (posture stabilization, foot clearance, slip minimization, smooth actions, contact and collision penalties), robustness via domain randomization, and clear benchmarking metrics for evaluation as described in the course guidelines.  
- Keep your repository organized, document your changes in the README, and ensure your scripts are reproducible, as these factors are part of grading alongside policy quality and the short demo video deliverable.

## Resources

- [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/ecosystem.html) — Everything you need to know about IsaacLab, and more!
- [Isaac Lab ANYmal C environment](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/direct/anymal_c) — This targets ANYmal C (not Unitree Go2), so use it as a reference and adapt robot config, assets, and reward to Go2.
- [DMO (IsaacGym) Go2 walking project page](https://machines-in-motion.github.io/DMO/) • [Go2 walking environment used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/tasks/go2_terrain.py) • [Config file used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/cfg/task/Go2Terrain.yaml) — Look at the function `compute_reward_CaT` (beware that some reward terms have a weight of 0 and thus are deactivated, check weights in the config file); this implementation includes strong reward shaping, domain randomization, and training disturbances for robust sim‑to‑real, but it is written for legacy IsaacGym and the challenge is to re-implement it in Isaac Lab.
- **API References**:
    - [ArticulationData (`robot.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html#isaaclab.assets.ArticulationData) — Contains `root_pos_w`, `joint_pos`, `projected_gravity_b`, etc.
    - [ContactSensorData (`_contact_sensor.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html#isaaclab.sensors.ContactSensorData) — Contains `net_forces_w` (contact forces).

---
Students should only edit README.md below this line.

-------------------------------------------------------------------


# ROB6323 Go2 Project - Code Modifications (Run_12 selected as 'best' run)

----
Note: **Logs for all runs available here:** [Google Drive folder](https://drive.google.com/drive/folders/1iKTXRSzleZ_O_0G2LWdkNHcRCAVvaGXI?usp=sharing)
----

This document summarizes the **functional changes** we implemented on top of the starter repository to improve locomotion learning and prepare for domain randomization. It focuses on **new reward/penalty terms and environment functionality**, not on per-run hyperparameter sweeps.

**Selected run:** **Run_12** (high tracking rewards + Stage 1 surface-friction randomization + joint friction).

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
| 12 | Run_11b | Joint friction + high rewards | Joint friction: stiction `U(0.0, 2.5)`, viscous `U(0.0, 0.3)` (1 scalar/env)<br>High tracking rewards: `lin_vel_reward_scale=16.0`, `yaw_rate_reward_scale=8.0` | 15.77 | 135705 |

## Where the changes live

- Environment: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py`
- Environment config: `source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py`

## Run_12 - Selected parameter values

These are the key parameters used for **Run_12** (high rewards + Stage 1 surface friction DR + joint friction):

- **Selected Gym task ID:** `Template-Rob6323-Go2-Direct-Run12-v0`
- **Simulation / scene**
  - `dt=1/200`, `decimation=4`, `episode_length_s=20.0`
  - `num_envs=4096`, `env_spacing=4.0`
  - Physics materials: `friction_combine_mode="multiply"`, `restitution_combine_mode="multiply"`
- **Spaces**
  - `action_space=12`, `action_scale=0.25`
  - `observation_space=52` (includes 4 gait clock inputs)
- **Controller**
  - `Kp=20.0`, `Kd=0.5`, `torque_limits=100.0`
- **Tracking rewards (Run_10-level)**
  - `lin_vel_reward_scale=16.0`, `yaw_rate_reward_scale=8.0`
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
- **Joint friction (enabled)**
  - `enable_joint_friction=True`, `randomize_joint_friction=True`
  - `stiction_range=(0.0, 2.5)`, `viscous_range=(0.0, 0.3)`, `stiction_velocity_scale=0.1`
- **Contact sensor**
  - `history_length=3`, `update_period=0.005`, `track_air_time=True`


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

## Domain randomization

We implemented domain randomization using Isaac Lab's `EventManager` interface:

- Randomize rigid-body material properties at reset using narrower friction ranges (0.8-1.2) to avoid performance collapse.
- When `events` are enabled, reduce commanded linear velocity ranges to ±0.6 m/s (easier curriculum under DR).

## Part 7 - Joint friction torque model (used in Run_12 plan)

To better match real hardware, we added an optional joint friction model that subtracts stiction + viscous torques from the PD torques:

- `tau = tau_PD - (tau_stiction + tau_viscous)`
- `tau_stiction = k_stiction * tanh(dq / v0)` with `k_stiction ~ U(0.0, 2.5)` sampled per-environment at reset
- `tau_viscous = k_viscous * dq` with `k_viscous ~ U(0.0, 0.3)` sampled per-environment at reset

Config knobs: `enable_joint_friction`, `randomize_joint_friction`, `stiction_range`, `viscous_range`, `stiction_velocity_scale`.
