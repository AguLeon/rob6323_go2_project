# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.markers.config import BLUE_ARROW_X_MARKER_CFG, FRAME_MARKER_CFG, GREEN_ARROW_X_MARKER_CFG
from isaaclab.actuators import ImplicitActuatorCfg

@configclass
class Rob6323Go2EventCfgStage1:
    """Domain randomization events (Stage 1).

    Narrow friction ranges to make DR learnable without collapsing performance.
    """

    robot_physics_material = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 1.2),
            "dynamic_friction_range": (0.8, 1.2),
            "restitution_range": (0.0, 0.05),
            "num_buckets": 100,
        },
    )


@configclass
class Rob6323Go2EventCfgStage2:
    """Domain randomization events (Stage 2).

    Wider friction ranges to improve robustness after Stage 1 is stable.
    """

    robot_physics_material = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.5, 1.25),
            "dynamic_friction_range": (0.5, 1.25),
            "restitution_range": (0.0, 0.1),
            "num_buckets": 250,
        },
    )

    robot_base_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            # Additive payload mass in kg. Negative values simulate lighter configurations.
            "mass_range": (-1.0, 3.0),
            "operation": "add",
        },
    )


@configclass
class Rob6323Go2EnvCfg(DirectRLEnvCfg):
    # env
    decimation = 4
    episode_length_s = 20.0
    # - spaces definition
    action_scale = 0.25
    action_space = 12
    observation_space = 52  # 48 base + 4 clock inputs
    state_space = 0

    # command sampling ranges (used in Rob6323Go2Env._reset_idx)
    command_lin_vel_x_range = (-1.0, 1.0)
    command_lin_vel_y_range = (-1.0, 1.0)
    command_yaw_rate_range = (-1.0, 1.0)

    # When DR events are enabled, optionally use narrower command ranges (matches reference implementation).
    use_dr_command_ranges = True
    command_lin_vel_x_range_dr = (-0.6, 0.6)
    command_lin_vel_y_range_dr = (-0.6, 0.6)
    command_yaw_rate_range_dr = (-1.0, 1.0)
    debug_vis = True
    raibert_heuristic_reward_scale = -1.0
    feet_clearance_reward_scale = -30.0
    tracking_contacts_shaped_force_reward_scale = 0.4

    # PD control gains
    Kp = 20.0  # Proportional gain
    Kd = 0.5   # Derivative gain
    torque_limits = 100.0  # Max torque

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 200,
        render_interval=decimation,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )
    # robot(s)
    robot_cfg: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # "base_legs" is an arbitrary key we use to group these actuators
    robot_cfg.actuators["base_legs"] = ImplicitActuatorCfg(
        joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
        effort_limit=23.5,
        velocity_limit=30.0,
        stiffness=0.0,  # CRITICAL: Set to 0 to disable implicit P-gain
        damping=0.0,    # CRITICAL: Set to 0 to disable implicit D-gain
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*", history_length=3, update_period=0.005, track_air_time=True
    )
    goal_vel_visualizer_cfg: VisualizationMarkersCfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/velocity_goal"
    )
    """The configuration for the goal velocity visualization marker. Defaults to GREEN_ARROW_X_MARKER_CFG."""

    current_vel_visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/velocity_current"
    )
    """The configuration for the current velocity visualization marker. Defaults to BLUE_ARROW_X_MARKER_CFG."""

    # Set the scale of the visualization markers to (0.5, 0.5, 0.5)
    goal_vel_visualizer_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)
    current_vel_visualizer_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)

    # reward scales
    lin_vel_reward_scale = 3.0
    yaw_rate_reward_scale = 1.5
    action_rate_reward_scale = -0.1

    # Additional reward scales
    orient_reward_scale = -5.0
    lin_vel_z_reward_scale = -0.02
    dof_vel_reward_scale = -0.0001
    ang_vel_xy_reward_scale = -0.001
    foot_slip_reward_scale = -0.1

    # termination
    base_height_min = 0.05

    # collision penalty
    base_collision_penalty_scale = -1.0

    # events (Run_11b): Stage 1 enabled by default; switch to `Rob6323Go2EventCfgStage2()` once stable.
    events: Rob6323Go2EventCfgStage1 | Rob6323Go2EventCfgStage2 | None = Rob6323Go2EventCfgStage1()


@configclass
class Rob6323Go2EnvCfgRun11c(Rob6323Go2EnvCfg):
    """Run_11c: Stage 2 domain randomization (wider friction + base mass)."""

    events: Rob6323Go2EventCfgStage1 | Rob6323Go2EventCfgStage2 | None = Rob6323Go2EventCfgStage2()
