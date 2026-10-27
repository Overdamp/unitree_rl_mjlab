"""Unitree G1-23DOF velocity environment configurations."""

from dataclasses import replace

from mjlab.actuator import DelayedActuatorCfg
from src.assets.robots import (
  G1_23DOF_ACTION_SCALE,
  get_g1_23dof_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, RayCastSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from src.tasks.velocity.mdp import delayed_effort_limits
from src.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

# Max action latency in physics steps (sim dt = 0.005 s -> 20 ms).
# 0-40 ms (lag 8) collapsed forward walking to ~0.25 m/s; see tag phase3a-lag0-40ms-collapsed.
G1_23DOF_MAX_ACTION_LAG = 4


def unitree_g1_23dof_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Unitree G1-23DOF rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  cfg.sim.nconmax = 48

  # Wrap every actuator so its position target can be delayed. Lags are held fixed
  # (hold_prob=1.0) and only set per episode by the "action_delay" reset event.
  robot_cfg = get_g1_23dof_robot_cfg()
  assert robot_cfg.articulation is not None
  robot_cfg.articulation = replace(
    robot_cfg.articulation,
    actuators=tuple(
      DelayedActuatorCfg(
        base_cfg=actuator,
        delay_target="position",
        delay_min_lag=0,
        delay_max_lag=G1_23DOF_MAX_ACTION_LAG,
        delay_hold_prob=1.0,
      )
      for actuator in robot_cfg.articulation.actuators
    ),
  )
  cfg.scene.entities = {"robot": robot_cfg}

  # Set raycast sensor frame to G1-23DOF pelvis.
  for sensor in cfg.scene.sensors or ():
    if sensor.name == "terrain_scan":
      assert isinstance(sensor, RayCastSensorCfg)
      sensor.frame.name = "pelvis"

  site_names = ("left_foot", "right_foot")
  geom_names = tuple(
    f"{side}_foot{i}_collision" for side in ("left", "right") for i in range(1, 8)
  )

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree",
      pattern=r"^(left_ankle_roll_link|right_ankle_roll_link)$",
      entity="robot",
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  self_collision_cfg = ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern="pelvis", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="pelvis", entity="robot"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )
  cfg.scene.sensors = (cfg.scene.sensors or ()) + (
    feet_ground_cfg,
    self_collision_cfg,
  )

  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = G1_23DOF_ACTION_SCALE

  cfg.viewer.body_name = "torso_link"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 1.15

  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names

  cfg.events["foot_friction"].params["asset_cfg"].geom_names = geom_names
  cfg.events["base_com"].params["asset_cfg"].body_names = ("torso_link",)

  # Phase 2 Sim-to-Real Domain Randomization:
  # 1. Randomize torso mass payload (-1.0 to +3.5 kg) to cover real battery/hardware mass (~34.1-35.0 kg total).
  cfg.events["torso_mass_payload"] = EventTermCfg(
    mode="startup",
    func=dr.body_mass,
    params={
      "asset_cfg": SceneEntityCfg("robot", body_names=("torso_link",)),
      "operation": "add",
      "ranges": (-1.0, 3.5),
    },
  )
  # 2. Scale pelvis, leg, and arm link masses by +-10%.
  cfg.events["limb_mass_scale"] = EventTermCfg(
    mode="startup",
    func=dr.body_mass,
    params={
      "asset_cfg": SceneEntityCfg("robot", body_names=(r"^(?!torso_link$).*",)),
      "operation": "scale",
      "ranges": (0.90, 1.10),
    },
  )
  # 3. Randomize static joint friction loss (0.0 to 0.30) to cover planetary gear / 4-bar linkage stiction (0.2 in scene_g1_23dof.xml).
  cfg.events["joint_friction"] = EventTermCfg(
    mode="startup",
    func=dr.joint_friction,
    params={
      "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      "operation": "abs",
      "ranges": (0.0, 0.30),
    },
  )
  # 4. Randomize viscous joint damping (0.0 to 0.10) to cover mechanical damping (0.05 in scene_g1_23dof.xml).
  cfg.events["joint_damping"] = EventTermCfg(
    mode="startup",
    func=dr.joint_damping,
    params={
      "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      "operation": "abs",
      "ranges": (0.0, 0.10),
    },
  )
  # 5. Randomize actuator PD gains (+-15%) and effort limits (85% to 105%) for battery voltage & thermal variation.
  cfg.events["motor_pd_gains"] = EventTermCfg(
    mode="startup",
    func=dr.pd_gains,
    params={
      "asset_cfg": SceneEntityCfg("robot"),
      "operation": "scale",
      "kp_range": (0.85, 1.15),
      "kd_range": (0.85, 1.15),
    },
  )
  cfg.events["motor_effort_limits"] = EventTermCfg(
    mode="startup",
    # Same as dr.effort_limits, but accepts the Phase 3 DelayedActuator wrappers.
    func=delayed_effort_limits,
    params={
      "asset_cfg": SceneEntityCfg("robot"),
      "operation": "scale",
      "effort_limit_range": (0.85, 1.05),
    },
  )

  # Phase 3 Sim-to-Real Domain Randomization:
  # 6. Action latency. One whole-body lag per episode, 0-G1_23DOF_MAX_ACTION_LAG physics
  #    steps (0-20 ms at dt=0.005). The Exp 5 sim2sim sweep showed standing oscillation
  #    from ~20-30 ms and falls at 40 ms for the Phase 2 policy trained without delay.
  cfg.events["action_delay"] = EventTermCfg(
    mode="reset",
    func=dr.sync_actuator_delays,
    params={
      "asset_cfg": SceneEntityCfg("robot"),
      "lag_range": (0, G1_23DOF_MAX_ACTION_LAG),
    },
  )

  # Rationale for std values:
  # - Knees/hip_pitch get the loosest std to allow natural leg bending during stride.
  # - Hip roll/yaw stay tighter to prevent excessive lateral sway and keep gait stable.
  # - Ankle roll is very tight for balance; ankle pitch looser for foot clearance.
  # - Waist roll/pitch stay tight to keep the torso upright and stable.
  # - Shoulders/elbows get moderate freedom for natural arm swing during walking.
  # - Wrists are loose (0.3) since they don't affect balance much.
  # Running values are ~1.5-2x walking values to accommodate larger motion range.
  cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
  cfg.rewards["pose"].params["std_walking"] = {
    # Lower body.
    r".*hip_pitch.*": 0.5,
    r".*hip_roll.*": 0.15,
    r".*hip_yaw.*": 0.15,
    r".*knee.*": 0.5,
    r".*ankle_pitch.*": 0.15,
    r".*ankle_roll.*": 0.1,
    # Waist.
    r".*waist_yaw.*": 0.15,
    # Arms.
    r".*shoulder_pitch.*": 0.15,
    r".*shoulder_roll.*": 0.1,
    r".*shoulder_yaw.*": 0.1,
    r".*elbow.*": 0.1,
    r".*wrist.*": 0.1,
  }
  cfg.rewards["pose"].params["std_running"] = {
    # Lower body.
    r".*hip_pitch.*": 0.5,
    r".*hip_roll.*": 0.25,
    r".*hip_yaw.*": 0.25,
    r".*knee.*": 0.5,
    r".*ankle_pitch.*": 0.25,
    r".*ankle_roll.*": 0.1,
    # Waist.
    r".*waist_yaw.*": 0.25,
    # Arms.
    r".*shoulder_pitch.*": 0.25,
    r".*shoulder_roll.*": 0.1,
    r".*shoulder_yaw.*": 0.1,
    r".*elbow.*": 0.1,
    r".*wrist.*": 0.1,
  }

  cfg.rewards["body_orientation_l2"].params["asset_cfg"].body_names = ("torso_link",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("torso_link",)
  cfg.rewards["foot_clearance"].params["asset_cfg"].site_names = site_names
  cfg.rewards["foot_slip"].params["asset_cfg"].site_names = site_names
  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-1.0,
    params={"sensor_name": self_collision_cfg.name, "force_threshold": 10.0},
  )

  # Apply play mode overrides.
  if play:
    # Effectively infinite episode length.
    cfg.episode_length_s = int(1e9)

    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum = {}
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=envs_mdp.randomize_terrain,
      mode="reset",
      params={},
    )

    if cfg.scene.terrain is not None:
      if cfg.scene.terrain.terrain_generator is not None:
        cfg.scene.terrain.terrain_generator.curriculum = False
        cfg.scene.terrain.terrain_generator.num_cols = 5
        cfg.scene.terrain.terrain_generator.num_rows = 5
        cfg.scene.terrain.terrain_generator.border_width = 10.0

  return cfg


def unitree_g1_23dof_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Unitree G1-23DOF flat terrain velocity configuration."""
  cfg = unitree_g1_23dof_rough_env_cfg(play=play)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  # Switch to flat terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # Remove raycast sensor and height scan (no terrain to scan).
  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"
  )
  del cfg.observations["actor"].terms["height_scan"]
  del cfg.observations["critic"].terms["height_scan"]

  # Disable terrain curriculum (not present in play mode since rough clears all).
  cfg.curriculum.pop("terrain_levels", None)

  if play:
    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    twist_cmd.ranges.lin_vel_x = (-0.5, 1.0)
    twist_cmd.ranges.lin_vel_y = (-0.5, 0.5)
    twist_cmd.ranges.ang_vel_z = (-0.5, 0.5)

  return cfg
