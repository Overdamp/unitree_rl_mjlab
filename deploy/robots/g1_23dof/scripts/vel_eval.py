#!/usr/bin/env python3
"""Headless sim2sim velocity-tracking check for G1-23DoF velocity policies.

Replicates the g1_ctrl Velocity state on the unitree_mujoco scene: obs/action from
deploy.yaml (including per-term history, filled with the first frame on reset and
concatenated oldest -> newest like the deploy ObservationManager), 50 Hz policy,
PD on 500 Hz physics, joints the policy does not drive held with FixStand gains.
No DDS latency is simulated.

Per case it also reports efficiency and hardware load over the measure window:
mechanical power and cost of transport (CoT = mean sum|tau * qdot| / (m g v)),
peak and RMS torque per joint against the effort limit, and the share of time a
motor sat at its torque limit. --push sweeps base velocity kicks (like the
training push_robot event) and reports the largest kick survived per direction.

  python deploy/robots/g1_23dof/scripts/vel_eval.py \
    --policy deploy/robots/g1_23dof/config/policy/velocity/v0/exported/policy_phase3b_16000.onnx \
    --deploy deploy/robots/g1_23dof/config/policy/velocity/v0/params/deploy.yaml
  python .../vel_eval.py --policy ... --deploy ... --cases "0.4,0,0;0,0,0" --push
"""

import argparse
import json
import re
from collections import deque
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
XML = REPO_ROOT / "src/assets/robots/unitree_g1/xmls/scene_g1_23dof.xml"
FSM_CFG = REPO_ROOT / "deploy/robots/g1_23dof/config/config.yaml"

# (vx, vy, wz) commands: forward speeds, then turning and sideways in place.
DEFAULT_CASES = [
  (0.4, 0.0, 0.0),
  (0.8, 0.0, 0.0),
  (1.0, 0.0, 0.0),
  (0.0, 0.0, 0.5),
  (0.0, 0.0, -0.5),
  (0.0, 0.0, 1.0),
  (0.4, 0.0, 0.5),
  (0.0, 0.3, 0.0),
]
FALL_TILT = 0.85  # rad
HANG_HEIGHT = 1.3  # m, feet clear of the ground
G = 9.81
# Push sweep: base velocity kicks (m/s) applied once, 1 s into the measure window.
PUSH_DIRS = {"front": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0)}
PUSH_LEVELS = (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0)
PUSH_AT = 1.0  # s after the measure window starts
# Motor velocity limits in rad/s (src/assets/robots/unitree_g1/g1_23dof_constants.py).
VEL_LIMITS = [
  (r".*_(hip_roll|knee)_joint", 20.0),
  (r".*_(hip_pitch|hip_yaw)_joint|waist_yaw_joint", 32.0),
  (r".*", 37.0),
]


def quat_rot(q):
  R = np.zeros(9)
  mujoco.mju_quat2Mat(R, q)
  return R.reshape(3, 3)


class Sim:
  def __init__(self, deploy_yaml: Path):
    cfg = yaml.safe_load(open(deploy_yaml))
    fsm = yaml.safe_load(open(FSM_CFG))["FSM"]
    self.ids = np.array(cfg["joint_ids_map"])
    self.default = np.array(cfg["default_joint_pos"])
    act = cfg["actions"]["JointPositionAction"]
    self.scale = np.array(act["scale"])
    self.offset = np.array(act["offset"])
    self.clip = np.array(act["clip"]) if act.get("clip") else None
    obs_cfg = cfg["observations"]
    self.terms = list(obs_cfg)
    self.history = {k: obs_cfg[k].get("history_length", 1) or 1 for k in self.terms}
    self.obs_scale = {k: np.array(obs_cfg[k]["scale"]) if obs_cfg[k].get("scale") else None for k in self.terms}
    self.period = obs_cfg["gait_phase"]["params"]["period"]
    self.dt = cfg["step_dt"]

    self.m = mujoco.MjModel.from_xml_path(str(XML))
    self.d = mujoco.MjData(self.m)
    m = self.m
    self.dec = int(round(self.dt / m.opt.timestep))
    self.qadr = np.array([m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(m.nu)])
    self.vadr = np.array([m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in range(m.nu)])
    self.lo, self.hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    self.gyro_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")]
    jids = [m.actuator_trnid[i, 0] for i in self.ids]
    self.names = [m.joint(j).name for j in jids]
    self.jrange = np.array([m.jnt_range[j] for j in jids])
    self.vlim = np.array([next(v for p, v in VEL_LIMITS if re.fullmatch(p, n)) for n in self.names])
    self.mass = float(np.sum(m.body_mass))

    # Full 29-motor targets: FixStand pose/gains, policy joints overwritten.
    self.kp = np.array(fsm["FixStand"]["kp"], float)
    self.kd = np.array(fsm["FixStand"]["kd"], float)
    self.q_stand = np.array(fsm["FixStand"]["qs"][1], float)
    self.kp[self.ids] = cfg["stiffness"]
    self.kd[self.ids] = cfg["damping"]

  def frame(self, cmd, phase, last):
    d = self.d
    R = quat_rot(d.qpos[3:7])
    if np.linalg.norm(cmd) >= 0.1:
      ph = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]
    else:
      ph = [0.0, 0.0]
    raw = {
      "base_ang_vel": d.sensordata[self.gyro_adr : self.gyro_adr + 3].copy(),
      "projected_gravity": R.T @ np.array([0, 0, -1.0]),
      "velocity_commands": cmd.copy(),
      "gait_phase": np.array(ph),
      "joint_pos_rel": d.qpos[self.qadr[self.ids]] - self.default,
      "joint_vel_rel": d.qvel[self.vadr[self.ids]].copy(),
      "last_action": last.copy(),
    }
    return {k: raw[k] * self.obs_scale[k] if self.obs_scale[k] is not None else raw[k] for k in self.terms}

  def run(self, sess, cmd_target, settle=2.0, walk=12.0, measure=8.0, smooth=True, hang=False, push=None):
    """push: (dvx, dvy) in m/s, world frame, added to the base velocity once."""
    m, d = self.m, self.d
    iname = sess.get_inputs()[0].name
    mujoco.mj_resetData(m, d)
    d.qpos[2] = HANG_HEIGHT if hang else 0.793
    d.qpos[3:7] = [1, 0, 0, 0]
    d.qpos[self.qadr] = self.q_stand
    d.qpos[self.qadr[self.ids]] = self.default
    mujoco.mj_forward(m, d)

    last = np.zeros(len(self.ids))
    cmd = np.zeros(3)
    phase = 0.0
    q_target = self.q_stand.copy()
    q_target[self.ids] = self.default
    first = self.frame(cmd, phase, last)
    buf = {k: deque([first[k]] * self.history[k], maxlen=self.history[k]) for k in self.terms}

    n_settle = int(settle / self.dt)
    n_total = n_settle + int(walk / self.dt)
    n_meas0 = n_total - int(measure / self.dt)
    vx, vy, wz, jv, tilt_max, fell = [], [], [], [], 0.0, False
    n = len(self.ids)
    lo, hi = self.jrange[:, 0], self.jrange[:, 1]
    safety = dict(
      entry_jump=0.0, target_out=0, target_over=np.zeros(n), q_margin=np.full(n, np.inf),
      speed_ratio=np.zeros(n), tau_ratio=np.zeros(n), steps=0,
    )
    # Measure-window load, all 29 motors for power, policy joints for torque.
    energy, meas_t = 0.0, 0.0
    tau_peak, tau_sq, tau_sat, n_sub = np.zeros(n), np.zeros(n), np.zeros(n), 0
    k_push = n_meas0 + int(PUSH_AT / self.dt) if push is not None else -1
    for k in range(n_total):
      target = np.array(cmd_target) if k >= n_settle else np.zeros(3)
      # g1_ctrl smooths keyboard commands; the real gamepad stick goes straight in.
      cmd = cmd + (target - cmd) * 0.15 if smooth else target.astype(float)
      phase = (phase + self.dt / self.period) % 1.0
      f = self.frame(cmd, phase, last)
      for t in self.terms:
        buf[t].append(f[t])
      obs = np.concatenate([np.concatenate(buf[t]) for t in self.terms]).astype(np.float32)
      act = sess.run(None, {iname: obs[None]})[0][0]
      last = act.astype(np.float64)
      tgt = act * self.scale + self.offset
      if self.clip is not None:
        tgt = np.clip(tgt, self.clip[:, 0], self.clip[:, 1])
      q_target[self.ids] = tgt
      if k == 0:
        safety["entry_jump"] = float(np.max(np.abs(tgt - self.default)))
      over = np.maximum(tgt - hi, lo - tgt)
      safety["target_out"] += int(np.sum(over > 0))
      safety["target_over"] = np.maximum(safety["target_over"], over)
      safety["steps"] += 1
      if k == k_push:
        d.qvel[0] += push[0]
        d.qvel[1] += push[1]
      for _ in range(self.dec):
        if hang:  # gantry: base held in the air, feet off the ground
          d.qpos[0:3] = (0.0, 0.0, HANG_HEIGHT)
          d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
          d.qvel[0:6] = 0.0
        tau = self.kp * (q_target - d.qpos[self.qadr]) - self.kd * d.qvel[self.vadr]
        d.ctrl[:] = np.clip(tau, self.lo, self.hi)
        mujoco.mj_step(m, d)
        qj = d.qpos[self.qadr[self.ids]]
        safety["q_margin"] = np.minimum(safety["q_margin"], np.minimum(qj - lo, hi - qj))
        safety["speed_ratio"] = np.maximum(safety["speed_ratio"], np.abs(d.qvel[self.vadr[self.ids]]) / self.vlim)
        safety["tau_ratio"] = np.maximum(safety["tau_ratio"], np.abs(d.ctrl[self.ids]) / self.hi[self.ids])
        if k >= n_meas0:
          energy += float(np.sum(np.abs(d.ctrl * d.qvel[self.vadr]))) * m.opt.timestep
          meas_t += m.opt.timestep
          tj = np.abs(d.ctrl[self.ids])
          tau_peak = np.maximum(tau_peak, tj)
          tau_sq += tj * tj
          tau_sat += tj >= 0.99 * self.hi[self.ids]
          n_sub += 1
      R = quat_rot(d.qpos[3:7])
      tilt = np.arccos(np.clip(-(R.T @ np.array([0, 0, -1.0]))[2], -1, 1))
      tilt_max = max(tilt_max, tilt)
      if tilt > FALL_TILT:
        fell = True
        break
      if k >= n_meas0:
        yaw = np.arctan2(R[1, 0], R[0, 0])
        v = d.qvel[0:3]
        vx.append(np.cos(yaw) * v[0] + np.sin(yaw) * v[1])
        vy.append(-np.sin(yaw) * v[0] + np.cos(yaw) * v[1])
        wz.append(d.qvel[5])
        jv.append(d.qvel[self.vadr[self.ids]].copy())

    def stat(x):
      return round(float(np.mean(x)), 3) if x else None

    def worst(a, fn=np.argmax):
      i = int(fn(a))
      return [self.names[i].removesuffix("_joint"), round(float(a[i]), 3)]

    power = energy / meas_t if meas_t > 0 else None
    speed = float(np.hypot(np.mean(vx), np.mean(vy))) if vx else 0.0
    if n_sub:
      lim = self.hi[self.ids]
      tau_rms = np.sqrt(tau_sq / n_sub)
      load = dict(
        # Mechanical power, sum |tau * qdot| over all motors (W). Upper bound of
        # battery draw minus electrical losses, which this does not model.
        power_w=round(power, 1),
        # Dimensionless; only meaningful when the robot actually travels.
        cot=round(power / (self.mass * G * speed), 3) if speed >= 0.1 else None,
        peak_torque_nm=worst(tau_peak),
        peak_torque_ratio=worst(tau_peak / lim),
        rms_torque_ratio=worst(tau_rms / lim),
        # Share of physics steps a joint spent at >= 99% of its torque limit.
        torque_sat_pct=worst(100.0 * tau_sat / n_sub) if tau_sat.any() else 0.0,
      )
    else:
      load = None

    return dict(
      cmd=list(cmd_target),
      fell=fell,
      tilt_max_deg=round(float(np.degrees(tilt_max)), 1),
      vx=stat(vx),
      vx_std=round(float(np.std(vx)), 3) if vx else None,
      vy=stat(vy),
      wz=stat(wz),
      # Sway over the measure window; at cmd = 0 a still stance gives ~0 for both.
      joint_vel_std=round(float(np.mean(np.std(jv, axis=0))), 3) if jv else None,
      yaw_rate_std=round(float(np.std(wz)), 3) if wz else None,
      load=load,
      safety=dict(
        # Largest |target - default pose| on the first policy step after entering Velocity.
        entry_jump_rad=round(safety["entry_jump"], 3),
        # Share of joint targets beyond the MJCF joint range, and the worst overshoot.
        target_out_pct=round(100.0 * safety["target_out"] / (safety["steps"] * n), 2),
        target_over_rad=worst(safety["target_over"]),
        min_q_margin_rad=worst(safety["q_margin"], np.argmin),
        max_speed_ratio=worst(safety["speed_ratio"]),
        max_torque_ratio=worst(safety["tau_ratio"]),
      ),
    )


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--policy", type=Path, required=True, help="policy.onnx")
  parser.add_argument("--deploy", type=Path, required=True, help="deploy.yaml matching the policy")
  parser.add_argument("--cases", type=str, default=None, help="'vx,vy,wz;vx,vy,wz' (default: built-in set)")
  parser.add_argument("--no-smooth", action="store_true", help="step commands like the real gamepad (no 0.15 smoothing)")
  parser.add_argument("--hang", action="store_true", help="hold the base in the air like the gantry (use --cases 0,0,0)")
  parser.add_argument("--push", action="store_true", help="per case, sweep base kicks front/back/left/right until it falls")
  args = parser.parse_args()

  sim = Sim(args.deploy)
  sess = ort.InferenceSession(str(args.policy), providers=["CPUExecutionProvider"])
  n_in = sess.get_inputs()[0].shape[-1]
  n_obs = sum(len(sim.frame(np.zeros(3), 0.0, np.zeros(len(sim.ids)))[t]) * sim.history[t] for t in sim.terms)
  assert n_in == n_obs, f"policy input {n_in} != deploy.yaml obs {n_obs}"

  cases = DEFAULT_CASES
  if args.cases:
    cases = [tuple(float(x) for x in c.split(",")) for c in args.cases.split(";")]
  for c in cases:
    if args.push:
      # Largest kick survived per direction; levels stop at the first fall.
      res = {}
      for name, (ux, uy) in PUSH_DIRS.items():
        ok = 0.0
        for lv in PUSH_LEVELS:
          if sim.run(sess, c, smooth=not args.no_smooth, push=(ux * lv, uy * lv))["fell"]:
            break
          ok = lv
        res[name] = ok
      r = dict(cmd=list(c), push_max_ok_mps=res, push_levels=list(PUSH_LEVELS))
    else:
      r = sim.run(sess, c, smooth=not args.no_smooth, hang=args.hang)
    r["policy"] = args.policy.name
    print(json.dumps(r), flush=True)


if __name__ == "__main__":
  main()
