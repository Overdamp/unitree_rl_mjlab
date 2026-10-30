#!/usr/bin/env python3
"""Scripted joint motions on the gantry for actuator identification (G1 23-DoF).

Replaces g1_ctrl (never run both): publishes rt/lowcmd itself, moves to the
stand pose, then moves one joint at a time with
  step   +A, 0, -A, 0 (1.5 s each, 100 ms smooth edges)
  chirp  A * sin, 0.3 -> 2.5 Hz over 12 s (1 s fade in/out)
around that pose, with the training gains (deploy.yaml, default) or Unitree's
FixStand gains. Because the motion comes from the commands, pd_check.py
--replay can compare the real response with MuJoCo (armature, friction,
delay), which pushing by hand cannot.

Record with lowstate_monitor.py / lowcmd_monitor.py --log as usual; --segments
writes when each test ran (time.monotonic, same clock as the monitor logs).

Safety (robot hung on the gantry only):
  * the whole plan is checked against the MJCF limits (0.1 rad margin) and a
    target speed limit before anything is sent; --dry-run prints it only
  * damping (kp 0, Passive kd) on: remote L2 + B, Ctrl-C, LowState older than
    50 ms, a joint more than 0.5 rad from its target, motor > 70 C, a motor
    offline (vol 0 or motorstate != 0, real robot only); also at the end
  * refuses to start while rt/lowcmd is published (g1_ctrl) or, on the real
    robot, while the built-in controller runs (motion_mode.py check / release)

  python deploy/robots/g1_23dof/scripts/sysid_motion.py --dry-run
  python deploy/robots/g1_23dof/scripts/sysid_motion.py --iface lo --domain 1 --yes   # unitree_mujoco
  python deploy/robots/g1_23dof/scripts/sysid_motion.py --groups armL,armR,waist --segments ~/sysid_seg.csv
"""

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lowcmd_monitor import CONFIG_YAML, DEFAULT_DEPLOY  # noqa: E402
from lowstate_monitor import DEFAULT_MJCF, G1_23DOF_JOINTS, NUM_SLOTS, load_joint_limits  # noqa: E402

CONTROL_DT = 0.002
RAMP_S = 3.0
HOLD_S = 2.0
REST_S = 1.0
STEP_HOLD_S = 1.5
STEP_EDGE_S = 0.1
CHIRP_S = 12.0
CHIRP_F0, CHIRP_F1 = 0.3, 2.5
FADE_S = 1.0
END_HOLD_S = 1.0
DAMP_EXIT_S = 1.0

LIMIT_MARGIN = 0.1  # rad from the MJCF range
MAX_TARGET_SPEED = 6.0  # rad/s, planned q_des speed
STALE_S = 0.05
MAX_TRACK_ERR = 0.5  # rad, after the ramp
MAX_TEMP_C = 70
LOWCMD_LISTEN_S = 0.5
STATUS_S = 0.5

# Remote buttons in LowState.wireless_remote (unitree_joystick.hpp BtnUnion).
L2_BYTE, L2_MASK = 2, 0x20
B_BYTE, B_MASK = 3, 0x02

# Joint name suffix -> (step amplitude, chirp amplitude), rad.
AMPLITUDE = {
  "hip_pitch": (0.15, 0.10),
  "hip_roll": (0.10, 0.08),
  "hip_yaw": (0.10, 0.08),
  "knee": (0.15, 0.10),
  "ankle_pitch": (0.10, 0.08),
  "ankle_roll": (0.08, 0.05),
  "waist_yaw": (0.15, 0.10),
  "shoulder_pitch": (0.20, 0.15),
  "shoulder_roll": (0.12, 0.08),
  "shoulder_yaw": (0.20, 0.15),
  "elbow": (0.20, 0.15),
  "wrist_roll": (0.30, 0.20),
}
GROUPS = {
  "armL": [15, 16, 17, 18, 19],
  "armR": [22, 23, 24, 25, 26],
  "waist": [12],
  "legL": [0, 1, 2, 3, 4, 5],
  "legR": [6, 7, 8, 9, 10, 11],
}


@dataclass
class Segment:
  name: str
  slot: int  # -1 = all joints (ramp, hold)
  kind: str
  t0: float
  t1: float
  amp: float = 0.0


def amplitudes(slot: int) -> tuple[float, float]:
  name = G1_23DOF_JOINTS[slot].removesuffix("_joint")
  for key, amp in AMPLITUDE.items():
    if name.endswith(key):
      return amp
  raise KeyError(name)


def smooth(x: np.ndarray | float) -> np.ndarray | float:
  """0 -> 1 raised cosine on x in [0, 1]."""
  x = np.clip(x, 0.0, 1.0)
  return 0.5 - 0.5 * np.cos(np.pi * x)


def step_offset(t: float, amp: float) -> float:
  levels = [amp, 0.0, -amp, 0.0]
  k = min(int(t // STEP_HOLD_S), 3)
  prev = 0.0 if k == 0 else levels[k - 1]
  return prev + (levels[k] - prev) * smooth((t - k * STEP_HOLD_S) / STEP_EDGE_S)


def chirp_offset(t: float, amp: float) -> float:
  phase = 2 * np.pi * (CHIRP_F0 * t + (CHIRP_F1 - CHIRP_F0) * t * t / (2 * CHIRP_S))
  fade = smooth(t / FADE_S) * smooth((CHIRP_S - t) / FADE_S)
  return amp * fade * np.sin(phase)


def build_plan(slots: list[int], kinds: list[str]) -> list[Segment]:
  plan = [Segment("ramp to stand pose", -1, "ramp", 0.0, RAMP_S), Segment("hold", -1, "hold", RAMP_S, RAMP_S + HOLD_S)]
  t = RAMP_S + HOLD_S
  for slot in slots:
    a_step, a_chirp = amplitudes(slot)
    for kind in kinds:
      dur = 4 * STEP_HOLD_S if kind == "step" else CHIRP_S
      amp = a_step if kind == "step" else a_chirp
      plan.append(Segment(G1_23DOF_JOINTS[slot].removesuffix("_joint"), slot, kind, t, t + dur, amp))
      t += dur + REST_S
  plan.append(Segment("hold", -1, "hold", t, t + END_HOLD_S))
  return plan


def offsets(plan: list[Segment], t: float) -> np.ndarray:
  off = np.zeros(NUM_SLOTS)
  for seg in plan:
    if seg.slot >= 0 and seg.t0 <= t < seg.t1:
      tau = t - seg.t0
      off[seg.slot] = step_offset(tau, seg.amp) if seg.kind == "step" else chirp_offset(tau, seg.amp)
  return off


def check_plan(plan: list[Segment], base: np.ndarray, limits: dict) -> list[str]:
  problems = []
  ts = np.arange(plan[1].t1, plan[-1].t1, CONTROL_DT)
  q = np.array([base + offsets(plan, t) for t in ts])
  speed = np.abs(np.diff(q, axis=0)) / CONTROL_DT
  for slot, name in G1_23DOF_JOINTS.items():
    lo, hi = limits[name]
    if q[:, slot].min() < lo + LIMIT_MARGIN or q[:, slot].max() > hi - LIMIT_MARGIN:
      problems.append(f"{name}: target {q[:, slot].min():+.3f}..{q[:, slot].max():+.3f} outside [{lo:+.3f}, {hi:+.3f}] - margin")
    if speed[:, slot].max() > MAX_TARGET_SPEED:
      problems.append(f"{name}: target speed {speed[:, slot].max():.1f} rad/s > {MAX_TARGET_SPEED}")
  return problems


def load_gains(which: str, deploy: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
  """29-slot kp, kd, stand pose and Passive kd (used slots only, others 0)."""
  dep = yaml.safe_load(open(deploy))
  fsm = yaml.safe_load(open(CONFIG_YAML))["FSM"]
  ids = dep["joint_ids_map"]
  kp, kd, base = np.zeros(NUM_SLOTS), np.zeros(NUM_SLOTS), np.zeros(NUM_SLOTS)
  base[ids] = dep["default_joint_pos"]
  if which == "train":
    kp[ids], kd[ids] = dep["stiffness"], dep["damping"]
  else:
    kp[ids] = np.array(fsm["FixStand"]["kp"], float)[ids]
    kd[ids] = np.array(fsm["FixStand"]["kd"], float)[ids]
  passive_kd = np.zeros(NUM_SLOTS)
  passive_kd[ids] = np.array(fsm["Passive"]["kd"], float)[ids]
  return kp, kd, base, passive_kd


def print_plan(plan: list[Segment], kp: np.ndarray, kd: np.ndarray, base: np.ndarray) -> None:
  print(f"{'t start':>8} {'dur':>5}  {'test':<20} {'kind':<6} {'amp':>5} {'base':>6} {'kp':>6} {'kd':>4}")
  for seg in plan:
    if seg.slot < 0:
      print(f"{seg.t0:8.1f} {seg.t1 - seg.t0:5.1f}  {seg.name:<20} {seg.kind:<6}")
    else:
      s = seg.slot
      print(f"{seg.t0:8.1f} {seg.t1 - seg.t0:5.1f}  {seg.name:<20} {seg.kind:<6} {seg.amp:5.2f} {base[s]:+6.2f} {kp[s]:6.1f} {kd[s]:4.1f}")
  print(f"total {plan[-1].t1:.0f} s ({plan[-1].t1 / 60:.1f} min)")


class Robot:
  """Latest LowState plus the rt/lowcmd publisher."""

  def __init__(self) -> None:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    self.state = None
    self.t_state = 0.0
    self.sub = ChannelSubscriber("rt/lowstate", LowState_)
    self.sub.Init(self._on_state, 10)
    self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    self.pub.Init()
    self.cmd = unitree_hg_msg_dds__LowCmd_()
    self.crc = CRC()

  def _on_state(self, msg) -> None:
    self.state = msg
    self.t_state = time.monotonic()

  def q(self) -> np.ndarray:
    return np.array([self.state.motor_state[i].q for i in range(NUM_SLOTS)])

  def send(self, q: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> None:
    self.cmd.mode_pr = 0
    self.cmd.mode_machine = self.state.mode_machine
    for i in range(NUM_SLOTS):
      m = self.cmd.motor_cmd[i]
      m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 1, float(q[i]), 0.0, 0.0, float(kp[i]), float(kd[i])
    self.cmd.crc = self.crc.Crc(self.cmd)
    self.pub.Write(self.cmd)


def lowcmd_traffic(listen_s: float) -> int:
  from unitree_sdk2py.core.channel import ChannelSubscriber
  from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

  count = [0]
  sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
  sub.Init(lambda _: count.__setitem__(0, count[0] + 1), 10)
  time.sleep(listen_s)
  sub.Close()
  return count[0]


def fault(robot: Robot, q_des: np.ndarray, check_err: bool, real: bool) -> str | None:
  """Reason to switch to damping, or None."""
  if time.monotonic() - robot.t_state > STALE_S:
    return f"LowState older than {STALE_S * 1e3:.0f} ms"
  st = robot.state
  rc = st.wireless_remote
  if rc[L2_BYTE] & L2_MASK and rc[B_BYTE] & B_MASK:
    return "remote L2 + B"
  for slot, name in G1_23DOF_JOINTS.items():
    ms = st.motor_state[slot]
    if check_err and abs(ms.q - q_des[slot]) > MAX_TRACK_ERR:
      return f"{name} {abs(ms.q - q_des[slot]):.2f} rad from target"
    if ms.temperature[0] > MAX_TEMP_C:
      return f"{name} {ms.temperature[0]} C"
    if real and (ms.vol == 0 or ms.motorstate != 0):
      return f"{name} offline (vol {ms.vol}, motorstate {ms.motorstate:#x})"
  return None


def write_segments(path: Path, plan: list[Segment], t_start: float, t_stop: float) -> None:
  with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["test", "slot", "kind", "amp", "t0", "t1", "completed"])
    for seg in plan:
      if seg.t0 + t_start < t_stop:
        w.writerow([seg.name, seg.slot, seg.kind, seg.amp, f"{t_start + seg.t0:.4f}",
                    f"{t_start + min(seg.t1, t_stop - t_start):.4f}", int(seg.t1 + t_start <= t_stop)])
  print(f"segments written to {path}")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--iface", default="enp43s0", help="enp43s0 for the robot, lo for unitree_mujoco")
  parser.add_argument("--domain", type=int, default=0)
  parser.add_argument("--gains", choices=("train", "unitree"), default="train")
  parser.add_argument("--deploy", type=Path, default=DEFAULT_DEPLOY, help="deploy.yaml for train gains and the pose")
  parser.add_argument("--groups", default="armL,armR,waist,legL,legR", help=f"comma list of {','.join(GROUPS)}")
  parser.add_argument("--joints", default="", help="only joints whose name contains one of these (comma list)")
  parser.add_argument("--kinds", default="step,chirp")
  parser.add_argument("--segments", type=Path, default=None, help="CSV with the start/end time of each test")
  parser.add_argument("--dry-run", action="store_true", help="print and check the plan, send nothing")
  parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
  args = parser.parse_args()

  slots = [s for g in args.groups.split(",") for s in GROUPS[g]]
  if args.joints:
    keys = args.joints.split(",")
    slots = [s for s in slots if any(k in G1_23DOF_JOINTS[s] for k in keys)]
  kinds = args.kinds.split(",")
  if not slots or any(k not in ("step", "chirp") for k in kinds):
    parser.error("no joints selected or unknown kind")

  kp, kd, base, passive_kd = load_gains(args.gains, args.deploy)
  plan = build_plan(slots, kinds)
  print(f"gains: {args.gains} ({args.deploy.parent.parent.name if args.gains == 'train' else 'config.yaml FixStand'})")
  print_plan(plan, kp, kd, base)
  problems = check_plan(plan, base, load_joint_limits(DEFAULT_MJCF))
  for p in problems:
    print(f"PLAN ERROR: {p}")
  if problems:
    return 1
  print("plan OK: inside MJCF limits with margin, target speed limited")
  if args.dry_run:
    return 0

  from unitree_sdk2py.core.channel import ChannelFactoryInitialize

  real = args.iface != "lo"
  ChannelFactoryInitialize(args.domain, args.iface)
  n_cmd = lowcmd_traffic(LOWCMD_LISTEN_S)
  if n_cmd:
    print(f"refusing: {n_cmd} rt/lowcmd messages in {LOWCMD_LISTEN_S} s (stop g1_ctrl first)")
    return 1
  if real:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    code, data = msc.CheckMode()
    if code != 0 or (data or {}).get("name"):
      print(f"refusing: built-in controller not released (code {code}, mode {data}); run motion_mode.py release")
      return 1

  robot = Robot()
  t_wait = time.monotonic()
  while robot.state is None and time.monotonic() - t_wait < 2.0:
    time.sleep(0.01)
  if robot.state is None:
    print("no rt/lowstate: wrong interface/domain, robot off, or lo multicast off")
    return 1
  if not args.yes:
    print("The robot will move to the stand pose and then move each joint. Is it hung on the gantry, area clear?")
    if input("Type 'yes' to start: ").strip().lower() != "yes":
      return 1

  q_start = robot.q()
  t_start = time.monotonic()
  reason, next_status, next_tick = None, 0.0, time.perf_counter()
  seg_i = 0
  try:
    while True:
      t = time.monotonic() - t_start
      if t >= plan[-1].t1:
        reason = "plan finished"
        break
      a = smooth(t / RAMP_S)
      q_des = q_start + (base - q_start) * a + offsets(plan, t)
      q_des[[s for s in range(NUM_SLOTS) if s not in G1_23DOF_JOINTS]] = 0.0
      reason = fault(robot, q_des, t > RAMP_S, real)
      if reason:
        break
      robot.send(q_des, kp, kd)
      while seg_i + 1 < len(plan) and t >= plan[seg_i + 1].t0:
        seg_i += 1
      if t >= next_status:
        seg = plan[seg_i]
        print(f"\r{t:6.1f}/{plan[-1].t1:.0f} s  {seg.name:<20} {seg.kind:<6}", end="", flush=True)
        next_status = t + STATUS_S
      next_tick += CONTROL_DT
      time.sleep(max(0.0, next_tick - time.perf_counter()))
  except KeyboardInterrupt:
    reason = "Ctrl-C"
  t_stop = time.monotonic()
  print(f"\n-> damping ({reason})")

  # Damping until the end: the last command the motors keep is kp 0.
  zeros = np.zeros(NUM_SLOTS)
  t_damp = time.monotonic()
  try:
    while time.monotonic() - t_damp < DAMP_EXIT_S:
      if robot.state is not None:
        robot.send(robot.q(), zeros, passive_kd)
      time.sleep(CONTROL_DT)
  except KeyboardInterrupt:
    pass
  if args.segments:
    write_segments(args.segments, plan, t_start, t_stop)
  return 0 if reason == "plan finished" else 2


if __name__ == "__main__":
  sys.exit(main())
