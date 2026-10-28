#!/usr/bin/env python3
"""Read-only LowState monitor for the G1 23-DoF.

Subscribes to ``rt/lowstate`` only and never publishes ``rt/lowcmd``, so the
robot is not commanded. Use it to validate the connection and the joint map
before running ``g1_ctrl`` (see doc/g1_23dof_real_robot_setup_checklist.md).

It shows the message rate and gaps, mode_machine/mode_pr, IMU, and per joint
q / dq / temperature, the min/max seen so far, and the MJCF limits. On exit
(Ctrl-C or --duration) it prints a summary and can save a CSV log.

Run with the conda env that has unitree_sdk2py (``g1_sdk``):

  python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --iface enp43s0
  python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --iface lo      # unitree_mujoco
  python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --iface enp43s0 --log /tmp/lowstate.csv
"""

import argparse
import csv
import math
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

# SDK motor index -> joint name. Same order as joint_ids_map in deploy.yaml.
G1_23DOF_JOINTS = {
  0: "left_hip_pitch_joint",
  1: "left_hip_roll_joint",
  2: "left_hip_yaw_joint",
  3: "left_knee_joint",
  4: "left_ankle_pitch_joint",
  5: "left_ankle_roll_joint",
  6: "right_hip_pitch_joint",
  7: "right_hip_roll_joint",
  8: "right_hip_yaw_joint",
  9: "right_knee_joint",
  10: "right_ankle_pitch_joint",
  11: "right_ankle_roll_joint",
  12: "waist_yaw_joint",
  15: "left_shoulder_pitch_joint",
  16: "left_shoulder_roll_joint",
  17: "left_shoulder_yaw_joint",
  18: "left_elbow_joint",
  19: "left_wrist_roll_joint",
  22: "right_shoulder_pitch_joint",
  23: "right_shoulder_roll_joint",
  24: "right_shoulder_yaw_joint",
  25: "right_elbow_joint",
  26: "right_wrist_roll_joint",
}
# Slots that exist on the 29-DoF but are locked or absent on the 23-DoF.
UNUSED_SLOTS = (13, 14, 20, 21, 27, 28)
NUM_SLOTS = 29

EXPECTED_MODE_MACHINE = 4  # Checked by main.cpp for the 23-DoF.
GAP_WARN_S = 0.1  # Matches the LowState timeout in main.cpp.
MOVING_DQ = 0.2  # rad/s, highlight joints being moved by hand.
LIMIT_MARGIN = 0.05  # rad, warn when this close to an MJCF limit.

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_MJCF = REPO_ROOT / "src/assets/robots/unitree_g1/xmls/g1_23dof.xml"


def load_joint_limits(mjcf_path: Path) -> dict[str, tuple[float, float]]:
  limits = {}
  for joint in ET.parse(mjcf_path).getroot().iter("joint"):
    name, rng = joint.get("name"), joint.get("range")
    if name and rng:
      lo, hi = (float(v) for v in rng.split())
      limits[name] = (lo, hi)
  return limits


def quat_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
  roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
  pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
  yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  return roll, pitch, yaw


def projected_gravity(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
  # World gravity (0, 0, -1) rotated into the body frame, same as the deploy obs.
  return (
    2 * (w * y - x * z),
    -2 * (w * x + y * z),
    -(w * w - x * x - y * y + z * z),
  )


class LowStateMonitor:
  def __init__(self, log_path: Path | None):
    self._lock = threading.Lock()
    self.msg = None
    self.count = 0
    self.t_first = None
    self.t_last = None
    self.max_gap = 0.0
    self.gaps_over_warn = 0
    self.window = []  # receive times in the last second
    self.q_min = [math.inf] * NUM_SLOTS
    self.q_max = [-math.inf] * NUM_SLOTS
    self.mode_machine_seen = set()
    self.motor_error_seen = {}
    self._log_file = None
    self._log = None
    if log_path is not None:
      self._log_file = open(log_path, "w", newline="")
      self._log = csv.writer(self._log_file)
      header = ["t", "tick", "mode_machine", "mode_pr"]
      header += ["quat_w", "quat_x", "quat_y", "quat_z", "gyro_x", "gyro_y", "gyro_z"]
      header += [f"q{i}" for i in range(NUM_SLOTS)] + [f"dq{i}" for i in range(NUM_SLOTS)]
      header += [f"temp{i}" for i in range(NUM_SLOTS)]
      self._log.writerow(header)

  def handler(self, msg: LowState_):
    now = time.monotonic()
    with self._lock:
      if self.t_last is not None:
        gap = now - self.t_last
        self.max_gap = max(self.max_gap, gap)
        if gap > GAP_WARN_S:
          self.gaps_over_warn += 1
      else:
        self.t_first = now
      self.t_last = now
      self.count += 1
      self.window.append(now)
      while self.window and now - self.window[0] > 1.0:
        self.window.pop(0)
      self.msg = msg
      self.mode_machine_seen.add(msg.mode_machine)
      for i in range(NUM_SLOTS):
        m = msg.motor_state[i]
        self.q_min[i] = min(self.q_min[i], m.q)
        self.q_max[i] = max(self.q_max[i], m.q)
        if m.motorstate:
          self.motor_error_seen[i] = m.motorstate
      if self._log is not None:
        imu = msg.imu_state
        row = [f"{now:.4f}", msg.tick, msg.mode_machine, msg.mode_pr]
        row += list(imu.quaternion) + list(imu.gyroscope)
        row += [msg.motor_state[i].q for i in range(NUM_SLOTS)]
        row += [msg.motor_state[i].dq for i in range(NUM_SLOTS)]
        row += [max(msg.motor_state[i].temperature) for i in range(NUM_SLOTS)]
        self._log.writerow(row)

  def snapshot(self):
    with self._lock:
      return {
        "msg": self.msg,
        "count": self.count,
        "rate": len(self.window),
        "since_last": None if self.t_last is None else time.monotonic() - self.t_last,
        "max_gap": self.max_gap,
        "gaps_over_warn": self.gaps_over_warn,
        "q_min": list(self.q_min),
        "q_max": list(self.q_max),
        "mode_machine_seen": set(self.mode_machine_seen),
        "motor_error_seen": dict(self.motor_error_seen),
        "elapsed": 0.0 if self.t_first is None else self.t_last - self.t_first,
      }

  def close(self):
    if self._log_file is not None:
      self._log_file.close()


def limit_flag(q: float, lim: tuple[float, float] | None) -> str:
  if lim is None:
    return ""
  lo, hi = lim
  if q < lo or q > hi:
    return "OUT"
  if q < lo + LIMIT_MARGIN or q > hi - LIMIT_MARGIN:
    return "near"
  return ""


def render(s: dict, limits: dict) -> str:
  msg = s["msg"]
  lines = []
  if msg is None:
    return "Waiting for rt/lowstate ... (check interface, IP, multicast, debug mode)"

  since = s["since_last"]
  stale = " STALE" if since is not None and since > GAP_WARN_S else ""
  lines.append(
    f"rt/lowstate {s['rate']:4d} Hz | msgs {s['count']} | max gap {s['max_gap'] * 1e3:6.1f} ms"
    f" | gaps>{GAP_WARN_S * 1e3:.0f}ms {s['gaps_over_warn']}{stale}"
  )
  mm_ok = "OK" if msg.mode_machine == EXPECTED_MODE_MACHINE else f"!! expected {EXPECTED_MODE_MACHINE}"
  lines.append(f"mode_machine {msg.mode_machine} ({mm_ok}) | mode_pr {msg.mode_pr} | tick {msg.tick}")

  imu = msg.imu_state
  w, x, y, z = imu.quaternion
  roll, pitch, yaw = quat_to_rpy(w, x, y, z)
  g = projected_gravity(w, x, y, z)
  gx, gy, gz = imu.gyroscope
  lines.append(
    f"IMU rpy(deg) {math.degrees(roll):7.2f} {math.degrees(pitch):7.2f} {math.degrees(yaw):7.2f}"
    f" | gyro(rad/s) {gx:6.3f} {gy:6.3f} {gz:6.3f}"
    f" | proj_g {g[0]:6.3f} {g[1]:6.3f} {g[2]:6.3f} | imu temp {imu.temperature}"
  )
  lines.append("")
  lines.append(
    f"{'idx':>3} {'joint':<27} {'q':>8} {'dq':>8} {'temp':>5} {'min seen':>9} {'max seen':>9}"
    f" {'MJCF lo':>8} {'MJCF hi':>8}  flag"
  )
  for idx, name in G1_23DOF_JOINTS.items():
    m = msg.motor_state[idx]
    lim = limits.get(name)
    flags = []
    if abs(m.dq) > MOVING_DQ:
      flags.append("<< moving")
    lf = limit_flag(m.q, lim)
    if lf:
      flags.append(lf)
    if m.motorstate:
      flags.append(f"err=0x{m.motorstate:x}")
    lo, hi = lim if lim else (float("nan"), float("nan"))
    lines.append(
      f"{idx:>3} {name:<27} {m.q:8.3f} {m.dq:8.3f} {max(m.temperature):5d}"
      f" {s['q_min'][idx]:9.3f} {s['q_max'][idx]:9.3f} {lo:8.3f} {hi:8.3f}  {' '.join(flags)}"
    )
  lines.append("")
  unused = "  ".join(f"{i}:{msg.motor_state[i].q:+.3f}" for i in UNUSED_SLOTS)
  lines.append(f"unused slots q: {unused}")
  lines.append("Read-only: this script never publishes rt/lowcmd. Ctrl-C to stop and print the summary.")
  return "\n".join(lines)


def summary(s: dict, limits: dict) -> str:
  lines = ["", "=" * 60, "Summary"]
  avg_rate = (s["count"] - 1) / s["elapsed"] if s["elapsed"] > 0 else 0.0
  lines.append(f"messages {s['count']} over {s['elapsed']:.1f} s, average {avg_rate:.1f} Hz")
  lines.append(f"max gap {s['max_gap'] * 1e3:.1f} ms, gaps > {GAP_WARN_S * 1e3:.0f} ms: {s['gaps_over_warn']}")
  seen = sorted(s["mode_machine_seen"])
  mm_ok = "OK" if seen == [EXPECTED_MODE_MACHINE] else f"!! expected only {EXPECTED_MODE_MACHINE}"
  lines.append(f"mode_machine seen {seen} ({mm_ok})")
  if s["motor_error_seen"]:
    errs = ", ".join(f"{i}:0x{v:x}" for i, v in sorted(s["motor_error_seen"].items()))
    lines.append(f"motor error codes seen: {errs}")
  else:
    lines.append("motor error codes seen: none")
  lines.append("")
  lines.append(f"{'idx':>3} {'joint':<27} {'min seen':>9} {'max seen':>9} {'range':>7} {'MJCF lo':>8} {'MJCF hi':>8}  note")
  for idx, name in G1_23DOF_JOINTS.items():
    q_lo, q_hi = s["q_min"][idx], s["q_max"][idx]
    lim = limits.get(name)
    note = []
    if q_hi - q_lo < 0.05:
      note.append("not moved")
    if lim and (q_lo < lim[0] or q_hi > lim[1]):
      note.append("OUTSIDE MJCF limit")
    lo, hi = lim if lim else (float("nan"), float("nan"))
    lines.append(
      f"{idx:>3} {name:<27} {q_lo:9.3f} {q_hi:9.3f} {q_hi - q_lo:7.3f} {lo:8.3f} {hi:8.3f}  {', '.join(note)}"
    )
  return "\n".join(lines)


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--iface", default="enp43s0", help="DDS network interface (enp43s0 for the robot, lo for sim)")
  parser.add_argument("--domain", type=int, default=0, help="Unitree DDS domain (default 0)")
  parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl-C)")
  parser.add_argument("--rate", type=float, default=5.0, help="screen refresh rate in Hz")
  parser.add_argument("--log", type=Path, default=None, help="save every message to this CSV file")
  parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF, help="MJCF used for the joint limits")
  parser.add_argument("--no-clear", action="store_true", help="do not clear the screen between refreshes")
  args = parser.parse_args()

  limits = load_joint_limits(args.mjcf)
  missing = [n for n in G1_23DOF_JOINTS.values() if n not in limits]
  if missing:
    print(f"warning: no MJCF limit for {missing}", file=sys.stderr)

  ChannelFactoryInitialize(args.domain, args.iface)
  monitor = LowStateMonitor(args.log)
  sub = ChannelSubscriber("rt/lowstate", LowState_)
  sub.Init(monitor.handler, 10)

  t_start = time.monotonic()
  try:
    while args.duration <= 0 or time.monotonic() - t_start < args.duration:
      text = render(monitor.snapshot(), limits)
      if not args.no_clear:
        sys.stdout.write("\x1b[H\x1b[2J")
      sys.stdout.write(text + "\n")
      sys.stdout.flush()
      time.sleep(1.0 / args.rate)
  except KeyboardInterrupt:
    pass
  finally:
    sub.Close()
    monitor.close()
    snap = monitor.snapshot()
    if snap["msg"] is not None:
      print(summary(snap, limits))
    else:
      print("\nNo rt/lowstate received.")
    if args.log is not None:
      print(f"log saved to {args.log}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
