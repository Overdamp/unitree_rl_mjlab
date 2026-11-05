#!/usr/bin/env python3
"""Check the joint zero offsets against the 28/10/2026 hard stops (read-only).

A joint calibration (Unitree Explore app) or a slipped encoder moves the zero:
the mechanical stops then read at different angles. The relaxed hanging pose
cannot show this (it differs by up to 0.3 rad between days), the stops can.

Move each joint slowly by hand to both stops while this runs (robot hung, in
damping or zero torque), then Ctrl-C. Per joint it compares the q min / max
seen with the stops measured on 28/10 (analyze_joint_sweep.py on
g1_joints_20261028_1848.csv / _1927.csv, table in the real robot checklist):

  OK            both stops within TOL of 28/10
  SHIFT +x      both stops moved the same way: the zero changed by ~x rad
  not reached   a side stayed short of its 28/10 stop: push further
  past 28/10    one side went further than on 28/10 (that day it was not pushed
                fully, or the zero moved: check the other side)

The final table is also saved to ~/offset_check_<date>_<time>.txt (--out).

Never publishes rt/lowcmd.

  python deploy/robots/g1_23dof/scripts/offset_check.py --iface enp43s0 --joints right_elbow,right_wrist_roll
  python deploy/robots/g1_23dof/scripts/offset_check.py --iface enp43s0       # all 23 joints (default lo = sim)
  python deploy/robots/g1_23dof/scripts/offset_check.py --csv ~/g1_joints_20261028_1848.csv
"""

import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lowstate_monitor import G1_23DOF_JOINTS, NUM_SLOTS  # noqa: E402

# q min / max at the stops by hand on 28/10/2026 (rad). Shoulder roll inwards is
# stopped by the torso, not by the joint: less repeatable, still compared.
REF_STOPS = {
  0: (-2.534, 2.808), 1: (-0.524, 2.962), 2: (-2.761, 2.747), 3: (-0.086, 2.877),
  4: (-0.868, 0.514), 5: (-0.284, 0.271),
  6: (-2.536, 2.867), 7: (-2.963, 0.531), 8: (-2.750, 2.760), 9: (-0.087, 2.895),
  10: (-0.868, 0.525), 11: (-0.319, 0.242),
  12: (-2.704, 2.704),
  15: (-3.065, 2.666), 16: (-0.146, 2.245), 17: (-2.613, 2.608), 18: (-1.038, 2.089), 19: (-1.998, 1.959),
  22: (-3.076, 2.669), 23: (-2.255, 0.149), 24: (-2.625, 2.603), 25: (-1.033, 2.073), 26: (-1.967, 1.979),
}
TOL = 0.03  # rad, stop-to-stop repeatability by hand
REACH = 0.15  # rad, a side this far short of its stop was not pushed there


def verdict(lo: float, hi: float, ref: tuple[float, float]) -> str:
  d_lo, d_hi = lo - ref[0], hi - ref[1]
  ok_lo, ok_hi = abs(d_lo) <= TOL, abs(d_hi) <= TOL
  if ok_lo and ok_hi:
    return "OK"
  if d_lo > REACH or d_hi < -REACH:
    side = [s for s, short in (("min", d_lo > REACH), ("max", d_hi < -REACH)) if short]
    return "not reached (" + ", ".join(side) + ")"
  if not ok_lo and not ok_hi and np.sign(d_lo) == np.sign(d_hi) and abs(d_lo - d_hi) <= 2 * TOL:
    return f"SHIFT {(d_lo + d_hi) / 2:+.3f}"
  past = [s for s, p in (("min", d_lo < -TOL), ("max", d_hi > TOL)) if p]
  short = [s for s, p in (("min", d_lo > TOL), ("max", d_hi < -TOL)) if p]
  parts = []
  if past:
    parts.append("past 28/10 (" + ", ".join(past) + ")")
  if short:
    parts.append("short (" + ", ".join(short) + ")")
  return "; ".join(parts)


def report(q_min: np.ndarray, q_max: np.ndarray, slots: list[int]) -> str:
  lines = [f"{'idx':>3} {'joint':<27} {'min':>7} {'ref':>7} {'d':>7}   {'max':>7} {'ref':>7} {'d':>7}   verdict"]
  for i in slots:
    ref = REF_STOPS[i]
    if not np.isfinite(q_min[i]):
      lines.append(f"{i:>3} {G1_23DOF_JOINTS[i]:<27} {'no data (offline or no message)':>59}")
      continue
    lines.append(
      f"{i:>3} {G1_23DOF_JOINTS[i]:<27} {q_min[i]:7.3f} {ref[0]:7.3f} {q_min[i] - ref[0]:+7.3f}   "
      f"{q_max[i]:7.3f} {ref[1]:7.3f} {q_max[i] - ref[1]:+7.3f}   {verdict(q_min[i], q_max[i], ref)}"
    )
  return "\n".join(lines)


def from_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
  header = open(path).readline().strip().split(",")
  cols = [header.index(f"q{i}") for i in range(NUM_SLOTS)]
  q_min = np.full(NUM_SLOTS, np.inf)
  q_max = np.full(NUM_SLOTS, -np.inf)
  # Chunks keep multi-GB logs out of memory.
  with open(path) as f:
    f.readline()
    while True:
      lines = [f.readline() for _ in range(200000)]
      lines = [ln for ln in lines if ln]
      if not lines:
        break
      q = np.loadtxt(lines, delimiter=",", usecols=cols, ndmin=2)
      q_min = np.minimum(q_min, q.min(axis=0))
      q_max = np.maximum(q_max, q.max(axis=0))
  return q_min, q_max


def live(args, slots: list[int]) -> tuple[np.ndarray, np.ndarray]:
  from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
  from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

  lock = threading.Lock()
  q_min = np.full(NUM_SLOTS, np.inf)
  q_max = np.full(NUM_SLOTS, -np.inf)
  offline = set()
  count = [0]

  def handler(msg: LowState_):
    q = np.array([msg.motor_state[i].q for i in range(NUM_SLOTS)])
    # unitree_mujoco reports vol 0 for every motor; on the robot vol 0 = offline.
    sim = all(msg.motor_state[i].vol == 0 for i in G1_23DOF_JOINTS)
    bad = {i for i in slots if (msg.motor_state[i].vol == 0 and not sim) or msg.motor_state[i].motorstate}
    with lock:
      count[0] += 1
      offline.clear()
      offline.update(bad)
      good = [i for i in range(NUM_SLOTS) if i not in bad]
      q_min[good] = np.minimum(q_min[good], q[good])
      q_max[good] = np.maximum(q_max[good], q[good])

  ChannelFactoryInitialize(args.domain, args.iface)
  sub = ChannelSubscriber("rt/lowstate", LowState_)
  sub.Init(handler, 10)
  t0 = time.time()
  try:
    while args.duration <= 0 or time.time() - t0 < args.duration:
      time.sleep(1.0)
      with lock:
        n, lo, hi, off = count[0], q_min.copy(), q_max.copy(), sorted(offline)
      if n == 0:
        print("waiting for rt/lowstate ... (interface, cable, robot on?)")
        continue
      print("\033[2J\033[H" + f"offset_check  {time.time() - t0:5.0f} s  {n} msgs  (Ctrl-C to finish)")
      if off:
        print(f"OFFLINE motors (vol 0 / motorstate != 0), not measured: {off}")
      print(report(lo, hi, slots))
  except KeyboardInterrupt:
    pass
  with lock:
    return q_min.copy(), q_max.copy()


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--csv", type=Path, default=None, help="read a lowstate_monitor.py log instead of the robot")
  parser.add_argument("--joints", default=None, help="comma-separated name parts, e.g. right_elbow,wrist (default all)")
  parser.add_argument("--iface", default="lo", help="lo = sim (default), enp43s0 = real robot (say it explicitly)")
  parser.add_argument("--domain", type=int, default=None, help="default: 1 on lo (sim), 0 otherwise (real robot)")
  parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl-C)")
  parser.add_argument("--out", default=None,
                      help="save the final table here (default ~/offset_check_<date>_<time>.txt, '' = do not save)")
  args = parser.parse_args()
  # Sim is the default; the real robot needs an explicit --iface enp43s0.
  if args.domain is None:
    args.domain = 1 if args.iface == "lo" else 0
  print(f"DDS: {'SIM' if args.iface == 'lo' else 'REAL ROBOT'} | iface {args.iface} | domain {args.domain}", flush=True)

  slots = list(REF_STOPS)
  if args.joints:
    parts = [p.strip() for p in args.joints.split(",") if p.strip()]
    slots = [i for i in slots if any(p in G1_23DOF_JOINTS[i] for p in parts)]
    if not slots:
      parser.error(f"no joint matches {args.joints!r}")

  t_start = datetime.now()
  q_min, q_max = from_csv(args.csv) if args.csv else live(args, slots)
  verdicts = [verdict(q_min[i], q_max[i], REF_STOPS[i]) for i in slots]
  shifted = [G1_23DOF_JOINTS[i] for i, v in zip(slots, verdicts) if v.startswith("SHIFT")]
  source = f"csv {args.csv}" if args.csv else f"live {args.iface} domain {args.domain}"
  text = (f"# offset_check {t_start:%Y-%m-%d %H:%M:%S}, {source}, joints {args.joints or 'all'}\n"
          f"Stops vs 28/10/2026 (TOL {TOL} rad):\n{report(q_min, q_max, slots)}\n")
  if shifted:
    text += f"\nZERO SHIFTED: {', '.join(shifted)}. Do not run policies until this is understood.\n"
  print()
  print(text, end="")
  out = args.out if args.out is not None else str(Path.home() / f"offset_check_{t_start:%Y%m%d_%H%M%S}.txt")
  if out:
    Path(out).expanduser().write_text(text)
    print(f"saved {Path(out).expanduser()}")
  return 1 if shifted else 0


if __name__ == "__main__":
  sys.exit(main())
