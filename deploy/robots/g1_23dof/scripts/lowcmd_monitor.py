#!/usr/bin/env python3
"""Read-only LowCmd monitor for the G1 23-DoF.

Subscribes to ``rt/lowcmd`` (what g1_ctrl sends to the motors) and never
publishes, so it does not command the robot and does not trip the g1_ctrl
"other process is using the lowcmd channel" check. It shows per slot the
target q / dq, kp, kd and feed-forward tau, recognises which FSM state the
gains belong to (Passive / FixStand / Velocity from config.yaml and
deploy.yaml) and flags every slot whose kp/kd differ from that state, including
unused 23-DoF slots that should get kp = kd = 0 in Velocity.

Record it together with lowstate_monitor.py --log and compare the torques with
pd_check.py.

  python deploy/robots/g1_23dof/scripts/lowcmd_monitor.py --iface lo
  python deploy/robots/g1_23dof/scripts/lowcmd_monitor.py --iface enp43s0 --log /tmp/lowcmd.csv
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lowstate_monitor import G1_23DOF_JOINTS, NUM_SLOTS, UNUSED_SLOTS  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
CONFIG_YAML = REPO_ROOT / "deploy/robots/g1_23dof/config/config.yaml"
DEFAULT_DEPLOY = REPO_ROOT / "deploy/robots/g1_23dof/config/policy/velocity/v0/params/deploy.yaml"

GAIN_TOL = 0.06  # deploy.yaml rounds kp/kd to one decimal (0.9 for 0.907)


def expected_gains(config_yaml: Path, deploy_yaml: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
  """29-slot (kp, kd) per FSM state, as g1_ctrl sets them on enter()."""
  fsm = yaml.safe_load(open(config_yaml))["FSM"]
  dep = yaml.safe_load(open(deploy_yaml))
  vel_kp = np.zeros(NUM_SLOTS)
  vel_kd = np.zeros(NUM_SLOTS)
  vel_kp[dep["joint_ids_map"]] = dep["stiffness"]
  vel_kd[dep["joint_ids_map"]] = dep["damping"]
  return {
    "Passive": (np.zeros(NUM_SLOTS), np.array(fsm["Passive"]["kd"], float)),
    "FixStand": (np.array(fsm["FixStand"]["kp"], float), np.array(fsm["FixStand"]["kd"], float)),
    "Velocity": (vel_kp, vel_kd),
  }


def gains_match(a: np.ndarray, b: np.ndarray) -> np.ndarray:
  return np.abs(a - b) <= np.maximum(GAIN_TOL * np.abs(b), 1e-3)


def classify(kp: np.ndarray, kd: np.ndarray, expected: dict) -> tuple[str, list[int]]:
  """Best matching state and the slots whose kp or kd differ from it."""
  best, best_bad = "unknown", list(range(NUM_SLOTS))
  for name, (ekp, ekd) in expected.items():
    bad = [i for i in range(NUM_SLOTS) if not (gains_match(kp[i], ekp[i]) and gains_match(kd[i], ekd[i]))]
    if len(bad) < len(best_bad):
      best, best_bad = name, bad
  if len(best_bad) > NUM_SLOTS // 2:
    return "unknown", best_bad
  return best, best_bad


class LowCmdMonitor:
  def __init__(self, log_path: Path | None):
    self._lock = threading.Lock()
    self.msg = None
    self.count = 0
    self.t_first = None
    self.t_last = None
    self.max_gap = 0.0
    self.window = []
    self.states_seen = {}  # state -> message count
    self._log_file = None
    self._log = None
    if log_path is not None:
      self._log_file = open(log_path, "w", newline="")
      self._log = csv.writer(self._log_file)
      header = ["t", "mode_pr", "mode_machine"]
      for key in ("mode", "q", "dq", "tau", "kp", "kd"):
        header += [f"{key}{i}" for i in range(NUM_SLOTS)]
      self._log.writerow(header)
    self.expected = None

  def handler(self, msg):
    now = time.monotonic()
    cmds = msg.motor_cmd[:NUM_SLOTS]
    kp = np.array([c.kp for c in cmds])
    kd = np.array([c.kd for c in cmds])
    state, _ = classify(kp, kd, self.expected)
    with self._lock:
      if self.t_last is not None:
        self.max_gap = max(self.max_gap, now - self.t_last)
      else:
        self.t_first = now
      self.t_last = now
      self.count += 1
      self.window.append(now)
      while self.window and now - self.window[0] > 1.0:
        self.window.pop(0)
      self.msg = msg
      self.states_seen[state] = self.states_seen.get(state, 0) + 1
      if self._log is not None:
        row = [f"{now:.4f}", msg.mode_pr, msg.mode_machine]
        row += [c.mode for c in cmds] + [c.q for c in cmds] + [c.dq for c in cmds]
        row += [c.tau for c in cmds] + list(kp) + list(kd)
        self._log.writerow(row)

  def snapshot(self):
    with self._lock:
      return {
        "msg": self.msg,
        "count": self.count,
        "rate": len(self.window),
        "since_last": None if self.t_last is None else time.monotonic() - self.t_last,
        "max_gap": self.max_gap,
        "states_seen": dict(self.states_seen),
        "elapsed": 0.0 if self.t_first is None else self.t_last - self.t_first,
      }

  def close(self):
    if self._log_file is not None:
      self._log_file.close()


def slot_name(i: int) -> str:
  return G1_23DOF_JOINTS.get(i, f"(unused slot {i})")


def render(s: dict, expected: dict) -> str:
  msg = s["msg"]
  if msg is None:
    return "Waiting for rt/lowcmd ... (is g1_ctrl running? check interface)"
  cmds = msg.motor_cmd[:NUM_SLOTS]
  kp = np.array([c.kp for c in cmds])
  kd = np.array([c.kd for c in cmds])
  state, bad = classify(kp, kd, expected)
  since = s["since_last"]
  stale = " STALE" if since is not None and since > 0.1 else ""
  lines = [
    f"rt/lowcmd {s['rate']:4d} Hz | msgs {s['count']} | max gap {s['max_gap'] * 1e3:6.1f} ms{stale}",
    f"mode_machine {msg.mode_machine} | mode_pr {msg.mode_pr}",
    f"gains match: {state}" + (f"  !! {len(bad)} slot(s) differ: {bad}" if bad and state != "unknown" else ""),
    "",
  ]
  ekp, ekd = expected.get(state, (np.full(NUM_SLOTS, np.nan), np.full(NUM_SLOTS, np.nan)))
  lines.append(
    f"{'idx':>3} {'joint':<27} {'mode':>4} {'q_des':>7} {'dq_des':>7} {'kp':>7} {'kd':>6} {'tau_ff':>7}"
    f" {'exp kp':>7} {'exp kd':>6}  flag"
  )
  for i in range(NUM_SLOTS):
    c = cmds[i]
    flag = "<< MISMATCH" if i in bad and state != "unknown" else ""
    lines.append(
      f"{i:>3} {slot_name(i):<27} {c.mode:>4} {c.q:7.3f} {c.dq:7.3f} {c.kp:7.2f} {c.kd:6.2f} {c.tau:7.2f}"
      f" {ekp[i]:7.2f} {ekd[i]:6.2f}  {flag}"
    )
  lines.append("")
  lines.append(f"unused 23-DoF slots: {UNUSED_SLOTS} (kp = kd = 0 expected in Velocity)")
  lines.append("Read-only: this script never publishes. Ctrl-C to stop.")
  return "\n".join(lines)


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--iface", default="enp43s0", help="DDS network interface (enp43s0 for the robot, lo for sim)")
  parser.add_argument("--domain", type=int, default=0, help="Unitree DDS domain (default 0)")
  parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl-C)")
  parser.add_argument("--rate", type=float, default=5.0, help="screen refresh rate in Hz")
  parser.add_argument("--log", type=Path, default=None, help="save every message to this CSV file")
  parser.add_argument("--deploy", type=Path, default=DEFAULT_DEPLOY, help="deploy.yaml that g1_ctrl loads")
  parser.add_argument("--no-clear", action="store_true", help="do not clear the screen between refreshes")
  args = parser.parse_args()

  from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
  from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

  expected = expected_gains(CONFIG_YAML, args.deploy)
  ChannelFactoryInitialize(args.domain, args.iface)
  monitor = LowCmdMonitor(args.log)
  monitor.expected = expected
  sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
  sub.Init(monitor.handler, 10)

  t_start = time.monotonic()
  try:
    while args.duration <= 0 or time.monotonic() - t_start < args.duration:
      text = render(monitor.snapshot(), expected)
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
    if snap["msg"] is None:
      print("\nNo rt/lowcmd received.")
    else:
      avg = (snap["count"] - 1) / snap["elapsed"] if snap["elapsed"] > 0 else 0.0
      print(f"\nmessages {snap['count']} over {snap['elapsed']:.1f} s, average {avg:.1f} Hz,"
            f" max gap {snap['max_gap'] * 1e3:.1f} ms")
      print("gain sets seen (messages): " + ", ".join(f"{k} {v}" for k, v in snap["states_seen"].items()))
    if args.log is not None:
      print(f"log saved to {args.log}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
