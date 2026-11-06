#!/usr/bin/env python3
"""One-shot, read-only health check of the G1 23-DoF before a test day.

Listens for a few seconds and reports, each with PASS / WARN / FAIL:
  - rt/lowstate rate and mode_machine (4 for the 23-DoF)
  - rt/lowcmd traffic (another controller still running?)
  - motion_switcher mode ('ai' = Unitree controller, '' = released / debug)
  - offline motors (vol 0 or motorstate != 0; 25/26 after the 30/10 fall)
  - motor temperature and voltage, IMU roll / pitch
  - battery from rt/lf/bmsstate (SOC, SOH, cycles, voltage, current)

Never publishes rt/lowcmd and never changes the mode.

  python deploy/robots/g1_23dof/scripts/robot_health.py --network enp43s0              # real robot
  python deploy/robots/g1_23dof/scripts/robot_health.py                              # sim (default lo, domain 1)
  python deploy/robots/g1_23dof/scripts/robot_health.py --network enp43s0 --seconds 5
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lowstate_monitor import EXPECTED_MODE_MACHINE, G1_23DOF_JOINTS  # noqa: E402

MIN_RATE_HZ = 400  # lowstate is ~1000 Hz once booted; ~350 Hz right after power-on
TEMP_WARN_C = 60
TEMP_FAIL_C = 70  # sysid_motion.py stops at this
TILT_WARN_DEG = 10  # hung upright
SOC_WARN = 30


def line(status: str, text: str) -> str:
  return f"[{status:4s}] {text}"


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--network", "--iface", dest="iface", default="lo",
                      help="DDS interface like g1_ctrl --network: lo = sim (default), enp43s0 = real robot (say it explicitly); --iface = old name")
  parser.add_argument("--domain", type=int, default=None, help="default: 1 on lo (sim), 0 otherwise (real robot)")
  parser.add_argument("--seconds", type=float, default=3.0, help="listening time")
  args = parser.parse_args()
  # Sim is the default; the real robot needs an explicit --network enp43s0.
  if args.domain is None:
    args.domain = 1 if args.iface == "lo" else 0
  print(f"DDS: {'SIM' if args.iface == 'lo' else 'REAL ROBOT'} | network {args.iface} | domain {args.domain}", flush=True)

  from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
  from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
  from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowCmd_, LowState_

  ChannelFactoryInitialize(args.domain, args.iface)
  states, bms, n_cmd = [], [], [0]
  ChannelSubscriber("rt/lowstate", LowState_).Init(lambda m: states.append(m), 10)
  ChannelSubscriber("rt/lowcmd", LowCmd_).Init(lambda m: n_cmd.__setitem__(0, n_cmd[0] + 1), 10)
  ChannelSubscriber("rt/lf/bmsstate", BmsState_).Init(lambda m: bms.append(m), 10)
  time.sleep(args.seconds)
  if not states:
    print(line("FAIL", f"no rt/lowstate on {args.iface}: robot on? cable? ping 192.168.123.161"))
    return 1

  out, worst = [], 0
  rank = {"PASS": 0, "WARN": 1, "FAIL": 2}

  def add(status: str, text: str):
    nonlocal worst
    worst = max(worst, rank[status])
    out.append(line(status, text))

  m = states[-1]
  rate = len(states) / args.seconds
  add("PASS" if rate >= MIN_RATE_HZ else "WARN", f"rt/lowstate {rate:.0f} Hz" + ("" if rate >= MIN_RATE_HZ else " (still booting? read again)"))
  add("PASS" if m.mode_machine == EXPECTED_MODE_MACHINE else "FAIL", f"mode_machine {m.mode_machine} (expected {EXPECTED_MODE_MACHINE})")
  add("PASS" if n_cmd[0] == 0 else "WARN", f"rt/lowcmd messages in {args.seconds:.0f} s: {n_cmd[0]}" + ("" if n_cmd[0] == 0 else " (g1_ctrl or another controller running)"))

  msc = MotionSwitcherClient()
  msc.SetTimeout(3.0)
  msc.Init()
  code, mode = msc.CheckMode()
  if code != 0:
    add("WARN", f"motion_switcher CheckMode failed, code {code}")
  else:
    name = (mode or {}).get("name", "")
    add("PASS", f"motion mode form={(mode or {}).get('form', '')!r} name={name!r} "
        + ("(released / debug: g1_ctrl and sysid can run)" if not name else "(Unitree controller: motion_mode.py release before g1_ctrl)"))

  # unitree_mujoco reports vol 0 for every motor: only a robot with some motors
  # powered can show one as offline by its voltage.
  sim = all(m.motor_state[i].vol == 0 for i in G1_23DOF_JOINTS)
  offline = [i for i in G1_23DOF_JOINTS if (m.motor_state[i].vol == 0 and not sim) or m.motor_state[i].motorstate]
  if sim:
    add("WARN", "every motor reports vol 0: simulation (offline check by motorstate only)")
  if offline:
    add("FAIL", "offline motors: " + ", ".join(f"{i} {G1_23DOF_JOINTS[i]} (vol {m.motor_state[i].vol:.0f}, "
                                               f"motorstate {m.motor_state[i].motorstate:#x})" for i in offline))
  else:
    add("PASS", "all 23 motors online")
  online = [i for i in G1_23DOF_JOINTS if i not in offline]
  if online:
    temps = sorted(((max(m.motor_state[i].temperature), i) for i in online), reverse=True)
    t_max = temps[0][0]
    status = "PASS" if t_max < TEMP_WARN_C else "WARN" if t_max < TEMP_FAIL_C else "FAIL"
    add(status, "motor temp max {} C, hottest: {}".format(t_max, ", ".join(f"{G1_23DOF_JOINTS[i]} {t}" for t, i in temps[:3])))
    if not sim:
      vols = [m.motor_state[i].vol for i in online]
      add("PASS", f"motor voltage {min(vols):.1f}-{max(vols):.1f} V")
  roll, pitch, yaw = (math.degrees(a) for a in m.imu_state.rpy)
  tilt = max(abs(roll), abs(pitch))
  add("PASS" if tilt < TILT_WARN_DEG else "WARN", f"IMU roll {roll:+.1f} pitch {pitch:+.1f} yaw {yaw:+.1f} deg")

  if bms and not any(bms[-1].bmsvoltage):
    add("WARN", "rt/lf/bmsstate has no voltage (simulation): battery unknown")
  elif bms:
    b = bms[-1]
    cells = [v / 1000 for v in b.bmsvoltage if v]
    status = "PASS" if b.soc >= SOC_WARN else "WARN"
    add(status, f"battery SOC {b.soc}%, SOH {b.soh}%, {b.cycle} cycles, "
        f"{max(cells) if cells else 0:.1f} V, current {b.current} (raw), "
        f"temp {max(b.temperature)} C")
  else:
    add("WARN", "no rt/lf/bmsstate message (battery unknown)")

  print("\n".join(out))
  print(json.dumps({"verdict": ["PASS", "WARN", "FAIL"][worst], "offline": offline}))
  return worst


if __name__ == "__main__":
  sys.exit(main())
