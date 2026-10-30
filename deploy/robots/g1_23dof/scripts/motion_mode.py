#!/usr/bin/env python3
"""Check, release or select the G1's built-in motion control mode (MotionSwitcher RPC).

The robot's own motion control service (sport / ai mode, driven by the remote)
must not run while g1_ctrl sends rt/lowcmd. The remote way is L2 + R2 (debug
mode, left by rebooting). This does the same from the PC with the SDK's
MotionSwitcherClient, like unitree_sdk2's g1_ankle_swing_example:

  check            read-only: print the active mode (name empty = released)
  release          stop the service (like debug mode); loops until released
  select <name>    start a service again ("ai", "normal", "advanced"; which one
                   the G1 23-DoF uses is not verified yet, see `check` first)

Do release / select only with the robot hung on the gantry: once released
nothing holds the joints, and select may start in a mode that moves. select
refuses while rt/lowcmd is being published (g1_ctrl still running).

  python deploy/robots/g1_23dof/scripts/motion_mode.py check
  python deploy/robots/g1_23dof/scripts/motion_mode.py release
  python deploy/robots/g1_23dof/scripts/motion_mode.py select ai
"""

import argparse
import sys
import time

LOWCMD_LISTEN_S = 0.5
RELEASE_TRIES = 10


def check_mode(msc) -> dict | None:
  code, data = msc.CheckMode()
  if code != 0:
    print(f"CheckMode failed, code {code} (no reply: wrong interface, robot off, or no motion_switcher service)")
    return None
  return data or {}


def lowcmd_traffic(listen_s: float) -> int:
  """Number of rt/lowcmd messages seen in listen_s (any process, read-only)."""
  from unitree_sdk2py.core.channel import ChannelSubscriber
  from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

  count = [0]

  def on_msg(_):
    count[0] += 1

  sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
  sub.Init(on_msg, 10)
  time.sleep(listen_s)
  sub.Close()
  return count[0]


def confirm(text: str, assume_yes: bool) -> bool:
  if assume_yes:
    return True
  print(text)
  return input("Type 'yes' to continue: ").strip().lower() == "yes"


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("action", choices=("check", "release", "select"))
  parser.add_argument("name", nargs="?", help="mode name for select (e.g. ai, normal)")
  parser.add_argument("--iface", default="enp43s0", help="DDS network interface")
  parser.add_argument("--domain", type=int, default=0)
  parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
  args = parser.parse_args()
  if args.action == "select" and not args.name:
    parser.error("select needs a mode name, e.g. `select ai`")

  from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
  from unitree_sdk2py.core.channel import ChannelFactoryInitialize

  ChannelFactoryInitialize(args.domain, args.iface)
  msc = MotionSwitcherClient()
  msc.SetTimeout(5.0)
  msc.Init()

  mode = check_mode(msc)
  if mode is None:
    return 1
  name = mode.get("name", "")
  print(f"current mode: form={mode.get('form', '')!r} name={name!r}" + ("  (released / debug)" if not name else ""))
  n_cmd = lowcmd_traffic(LOWCMD_LISTEN_S)
  print(f"rt/lowcmd messages in {LOWCMD_LISTEN_S:.1f} s: {n_cmd}")

  if args.action == "check":
    return 0

  if args.action == "release":
    if not name:
      print("already released, nothing to do")
      return 0
    if not confirm("Releasing stops the built-in controller: the robot goes limp. Is it hung on the gantry?", args.yes):
      return 1
    for _ in range(RELEASE_TRIES):
      code, _ = msc.ReleaseMode()
      print(f"ReleaseMode -> code {code}")
      time.sleep(1.0)
      mode = check_mode(msc)
      if mode is not None and not mode.get("name"):
        print("released: start g1_ctrl now")
        return 0
    print(f"still not released after {RELEASE_TRIES} tries (last mode {mode})")
    return 1

  # select
  if n_cmd > 0:
    print("refusing: rt/lowcmd is being published (stop g1_ctrl first, the two controllers would fight)")
    return 1
  if name:
    print(f"note: mode {name!r} is already active")
  if not confirm(f"Selecting {args.name!r} starts the built-in controller. Is the robot hung on the gantry?", args.yes):
    return 1
  code, _ = msc.SelectMode(args.name)
  print(f"SelectMode({args.name!r}) -> code {code}")
  time.sleep(1.0)
  mode = check_mode(msc)
  print(f"mode now: {mode}")
  return 0 if code == 0 and mode and mode.get("name") else 1


if __name__ == "__main__":
  sys.exit(main())
