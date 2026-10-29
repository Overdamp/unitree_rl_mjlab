#!/usr/bin/env python3
"""Walking-speed readout for sim2sim (unitree_mujoco + g1_ctrl).

Subscribes to rt/sportmodestate (base position/velocity that unitree_mujoco
publishes; the real robot in debug mode does not) and rt/lowstate (IMU), and
prints 2 s averages of forward/lateral speed in the heading frame, yaw rate and
tilt, so the keyboard commands (w = 0.4, q/e = +-0.3 rad/s, a/d = +-0.2) can be
compared with what the robot actually does. Read-only.

  python deploy/robots/g1_23dof/scripts/sim_speed_monitor.py --iface lo --log /tmp/speed.csv
"""

import argparse
import csv
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

WINDOW_S = 2.0


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--iface", default="lo")
  parser.add_argument("--domain", type=int, default=0)
  parser.add_argument("--rate", type=float, default=2.0, help="print rate in Hz")
  parser.add_argument("--log", type=Path, default=None, help="CSV of t, x, y, vx, vy, wz, tilt_deg")
  args = parser.parse_args()

  from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
  from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
  from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

  lock = threading.Lock()
  imu = {"quat": (1.0, 0.0, 0.0, 0.0), "gyro_z": 0.0}
  samples = deque()  # (t, vx_heading, vy_heading, wz, tilt)
  log = csv.writer(open(args.log, "w", newline="")) if args.log else None
  if log:
    log.writerow(["t", "x", "y", "vx", "vy", "wz", "tilt_deg"])

  def on_lowstate(msg):
    with lock:
      imu["quat"] = tuple(msg.imu_state.quaternion)
      imu["gyro_z"] = msg.imu_state.gyroscope[2]

  def on_sport(msg):
    now = time.monotonic()
    with lock:
      w, x, y, z = imu["quat"]
      gyro_z = imu["gyro_z"]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    vx_w, vy_w = msg.velocity[0], msg.velocity[1]
    vx = math.cos(yaw) * vx_w + math.sin(yaw) * vy_w
    vy = -math.sin(yaw) * vx_w + math.cos(yaw) * vy_w
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    with lock:
      samples.append((now, vx, vy, gyro_z, tilt))
      while samples and now - samples[0][0] > WINDOW_S:
        samples.popleft()
      if log:
        log.writerow([f"{now:.4f}", msg.position[0], msg.position[1], vx, vy, gyro_z, tilt])

  ChannelFactoryInitialize(args.domain, args.iface)
  subs = [ChannelSubscriber("rt/lowstate", LowState_), ChannelSubscriber("rt/sportmodestate", SportModeState_)]
  subs[0].Init(on_lowstate, 10)
  subs[1].Init(on_sport, 10)
  print(f"{WINDOW_S:.0f} s averages in the heading frame (Ctrl-C to stop)")
  try:
    while True:
      time.sleep(1.0 / args.rate)
      with lock:
        s = list(samples)
      if not s:
        print("waiting for rt/sportmodestate ...")
        continue
      n = len(s)
      vx, vy, wz = (sum(r[i] for r in s) / n for i in (1, 2, 3))
      tilt = max(r[4] for r in s)
      print(f"vx {vx:6.3f}  vy {vy:6.3f}  wz {wz:6.3f}  max tilt {tilt:5.1f} deg")
  except KeyboardInterrupt:
    pass
  finally:
    for sub in subs:
      sub.Close()
  return 0


if __name__ == "__main__":
  sys.exit(main())
