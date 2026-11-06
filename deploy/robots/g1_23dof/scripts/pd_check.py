#!/usr/bin/env python3
"""Check that the motors apply the commanded PD, from a lowcmd + lowstate recording.

Record both topics at the same time (same machine, both use time.monotonic()):

  python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --network enp43s0 --log state.csv
  python deploy/robots/g1_23dof/scripts/lowcmd_monitor.py   --network enp43s0 --log cmd.csv

then

  python deploy/robots/g1_23dof/scripts/pd_check.py state.csv cmd.csv
  python deploy/robots/g1_23dof/scripts/pd_check.py state.csv cmd.csv --replay --t0 5 --t1 25
  python deploy/robots/g1_23dof/scripts/pd_check.py state.csv cmd.csv --segments ~/sysid_seg.csv

1. Torque check (always): for each lowstate sample, take the lowcmd in effect
   (zero-order hold) and compare tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff
   with the motor's tau_est, per joint and per FSM gain set. The command delay is
   fitted first; then kp and kd ratios near 1 and a high correlation mean the
   motor applies the kp/kd we send.

2. Replay (--replay, robot hung on the gantry): play the recorded lowcmd into
   MuJoCo with the base held in the air (orientation from the IMU), starting from
   the recorded q, and compare simulated q with the real q. Large errors or lag
   mean the sim actuator (armature, friction, delay) differs from the real one.
   --armature train uses the training armature instead of the MJCF value (0.01).

3. Segments (--segments, from sysid_motion.py): fit each step / chirp test on
   its own joint and time window only, with a constant tau_est offset:
   tau_est - tau_ff = a*P + b*D + c. At the training gains the hold torques are
   small (a few Nm), so a tau_est offset of 0.1-0.3 Nm biases the whole-log fit
   (3/11/2026: right elbow 0.61 whole log, 0.96 on its step with the offset).
   Steps give kp; chirps move fast enough to give kd as well.
"""

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lowcmd_monitor import CONFIG_YAML, DEFAULT_DEPLOY, classify, expected_gains  # noqa: E402
from lowstate_monitor import G1_23DOF_JOINTS, NUM_SLOTS  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
SCENE_XML = REPO_ROOT / "src/assets/robots/unitree_g1/xmls/scene_g1_23dof.xml"

# Reflected inertia used in training (src/assets/robots/unitree_g1/g1_23dof_constants.py).
_ARM_5020 = 0.003609725
TRAIN_ARMATURE = [
  (r".*_(elbow|shoulder_pitch|shoulder_roll|shoulder_yaw|wrist_roll)_joint", _ARM_5020),
  (r".*_(hip_pitch|hip_yaw)_joint|waist_yaw_joint", 0.01017752),
  (r".*_(hip_roll|knee)_joint", 0.025101925),
  (r".*_ankle_(pitch|roll)_joint", 2 * _ARM_5020),
]

MIN_TAU_RMS = 0.3  # Nm, below this the torque fit is noise


def rms(x: np.ndarray) -> float:
  return float(np.sqrt(np.mean(x**2))) if len(x) else 0.0


def fit_pd(P: np.ndarray, D: np.ndarray, y: np.ndarray, offset: bool) -> tuple[float, float, float, np.ndarray]:
  """Least squares y ~ a*P + b*D (+ c): returns a, b, c and the fitted y.

  Both terms stay in the fit whenever they carry any torque, even when one is
  too small to score its gain: a dropped term that is correlated with the other
  biases the other gain (sim chirp, shoulder roll: kp x 0.67 without D, 0.99 with).
  """
  use_p, use_d = rms(P) > 1e-6, rms(D) > 1e-6
  cols = [P] * use_p + [D] * use_d + [np.ones_like(y)] * offset
  if not cols:
    return float("nan"), float("nan"), 0.0, np.zeros_like(y)
  X = np.stack(cols, axis=1)
  fit = np.linalg.lstsq(X, y, rcond=None)[0]
  a = float(fit[0]) if use_p else float("nan")
  b = float(fit[int(use_p)]) if use_d else float("nan")
  c = float(fit[-1]) if offset else 0.0
  return a, b, c, X @ fit
MIN_MOTION_STD = 0.02  # rad, joints that barely moved are not scored in the replay
MAX_LAG_S = 0.1
MAX_CMD_DELAY_S = 0.03
MIN_SEG_CORR = 0.8  # fit vs tau_est, per segment
SATURATION_FRAC = 0.95  # of the MJCF effort limit


def read_csv(path: Path, keys: list[str], decimate: int = 1) -> dict[str, np.ndarray]:
  """Columns named in keys ('t', 'q' -> q0..q28, ...) as arrays."""
  with open(path, newline="") as f:
    header = next(csv.reader(f))
  col = {n: i for i, n in enumerate(header)}
  idx = {}
  for k in keys:
    if k in col:
      idx[k] = [col[k]]
    elif f"{k}0" in col:
      idx[k] = [col[f"{k}{i}"] for i in range(NUM_SLOTS)]
    else:
      raise SystemExit(f"{path.name}: no column '{k}' (old log without it?)")
  # loadtxt keeps only the needed columns as floats, so multi-GB logs fit in memory.
  usecols = [c for cols in idx.values() for c in cols]
  data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=usecols, ndmin=2)[::decimate]
  out, start = {}, 0
  for k, cols in idx.items():
    a = data[:, start:start + len(cols)]
    out[k] = a[:, 0].copy() if len(cols) == 1 else a
    start += len(cols)
  return out


def expected_torque(state, cmd, delay: float):
  """tau from the lowcmd in effect `delay` s before each lowstate sample."""
  j = np.searchsorted(cmd["t"], state["t"] - delay, side="right") - 1
  ok = j >= 0
  st = {k: v[ok] for k, v in state.items()}
  j = j[ok]
  p_term = cmd["kp"][j] * (cmd["q"][j] - st["q"])
  d_term = cmd["kd"][j] * (cmd["dq"][j] - st["dq"])
  return st, j, p_term, d_term, p_term + d_term + cmd["tau"][j]


def effort_limits() -> np.ndarray:
  import mujoco

  m = mujoco.MjModel.from_xml_path(str(SCENE_XML))
  return np.abs(m.actuator_ctrlrange).max(axis=1)


def torque_check(state, cmd, expected) -> float:
  # The motor applies a command some time after g1_ctrl publishes it. Fit that
  # delay first (shared by all joints), or a few ms of latency looks like wrong gains.
  joints = list(G1_23DOF_JOINTS)
  # Score by correlation, which a wrong kp scale does not change.
  fits = []
  for delay in np.arange(0.0, MAX_CMD_DELAY_S + 1e-9, 0.001):
    st, _, _, _, tau = expected_torque(state, cmd, delay)
    e, r = tau[:, joints], st["tau"][:, joints]
    e, r = e - e.mean(axis=0), r - r.mean(axis=0)
    corr = (e * r).sum(axis=0) / np.sqrt((e**2).sum(axis=0) * (r**2).sum(axis=0) + 1e-12)
    fits.append((float(np.mean(corr)), delay))
  _, best_delay = max(fits)
  st, j, p_term, d_term, tau_exp = expected_torque(state, cmd, best_delay)
  tau_ff = tau_exp - p_term - d_term
  # Samples where the command asks for more than the effort limit are clipped by
  # the motor; leave them out of the fit and report how many there were.
  limit = effort_limits()
  unsat = np.abs(tau_exp) < SATURATION_FRAC * limit
  kp, kd = cmd["kp"][j], cmd["kd"][j]
  labels = np.array([classify(cmd["kp"][i], cmd["kd"][i], expected)[0] for i in range(len(cmd["t"]))])[j]

  print("Torque check: tau_est vs kp*(q_des-q) + kd*(dq_des-dq) + tau_ff")
  print("  kp x / kd x = least-squares fit tau_est - tau_ff = a*P + b*D (1.0 = motor applies the commanded gain)")
  print(f"  best-fit command delay {best_delay * 1e3:.0f} ms (lowcmd publish -> motor, searched 0-"
        f"{MAX_CMD_DELAY_S * 1e3:.0f} ms)")
  for state_name in dict.fromkeys(labels):
    m = labels == state_name
    dur = st["t"][m][-1] - st["t"][m][0] if m.sum() > 1 else 0.0
    print(f"\n[{state_name}] {m.sum()} samples, ~{dur:.1f} s")
    if m.sum() < 100:
      print("  too few samples")
      continue
    print(f"  {'idx':>3} {'joint':<27} {'kp':>6} {'kd':>5} {'rms exp':>8} {'rms est':>8} {'kp x':>5} {'kd x':>5} {'corr':>5} {'sat%':>5}  note")
    flagged = 0
    for idx, name in G1_23DOF_JOINTS.items():
      u = m & unsat[:, idx]
      sat = 100.0 * (1.0 - u.sum() / m.sum())
      e, r = tau_exp[u, idx], st["tau"][u, idx]
      P, D, ff = p_term[u, idx], d_term[u, idx], tau_ff[u, idx]
      rms_e, rms_r = rms(e), rms(r)
      corr = float(np.corrcoef(e, r)[0, 1]) if len(e) > 2 and np.std(e) > 1e-6 and np.std(r) > 1e-6 else float("nan")
      # Each gain is only scored when its term carries torque (kp = 0 in
      # Passive, small D term when joints move slowly), but both stay in the fit.
      p_scored, d_scored = rms(P) >= MIN_TAU_RMS, rms(D) >= MIN_TAU_RMS
      a, b, _, _ = fit_pd(P, D, r - ff, offset=False)
      a = a if p_scored else float("nan")
      b = b if d_scored else float("nan")
      note = ""
      if len(e) < 100:
        note = "saturated, not scored"
      elif not (p_scored or d_scored):
        note = "small torque, not scored"
      elif (p_scored and a < 0) or (d_scored and b < 0):
        note = "!! SIGN"
      elif (p_scored and not 0.8 <= a <= 1.2) or (d_scored and not 0.5 <= b <= 1.5) or not corr >= 0.9:
        note = "!! CHECK"
      if note in ("", "!! CHECK", "!! SIGN"):
        skipped = [g for g, ok in (("kp", p_scored), ("kd", d_scored)) if not ok]
        if skipped:
          note = (note + f" ({'/'.join(skipped)} not scored: small term)").strip()
      flagged += note.startswith("!!")
      print(
        f"  {idx:>3} {name:<27} {np.median(kp[m, idx]):6.1f} {np.median(kd[m, idx]):5.2f}"
        f" {rms_e:8.2f} {rms_r:8.2f} {a:5.2f} {b:5.2f} {corr:5.2f} {sat:5.1f}  {note}"
      )
    print(f"  -> {'OK' if not flagged else f'{flagged} joint(s) to check'}")
  return best_delay


def segment_check(state, cmd, delay: float, path: Path) -> None:
  with open(path, newline="") as f:
    segs = [r for r in csv.DictReader(f) if int(r["slot"]) >= 0]
  st, _, p_term, d_term, tau_exp = expected_torque(state, cmd, delay)
  tau_ff = tau_exp - p_term - d_term
  print(f"\nSegments ({path.name}): per test, own joint, fit tau_est - tau_ff = a*P + b*D + offset"
        f" (delay {delay * 1e3:.0f} ms)")
  print(f"  {'test':<22} {'kind':<5} {'amp':>5} {'rms P':>6} {'rms D':>6} {'kp x':>5} {'kd x':>5} {'off Nm':>7} {'corr':>5}  note")
  flagged = 0
  for s in segs:
    i = int(s["slot"])
    m = (st["t"] >= float(s["t0"])) & (st["t"] <= float(s["t1"]))
    if m.sum() < 100:
      print(f"  {s['test']:<22} {s['kind']:<5} {float(s['amp']):5.2f}  too few samples (log overlap?)")
      continue
    P, D, y = p_term[m, i], d_term[m, i], st["tau"][m, i] - tau_ff[m, i]
    p_ok, d_ok = rms(P) >= MIN_TAU_RMS, rms(D) >= MIN_TAU_RMS
    a, b, off, pred = fit_pd(P, D, y, offset=True)
    a = a if p_ok else float("nan")
    b = b if d_ok else float("nan")
    corr = float(np.corrcoef(pred, y)[0, 1]) if np.std(pred) > 1e-9 and np.std(y) > 1e-9 else float("nan")
    note = ""
    if not (p_ok or d_ok):
      note = "small torque, not scored"
    elif (p_ok and not 0.8 <= a <= 1.2) or (d_ok and not 0.5 <= b <= 1.5) or not corr >= MIN_SEG_CORR:
      note = "!! CHECK"
    if s.get("completed", "1") == "0":
      note = (note + " (stopped early)").strip()
    if not d_ok and note != "small torque, not scored":
      note = (note + " (kd not scored: small term)").strip()
    flagged += note.startswith("!!")
    print(f"  {s['test']:<22} {s['kind']:<5} {float(s['amp']):5.2f} {rms(P):6.2f} {rms(D):6.2f}"
          f" {a:5.2f} {b:5.2f} {off:+7.2f} {corr:5.2f}  {note}")
  print(f"  -> {'OK' if not flagged else f'{flagged} test(s) to check'}")


def replay(state, cmd, args) -> None:
  import mujoco

  m = mujoco.MjModel.from_xml_path(str(SCENE_XML))
  d = mujoco.MjData(m)
  assert m.nu == NUM_SLOTS, f"expected {NUM_SLOTS} actuators, got {m.nu}"
  jid = [m.actuator_trnid[i, 0] for i in range(m.nu)]
  qadr = np.array([m.jnt_qposadr[j] for j in jid])
  vadr = np.array([m.jnt_dofadr[j] for j in jid])
  lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
  if args.armature == "train":
    for i, j in enumerate(jid):
      name = m.joint(j).name
      for pattern, arm in TRAIN_ARMATURE:
        if re.fullmatch(pattern, name):
          m.dof_armature[vadr[i]] = arm
  arm_used = ", ".join(
    f"{G1_23DOF_JOINTS[i].removesuffix('_joint')} {m.dof_armature[vadr[i]]:.4f}" for i in (0, 1, 4, 15)
  )

  t_rel = state["t"] - state["t"][0]
  sel = (t_rel >= args.t0) & (t_rel <= (args.t1 if args.t1 > 0 else np.inf))
  if sel.sum() < 10:
    raise SystemExit("replay window has no samples")
  ts, q_real, dq_real = state["t"][sel], state["q"][sel], state["dq"][sel]
  quat = state["quat"][sel]
  delay = args.delay_ms * 1e-3

  mujoco.mj_resetData(m, d)
  d.qpos[qadr] = q_real[0]
  d.qvel[vadr] = dq_real[0]
  dt = m.opt.timestep
  n_steps = int((ts[-1] - ts[0]) / dt)
  sim_t = ts[0] + np.arange(n_steps + 1) * dt
  ci = np.searchsorted(cmd["t"], sim_t - delay, side="right") - 1
  si = np.clip(np.searchsorted(ts, sim_t, side="right") - 1, 0, len(ts) - 1)
  q_sim_steps = np.zeros((n_steps + 1, NUM_SLOTS))
  for k in range(n_steps + 1):
    d.qpos[0:3] = (0.0, 0.0, 2.0)  # held in the air, feet off the ground
    d.qpos[3:7] = quat[si[k]]
    d.qvel[0:6] = 0.0
    q_sim_steps[k] = d.qpos[qadr]
    c = ci[k]
    if c < 0:
      d.ctrl[:] = 0.0
    else:
      tau = cmd["kp"][c] * (cmd["q"][c] - d.qpos[qadr]) + cmd["kd"][c] * (cmd["dq"][c] - d.qvel[vadr]) + cmd["tau"][c]
      d.ctrl[:] = np.clip(tau, lo, hi)
    mujoco.mj_step(m, d)
  q_sim = q_sim_steps[np.clip(np.round((ts - ts[0]) / dt).astype(int), 0, n_steps)]

  print(f"\nReplay: {ts[-1] - ts[0]:.1f} s, base held in the air, armature={args.armature}"
        f" ({arm_used}), command delay {args.delay_ms:.0f} ms")
  print("  nrmse = rms(q_sim - q_real) / std(q_real); lag > 0 means the real joint lags the sim")
  print(f"  {'idx':>3} {'joint':<27} {'std real':>8} {'rms err':>8} {'nrmse':>6} {'lag ms':>7}  note")
  dt_s = np.median(np.diff(ts))
  max_shift = int(MAX_LAG_S / dt_s)
  flagged, scored, nrmses = 0, 0, []
  for idx, name in G1_23DOF_JOINTS.items():
    r, s = q_real[:, idx], q_sim[:, idx]
    std = np.std(r)
    err = np.sqrt(np.mean((s - r) ** 2))
    nrmse = err / std if std > 1e-9 else float("nan")
    note, lag = "", float("nan")
    if std < MIN_MOTION_STD:
      note = "barely moved, not scored"
    else:
      rc, sc = r - r.mean(), s - s.mean()
      shifts = range(-max_shift, max_shift + 1)
      xc = [np.dot(rc[max(0, k):len(rc) + min(0, k)], sc[max(0, -k):len(sc) - max(0, k)]) for k in shifts]
      lag = shifts[int(np.argmax(xc))] * dt_s * 1e3
      scored += 1
      nrmses.append(nrmse)
      if nrmse > 0.3:
        note = "!! sim differs"
        flagged += 1
    print(f"  {idx:>3} {name:<27} {std:8.3f} {err:8.3f} {nrmse:6.2f} {lag:7.1f}  {note}")
  if scored:
    print(f"  -> {scored} joints scored, median nrmse {np.median(nrmses):.2f},"
          f" {'OK' if not flagged else f'{flagged} joint(s) differ from the sim'}")
  if args.save:
    with open(args.save, "w", newline="") as f:
      w = csv.writer(f)
      w.writerow(["t"] + [f"real_q{i}" for i in G1_23DOF_JOINTS] + [f"sim_q{i}" for i in G1_23DOF_JOINTS])
      for k in range(len(ts)):
        w.writerow([f"{ts[k] - ts[0]:.4f}"] + [q_real[k, i] for i in G1_23DOF_JOINTS] + [q_sim[k, i] for i in G1_23DOF_JOINTS])
    print(f"  saved {args.save}")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("lowstate_csv", type=Path)
  parser.add_argument("lowcmd_csv", type=Path)
  parser.add_argument("--deploy", type=Path, default=DEFAULT_DEPLOY, help="deploy.yaml g1_ctrl loaded")
  parser.add_argument("--decimate", type=int, default=1, help="keep every Nth lowstate row for the torque check")
  parser.add_argument("--replay", action="store_true", help="also replay the commands in MuJoCo (gantry recordings)")
  parser.add_argument("--t0", type=float, default=0.0, help="replay start, s from the first lowstate sample")
  parser.add_argument("--t1", type=float, default=0.0, help="replay end (0 = end of log)")
  parser.add_argument("--armature", choices=("mjcf", "train"), default="mjcf")
  parser.add_argument("--delay-ms", type=float, default=0.0, help="extra command delay in the replay")
  parser.add_argument("--save", type=Path, default=None, help="write real and simulated q to this CSV")
  parser.add_argument("--segments", type=Path, default=None, help="sysid_motion.py --segments CSV: fit per test")
  args = parser.parse_args()

  expected = expected_gains(CONFIG_YAML, args.deploy)
  cmd = read_csv(args.lowcmd_csv, ["t", "q", "dq", "tau", "kp", "kd"])
  state = read_csv(args.lowstate_csv, ["t", "q", "dq", "tau"], args.decimate)
  print(f"lowstate {len(state['t'])} rows, lowcmd {len(cmd['t'])} rows,"
        f" overlap {max(0.0, min(state['t'][-1], cmd['t'][-1]) - max(state['t'][0], cmd['t'][0])):.1f} s\n")
  delay = torque_check(state, cmd, expected)
  if args.segments:
    segment_check(state, cmd, delay, args.segments)
  if args.replay:
    full = read_csv(args.lowstate_csv, ["t", "q", "dq", "quat_w", "quat_x", "quat_y", "quat_z"])
    full["quat"] = np.stack([full.pop(k) for k in ("quat_w", "quat_x", "quat_y", "quat_z")], axis=1)
    replay(full, cmd, args)
  return 0


if __name__ == "__main__":
  sys.exit(main())
