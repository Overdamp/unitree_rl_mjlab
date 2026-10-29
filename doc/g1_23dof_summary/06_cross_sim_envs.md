# 06: unitree_rl_lab (Isaac Lab) และ environment

[← สารบัญ](README.md)

- [1. แผนที่ environment](#1-แผนที่-environment)
- [2. unitree_rl_lab กับ G1-23dof](#2-unitree_rl_lab-กับ-g1-23dof)
- [3. เชื่อมสองสาย: deploy contract](#3-เชื่อมสองสาย-deploy-contract)
- [4. MicroDuck](#4-microduck)

## 1. แผนที่ environment

| โปรเจค | environment |
|---|---|
| unitree_rl_mjlab (งานหลัก) | conda `unitree_rl_mjlab` (editable install) |
| สคริปต์ที่ต่อหุ่น / sim2sim (`*_monitor.py`, `vel_eval.py`, `pd_check.py`) | conda `g1_sdk` (มี `unitree_sdk2py`, `onnxruntime`, `mujoco`) |
| unitree_rl_lab | worktree `~/nvidia/IsaacLab-unitree` @ `207f319d9` + uv venv `env_isaaclab` (Isaac Sim 6.0.0) รัน `python scripts/rsl_rl/...` ไม่ใช้ `unitree_rl_lab.sh` (อิง `$CONDA_PREFIX`) |
| MicroDuck | `~/nvidia/IsaacLab-main` (origin/main) |
| IsaacLab v3.0.0 | `~/nvidia/IsaacLab/.venv` ต้อง `uv sync --extra isaacsim` ไม่อย่างนั้น isaacsim ถูกลบ |

conda `env_isaaclab` เลิกใช้แล้ว

## 2. unitree_rl_lab กับ G1-23dof

- unitree_rl_lab ใช้กับ IsaacLab v3.0.0 ไม่ได้ (`ModuleNotFoundError isaaclab_tasks.manager_based`, ไม่มี `track_lin_vel_xy_yaw_frame_exp`) จึงใช้ IsaacLab รุ่นที่ pin ไว้ใน worktree แยก
- เขียน task ใหม่ (ยังไม่ commit, repo เป็น fork ของ unitreerobotics):
  - `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/23dof/velocity_env_cfg.py`: สืบทอดจาก 29dof, ใช้ `UNITREE_G1_23DOF_CFG` แต่แทน actuator ด้วยค่าแบบ mjlab (armature·ω², ζ = 2) และ action scale 0.25·effort/kp
  - `.../23dof/__init__.py`: register `Unitree-G1-23dof-Velocity` และ `-Flat`
- แผนงาน: `~/nvidia/unitree/unitree_rl_lab/doc/g1_23dof_cross_sim_plan.md`

| ขั้น | งาน | สถานะ |
|---|---|---|
| 0, 0.1 | uv sync, เลือก IsaacLab รุ่นเก่า | ✅ |
| 1, 2 | task 23dof + พารามิเตอร์แบบ mjlab | ✅ เขียนโค้ดแล้ว |
| 2.1 | smoke test (list_envs, 16 env × 5 iter, ตรวจ params/deploy.yaml) | ⏳ |
| 3 / 3.1 | เทรน Flat แล้ว Rough | – |
| 4 / 4.1 | ทำ `deploy.yaml` ให้เข้ากัน, รันใน `g1_ctrl` ของ mjlab | – |
| 5 | play P3b ใน Isaac | – |
| 6 | สรุป | – |

## 3. เชื่อมสองสาย: deploy contract

สิ่งที่ใช้ร่วมกันคือ `policy.onnx` + `deploy.yaml` (joint_ids_map, kp/kd, default pose, action scale, obs และ history)

- deploy C++ ของทั้งสอง repo รองรับ obs: base_ang_vel, projected_gravity, joint_pos, joint_pos_rel, joint_vel_rel, last_action, velocity_commands, gait_phase
- ต่างกัน: unitree_rl_lab export stiffness เรียงตาม SDK (27 ช่อง) แต่ mjlab เรียงตาม policy (23 ช่อง) ต้องแก้ในขั้น 4

## 4. MicroDuck

ไม่ขึ้นใน `list_envs` ของ IsaacLab v3.0.0 เพราะอยู่บน IsaacLab main ใช้ `~/nvidia/IsaacLab-main` ตาม [HF blog](https://huggingface.co/blog/nvidia/microduck-isaac-lab-sim-to-real) ระวัง `uv sync` ลบ isaacsim ออก (ดูข้อ 1)
