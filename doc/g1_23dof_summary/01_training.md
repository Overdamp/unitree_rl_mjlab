# 01: การเทรน policy

[← สารบัญ](README.md)

- [1. สาย policy ทั้งหมด](#1-สาย-policy-ทั้งหมด)
- [2. เทรนแบบ best practice หรือยัง](#2-เทรนแบบ-best-practice-หรือยัง)
- [3. Hist5: เทรนจากศูนย์](#3-hist5-เทรนจากศูนย์)
- [4. yaw_ft: fine-tune การหมุน](#4-yaw_ft-fine-tune-การหมุน)
- [5. YawStand: fine-tune ให้ยืนนิ่ง](#5-yawstand-fine-tune-ให้ยืนนิ่ง)
- [6. คำสั่งบน DGX](#6-คำสั่งบน-dgx)
- [7. observation ที่ policy ใช้ (blind)](#7-observation-ที่-policy-ใช้-blind)

## 1. สาย policy ทั้งหมด

| ชื่อ | task | เริ่มจาก | สิ่งที่เปลี่ยน | ผล | ไฟล์ |
|---|---|---|---|---|---|
| P1 | Flat | ศูนย์ | DR พื้นฐาน | เดินได้ | `v0/exported/policy_phase1_10000.onnx` |
| P2 | Flat | P1 | DR มวล/ข้อต่อ/มอเตอร์ | เดินได้, สั่นเมื่อ delay 20–30 ms | `policy_phase2_13000.onnx` |
| P3a | Flat | P2 | action delay 0–40 ms | ❌ เดินได้แค่ ~0.25 m/s | tag `phase3a-lag0-40ms-collapsed` |
| **P3b** | Flat | P2 | action delay 0–20 ms | ✅ **ใช้ deploy รอบแรก** | `v0/exported/policy.onnx` (= `policy_phase3b_16000.onnx`) |
| Phase 4 | Rough-Blind | P3b | พื้นขรุขระ (blind), reward linear ×2 | — | `run_phase4_train.sh` |
| Hist5 | Rough-Blind-Hist5 | ศูนย์ | actor เห็น 5 เฟรม (80 → 400 input) | เดินเร็วดี, หมุนแย่ | DGX run `2026-10-28_04-53-15` |
| yaw_ft | Rough-Blind-Hist5-Yaw | Hist5 `model_14999` | reward หมุน ×2, `rel_heading_envs` 0.3 | หมุน ~78%, ยืนส่าย | `config/policy/velocity/hist5_yawft/` |
| yaw_stand | Rough-Blind-Hist5-YawStand | yaw_ft `model_17998` | `rel_standing_envs` 0.05 → 0.15 | ✅ ยืนนิ่ง, vx 0.8/1.0 ≥ 94%, หมุน 72–80%; ❌ เตะขาตอนแขวน | `config/policy/velocity/hist5_yawstand/` (run `2026-10-29_17-01-49_yaw_stand_ft`, `model_19497`) |

รายละเอียด DR ของ P1–P3b: [`../domain_randomization_phases_g1_23dof.md`](../domain_randomization_phases_g1_23dof.md)

## 2. เทรนแบบ best practice หรือยัง

ใช่ ไม่ต้องเทรนใหม่ ที่ใช้กันมีสองแบบ:

| แบบ | ใช้ที่ไหน | ทำอย่างไร |
|---|---|---|
| A. เทรนพื้นขรุขระตั้งแต่เริ่ม + terrain curriculum | legged_gym (Rudin et al. 2021), Isaac Lab, mjlab | เริ่มที่แถวง่ายสุด (เกือบเรียบ), เดินได้ไกลพอ → เลื่อนไปแถวยากขึ้น, เดินได้น้อย → ลดลงมา |
| B. เทรนพื้นเรียบก่อน แล้ว fine-tune บนพื้นขรุขระ | เมื่อมี policy พื้นเรียบที่ดีอยู่แล้ว | resume checkpoint (obs ต้องขนาดเท่าเดิม) |

"เทรนพื้นขรุขระก่อนพื้นเรียบ" ไม่ใช่วิธีหลัก พื้นเรียบรวมอยู่ใน curriculum อยู่แล้ว

- สายหลักใช้แบบ B: Flat (P1 → P3b) → Phase 4 Rough-Blind (resume P3b)
- Hist5 ใช้แบบ A และ**ต้อง**ใช้ เพราะ input 400 ตัว (จาก 80) โหลด P3b ไม่ได้
- config ของ Hist5 ตรงกับแบบ A: `terrain_generator.curriculum = True`, `max_init_terrain_level = 1`, ทุกระดับมีพื้นเรียบ 20%, ลดความยากสำหรับ blind (บันได ≤ 8 cm, ทางลาด ≤ ~22°)

**อ่านค่า terrain level:** มี 10 ระดับ (0–9) หุ่นที่ผ่านระดับสูงสุดถูกสุ่มกลับไประดับ 0–9 ค่าเฉลี่ยตอนผ่านหมดจึงอยู่ที่ ~4.5 ไม่ใช่ 9 Hist5 จบที่ 5.3 = ผ่านเกือบครบ

## 3. Hist5: เทรนจากศูนย์

- task `Unitree-G1-23Dof-Rough-Blind-Hist5`, `run_hist5_train.sh`, 15000 iteration, 1024 env, ~19 ชั่วโมงบน DGX
- actor ใช้ `history_length = 5`, `flatten_history_dim = True` (ต่อแต่ละ term จากเก่าไปใหม่ ตรงกับ deploy `use_gym_history: false`)
- ผลตอนเทรน (iteration 14999): error_vel_xy 0.62, error_vel_yaw 2.20, fell_over 0.63 / time_out 3.46 (ล้ม ~15%), action std 0.46

**หลังรอบสุดท้ายหน้าจอค้าง:** rsl_rl บันทึก `model_14999.pt` → `VelocityOnPolicyRunner.save` export `policy.onnx` → ปิด tensorboard writer → `env.close()` ที่ค้างคือสองขั้นท้าย ถ้ามี `model_<last>.pt` และ `policy.onnx` เวลาตรงกันแล้ว กด Ctrl-C ได้

## 4. yaw_ft: fine-tune การหมุน

ปัญหา: sim2sim ของ Hist5 หมุนได้แค่ 30–40% ของคำสั่ง (P3b ~70%) สาเหตุที่น่าจะเป็น:

1. Rough-Blind เพิ่มน้ำหนัก `track_linear_velocity` เป็น 2.0 แต่ angular ยัง 1.0
2. `rel_heading_envs = 1.0` (ค่า default ของ mjlab): ทุก env ที่ไม่ยืนได้ wz จาก heading controller (`0.5 × heading error`) ซึ่งลดลงเป็นศูนย์ ไม่เหมือนจอยที่ส่ง wz คงที่

แก้ใน task `Unitree-G1-23Dof-Rough-Blind-Hist5-Yaw`: `track_angular_velocity.weight` 2.0, `rel_heading_envs` 0.3 resume `model_14999` อีก 3000 iteration (`run_hist5_yaw_finetune.sh`, run `2026-10-29_14-08-55_yaw_ft`)

ผลตอนเทรน: ล้มเหลือ ~4% (0.08 / 1.88), error_vel_yaw 1.96 ผล sim2sim ดู [02_sim2sim_eval.md](02_sim2sim_eval.md)

## 5. YawStand: fine-tune ให้ยืนนิ่ง

ปัญหาของ yaw_ft: ตอน cmd = 0 ยืนส่าย (joint vel std 0.055 rad/s, yaw rate std 0.065 rad/s; P3b และ Hist5 ≈ 0) สมมติฐาน: หุ่นที่ถูกสั่งให้ยืนมีแค่ 5% และคำสั่งหมุนสุ่มแบบ uniform ทำให้ฝึกยืนนิ่งน้อย

แก้ใน task `Unitree-G1-23Dof-Rough-Blind-Hist5-YawStand`: `rel_standing_envs` 0.15 (ค่าอื่นเหมือน yaw_ft) resume `model_17998` อีก 1500 iteration (`run_hist5_yaw_stand_finetune.sh`, run name `yaw_stand_ft`) smoke test ผ่าน

เป้าหมาย: `vel_eval.py` ที่ cmd = 0 ได้ `joint_vel_std` และ `yaw_rate_std` ≈ 0 โดยหมุนยังได้ ≥ 70%

ผล (run `2026-10-29_17-01-49_yaw_stand_ft`, `model_19497`): ยืนนิ่ง 0.000 / 0.000 ✅, vx 0.4/0.8/1.0 = 88/94/95%, หมุน 72–80%, ไม่ล้ม; ตอนเทรน (เฉลี่ย 200 iteration สุดท้าย) ล้ม 0.25 / time_out 1.41 ใกล้ yaw_ft แต่ยังเตะขาตอนแขวน (`--hang`: ความเร็ว joint 131% ของ limit) ดู [02_sim2sim_eval.md](02_sim2sim_eval.md)

## 6. คำสั่งบน DGX

```bash
cd <worktree hist5 บน DGX>
git pull origin hist5-from-scratch
conda activate unitree_rl_mjlab
byobu new -s train                                   # กัน ssh หลุด
./run_hist5_yaw_stand_finetune.sh 2>&1 | tee yaw_stand_ft.log
```

- worktree ต้องใส่ checkout ตัวเองไว้หน้า PYTHONPATH (สคริปต์ทำให้แล้ว) ไม่อย่างนั้น `import src` ไปเจอ checkout หลัก
- resume ต้องใช้ experiment เดียวกัน (`g1_23dof_velocity_hist5`) และ run เดิมต้องอยู่ใน `logs/rsl_rl/g1_23dof_velocity_hist5/`
- ค่าที่ดูระหว่างเทรน: `Metrics/twist/error_vel_yaw`, `error_vel_xy`, `Episode_Termination/fell_over`
- ดึงผลกลับ: โฟลเดอร์ run ทั้งโฟลเดอร์ (ต้องมี `model_*.pt`, `policy.onnx`, `params/`)

## 7. observation ที่ policy ใช้ (blind)

ไม่ใช้กล้องหรือ lidar ตอน deploy ไม่ต้องต่อ:

| obs | มาจาก |
|---|---|
| `base_ang_vel`, `projected_gravity` | IMU ในเชิงกราน (`rt/lowstate`) |
| `velocity_commands` | จอย / คีย์บอร์ด |
| `gait_phase` | นาฬิกาใน `g1_ctrl` (รอบ 0.6 s, เป็น 0 เมื่อ \|cmd\| < 0.1) |
| `joint_pos_rel`, `joint_vel_rel` | encoder (`rt/lowstate`) |
| `last_action` | action รอบก่อน |

`Unitree-G1-23Dof-Rough` (ไม่มีคำว่า Blind) ใช้ `height_scan` จึง deploy กับ `g1_ctrl` ไม่ได้ lidar ใช้ทีหลังในระดับนำทาง (g1pilot ส่ง vx, vy, wz แทนจอย)
