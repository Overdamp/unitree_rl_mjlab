# 02: ทดสอบ sim2sim

[← สารบัญ](README.md)

- [1. สองวิธีที่ใช้](#1-สองวิธีที่ใช้)
- [2. ผลความเร็ว: P3b vs Hist5 vs yaw_ft](#2-ผลความเร็ว-p3b-vs-hist5-vs-yaw_ft)
- [3. ผลความปลอดภัยและการส่าย](#3-ผลความปลอดภัยและการส่าย)
- [4. รัน unitree_mujoco + g1_ctrl กับ policy ใดก็ได้](#4-รัน-unitree_mujoco--g1_ctrl-กับ-policy-ใดก็ได้)
- [5. ปัญหาที่เจอ](#5-ปัญหาที่เจอ)

## 1. สองวิธีที่ใช้

| วิธี | เครื่องมือ | ใช้ทำอะไร |
|---|---|---|
| headless | `deploy/robots/g1_23dof/scripts/vel_eval.py` | วัดเร็ว ทำซ้ำได้ ไม่ต้องมีจอ จำลอง obs/action ตาม `deploy.yaml` (รวม history) แบบเดียวกับ `g1_ctrl`, policy 50 Hz, PD 500 Hz, ไม่จำลอง delay ของ DDS |
| ครบระบบ | `unitree_mujoco` + `g1_ctrl` + `sim2sim_policy.sh` + `sim_speed_monitor.py` | ทดสอบโค้ด C++ จริง (obs, history, FSM) ต้องกดใน GUI |

`vel_eval.py` ตรวจแล้วว่าตรงกับของจริง: P3b ได้ 0.388 / 0.594 / 0.413 m/s ที่ vx 0.4 / 0.8 / 1.0 ตรงกับตารางใน [`../g1_23dof_metrics_manager.md`](../g1_23dof_metrics_manager.md) และ `unitree_mujoco` + `g1_ctrl` ที่ปุ่ม `w` ได้ 0.398

```bash
# env g1_sdk (env unitree_rl_mjlab ไม่มี onnxruntime)
D=deploy/robots/g1_23dof/config/policy/velocity
python deploy/robots/g1_23dof/scripts/vel_eval.py --policy $D/v0/exported/policy.onnx --deploy $D/v0/params/deploy.yaml
python deploy/robots/g1_23dof/scripts/vel_eval.py ... --cases "0,0,0;0.4,0,0" --no-smooth   # สั่งแบบ stick จริง
```

ผลลัพธ์ต่อกรณี (JSON): `vx`, `vy`, `wz`, `fell`, `tilt_max_deg`, `joint_vel_std`, `yaw_rate_std` และ `safety` (`entry_jump_rad`, `target_out_pct`, `target_over_rad`, `min_q_margin_rad`, `max_speed_ratio`, `max_torque_ratio`)

## 2. ผลความเร็ว: P3b vs Hist5 vs yaw_ft vs yaw_stand

พื้นเรียบ ยืน 2 s แล้วเดิน 12 s เฉลี่ย 8 s สุดท้าย

| คำสั่ง | P3b | Hist5 | yaw_ft | yaw_stand |
|---|---|---|---|---|
| vx 0.4 | **97%** | 82% | 76% | 88% |
| vx 0.8 | 74% | 91% | 83% | **94%** |
| vx 1.0 | 41% | 94% | 86% | **95%** |
| wz ±0.5 | 69% | 30–41% | **78–79%** | 72–79% |
| wz 1.0 | 73% | 34% | 78% | **80%** |
| (0.4, 0, 0.5) wz | 68% | 36% | 61% | **77%** |
| vy 0.3 | 59% | **72%** | 56% | 56% |
| ยืน: joint vel std / yaw rate std | 0 / 0 | ≈ 0 | 0.055 / 0.065 | **0 / 0** |

- ทุกตัวไม่ล้ม เอียง ≤ 5.7°
- P3b ค่อยๆ เลี้ยวขวาเอง ~2°/s, yaw_ft ที่ vx 1.0 เลี้ยวซ้ายเอง ~6°/s
- สรุป: P3b แม่นที่สุดที่ความเร็วต่ำ, Hist5 เดินเร็วดีที่สุดแต่หมุนแย่, yaw_ft หมุนดีที่สุดแต่ช้าลงบ้าง

## 3. ผลความปลอดภัยและการส่าย

7 กรณี (ยืน, vx 0.4/1.0/−0.5, wz 1.0, vy 0.5, ผสม) ทั้งแบบ smooth และแบบกระโดด (`--no-smooth`)

| รายการ | P3b | yaw_ft |
|---|---|---|
| ความเร็ว joint สูงสุด / limit มอเตอร์ | ≤ 29% (เข่า) | ≤ 45% (เข่า) |
| แรงบิดสูงสุด / effort limit | ≤ 66% (ข้อเท้า) | ≤ 66% |
| q จริงห่าง limit อย่างน้อย | 0.17 rad | 0.12 rad |
| เป้าหมายเกิน limit | ≤ 0.5% ของ step, ≤ 0.3 rad (ข้อเท้า) | ≤ 1.1%, ≤ 0.29 rad |
| กระตุก step แรกตอนเข้า Velocity | 0.28 rad (shoulder roll) | **1.00 rad (left shoulder roll)** |
| ยืน cmd = 0: joint vel std / yaw rate std | 0.000 / 0.000 | **0.055 / 0.065** |
| เดิน vx 0.4: target step rms | 0.037 rad | 0.044 rad |

- armature ใน sim (0.01 vs ค่าตอนเทรน) ไม่มีผลกับการส่าย ปัญหาอยู่ที่ policy
- การกระตุก step แรกเกิดเพราะ history 5 เฟรมถูกเติมด้วยข้อมูลชุดเดียวกัน และ `last_action` เป็นศูนย์ → แก้ด้วย soft start ([05_deploy_safety.md](05_deploy_safety.md))
- การส่ายตอนยืน → แก้แล้วด้วย YawStand ([01_training.md](01_training.md#5-yawstand-fine-tune-ให้ยืนนิ่ง))

**แขวนกลางอากาศ (`vel_eval.py --cases 0,0,0 --hang`, เหมือนกด RT + A ตอนแขวนบน gantry):**

| | P3b | yaw_ft | Hist5 |
|---|---|---|---|
| joint vel std | 0.000 | **2.04 rad/s** | – |
| ความเร็ว joint สูงสุด / limit | 8% | **87%** (เข่าขวา) | – |
| แรงบิดสูงสุด / limit | 16% | **100%** (hip pitch ขวา) | – |
| ข้อต่อจริงเกิน limit | ไม่ | **ข้อเท้าซ้ายเกิน 0.115 rad** | – |

yaw_stand แขวน: joint vel std **3.25 rad/s**, ความเร็ว joint **131%** ของ limit, แรงบิด 100%, ข้อเท้าขวาเกิน limit 0.072 rad (แย่กว่า yaw_ft)
| เป้าหมายเบี่ยงจากท่ายืนสูงสุด (ไม่มี / มี soft start) | 0.28 / 0.10 rad | 1.62 / 1.66 rad | 2.38 / 2.19 rad |

policy สาย Hist5 เตะขาไปมาเมื่อเท้าไม่แตะพื้น (น่าจะเพราะ history บอกว่าไม่มีพื้นรับ ซึ่งไม่เคยเจอตอนเทรน) soft start ไม่ใช่สาเหตุและแก้ไม่ได้ **ห้ามเข้า Velocity ตอนแขวนด้วย policy สาย Hist5**

## 4. รัน unitree_mujoco + g1_ctrl กับ policy ใดก็ได้

`g1_ctrl` อ่าน `config.yaml` ข้างไฟล์ binary ตายตัว `sim2sim_policy.sh` จึงคัดลอก binary + `config/` ไปโฟลเดอร์แยก แล้วตั้ง `policy_dir` ในสำเนา config จริงไม่ถูกแตะ

```bash
deploy/robots/g1_23dof/scripts/sim2sim_policy.sh hist5_yawft     # หรือ v0
tmux attach -t sim2sim
```

1. MuJoCo: Pause → Load key 0 (`stand_ready`)
2. window `g1_ctrl`: `u` (FixStand) แล้ว `v` (Velocity)
3. MuJoCo: Run
4. `w` (0.4), `s` (−0.3), `a`/`d` (±0.2), `q`/`e` (±0.3 rad/s), `x` (หยุด) ดูผลใน window `speed` (Ctrl-b 2)
5. ปิด: `tmux kill-session -t sim2sim`

`sim_speed_monitor.py` อ่าน `rt/sportmodestate` (มีเฉพาะใน sim) + IMU แสดงค่าเฉลี่ย 2 s ของ vx, vy, wz และเอียงสูงสุด บันทึกที่ `/tmp/g1_sim2sim_<policy>/speed.csv`

## 5. ปัญหาที่เจอ

| อาการ | สาเหตุ | แก้ |
|---|---|---|
| `unitree_mujoco` / `g1_ctrl` crash ที่ `dds_write.c ... iox_pub` | shell ที่ source ROS โหลด `libddsc` ของ ROS Humble | `env -u CYCLONEDDS_URI LD_LIBRARY_PATH=/opt/unitree_robotics/lib ./unitree_mujoco` (สคริปต์ทำให้แล้ว) |
| `g1_ctrl` ค้างที่ `Waiting for connection`, ขึ้น `lo is not multicast-capable` | multicast บน `lo` ถูกปิด | `sudo ip link set lo multicast on` |
| หุ่นใน sim ล้มทั้งที่ policy ปกติ | มี `g1_ctrl` สองตัวส่ง `rt/lowcmd` แย่งกัน (ตัวหนึ่งค้างใน Passive) | ปิดตัวที่เกิน `g1_ctrl` เตือน `The other process is using the lowcmd channel` แต่ไม่หยุด (`exit(0)` ถูก comment) |
| หุ่นล้มก่อนเข้า Velocity, ขึ้น `bad_orientation` | ไม่ได้ Load key 0 ก่อน | ทำตามลำดับในข้อ 4 |
| แขน/เอวใน FixStand สั่นไปมา ~160 ครั้ง/s แรงบิดชน ±25 Nm | kd 10 ของ FixStand สูงเกินสำหรับ PD แบบ explicit ที่ dt 2 ms ในแขนที่เบา | ข้อจำกัดของ sim, driver มอเตอร์จริงคำนวณ PD ถี่กว่ามาก แต่ให้ดูแขนตอนทดสอบจริง |
| `scene_g1_23dof.xml` ตั้ง armature 0.01 ทุก joint | ไม่ตรงกับค่าตอนเทรน (0.0036–0.025) | ผลต่อการเดินน้อย, `pd_check --armature train` ใช้เทียบได้ |
