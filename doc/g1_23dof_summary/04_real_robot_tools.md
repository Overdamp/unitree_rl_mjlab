# 04: ตรวจหุ่นจริงและเครื่องมือ

[← สารบัญ](README.md)

- [1. การเชื่อมต่อ](#1-การเชื่อมต่อ)
- [2. ผลตรวจแบบอ่านอย่างเดียว 28/10/2026](#2-ผลตรวจแบบอ่านอย่างเดียว-28102026)
- [3. เครื่องมือ](#3-เครื่องมือ)
- [4. ข้อมูลที่บันทึกไว้](#4-ข้อมูลที่บันทึกไว้)

## 1. การเชื่อมต่อ

| รายการ | ค่า |
|---|---|
| PC | `192.168.123.123/24` บน `enp43s0` |
| บอร์ด low-level | `192.168.123.161` |
| Unitree DDS | domain 0, CycloneDDS |
| ROS 2 (g1pilot) | `ROS_DOMAIN_ID=1` |
| `mode_machine` | 4 = 23dof |
| `mode_pr` | 0 = PR (`g1_ctrl` ไม่ได้ตั้ง) |
| slot ที่ไม่ใช้ | 13, 14, 20, 21, 27, 28 |
| IMU | `imu_state` อยู่ในเชิงกราน (`imu_in_pelvis`) |

ขั้นตอนเต็ม: checklist หัวข้อ 1–2

## 2. ผลตรวจแบบอ่านอย่างเดียว 28/10/2026

หุ่นแขวนบน gantry อยู่ใน damping ของ Unitree ไม่ส่ง `rt/lowcmd`

| รายการ | ผล |
|---|---|
| LowState | 1044 Hz, ไม่มี gap > 100 ms (สูงสุด 60 ms) |
| IMU | ตั้งตรง, gyro bias ≤ 0.0014 rad/s, แกนตรงกับ RViz |
| encoder noise | ~0.0001 rad |
| อุณหภูมิ / แบต | 36–51 °C / 52 V |
| joint map | index ถูกครบ 23 ตัว, ทิศยืนยันจากข้อมูล (joint ที่ limit ไม่สมมาตร) และ RViz |

เกิน limit ของ MJCF เล็กน้อย: ankle roll (ซ้าย ~0.02, ขวาฝั่งลบ 0.057 rad), waist yaw ±2.704 (MJCF ±2.618), left wrist roll 0.026, right knee 0.015 rad
shoulder roll หุบเข้าได้แค่ ~62% เพราะแขนชนลำตัว

ความสัมพันธ์ของทิศ: pitch+ ↔ gyro y+ ↔ projected gravity x+; roll+ ↔ gyro x+ ↔ projected gravity y−; yaw+ ↔ gyro z+
projected gravity = (2(wy−xz), −2(wx+yz), −(w²−x²−y²+z²))

ตารางเต็ม: checklist ส่วน "ผลการตรวจ 28/10/2026"

## 3. เครื่องมือ

ทุกตัวอยู่ใน `deploy/robots/g1_23dof/scripts/` ใช้ env `g1_sdk` ทุกตัวที่ต่อหุ่น**อ่านอย่างเดียว** (ไม่ publish)

| เครื่องมือ | อ่าน | ทำอะไร |
|---|---|---|
| `lowstate_monitor.py` | `rt/lowstate` | Hz, gap, `mode_machine`, IMU, q / dq / `tau_est` / อุณหภูมิ, min/max, limit MJCF, `--log` CSV |
| `analyze_joint_sweep.py` | CSV จากข้างบน | หาว่าขยับ joint ไหนเมื่อไร (ไม่ต้องตามลำดับ), crosstalk, range เทียบ MJCF |
| `lowcmd_monitor.py` | `rt/lowcmd` | q_des / kp / kd / tau ทั้ง 29 slot, บอกว่า gain ตรงกับ Passive / FixStand / Velocity, ไฮไลต์ `MISMATCH`, `--log` CSV |
| `pd_check.py` | CSV state + cmd | torque check (`kp x`, `kd x`, delay), `--replay` เทียบกับ MuJoCo ([03_kp_kd.md](03_kp_kd.md#4-ตรวจ-4-ชั้น)) |
| `gain_check.py` | ไฟล์ policy | ตรวจ `deploy.yaml` กับ metadata ใน ONNX (ไม่ใช้หุ่น) |
| `vel_eval.py` | – | sim2sim headless ([02_sim2sim_eval.md](02_sim2sim_eval.md)) |
| `sim2sim_policy.sh`, `sim_speed_monitor.py` | – | `unitree_mujoco` + `g1_ctrl` กับ policy ที่เลือก |
| g1pilot `src/docker/validate_viewer.py`, `validate_byobu.sh` | ROS 2 topics | dashboard `/joint_states`, `/g1pilot/motor_state`, `/g1pilot/imu` ใน container `g1pilot-dev` |

```bash
conda activate g1_sdk
cd ~/nvidia/unitree/unitree_rl_mjlab
T=$(date +%Y%m%d_%H%M)
python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --iface enp43s0 --log ~/g1_state_$T.csv
python deploy/robots/g1_23dof/scripts/lowcmd_monitor.py   --iface enp43s0 --log ~/g1_cmd_$T.csv
python deploy/robots/g1_23dof/scripts/pd_check.py ~/g1_state_$T.csv ~/g1_cmd_$T.csv
```

g1pilot `robot_state` อ่าน `rt/lowstate` แล้วส่งออกเป็น ROS 2 ที่ ~1050 Hz ใช้ร่วมกับ monitor ได้ แต่ห้ามรัน `bringup_23dof` / `loco_client` พร้อม `g1_ctrl`

## 4. ข้อมูลที่บันทึกไว้

ไฟล์ใน home (ไม่อยู่ใน git):

| ไฟล์ | เนื้อหา |
|---|---|
| `~/g1_baseline_20261028_1826.csv` | baseline 60 s (76 MB) |
| `~/g1_joints_20261028_1848.csv` | ขยับทุก joint (2.3 GB) |
| `~/g1_joints_20261028_1927.csv` | เข่าขวา |
| `~/g1_joints_20261028_1930.csv` | IMU (หมุนตัวหุ่น) |

CSV จาก `lowstate_monitor.py` รุ่นใหม่มีคอลัมน์ `tau0..28` เพิ่ม `analyze_joint_sweep.py` อ่านได้ทั้งรุ่นเก่าและใหม่
