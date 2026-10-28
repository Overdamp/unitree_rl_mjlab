# G1-23DoF: เช็คลิสต์เชื่อมต่อและตรวจ joint กับหุ่นจริง

บันทึกเมื่อ 28/10/2026 · branch `phase3-delay-dr` · ยังไม่ deploy policy

## ขอบเขต

รอบนี้ทำแค่ **เชื่อมต่อ อ่านค่า และ map joint ให้ตรงกับ config** ห้ามรัน policy (state Velocity) เด็ดขาด ผ่านเมื่อทุก joint ใน 23 ตัว ตรงทั้ง index, ชื่อ, ทิศ (sign), ท่ายืน และ limit กับ `deploy.yaml` และ MJCF โดยมีบันทึกหลักฐานครบ

| ด่าน | ผ่านเมื่อ |
|---|---|
| A. เชื่อมต่อ | ได้ `rt/lowstate` ต่อเนื่อง ~1000 Hz (วัดจริง 1044 Hz เมื่อ 28/10/2026), `mode_machine` = 4 (23dof) |
| B. อ่านค่า (ไม่ส่งคำสั่ง) | IMU, อุณหภูมิมอเตอร์ปกติ; encoder ทุก joint ขยับตามมือ |
| C. Map joint | ทั้ง 23 joint: index, ชื่อ, sign, limit ตรงกับ config |
| D. ทดสอบมีแรงต่ำ (ถ้าเวลาพอ) | FixStand แบบ kp ต่ำบน gantry ไปถึงท่ายืนได้นิ่ง ไม่สั่น |

## 1. ก่อนเปิดหุ่น

**ความปลอดภัย**

- [ ] แขวนหุ่นบน gantry เท้าลอยพ้นพื้น ~5 cm ตลอดการทดสอบ
- [ ] รีโมท Unitree อยู่ในมือคนที่สองตลอด รู้ปุ่มเข้า damping / ปิดมอเตอร์
- [ ] รัศมีแขน/ขา ~1 m ว่าง ไม่มีคนยืนหน้าหุ่นตอนมีแรงบิด
- [ ] ตกลงคำพูด "หยุด" กันก่อนเริ่ม

**ฮาร์ดแวร์**

- [ ] แบตเตอรี่ > 50% (จดค่าเริ่มต้นไว้)
- [ ] สาย LAN จากหุ่นเข้า `enp43s0` ของ luke-mini
- [ ] เช็คสภาพภายนอก: สายไม่พัน, ขายึดไหล่ไม่แตก, น็อตไม่หลวม

**ฝั่ง PC**

- [ ] ไม่มีงานหนักรันอยู่: ติดตั้ง Isaac Sim, เทรน, CVAT (`uptime` load < 2)
- [ ] ปิด `unitree_mujoco` และ `g1_ctrl` ที่รันกับ `lo` ทั้งหมด
- [ ] build `g1_ctrl` จาก branch `phase3-delay-dr` ล่าสุด (มี LowState timeout 100 ms)

## 2. เชื่อมต่อเครือข่ายและ DDS

หุ่น G1 อยู่ในวง `192.168.123.0/24` ตั้ง IP เครื่องเราเป็น `192.168.123.123` (ค่าที่ g1pilot และ config Livox ใช้) บอร์ด low-level อยู่ที่ `192.168.123.161`

1. เปิดหุ่น รอจนเข้าโหมด zero-torque/damping (หุ่นห้อยบน gantry)
2. เข้า **debug mode** ด้วยรีโมท (L2+R2 ตามคู่มือ Unitree) เพื่อปิด sport mode ไม่ให้แย่ง `rt/lowcmd` — เช็คคู่มือก่อนกด
3. ตั้ง IP บน PC:
   ```bash
   sudo ip addr add 192.168.123.123/24 dev enp43s0
   sudo ip link set enp43s0 up
   ip -br addr show enp43s0
   ```
4. `ping 192.168.123.161` ต้องตอบทุกครั้ง ไม่มี packet loss
5. `ip link show enp43s0` ต้องมี `MULTICAST`
6. ใน terminal ที่ใช้กับหุ่น: `unset ROS_DOMAIN_ID` หรือตั้งเป็น 1 (กันชนกับ Unitree SDK domain 0)
7. รัน monitor แบบ read-only (หัวข้อ 7) แล้วเช็ค:
   - [ ] LowState เข้า ~1000 Hz ต่อเนื่อง 1 นาที ไม่มี gap > 100 ms
   - [ ] `mode_machine` = 4 (ค่าที่ `main.cpp` เช็คสำหรับ 23dof) — ถ้าไม่ใช่ หยุดและจดค่าไว้
   - [ ] `mode_pr` (โหมด ankle PR/AB) จดค่าไว้
   - [ ] ไม่มี process อื่นส่ง `rt/lowcmd` (debug mode ติดจริง)

## 3. อ่านค่าอย่างเดียว (ไม่ส่งคำสั่งมอเตอร์)

ทุกข้อในหัวข้อนี้ใช้ `lowstate_monitor.py` หรือ `robot_state` ของ g1pilot ที่ subscribe `rt/lowstate` อย่างเดียว ไม่ publish `rt/lowcmd` หุ่นอยู่ใน damping ของ Unitree

- [ ] **IMU ท่าตั้งตรง:** projected gravity ≈ (0, 0, −1), roll/pitch < 3°
- [ ] **IMU เอียง:** เอียงหุ่นไปข้างหน้า → pitch เปลี่ยนตามทิศที่ MuJoCo/RViz ให้
- [ ] **gyro นิ่ง:** ตอนหุ่นนิ่ง |ω| < 0.02 rad/s ทุกแกน (จด bias)
- [ ] **หมุน yaw:** หมุนหุ่นทวนเข็ม (มองจากบน) → ωz เป็นบวก
- [ ] **แบต:** แรงดัน/เปอร์เซ็นต์จดไว้ (ดูจากแอป Unitree; LowState ของ G1 ไม่มีฟิลด์แบต)
- [ ] **อุณหภูมิมอเตอร์:** ทุกตัวใกล้อุณหภูมิห้อง ไม่มีตัวไหนสูงผิดปกติ
- [ ] **error code:** ไม่มีมอเตอร์ตัวไหนรายงาน error (คอลัมน์ flag `err=` ใน monitor)
- [ ] **encoder นิ่ง:** ตอนไม่แตะหุ่น q แกว่ง < 0.005 rad, dq ≈ 0
- [ ] **ช่องที่ไม่ใช้:** index 13, 14 (waist roll/pitch ที่ถูกล็อก), 20, 21, 27, 28 (wrist pitch/yaw) จดค่าไว้ ต้องไม่ขยับตามมือ

## 4. Map joint ทีละตัว (หุ่นอยู่ใน damping)

หลักฐานหลักคือ **mirror**: เอา q จริงทั้ง 23 ตัวไปแสดงในโมเดลหุ่น แล้วดูว่าท่าในจอตรงกับหุ่นจริง วิธีนี้จับได้ทั้ง index สลับ, sign กลับ และ offset ผิด ในครั้งเดียว ใช้ RViz จาก docker g1pilot (หัวข้อ 7)

ขั้นตอนต่อ joint:

1. ขยับ joint นั้นด้วยมือช้าๆ ทีละตัว
2. เช็คว่า index ที่ขึ้น `<< moving` ใน monitor ตรงตามตาราง และ index อื่นนิ่ง
3. ดู mirror ว่า joint เดียวกันขยับไปทิศเดียวกัน
4. ดันไปใกล้สุดทางเบาๆ (ไม่กระแทก) จด min/max เทียบ limit ใน MJCF (monitor แสดง `min seen` / `max seen` ให้)

| idx | joint | limit MJCF (rad) | ท่ายืน (rad) | +q ควรเป็น | index/sign ตรง | min/max ที่วัด |
|---|---|---|---|---|---|---|
| 0 | left_hip_pitch | −2.53 / 2.88 | −0.1 | ขาไปข้างหลัง | | |
| 1 | left_hip_roll | −0.52 / 2.97 | 0 | ขากางออกซ้าย | | |
| 2 | left_hip_yaw | −2.76 / 2.76 | 0 | ปลายเท้าหันซ้าย (CCW มองบน) | | |
| 3 | left_knee | −0.09 / 2.88 | 0.3 | งอเข่า | | |
| 4 | left_ankle_pitch | −0.87 / 0.52 | −0.2 | ปลายเท้ากดลง | | |
| 5 | left_ankle_roll | −0.26 / 0.26 | 0 | เทียบ mirror | | |
| 6 | right_hip_pitch | −2.53 / 2.88 | −0.1 | ขาไปข้างหลัง | | |
| 7 | right_hip_roll | −2.97 / 0.52 | 0 | ขาหุบเข้าด้านใน (ไปทางซ้าย) | | |
| 8 | right_hip_yaw | −2.76 / 2.76 | 0 | ปลายเท้าหันซ้าย (CCW มองบน) | | |
| 9 | right_knee | −0.09 / 2.88 | 0.3 | งอเข่า | | |
| 10 | right_ankle_pitch | −0.87 / 0.52 | −0.2 | ปลายเท้ากดลง | | |
| 11 | right_ankle_roll | −0.26 / 0.26 | 0 | เทียบ mirror | | |
| 12 | waist_yaw | −2.62 / 2.62 | 0 | ลำตัวหันซ้าย (CCW มองบน) | | |
| 15 | left_shoulder_pitch | −3.09 / 2.67 | 0.35 | แขนไปข้างหลัง | | |
| 16 | left_shoulder_roll | −1.59 / 2.25 | 0.18 | แขนกางออกซ้าย | | |
| 17 | left_shoulder_yaw | −2.62 / 2.62 | 0 | เทียบ mirror | | |
| 18 | left_elbow | −1.05 / 2.09 | 0.87 | เทียบ mirror | | |
| 19 | left_wrist_roll | −1.97 / 1.97 | 0 | เทียบ mirror | | |
| 22 | right_shoulder_pitch | −3.09 / 2.67 | 0.35 | แขนไปข้างหลัง | | |
| 23 | right_shoulder_roll | −2.25 / 1.59 | −0.18 | แขนหุบเข้าลำตัว (ไปทางซ้าย) | | |
| 24 | right_shoulder_yaw | −2.62 / 2.62 | 0 | เทียบ mirror | | |
| 25 | right_elbow | −1.05 / 2.09 | 0.87 | เทียบ mirror | | |
| 26 | right_wrist_roll | −1.97 / 1.97 | 0 | เทียบ mirror | | |

คอลัมน์ "+q ควรเป็น" คำนวณจากแกนใน MJCF (pitch = แกน y, roll = x, yaw = z) เป็นแค่ตัวช่วย ถ้าขัดกับ mirror ให้เชื่อ mirror และจดไว้ ท่ายืนมาจาก `default_joint_pos` ใน `deploy.yaml`

## 5. เทียบ config กับหุ่นจริง

ไฟล์อยู่ใน `deploy/robots/g1_23dof/config/` เทียบทีละค่ากับผลจากหัวข้อ 3–4

| ค่า | อยู่ที่ | ค่าใน config | เช็คกับหุ่นอย่างไร |
|---|---|---|---|
| `joint_ids_map` | `policy/velocity/v0/params/deploy.yaml` | 0–12, 15–19, 22–26 | ตรงกับ index ที่ขยับจริงในตารางหัวข้อ 4 |
| `default_joint_pos` | deploy.yaml | ตามคอลัมน์ท่ายืน | ทุกค่าอยู่ใน limit ที่วัดได้ มี margin ≥ 0.2 rad |
| stiffness / damping (policy) | deploy.yaml | 40.2 / 99.1 / 28.5 / 14.3 · 2.6 / 6.3 / 1.8 / 0.9 | ยังเช็คไม่ได้รอบนี้ ต้องทำ system ID บน gantry ภายหลัง |
| FixStand kp / kd | `config.yaml` | 100/150/200/40 · 2/4/5/10 | คนละชุดกับ policy ใช้แค่ตอนยืน ถ้าทำขั้น D ให้ลดเหลือ ~30% ก่อน |
| FixStand `qs` | config.yaml | เท่า `default_joint_pos` (29 ช่อง SDK) | ช่อง 13, 14, 20, 21, 27, 28 เป็น 0 |
| Passive kd | config.yaml | 4 / 6 / 2.5 / 4.5 | ตอน Passive หุ่นต้องหนืดพอดี ไม่สั่น |
| `mode_machine` | `main.cpp` | 4 | ตรงกับที่อ่านได้ในหัวข้อ 2 |
| effort limit | MJCF / constants | 88 / 139 / 50 / 25 Nm | เทียบสเปคมอเตอร์จาก Unitree (ไม่ต้องวัด) |
| ทิศ IMU | `deploy/include/unitree_articulation.h` (`projected_gravity`, `base_ang_vel`) | gyroscope + quaternion จาก `imu_state` ตรงๆ | sign ตรงกับหัวข้อ 3 |
| `step_dt` | deploy.yaml | 0.02 s (50 Hz) | ไม่ต้องเช็คกับหุ่น |

ถ้าค่าไหนไม่ตรง **ห้ามแก้หน้างาน** ให้จดไว้ แล้วแก้ใน sim (MJCF/constants) ก่อน เพราะค่าใน deploy ต้องตรงกับตอนเทรน

## 6. ทดสอบแบบมีแรงต่ำบน gantry (ทำเมื่อหัวข้อ 2–5 ผ่านหมดเท่านั้น)

ขั้นนี้ใช้ `g1_ctrl` แต่ไปแค่ Passive → FixStand ห้ามกดเข้า Velocity (RT + A) คนถือรีโมทพร้อมกด LT + B (กลับ Passive) ตลอด ปิด `bringup_23dof.launch.py` ของ g1pilot ก่อน (`loco_client` กับ `g1_ctrl` ห้ามรันพร้อมกัน)

- [ ] สำเนา `config.yaml` แล้วลด FixStand kp เหลือ ~30% และยืด `ts` เป็น `[0, 6]` (ไปท่ายืนช้าลง)
- [ ] รัน `g1_ctrl --network enp43s0` ต้องเห็น "Connected to robot" และไม่ขึ้น "Unmatched robot type"
- [ ] อยู่ใน Passive: หุ่นหนืดตาม kd ไม่มี joint ไหนกระตุก
- [ ] กด LT + up → FixStand: ทุก joint เคลื่อนเข้าท่ายืนนุ่มๆ ไม่มีตัวไหนไปผิดทิศ (ถ้ามี → LT + B ทันที)
- [ ] ค้าง FixStand 30 s: ความคลาดเคลื่อนจาก `qs` < 0.05 rad ทุก joint ไม่สั่น อุณหภูมิมอเตอร์ไม่พุ่ง
- [ ] ดึงสาย LAN ออกขณะ FixStand (ทดสอบ timeout 100 ms): หุ่นต้องไม่ค้างคำสั่งเก่า จดอาการที่เกิดขึ้น — ทำตอน kp ต่ำเท่านั้น
- [ ] กด LT + B → Passive หุ่นคลายแรงแบบหนืด ไม่ตกกระแทก

## 7. เครื่องมือ

### 7.1 `lowstate_monitor.py` (read-only)

อยู่ที่ `deploy/robots/g1_23dof/scripts/lowstate_monitor.py` subscribe `rt/lowstate` อย่างเดียว ไม่ publish `rt/lowcmd` แสดง Hz, gap, `mode_machine`, `mode_pr`, IMU (rpy, gyro, projected gravity), q / dq / อุณหภูมิ / min-max ที่เห็น / limit MJCF ต่อ joint, ไฮไลต์ joint ที่กำลังขยับ (`<< moving`) และ joint ที่หลุดหรือใกล้ limit (`OUT` / `near`) กด Ctrl-C แล้วพิมพ์สรุป (Hz เฉลี่ย, gap > 100 ms, joint ที่ยังไม่ได้ขยับ, joint ที่เกิน limit)

```bash
conda activate g1_sdk
cd ~/nvidia/unitree/unitree_rl_mjlab
python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --iface enp43s0 --log ~/g1_lowstate_$(date +%Y%m%d_%H%M).csv
```

- `--iface lo` ใช้กับ `unitree_mujoco` (ต้องเปิด multicast บน `lo` ก่อน)
- `--duration 60` หยุดเองหลัง 60 s (ใช้กับข้อ "LowState ต่อเนื่อง 1 นาที")
- `--log` เก็บทุกข้อความเป็น CSV (q, dq, อุณหภูมิ 29 ช่อง, quaternion, gyro) ไว้เทียบกับ sim ภายหลัง

### 7.2 Mirror ด้วย RViz (docker g1pilot)

`robot_state` ของ g1pilot subscribe `rt/lowstate` อย่างเดียว แล้วแสดง URDF `g1_23dof_rev_1_0` ตาม joint จริงใน RViz

```bash
# host
cd ~/Ros2_Directory/g1pilot_ws/src/docker && ./run_real.sh
# ใน container
./cbuild g1pilot
source /ros2_ws/setup_uri.sh $G1_INTERFACE
ros2 launch g1pilot robot_state_23dof.launch.py use_rviz:=true
```

- รันพร้อม `lowstate_monitor.py` ได้ (ทั้งคู่อ่านอย่างเดียว)
- ห้ามรัน `bringup_23dof.launch.py` รอบนี้ (`loco_client` เรียก `Damp()` ทันทีที่เริ่ม)

### 7.3 บันทึกที่ต้องเก็บ

- [ ] ตารางหัวข้อ 4 กรอกครบ 23 แถว
- [ ] CSV จาก monitor (หุ่นนิ่ง 1 นาที + ช่วงขยับทุก joint)
- [ ] รูป/วิดีโอหุ่นจริงคู่กับจอ RViz ตอนอยู่ท่ายืน
- [ ] ค่าที่ไม่ตรงทุกค่า + แบต/อุณหภูมิตอนเริ่มและตอนจบ

## 8. ห้ามทำรอบนี้

- ห้ามเข้า state Velocity / ห้ามรัน ONNX ทุกตัว
- ห้ามวางหุ่นลงพื้นตอนมีแรงบิด
- ห้ามใช้ kp เต็มจาก `config.yaml` ก่อนผ่านข้อค้าง FixStand 30 s ในหัวข้อ 6
- ห้ามแก้ config หน้างานแล้วรันต่อทันที ยกเว้นการลด kp
- ห้ามรันงานหนัก (ติดตั้ง Isaac Sim, เทรน, docker อื่น) บน luke-mini ระหว่างต่อหุ่น

## ผลการตรวจ 28/10/2026 (หุ่นแขวนบน gantry, damping ของ Unitree, ไม่ส่ง `rt/lowcmd`)

ผ่านครบส่วน read-only: เชื่อมต่อ, baseline, map joint 23/23 และ IMU ยังไม่ได้ทำหัวข้อ 6 (FixStand kp ต่ำ)

### Baseline 60 s (`g1_baseline_20261028_1826.csv`)

| เช็ค | ผล |
|---|---|
| อัตรา LowState | 1044 Hz (dt median 0.9 ms) |
| gap > 100 ms | 0 (สูงสุด 60 ms, > 20 ms 3 ครั้ง — ปิดโปรแกรมที่ไม่จำเป็นก่อน FixStand) |
| `mode_machine` / `mode_pr` | 4 / 0 (PR) ตลอด |
| IMU ท่าห้อย | roll 1.0°, pitch −2.2°, projected gravity (−0.04, −0.02, −1.00) |
| gyro bias / noise | ≤ 0.0014 / ≤ 0.0024 rad/s |
| encoder noise | peak-to-peak 0.0001 rad ทุก joint |
| ช่องที่ไม่ใช้ 13, 14, 20, 21, 27, 28 | q = 0, อุณหภูมิ = 0 (ไม่มีมอเตอร์) |
| อุณหภูมิ / แรงดัน | 36–51 °C (ไหล่ขวาร้อนสุด), 52 V, ไม่เพิ่มใน 60 s |

### Map joint (`g1_joints_20261028_1848.csv` + เข่าขวา `g1_joints_20261028_1927.csv`)

วิเคราะห์ด้วย `deploy/robots/g1_23dof/scripts/analyze_joint_sweep.py` index ถูกทุก joint; sign ยืนยันจากข้อมูลสำหรับ joint ที่ limit ไม่สมมาตร และยืนยันด้วย RViz สำหรับ joint ที่ limit สมมาตร (hip yaw, waist yaw, shoulder yaw, wrist roll, ankle roll)

| idx | joint | q min / max ที่วัด (rad) | MJCF (rad) | หมายเหตุ |
|---|---|---|---|---|
| 0 | left_hip_pitch | −2.534 / 2.808 | −2.531 / 2.880 | |
| 1 | left_hip_roll | −0.524 / 2.962 | −0.524 / 2.967 | |
| 2 | left_hip_yaw | −2.761 / 2.747 | −2.758 / 2.758 | |
| 3 | left_knee | −0.086 / 2.877 | −0.087 / 2.880 | |
| 4 | left_ankle_pitch | −0.868 / 0.514 | −0.873 / 0.524 | |
| 5 | left_ankle_roll | −0.284 / 0.271 | −0.262 / 0.262 | เกิน ~0.02 |
| 6 | right_hip_pitch | −2.536 / 2.867 | −2.531 / 2.880 | |
| 7 | right_hip_roll | −2.963 / 0.531 | −2.967 / 0.524 | |
| 8 | right_hip_yaw | −2.750 / 2.760 | −2.758 / 2.758 | |
| 9 | right_knee | −0.087 / 2.895 | −0.087 / 2.880 | |
| 10 | right_ankle_pitch | −0.868 / 0.525 | −0.873 / 0.524 | |
| 11 | right_ankle_roll | −0.319 / 0.242 | −0.262 / 0.262 | ด้านลบเกิน 0.057 |
| 12 | waist_yaw | −2.704 / 2.704 | −2.618 / 2.618 | เกินสองข้าง 0.086 (MJCF ระวังกว่าของจริง) |
| 15 | left_shoulder_pitch | −3.065 / 2.666 | −3.089 / 2.670 | |
| 16 | left_shoulder_roll | −0.146 / 2.245 | −1.588 / 2.252 | หุบเข้าได้ไม่สุดเพราะชนลำตัว |
| 17 | left_shoulder_yaw | −2.613 / 2.608 | −2.618 / 2.618 | |
| 18 | left_elbow | −1.038 / 2.089 | −1.047 / 2.094 | |
| 19 | left_wrist_roll | −1.998 / 1.959 | −1.972 / 1.972 | ด้านลบเกิน 0.026 |
| 22 | right_shoulder_pitch | −3.076 / 2.669 | −3.089 / 2.670 | |
| 23 | right_shoulder_roll | −2.255 / 0.149 | −2.252 / 1.588 | หุบเข้าได้ไม่สุดเพราะชนลำตัว |
| 24 | right_shoulder_yaw | −2.625 / 2.603 | −2.618 / 2.618 | |
| 25 | right_elbow | −1.033 / 2.073 | −1.047 / 2.094 | |
| 26 | right_wrist_roll | −1.967 / 1.979 | −1.972 / 1.972 | |

ไม่มีค่าไหนต้องแก้ config: ส่วนที่เกิน MJCF เล็กน้อยอยู่ที่ ankle roll (กลไกขนาน, roll คำนวณในโหมด PR), waist yaw และ wrist roll

### IMU (`g1_joints_20261028_1930.csv`)

เอียงหน้า/หลัง, ซ้าย/ขวา และบิด yaw 14 ครั้ง gyro ที่สะสม, มุมจาก quaternion และ projected gravity สอดคล้องกันทุกครั้ง (pitch+ ↔ gyro y+ ↔ proj g x+, roll+ ↔ gyro x+ ↔ proj g y−, yaw+ ↔ gyro z+) และโมเดลใน RViz เอียง/บิดตามหุ่นจริงทุกครั้ง
