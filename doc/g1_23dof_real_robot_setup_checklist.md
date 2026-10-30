# G1-23DoF: เช็คลิสต์เชื่อมต่อและตรวจ joint กับหุ่นจริง

บันทึกเมื่อ 28/10/2026 · branch `phase3-delay-dr` · หัวข้อ 1–8 รอบ setup (ไม่ deploy), หัวข้อ 9 deploy รอบแรก (เพิ่ม 29/10/2026)

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
2. เข้า **debug mode** ด้วยรีโมท: L2+B (damping) แล้ว L2+R2 เพื่อปิด sport mode ไม่ให้แย่ง `rt/lowcmd` ทำ**ตอนหุ่นแขวนเท่านั้น** ห้ามเข้าตอนนั่งหรือนอน

> **L2+A ใน debug mode:** เอกสาร Unitree ให้กด L2+A เพื่อยืนยันว่าเข้า debug แล้ว ([G1 remote control](https://support.unitree.com/home/en/G1_developer/remote_control)) หุ่นจะขยับเหยียดตัวตรง (ไม่ใช่ Squat ↔ Stand แบบโหมดปกติ) และ Unitree รองรับการเข้า debug เฉพาะตอน zero torque หรือ damping
> เหตุการณ์ 30/10/2026: เปิดหุ่นตอนนั่ง → L2+R2 → L2+A → หุ่นเหยียดตัวจากท่านั่ง หลังจากนั้นมอเตอร์ right elbow (25) และ right wrist roll (26) หลุด (`vol` 0, `motorstate` 0x80000000) รีบูตแล้วไม่หาย (28/10 สองตัวนี้ยังปกติ) ยังไม่รู้ว่าการเหยียดตัวเป็นสาเหตุหรือไม่
> แนวทางของเรา: **ข้าม L2+A** ใช้ `motion_mode.py check` (`name=''`) ยืนยัน debug แทน เพราะไม่ทำให้หุ่นขยับ ถ้าจะกด L2+A ต้องแขวนอยู่และรอบตัวว่าง ห้ามกดตอนนั่ง/นอน · L2+B (damping) ใช้เป็นปุ่มหยุดฉุกเฉินได้แม้ใน debug
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

- [ ] (ไม่บังคับ) สำเนา `config.yaml` แล้วลด FixStand kp เหลือ ~30% และยืด `ts` เป็น `[0, 6]` ค่าเดิมเป็นของ Unitree ใช้ได้เลยเพราะหัวข้อ 4 ตรวจ index/ทิศแล้ว
- [ ] เปิด `lowstate_monitor.py --log` และ `lowcmd_monitor.py --log` พร้อมกันก่อนรัน `g1_ctrl` (หัวข้อ 7.4) เพื่อใช้ `pd_check.py` ทีหลัง
- [ ] รัน `g1_ctrl --network enp43s0` ต้องเห็น "Connected to robot" และไม่ขึ้น "Unmatched robot type"
- [ ] อยู่ใน Passive: หุ่นหนืดตาม kd ไม่มี joint ไหนกระตุก
- [ ] กด LT + up → FixStand: ทุก joint เคลื่อนเข้าท่ายืนนุ่มๆ ไม่มีตัวไหนไปผิดทิศ (ถ้ามี → LT + B ทันที)
- [ ] ค้าง FixStand 30 s: ความคลาดเคลื่อนจาก `qs` < 0.05 rad ทุก joint ไม่สั่น อุณหภูมิมอเตอร์ไม่พุ่ง
- [ ] ดึงสาย LAN ออกขณะ FixStand (ทดสอบ timeout 100 ms): หุ่นต้องไม่ค้างคำสั่งเก่า จดอาการที่เกิดขึ้น — ทำตอน kp ต่ำเท่านั้น
- [ ] กด LT + B → Passive หุ่นคลายแรงแบบหนืด ไม่ตกกระแทก
- [ ] `lowcmd_monitor.py` ขึ้น `gains match: Passive` / `FixStand` ตรงกับโหมด ไม่มี `MISMATCH`
- [ ] `pd_check.py` ของช่วง FixStand: `kp x` 0.8–1.2 และ corr ≥ 0.9 ทุก joint ที่ถูก score (หัวข้อ 7.5)

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
- `--log` เก็บทุกข้อความเป็น CSV (q, dq, tau_est, อุณหภูมิ 29 ช่อง, quaternion, gyro) ไว้เทียบกับ sim ภายหลัง

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

### 7.4 `lowcmd_monitor.py` (read-only)

subscribe `rt/lowcmd` ที่ `g1_ctrl` ส่งไปมอเตอร์ ไม่ publish (ไม่ชนกับการเช็ค "other process is using the lowcmd channel" ของ `g1_ctrl`) แสดง q_des / dq_des / kp / kd / tau_ff ทั้ง 29 slot บอกว่า gain ตรงกับโหมดไหน (Passive / FixStand / Velocity จาก `config.yaml` + `deploy.yaml`) และไฮไลต์ slot ที่ไม่ตรง รวมถึง slot 13, 14, 20, 21, 27, 28 ที่ต้องได้ kp = kd = 0 ใน Velocity

```bash
# terminal 1
python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --iface enp43s0 --log ~/g1_state_$(date +%Y%m%d_%H%M).csv
# terminal 2
python deploy/robots/g1_23dof/scripts/lowcmd_monitor.py --iface enp43s0 --log ~/g1_cmd_$(date +%Y%m%d_%H%M).csv
```

`--deploy` ระบุ `deploy.yaml` ที่ `g1_ctrl` โหลด (ค่าเริ่มต้น `config/policy/velocity/v0`)

### 7.5 `pd_check.py` (วิเคราะห์ CSV จาก 7.1 + 7.4)

```bash
python deploy/robots/g1_23dof/scripts/pd_check.py ~/g1_state_X.csv ~/g1_cmd_X.csv
python deploy/robots/g1_23dof/scripts/pd_check.py ~/g1_state_X.csv ~/g1_cmd_X.csv --replay --t0 5 --t1 25
```

1. **Torque check:** หา delay ของคำสั่งก่อน (0–30 ms) แล้ว fit `tau_est − tau_ff = a·kp(q_des−q) + b·kd(dq_des−dq)` ต่อ joint ต่อโหมด `kp x` (a) และ `kd x` (b) ใกล้ 1 แปลว่ามอเตอร์ใช้ gain ตามที่ส่ง ตัด sample ที่แรงบิดเกิน 95% ของ effort limit ออก (คอลัมน์ `sat%`) gain ที่ term ของมันเล็กเกินไป (kp = 0 ใน Passive, D term ตอนข้อต่อนิ่ง) จะไม่ถูก score
2. **Replay (`--replay`, บันทึกตอนแขวน):** ส่ง lowcmd ที่บันทึกไว้เข้า MuJoCo โดยยึดลำตัวกลางอากาศ (orientation จาก IMU) แล้วเทียบ q จาก sim กับ q จริง `nrmse` > 0.3 หรือ `lag ms` มาก แปลว่า actuator ใน sim (armature, friction, delay) ต่างจากจริง ลอง `--armature train` (armature ตอนเทรน แทน 0.01 ใน MJCF) และ `--delay-ms N`

ทดสอบแล้ว (29/10/2026): ข้อมูลจำลองที่ตั้ง kp ×0.7 + delay 10 ms ได้ `kp x` 0.70, `kd x` 1.00, delay 10 ms; `unitree_mujoco` + `g1_ctrl` ช่วง FixStand ขาได้ `kp x` 0.98–1.01 ส่วนแขนและเอวใน `unitree_mujoco` สั่นกลับไปมา ~160 ครั้ง/s แรงบิดชน ±25 Nm เพราะ kd 10 ของ FixStand สูงเกินไปสำหรับ PD แบบ explicit ที่ dt 2 ms (เป็นข้อจำกัดของ sim, driver มอเตอร์จริงคำนวณ PD ที่ความถี่สูงกว่ามาก)

> รัน `unitree_mujoco` / `g1_ctrl` จาก shell ที่ source ROS แล้วจะ crash ที่ `dds_write.c ... iox_pub` เพราะโหลด `libddsc` ของ ROS ให้รันด้วย `env -u CYCLONEDDS_URI LD_LIBRARY_PATH=/opt/unitree_robotics/lib ./unitree_mujoco`

## 8. ห้ามทำรอบ setup (หัวข้อ 1–7)

- ห้ามเข้า state Velocity / ห้ามรัน ONNX ทุกตัว
- ห้ามวางหุ่นลงพื้นตอนมีแรงบิด
- ถ้าเริ่มด้วย kp ต่ำ ห้ามเพิ่มเป็นค่าเต็มก่อนผ่านข้อค้าง FixStand 30 s ในหัวข้อ 6 (ถ้าใช้ค่าเดิมของ Unitree ตั้งแต่แรก ต้องแขวนหุ่นและพร้อมกด LT + B)
- ห้ามแก้ config หน้างานแล้วรันต่อทันที ยกเว้นการลด kp
- ห้ามรันงานหนัก (ติดตั้ง Isaac Sim, เทรน, docker อื่น) บน luke-mini ระหว่างต่อหุ่น

## 9. Deploy รอบแรก: P3b บน gantry แล้วค่อยลงพื้น

ทำเมื่อหัวข้อ 1–7 ผ่านแล้ว (ผ่านเมื่อ 28/10/2026) ใช้ policy P3b ที่อยู่ใน `config/policy/velocity/v0/exported/policy.onnx` (md5 `727da8a2…` เท่ากับ `policy_phase3b_16000.onnx`) เพราะเป็นตัวเดียวที่ผ่าน sim2sim กับ `g1_ctrl` + `unitree_mujoco` และแม่นที่สุดที่ vx ≤ 0.4 (97%) ห้ามเพิ่มโฟลเดอร์ใหม่ใน `config/policy/velocity/` ระหว่างรอบนี้ เพราะ `g1_ctrl` เลือกโฟลเดอร์ย่อยตัวสุดท้ายตามลำดับชื่อ

ทำทีละขั้น ขั้นถัดไปเริ่มได้เมื่อขั้นก่อนผ่านทุกข้อ ถ้าไม่ผ่านให้หยุด กลับ Passive และบันทึกอาการ

แขวนหุ่นไว้เพื่อตรวจ kp/kd ใน Passive และ FixStand เท่านั้น (9.1) **ไม่เข้า Velocity ตอนแขวน**: policy ยังทำงานแม้ไม่ดัน stick ขาจะขยับกลางอากาศโดยไม่ได้ข้อมูลเพิ่ม และ policy สาย Hist5 (Hist5, yaw_ft, yaw_stand) เตะขาแรงมากเมื่อเท้าไม่แตะพื้น (`vel_eval.py --hang`: yaw_stand ความเร็ว joint 131% ของ limit, แรงบิดชน 100%, ข้อเท้าเกิน limit) gain ของโหมด Velocity เป็นตัวเลขจากไฟล์ ตรวจใน sim (9.0) และดูอีกครั้งตอนเข้า Velocity บนพื้น (9.2)

### 9.0 ก่อนเริ่มทุกครั้ง

- [ ] หัวข้อ 1 (ความปลอดภัย, ฮาร์ดแวร์, PC) และหัวข้อ 2 (IP, ping, debug mode L2+R2) ผ่าน
- [ ] ปิด `bringup_23dof.launch.py` / `loco_client` ของ g1pilot, ปิด viewer และ RViz (เหลือแค่ monitor 2 ตัว)
- [ ] `md5sum deploy/robots/g1_23dof/config/policy/velocity/v0/exported/policy.onnx` = `727da8a2bc5293f617a8e06f8c13a07e`
- [ ] `python deploy/robots/g1_23dof/scripts/gain_check.py deploy/robots/g1_23dof/config/policy/velocity/v0` ขึ้น `PASS` (joint map, kp, kd, default pose, action scale, offset และขนาด obs ตรงกับตอนเทรน)
- [ ] คนถือรีโมทรู้ปุ่ม: **LT + ↑** = FixStand, **RT + A** = Velocity, **LT + B** = Passive (หยุดฉุกเฉิน ใช้ได้ทุกโหมด)
- [ ] รู้ว่า stick ส่งความเร็วตรงๆ ไม่มีการ smooth: stick ซ้ายแกนตั้ง = vx (เต็ม stick = 1.0 m/s ถูก clamp ที่ [-0.5, 1.0]), stick ซ้ายแกนนอน = vy (±0.5), stick ขวาแกนนอน = wz (±1.0) — **รอบนี้ดันไม่เกิน ~40% ของ stick** (P3b ที่สั่ง 1.0 เดินได้แค่ 0.41 m/s)
- [ ] เปิด 2 terminal บันทึกตลอดรอบ (env `g1_sdk`):
  ```bash
  cd ~/nvidia/unitree/unitree_rl_mjlab
  T=$(date +%Y%m%d_%H%M)
  python deploy/robots/g1_23dof/scripts/lowstate_monitor.py --iface enp43s0 --log ~/g1_state_$T.csv
  python deploy/robots/g1_23dof/scripts/lowcmd_monitor.py   --iface enp43s0 --log ~/g1_cmd_$T.csv
  ```
- [ ] terminal ที่ 3 (ไม่ source ROS หรือใช้ `env -u CYCLONEDDS_URI LD_LIBRARY_PATH=/opt/unitree_robotics/lib`):
  ```bash
  cd ~/nvidia/unitree/unitree_rl_mjlab/deploy/robots/g1_23dof/build
  ./g1_ctrl --network enp43s0
  ```
  ต้องเห็น `Connected to robot`, `LowState timeout set to 100 ms`, `ONNX policy loaded successfully: .../velocity/v0/exported/policy.onnx` และไม่ขึ้น `Unmatched robot type` หรือ `The other process is using the lowcmd channel`
- [ ] ทำก่อนวันทดสอบ (sim): `g1_ctrl` build เดียวกันกับ `unitree_mujoco` (`--network lo`) + `lowcmd_monitor --iface lo` กด `u` แล้ว `v`: ต้องขึ้น `gains match: FixStand` แล้ว `Velocity`, slot 13, 14, 20, 21, 27, 28 ได้ kp = kd = 0, ไม่มี `MISMATCH`

### 9.1 ขั้น 1: Passive แล้ว FixStand บน gantry (เท้าลอย)

**Passive (kp = 0, ตรวจ kd)**

- [ ] `lowcmd_monitor` = `gains match: Passive` (kp = 0 ทุก slot, kd 2.5–6)
- [ ] ใช้มือหมุนข้อต่อช้าแล้วเร็ว: หนืดมากขึ้นตามความเร็ว แต่ไม่ดึงกลับที่เดิม ไม่มี joint กระตุก
- [ ] `pd_check` ช่วง Passive: joint ที่หมุนเร็วพอได้ `kd x` 0.5–1.5 (joint ที่ไม่ได้หมุนขึ้น `not scored` เป็นเรื่องปกติ)

**FixStand (gain ของ Unitree, ตรวจ kp)**

- [ ] LT + ↑ → FixStand: ทุก joint เข้าท่ายืนใน ~3 s ไม่มีตัวไหนไปผิดทิศ, `lowcmd_monitor` = `FixStand` ไม่มี `MISMATCH`
- [ ] ค้าง 30 s: q ห่าง `qs` < 0.05 rad (ห่าง ≈ แรงโน้มถ่วง / kp ถ้าห่างกว่าที่คาดมาก kp จริงอ่อนกว่าที่ส่ง), ไม่สั่น (ดูแขนเป็นพิเศษ: kd 10 ทำให้แขนสั่นใน `unitree_mujoco`), อุณหภูมิไม่ขึ้นเกิน ~5 °C
- [ ] ดันข้อต่อแขน/ขาเบาๆ แล้วปล่อย: กลับเข้าที่นุ่มๆ (แกว่งหลายรอบ = kd น้อย, สั่นถี่ = kd มากหรือมี delay)
- [ ] LT + B → Passive: คลายแรงแบบหนืด
- [ ] หลังจบ: `python deploy/robots/g1_23dof/scripts/pd_check.py ~/g1_state_$T.csv ~/g1_cmd_$T.csv` ช่วง FixStand `kp x` 0.8–1.2, corr ≥ 0.9 ทุก joint ที่ถูก score; จด delay ที่ fit ได้

**อ่านผล `pd_check` ที่ไม่ผ่าน**

| อาการ | สาเหตุที่น่าจะเป็น | ทำอย่างไร |
|---|---|---|
| `MISMATCH` ใน `lowcmd_monitor` | config / `deploy.yaml` ผิด หรือ slot map ผิด | หยุด แก้ไฟล์ รัน `gain_check.py` ใหม่ |
| `kp x` นอก 0.8–1.2 ทุก joint ไปทางเดียวกัน | `tau_est` (ประมาณจากกระแส, คลาดได้ ~±10–20%) หรือแบตอ่อน | ดูแรงดันแบต เทียบกับการดันด้วยมือ |
| `kp x` ผิดเฉพาะบาง joint | มอเตอร์ตัวนั้นมีปัญหาหรือ friction สูง | ดูอุณหภูมิและ `motorstate` error ของตัวนั้น |
| `SIGN` | ทิศคำสั่งกลับกัน (ไม่ควรเกิดเพราะหัวข้อ 4 ตรวจแล้ว) | หยุดทันที |
| delay ที่ fit ได้ > 10 ms | network หรือ PC หน่วง | ดู gap ใน `lowstate_monitor` ปิดงานหนักบน PC |
| ข้อเท้า `kp x` แปลก | linkage ข้อเท้า firmware แปลงแรงบิดให้ | จดไว้ ถ้ายืนใน 9.2 ปกติถือว่าผ่าน |

ผล 30/10/2026: ผ่าน (FixStand `kp x` 0.89–1.09, แขน `kd x` 0.92–0.98, delay 3 ms; Passive `kd x` 0.94–0.98) right ankle roll ได้ 0.67 เพราะ `tau_est` มีออฟเซ็ตคงที่ ~−0.26 Nm ถ้า fit แบบมีออฟเซ็ตได้ 0.91 · วันนั้นกด L2 + R2 แล้วจอยสั่นอย่างเดียว (`name='ai'`) ใช้ `motion_mode.py release` แทนได้ผล · มอเตอร์ 25/26 หลุดหลังกด L2 + A ตอนนั่ง รีบูตไม่หาย ถอดแบต ~2 นาทีแล้วหาย

### 9.1b ขั้น 1b: FixStand ด้วย gain ที่เทรน (BeyondMimic) บน gantry

9.1 วัดได้แค่ช่วง kp 40–200 / kd 2.5–10 แต่ policy ใช้แขน kp 14.3 / kd 0.9 และขา kd 1.8–6.3 ขั้นนี้วัดค่าเหล่านั้นตรงๆ ก่อนลงพื้น ใช้สำเนา `g1_ctrl` ใน `/tmp` ที่ FixStand ใช้ stiffness/damping จาก `deploy.yaml` ของ policy (config จริงไม่ถูกแตะ) และ**ตัด RT + A ออก เข้า Velocity ไม่ได้**

```bash
cd ~/nvidia/unitree/unitree_rl_mjlab
deploy/robots/g1_23dof/scripts/fixstand_train_gains.sh            # ค่าเริ่มต้น v0 (P3b)
G1_CTRL_DIR=/tmp/g1_fixstand_train/build deploy/robots/g1_23dof/scripts/deploy_byobu.sh 1 bm
```

- [ ] หุ่นแขวน เท้าลอย, debug mode (`motion_mode.py check` → `name=''`), T1/T2 บันทึก, T3 ชื่อช่องลงท้าย `/tmp/g1_fixstand_train/build`
- [ ] Passive แล้ว LT + ↑: `lowcmd_monitor` ขึ้น **`gains match: Velocity`** (gain ชุดเดียวกับ Velocity เป็นเรื่องปกติ) kp 40.2/99.1/28.5/14.3, slot ที่ไม่ใช้ kp = kd = 0
- [ ] ค้าง 30 s: แขนห้อยห่างท่ามากกว่า 9.1 (kp 14.3) แต่ต้องนิ่ง ไม่สั่น ไม่แกว่งเอง
- [ ] ดันทีละกลุ่ม 0.1–0.2 rad ค้าง 1 s แล้วปล่อย + **ดึงออกแล้วปล่อยทันที** (ช่วงที่ joint เด้งกลับเองใช้เทียบ armature ใน `pd_check --replay` ได้) กลับช้ากว่า 9.1 เป็นเรื่องปกติ แกว่งหลายรอบไม่หยุด → LT + B
- [ ] LT + B → Ctrl-C T1/T2 → `pd_check.py ~/g1_state_<date>_bm1.csv ~/g1_cmd_<date>_bm1.csv` ช่วงที่ขึ้นชื่อ `Velocity`: `kp x` 0.8–1.2 และ `kd x` 0.5–1.5 (kd 0.9 ของแขนเป็นตัวที่ยังไม่เคยวัด)
- [ ] จบ: Ctrl-C T3, `tmux kill-session -t g1deploy` แล้วเปิด `deploy_byobu.sh` ปกติ (ไม่มี `G1_CTRL_DIR`) ก่อนไป 9.2 เพราะสำเนานี้เข้า Velocity ไม่ได้

log ใหญ่ (30/10: 4 GB ใน ~40 นาที) ถ้าจะ `--replay` ให้ตัดเฉพาะช่วงด้วย `awk` ตามคอลัมน์ `t` ก่อน

### 9.1c ขั้น 1c: สั่งท่าทีละ joint (`sysid_motion.py`) บน gantry

ดันด้วยมือสู้ kp 100–200 ไม่ไหว และ replay ใช้ไม่ได้เพราะ `q_des` คงที่ (30/10: nrmse ≈ 1 ทุก joint) สคริปต์นี้ส่ง `rt/lowcmd` เอง**แทน `g1_ctrl`** ไล่ท่ายืนใน 3 s แล้วขยับทีละ joint: step ±A (1.5 s) และ chirp 0.3→2.5 Hz (12 s) ด้วย gain ที่เทรน (`--gains train`, ค่าเริ่มต้น) หรือ Unitree (`--gains unitree`) ทั้งชุด ~8 นาที แผนถูกตรวจกับ limit MJCF (เผื่อ 0.1 rad) และความเร็วเป้าหมาย ≤ 6 rad/s ก่อนส่ง

หยุดเข้า damping (kp 0) เองเมื่อ: **L2 + B**, Ctrl-C, LowState ขาด > 50 ms, joint ห่างเป้า > 0.5 rad, มอเตอร์ > 70 °C, มอเตอร์หลุด (`vol` 0 / `motorstate` ≠ 0) และตอนจบแผน ไม่ยอมเริ่มถ้ายังมี `rt/lowcmd` (g1_ctrl) หรือ controller ในตัวยังไม่ปล่อย · ทดสอบใน `unitree_mujoco` (domain 1, elastic band) 30/10: ครบแผน, ไม่ยอมเริ่มซ้อน, Ctrl-C และ sim ตาย → damping ผ่าน · L2 + B ตรวจได้แค่ด้วย unit test (sim ไม่มีจอย) **ต้องลองกับหุ่นจริงก่อนรอบเต็ม**

```bash
# T3 ไม่ต้องเปิด g1_ctrl (Ctrl-C ถ้าเปิดอยู่) · T1/T2 บันทึกตามปกติ (deploy_byobu.sh 1 sysid)
python deploy/robots/g1_23dof/scripts/sysid_motion.py --dry-run                      # ดูแผน
python deploy/robots/g1_23dof/scripts/sysid_motion.py --groups armL --kinds step \
    --segments ~/sysid_seg_test.csv                                                   # รอบลอง ~30 s: กด L2 + B กลางทาง
python deploy/robots/g1_23dof/scripts/sysid_motion.py --segments ~/sysid_seg_$(date +%Y%m%d).csv   # รอบเต็ม
```

- [ ] หุ่นแขวน เท้าลอย รอบตัวว่าง, `motion_mode.py check` → `name=''`, `g1_ctrl` ไม่ได้รัน
- [ ] รอบลอง: ไล่ท่ายืนนุ่มๆ, แขนซ้ายขยับ, **กด L2 + B แล้วต้องขึ้น `-> damping (remote L2 + B)`** และแขนคลาย
- [ ] รอบเต็ม: ดูทุกท่า ถ้าแกว่งไม่หยุด / ชนตัว / เสียงผิดปกติ → L2 + B · เลือกกลุ่มได้ด้วย `--groups armL,armR,waist,legL,legR`, `--joints elbow,knee`
- [ ] Ctrl-C T1/T2 → `pd_check.py` (ช่วงจะขึ้นชื่อ `Velocity` ถ้าใช้ gain ที่เทรน) และ `--replay --t0 --t1` ตามเวลาใน `--segments` (คอลัมน์ `t0`/`t1` ใช้นาฬิกาเดียวกับ log ลบด้วย `t` แถวแรกของ lowstate) เทียบ `--armature mjcf` กับ `train`

### 9.2 ขั้น 2: ยืนบนพื้น (Velocity, stick = 0)

- [ ] เริ่มจาก FixStand บน gantry แล้วหย่อนลงจนเท้าแตะพื้นเต็มเท้า เชือกยังรับไว้แต่หย่อนพอให้หุ่นรับน้ำหนักเอง
- [ ] RT + A → Velocity (soft start: เป้าหมายค่อยๆ เปลี่ยนจากท่า FixStand ใน 0.5 s): ยืนนิ่ง ไม่โยกหรือย่ำเท้าแรง, IMU เอียง < 5° ใน `lowstate_monitor`
- [ ] `lowcmd_monitor` = `gains match: Velocity` (kp 40.2/99.1/28.5/14.3 ตาม slot), slot 13, 14, 20, 21, 27, 28 ได้ kp = kd = 0, ไม่มี `MISMATCH`
- [ ] ยืน 30 s แล้ว LT + B โดยให้เชือกรับน้ำหนักก่อนกด (Passive ไม่มี kp หุ่นจะทรุด)

### 9.3 ขั้น 3: เดินช้า (เชือกยังกันไว้)

- [ ] ดัน stick ซ้ายไปข้างหน้าเบาๆ ~20% (≈ 0.2 m/s) 3–5 s แล้วปล่อย ให้หุ่นหยุดเอง
- [ ] ถ้าผ่าน เพิ่มเป็น ~40% (≈ 0.4 m/s) ระยะสั้น
- [ ] คาดไว้ก่อนว่า P3b เลี้ยวขวาเองประมาณ 2°/s (วัดใน sim) แก้ด้วย stick ขวา
- [ ] ถอยหลังเบาๆ ~20%

### 9.4 ขั้น 4: หมุนและเดินด้านข้าง

- [ ] หมุนอยู่กับที่ด้วย stick ขวา ~30% ทั้งสองทิศ (P3b หมุนได้ ~70% ของคำสั่งใน sim)
- [ ] เดินด้านข้างเบาๆ ~30% (P3b ได้ ~60% ของคำสั่ง)

### 9.5 หยุดทันที (LT + B) เมื่อ

- joint ใดไปผิดทิศ, สั่นถี่ หรือมีเสียงผิดปกติจากเกียร์
- `lowcmd_monitor` ขึ้น `MISMATCH` หรือ `unknown`, `lowstate_monitor` ขึ้น `STALE`, gap > 100 ms หรือ joint ขึ้น `OUT`
- อุณหภูมิมอเตอร์ตัวใด > 70 °C หรือขึ้นเร็วผิดปกติ
- ลำตัวเอียงเกิน ~15° (ระบบตัดเองที่ 0.85 rad ≈ 49° ซึ่งช้าเกินไปสำหรับการป้องกันการล้ม)
- คนใดคนหนึ่งพูด "หยุด"

### 9.6 ความปลอดภัยของ policy และการหยุด (ตรวจใน sim และโค้ด 29/10/2026)

วัดด้วย `vel_eval.py` (safety stats) บนพื้นเรียบ 7 กรณี (ยืน, vx 0.4/1.0/−0.5, wz 1.0, vy 0.5, ผสม) ทั้งแบบคำสั่ง smooth และแบบกระโดดทันทีเหมือน stick จริง (`--no-smooth`)

| รายการ | P3b (v0) | yaw_ft | ความหมาย |
|---|---|---|---|
| ล้ม / เอียงสูงสุด | ไม่ล้ม / ≤ 4.4° | ไม่ล้ม / ≤ 5.7° | stick กระโดดทันทีก็ไม่ล้มทั้งคู่ |
| ความเร็ว joint สูงสุด / limit มอเตอร์ | ≤ 29% (เข่า) | ≤ 45% (เข่า) | ไม่เหวี่ยงใกล้ limit ความเร็ว |
| แรงบิดสูงสุด / effort limit | ≤ 66% (ข้อเท้า) | ≤ 66% | ไม่ชนเพดานแรงบิด |
| q จริงห่าง limit ข้อต่ออย่างน้อย | 0.17 rad (ankle roll) | 0.12 rad (ankle roll) | ข้อต่อจริงไม่ชน hard stop |
| เป้าหมายเกิน limit ข้อต่อ | ≤ 0.5% ของ step, สูงสุด 0.3 rad (ankle pitch) | ≤ 1.1%, สูงสุด 0.29 rad | `clip: null` ไม่มีการจำกัด แต่ข้อต่อจริงไม่ไปถึง |
| กระตุกตอนเข้า Velocity (step แรก, max \|target − default\|) | 0.28 rad (shoulder roll) | **1.00 rad (left shoulder roll)**, −0.54 (right shoulder roll), −0.41 (right knee) | เกิด step เดียว 20 ms แล้วกลับปกติ ข้อต่อจริงขยับ < 0.1 rad แต่ควรดูแขนตอนกด RT + A |

**กลไกหยุดใน `g1_ctrl` (จากโค้ด)**

| กลไก | ทำงานเมื่อ | เร็วแค่ไหน | ข้อจำกัด |
|---|---|---|---|
| LT + B → Passive | ทุกโหมด | FSM ตรวจทุก 1 ms + รอ policy thread หยุด ≤ 20 ms | Passive คือ kp = 0: บน gantry หุ่นห้อยหนืด, บนพื้นหุ่นทรุดลงแบบหน่วง |
| LowState timeout → Passive | ไม่ได้ `rt/lowstate` 100 ms | 100 ms | ถ้าสาย/เครือข่ายขาด คำสั่ง Passive ก็ไปไม่ถึงมอเตอร์ อาการขึ้นกับ firmware (ทดสอบด้วยการดึงสาย LAN ใน 9.1 ตอนแขวน) |
| bad_orientation → Passive | เฉพาะ Velocity, เอียง > 0.85 rad (49°) | ทันที | ช้าเกินกันล้ม ใช้คนกด LT + B ที่ ~15° |
| ไม่มี | จำกัดตำแหน่งเป้าหมาย (`clip: null`), จำกัดความเร็ว joint, ramp ตอนเข้า Velocity | – | พึ่ง limit ในมอเตอร์ firmware |

- [ ] ทดสอบตอนแขวนใน FixStand: ปิด `g1_ctrl` (Ctrl-C) แล้วจดว่ามอเตอร์ค้างท่า, หนืด หรือปล่อยตัว (ยังไม่เคยวัด)

### 9.7 บันทึก

- [ ] CSV จาก monitor ทั้งสองตัว ทุกขั้น (จดเวลาเริ่ม/จบของแต่ละขั้นไว้ใช้กับ `--t0/--t1`)
- [ ] วิดีโอทุกขั้น
- [ ] ผล `pd_check.py` ของขั้น 1 (และช่วง Velocity บนพื้นถ้าต้องการดู `kp x` / `kd x` ของ gain ชุด policy)
- [ ] แบต/อุณหภูมิตอนเริ่มและตอนจบ, อาการผิดปกติทุกอย่าง

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
