# 03: kp / kd

[← สารบัญ](README.md)

- [1. kp/kd คืออะไร และอยู่ที่ไหน](#1-kpkd-คืออะไร-และอยู่ที่ไหน)
- [2. ค่าที่เราใช้มาจากไหน](#2-ค่าที่เราใช้มาจากไหน)
- [3. เทียบกับที่อื่น](#3-เทียบกับที่อื่น)
  - [3.1 ทำไมใช้แบบ BeyondMimic ไม่ใช่ 100/150/40](#31-ทำไมใช้แบบ-beyondmimic-ไม่ใช่-10015040)
- [4. ตรวจ 4 ชั้น](#4-ตรวจ-4-ชั้น)
- [5. ผลทดสอบเครื่องมือ](#5-ผลทดสอบเครื่องมือ)
- [6. จุดที่ต้องตรวจต่อ](#6-จุดที่ต้องตรวจต่อ)

## 1. kp/kd คืออะไร และอยู่ที่ไหน

kp/kd คือ stiffness/damping ของ PD controller ที่ตั้งใน sim ตอนเทรน บนหุ่นจริง**ไม่ต้องอัปโหลดเข้า firmware** เราส่งเองทุกข้อความ `rt/lowcmd` (~500 Hz) แต่ละมอเตอร์มี `q`, `dq`, `kp`, `kd`, `tau` driver คำนวณเอง:

```
τ = kp·(q_des − q) + kd·(dq_des − dq) + τ_ff
```

| อยู่ใน firmware (ไม่แตะ) | เราส่งเองทุกรอบ |
|---|---|
| สูตร PD และ loop ความถี่สูง | kp, kd |
| แปลงข้อเท้า PR ↔ มอเตอร์ 2 ตัว | q, dq เป้าหมาย |
| จำกัดกระแส, ป้องกันความร้อน | tau |
| watchdog เมื่อคำสั่งขาด | `mode` (เปิด/ปิดมอเตอร์) |

- เปลี่ยน kp/kd ได้ทุกเมื่อ (FixStand เข่า 150 → Velocity เข่า 99.1) ปิด `g1_ctrl` แล้วค่าก็หายไป
- ค่าอยู่ในไฟล์ฝั่งเรา: `config.yaml` (Passive, FixStand), `deploy.yaml` (Velocity ของแต่ละ policy)
- firmware ไม่ตรวจว่าค่าสมเหตุสมผล ส่งผิดมอเตอร์ก็ทำตาม
- controller ในตัวของ Unitree ใช้ช่องทางเดียวกัน ต้องเข้า debug mode (L2+R2) ก่อน

## 2. ค่าที่เราใช้มาจากไหน

สูตรจาก BeyondMimic ผ่าน mjlab (`src/assets/robots/unitree_g1/g1_23dof_constants.py`):

```
kp = armature × ω²          ω = 2π × 10 Hz
kd = 2 × ζ × armature × ω    ζ = 2  (หน่วงเกินวิกฤต เพราะ armature ไม่นับมวล link)
armature = ความเฉื่อยโรเตอร์ × (อัตราทด)²   (สเปคมอเตอร์ "from Unitree" ตามคอมเมนต์ใน mjlab)
```

| มอเตอร์ | joint | armature | kp | kd | effort (Nm) | action scale |
|---|---|---|---|---|---|---|
| 7520-14 | hip pitch/yaw, waist yaw | 0.01018 | 40.18 | 2.558 | 88 | 0.548 |
| 7520-22 | hip roll, knee | 0.02510 | 99.10 | 6.309 | 139 | 0.351 |
| 5020 ×2 (linkage) | ankle pitch/roll | 0.00722 | 28.50 | 1.814 | 50 | 0.439 |
| 5020 | shoulder, elbow, wrist roll | 0.00361 | 14.25 | 0.907 | 25 | 0.439 |

action scale = 0.25 × effort / kp

**รู้ได้อย่างไรว่าเป็นค่านี้:** ยืนยันได้ว่าค่าตอนเทรน = ค่าใน `deploy.yaml` (`gain_check.py`) แต่**ยังยืนยันไม่ได้ว่าความเฉื่อยโรเตอร์ตรงกับมอเตอร์จริง** ไม่พบเอกสารต้นฉบับจาก Unitree

## 3. เทียบกับที่อื่น

| joint (kp / kd) | **ของเรา** | unitree_rl_gym | unitree_rl_lab | FixStand (Unitree) | SDK `g1_ankle_swing_example` |
|---|---|---|---|---|---|
| hip pitch/yaw | **40.2 / 2.56** | 100 / 2 | 100 / 2 | 100 / 2 | 60 / 1 |
| hip roll | **99.1 / 6.31** | 100 / 2 | 100 / 2 | 100 / 2 | 60 / 1 |
| knee | **99.1 / 6.31** | 150 / 4 | 150 / 4 | 150 / 4 | 100 / 2 |
| ankle | **28.5 / 1.81** | 40 / 2 | 40 / 2 | 40 / 2 | 40 / 1 |
| waist yaw | **40.2 / 2.56** | – | 200 / 5 | 200 / 5 | 60 / 1 |
| แขน | **14.3 / 0.91** | – | 40 / 1 | 40 / 10 | 40 / 1 |
| armature ใน sim | ตามมอเตอร์ | – | 0.01 | – | – |

- สองแนว: ปรับมือ (kp สูง, kd ต่ำ) กับคำนวณจาก armature (kp ต่ำกว่า 1.5–3 เท่า, kd สูงกว่า)
- ไม่มีชุดไหนถูกที่สุด kp/kd เป็นส่วนหนึ่งของ policy ต้อง deploy ด้วยชุดที่เทรนเท่านั้น
- FixStand ใช้ชุดของ Unitree ได้ เพราะไม่ใช่ RL และ `State_RLBase` ตั้ง gain ใหม่ทุกครั้งที่เข้า Velocity
- ตัวอย่าง SDK เป็นแค่ demo

แหล่งอ้างอิง: [unitree_rl_gym g1_config.py](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/legged_gym/envs/g1/g1_config.py), [unitree_rl_lab unitree.py](https://github.com/unitreerobotics/unitree_rl_lab/blob/main/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py), [mjlab g1_constants.py](https://github.com/mujocolab/mjlab/blob/main/src/mjlab/asset_zoo/robots/unitree_g1/g1_constants.py), [BeyondMimic arXiv 2508.08241](https://arxiv.org/pdf/2508.08241), [unitree_sdk2 g1_ankle_swing_example.cpp](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp)

### 3.1 ทำไมใช้แบบ BeyondMimic ไม่ใช่ 100/150/40

**เราไม่ได้เลือกเอง เป็นค่า default ของ repo ที่ fork มา:** `g1_23dof_constants.py` (สูตร armature·ω²) มีตั้งแต่ commit แรกของ unitree_rl_mjlab (ผู้ดูแลฝั่ง Unitree, `klavier`) ประวัติมีแค่การ reformat (`4a734a8`) ฝั่งเราไม่เคยแก้ไฟล์นี้ repo สร้างบน mjlab ซึ่งใช้สูตรเดียวกันกับ G1 ทุกรุ่น ที่เราแก้คือฝั่ง deploy (`233aac8` "Fix 23-dof gains mapping") ให้ `deploy.yaml` ส่งค่าเดียวกับตอนเทรนไปถูก slot

| repo | default |
|---|---|
| **unitree_rl_mjlab (ที่เราใช้)**, mjlab | BeyondMimic: armature·ω² (hip 40.2, knee 99.1, ankle 28.5, แขน 14.3) |
| unitree_rl_gym, unitree_rl_lab | ปรับมือ (hip 100, knee 150, ankle 40, แขน 40) |
| FixStand ใน `g1_ctrl` | ปรับมือ (ชุดเดียวกับข้างบน) |

**เหตุผลของแนวทางนี้ (BeyondMimic / mjlab):**

1. อิงฟิสิกส์ของมอเตอร์: kp = armature × ω² ทำให้ทุก joint มีความถี่ตอบสนองเท่ากัน (10 Hz) ไม่ว่ามอเตอร์จะใหญ่หรือเล็ก ค่าปรับมือให้ทั้งกลุ่ม 100 เท่ากันแม้มอเตอร์ต่างขนาด
2. ข้อต่อนุ่ม (10 Hz ถือว่าต่ำ) ช่วย sim2real: แรงกระแทกตอนเท้าลงพื้นเบาลง ไวต่อความต่าง sim กับจริงน้อยลง (มวล, friction, delay) BeyondMimic ใช้วิธีนี้ deploy บน G1 จริงสำเร็จ
3. ζ = 2: armature นับแค่โรเตอร์ ไม่นับมวล link ความเฉื่อยจริงสูงกว่าที่คำนวณ จึงหน่วงเผื่อไม่ให้แกว่ง
4. ค่าอื่นทั้งระบบตั้งให้เข้ากับ gain ชุดนี้: action scale = 0.25 × effort / kp, น้ำหนัก reward และช่วง DR ใน mjlab ถ้าเปลี่ยน gain ต้องปรับส่วนอื่นตาม

**ข้อเสีย:**

- ข้อต่ออ่อนกว่า ไปไม่ถึงเป้าพอดี policy ต้องเรียนสั่งเผื่อเอง
- แขน kp 14.3 อ่อน แขนตกจากท่าเล็กน้อยเพราะแรงโน้มถ่วง
- ความเฉื่อยโรเตอร์ในสูตรยังไม่ได้ยืนยันกับมอเตอร์จริง (`pd_check --replay` ใช้ตอบ)

**ควรเปลี่ยนเป็นชุดของ unitree_rl_gym ไหม:** ตอนนี้ไม่ควร policy ทุกตัว (P3b, yaw_ft, yaw_stand) เทรนกับชุดนี้และต้อง deploy ด้วยชุดเดียวกัน เปลี่ยนต้องเทรนใหม่และปรับ action scale และใน sim ยังไม่พบปัญหาที่ชี้ว่า gain ผิด ถ้าผล `pd_check` หรือการเดินจริงชี้ว่าข้อต่ออ่อนเกิน (ยืนแล้วทรุด, ตามคำสั่งไม่ทัน) ค่อยเทรนรุ่นเปรียบเทียบที่ใช้ ω สูงขึ้นหรือชุดของ unitree_rl_gym

## 4. ตรวจ 4 ชั้น

| ชั้น | คำถาม | เครื่องมือ | ใช้หุ่น |
|---|---|---|---|
| 1 | ไฟล์ตรงกับตอนเทรนไหม | `gain_check.py <policy folder>` | ไม่ |
| 2 | `g1_ctrl` ส่ง kp/kd ถูกค่า ถูก slot ถูกโหมดไหม | `lowcmd_monitor.py` | sim แล้วหุ่นจริง |
| 3 | มอเตอร์ใช้ kp/kd ตามที่ส่งไหม | `pd_check.py` (torque check) | หุ่นจริง (แขวน) |
| 4 | ข้อต่อจริงตอบสนองเหมือน sim ไหม | `pd_check.py --replay` | หุ่นจริง (แขวน) |

ลำดับ (checklist หัวข้อ 9.0–9.2):

| ด่าน | gain | ตรวจ | เกณฑ์ |
|---|---|---|---|
| Passive | kp = 0, kd 2.5–6 | kd (หมุนข้อต่อด้วยมือ) | `kd x` 0.5–1.5 |
| FixStand | Unitree (100/150/40/200) | kp (สู้แรงโน้มถ่วง), ระยะห่าง `qs` < 0.05 rad, ดันแล้วปล่อย | `kp x` 0.8–1.2, corr ≥ 0.9 |
| Velocity (sim ก่อน แล้วบนพื้น ไม่ใช่ตอนแขวน) | policy (40.2/99.1/28.5/14.3) | `lowcmd_monitor`: ค่าตรง, slot ที่ไม่ใช้ต้อง 0 | ไม่มี `MISMATCH`; `pd_check` ช่วงนี้เป็นข้อมูลเพิ่ม |

`pd_check.py` ทำอะไร:

1. หา delay ของคำสั่ง (0–30 ms) จาก correlation (ไม่ขึ้นกับ scale ของ kp)
2. fit `tau_est − tau_ff = a·kp(q_des−q) + b·kd(dq_des−dq)` ต่อ joint ต่อโหมด → `kp x` = a, `kd x` = b
3. ตัด sample ที่แรงบิดเกิน 95% ของ effort limit (`sat%`), ไม่ให้คะแนน gain ที่ term เล็กเกินไป
4. `--replay`: ส่ง lowcmd ที่บันทึกเข้า MuJoCo ยึดลำตัวกลางอากาศ เทียบ q (`nrmse`, `lag ms`), เลือก `--armature train` / `--delay-ms`

`tau_est` ประมาณจากกระแส คลาดได้ ~±10–20% ถ้าทุก joint เพี้ยนทางเดียวกัน น่าจะเป็นเพราะ `tau_est` มากกว่า gain ผิด

ตารางอ่านผลเมื่อไม่ผ่าน: checklist หัวข้อ 9.1

## 5. ผลทดสอบเครื่องมือ

| กรณี | ผล |
|---|---|
| log จำลองที่ตรงกับ sim | kp ×1.00, kd ×1.00, delay 0 ms, replay error 0 |
| log จำลอง kp ×0.7 + delay 10 ms + armature ตอนเทรน | **kp ×0.70, kd ×1.00, delay 10 ms**, replay เตือน |
| log จำลอง delay 10 ms อย่างเดียว | delay 10 ms, ไม่เตือนผิด |
| `unitree_mujoco` + `g1_ctrl` FixStand | ขาทุกข้อ kp ×0.98–1.01 (แขนสั่นใน sim ดู [02](02_sim2sim_eval.md#5-ปัญหาที่เจอ)) |

## 6. จุดที่ต้องตรวจต่อ

1. ความเฉื่อยโรเตอร์จริง: เทียบ `pd_check --replay` แบบ `mjcf` vs `train`
2. แรงบิดสูงสุดข้อเท้า: เทรนที่ 50 Nm (2 × 25) แต่ unitree_rl_lab ใช้ 35 Nm ยังไม่พบสเปคทางการ
3. แขน kp 14.3 อ่อน แขนตกจากท่าเล็กน้อย ไม่เป็นปัญหาถ้า deploy ด้วยค่าเดียวกับที่เทรน
4. `State_Mimic.cpp` ใส่ kp/kd ตาม index ตรงๆ ไม่ผ่าน `joint_ids_map` ผิดกับ 23dof (Mimic ปิดอยู่ ต้องแก้ก่อนเปิด)
