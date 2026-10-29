# 05: แผน deploy และความปลอดภัย

[← สารบัญ](README.md)

- [1. ใช้ policy ไหน](#1-ใช้-policy-ไหน)
- [2. FSM ของ g1_ctrl และ FixStand](#2-fsm-ของ-g1_ctrl-และ-fixstand)
- [3. ลำดับ deploy รอบแรก](#3-ลำดับ-deploy-รอบแรก)
- [4. กลไกหยุด](#4-กลไกหยุด)
- [5. สิ่งที่ไม่มีการป้องกัน และสิ่งที่แก้แล้ว/เสนอ](#5-สิ่งที่ไม่มีการป้องกัน-และสิ่งที่แก้แล้วเสนอ)
- [6. การเลือกโฟลเดอร์ policy](#6-การเลือกโฟลเดอร์-policy)

## 1. ใช้ policy ไหน

**รอบแรก: P3b** (`config/policy/velocity/v0`, md5 `727da8a2bc5293f617a8e06f8c13a07e`)

- เป็นตัวเดียวที่ผ่าน sim2sim ครบระบบกับ `g1_ctrl` + `unitree_mujoco`
- แม่นที่สุดที่ vx ≤ 0.4 (97%), ยืนนิ่ง, กระตุกตอนเข้า Velocity แค่ 0.28 rad
- ตัวที่สอง: **yaw_stand** (`config/policy/velocity/hist5_yawstand`) หลัง P3b ผ่าน และหลังทดสอบกับ `g1_ctrl` + `unitree_mujoco` ด้วย `sim2sim_policy.sh hist5_yawstand` เข้า Velocity เฉพาะตอนเท้าแตะพื้น

## 2. FSM ของ g1_ctrl และ FixStand

```
Passive  ──(LT + ↑)──▶  FixStand  ──(RT + A)──▶  Velocity (policy)
   ▲                        │                          │
   └────────(LT + B)────────┴──────────(LT + B)────────┘
```

| โหมด | ทำอะไร |
|---|---|
| Passive | kp = 0, kd 2.5–6: หนืดอย่างเดียว ไม่ดึงไปที่ไหน |
| FixStand | ไล่เป้าหมายเป็นเส้นตรงจากท่าปัจจุบันไป `qs` ใน `ts: [0, 3]` แล้วค้าง ใช้ gain ของ Unitree (ขา 100/150/40, เอว 200, แขน 40) `qs` = `default_joint_pos` ของ policy |
| Velocity | policy ควบคุม gain จาก `deploy.yaml` |

- FixStand ไม่ใช่ RL เป็นค่าเดิมของ Unitree (ตรงกับ `g1` 29dof, ไม่เคยแก้ตั้งแต่ `fc9587f`) ใช้ได้เลย การลด kp เป็นทางเลือก
- ใน Passive โค้ดคัดลอก q ที่วัดได้ไปเป็นคำสั่งทุกรอบ เข้า FixStand จึงเริ่มจากท่าจริง ไม่กระโดด
- บนหุ่นจริงสั่งความเร็วด้วยจอยรีโมต (คีย์บอร์ดใช้ได้เฉพาะ `--network lo`)

**จอยรีโมตส่งความเร็วตรงๆ ไม่มีการ smooth:** stick ซ้ายแกนตั้ง = vx (เต็ม stick = 1.0 m/s, clamp [−0.5, 1.0]), stick ซ้ายแกนนอน = vy (±0.5), stick ขวาแกนนอน = wz (±1.0) รอบแรกดันไม่เกิน ~40%

## 3. ลำดับ deploy รอบแรก

รายละเอียด, คำสั่ง และเกณฑ์: checklist หัวข้อ 9

| ขั้น | ทำอะไร | ผ่านเมื่อ |
|---|---|---|
| 0 | ตรวจแบบอ่านอย่างเดียว | ✅ 28/10 |
| 9.0 | md5, `gain_check.py` PASS, debug mode, monitor 2 ตัว, `g1_ctrl --network enp43s0` | ขึ้น `Connected`, `ONNX policy loaded ... v0` |
| sim | `g1_ctrl` + `unitree_mujoco` + `lowcmd_monitor`: gain ของ FixStand และ Velocity ตรง, slot ว่างได้ 0 | ไม่มี `MISMATCH` |
| 9.1 | แขวน: Passive (ตรวจ kd) → FixStand 30 s (ตรวจ kp) **ไม่เข้า Velocity ตอนแขวน** | `pd_check` ผ่าน, ห่าง `qs` < 0.05 rad, ไม่สั่น |
| 9.2 | หย่อนให้เท้าแตะพื้น (เชือกกันไว้), RT + A, ยืนใน Velocity | ยืนนิ่ง, เอียง < 5°, `gains match: Velocity` |
| 9.3 | เดินช้า stick ~20% → ~40% | เดินตรง (P3b เลี้ยวขวาเอง ~2°/s) |
| 9.4 | หมุน ~30%, เดินด้านข้าง ~30% | – |

## 4. กลไกหยุด

| กลไก | ทำงานเมื่อ | เร็วแค่ไหน | ข้อจำกัด |
|---|---|---|---|
| LT + B → Passive | ทุกโหมด | FSM ตรวจทุก 1 ms + รอ policy thread หยุด ≤ 20 ms | Passive = kp 0: บนพื้นหุ่นทรุดแบบหน่วง ให้เชือกรับน้ำหนักก่อนกด |
| LowState timeout → Passive | ไม่ได้ `rt/lowstate` 100 ms | 100 ms | เครือข่ายขาด คำสั่ง Passive ก็ไปไม่ถึง อาการขึ้นกับ firmware |
| bad_orientation → Passive | Velocity, เอียง > 0.85 rad (49°) | ทันที | ช้าเกินกันล้ม ให้คนกด LT + B ที่ ~15° |
| `g1_ctrl` crash | – | – | ยังไม่รู้อาการ ทดสอบด้วย Ctrl-C ตอนแขวนใน FixStand |

หยุดทันทีเมื่อ: joint ผิดทิศ/สั่นถี่/มีเสียง, `MISMATCH` / `unknown` / `STALE` / gap > 100 ms / `OUT`, อุณหภูมิ > 70 °C, เอียง > ~15°, มีคนพูด "หยุด"

## 5. สิ่งที่ไม่มีการป้องกัน และสิ่งที่แก้แล้ว/เสนอ

| เรื่อง | สถานะ |
|---|---|
| ตำแหน่งเป้าหมายไม่ถูกจำกัด (`clip: null`) | ⏳ เสนอ: ใส่ `clip` = limit MJCF ใน `deploy.yaml` (มีผลแค่ ≤ 1% ของ step, `gain_check` และ `vel_eval` รองรับแล้ว) |
| ไม่จำกัดความเร็ว joint | ใน sim สูงสุด 29–45% ของ limit มอเตอร์ |
| กระตุกตอนเข้า Velocity | ✅ ทดสอบใน `unitree_mujoco` (แขวนด้วย elastic band) แล้ว: P3b 20 ms แรกขยับ ≤ 0.015 rad, ก้าวสูงสุดระหว่างข้อความ 0.009 rad (เดิม step แรก 0.28 rad) — **soft start 0.5 s** ใน `robots/g1_23dof/src/State_RLBase.cpp` เป้าหมายเริ่มจากท่าที่ FixStand ค้างไว้แล้วค่อยๆ เปลี่ยนเป็นค่าของ policy, ไม่ส่งค่าค้างจากรอบก่อน (ตรวจการเข้าโหมดจากช่วงห่างของ `run()` > 50 ms) |
| controller สองตัวส่ง `rt/lowcmd` พร้อมกัน | ⏳ เสนอ: ให้ `robots/g1_23dof/main.cpp` ปิดตัวเมื่อเจอ (ตอนนี้แค่เตือน) |
| policy สาย Hist5 ตอนแขวน | ❌ เตะขา (ความเร็ว 87% ของ limit, แรงบิด 100%, ข้อเท้าชน limit) ห้ามเข้า Velocity ตอนแขวน ตรวจ policy ใหม่ด้วย `vel_eval.py --hang` |
| มุมเอียงที่ตัดเอง 0.85 rad | คงไว้ ถ้าต่ำเกินอาจตัดระหว่างเดินปกติแล้วหุ่นทรุด |
| arm EMA filter (`arm_ema_alpha`) | มีในโค้ดแต่ปิด (1.0) ไม่เปิดเพราะเปลี่ยน dynamics จากตอนเทรน |

## 6. การเลือกโฟลเดอร์ policy

ถ้า `policy_dir` ไม่มี `exported/` อยู่ตรงๆ `g1_ctrl` เลือก**โฟลเดอร์ย่อยตัวสุดท้ายตามชื่อ** เพิ่มโฟลเดอร์ใหม่อาจแทน policy ที่ deploy อยู่โดยไม่รู้ตัว จึง pin ไว้ใน `config.yaml`:

```yaml
Velocity:
  policy_dir: config/policy/velocity/v0     # P3b; hist5_yawft = yaw_ft (model_17998)
```

เปลี่ยน policy บนหุ่นจริง = แก้บรรทัดนี้ ทดสอบ policy อื่นใน sim ใช้ `sim2sim_policy.sh` (ไม่แตะ config จริง)
