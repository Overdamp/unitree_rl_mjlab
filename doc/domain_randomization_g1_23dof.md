# Domain Randomization: G1-23DoF Velocity

เอกสารนี้สรุปค่า Domain Randomization (DR) ที่ใช้ฝึก task `Unitree-G1-23Dof-Flat`
ค่าทั้งหมดอ่านจาก `params/env.yaml` ที่ rsl_rl บันทึกไว้ในแต่ละ run จึงเป็นค่าที่ใช้ฝึกจริง
ไม่ใช่ค่าที่คาดว่าใช้จากโค้ด

## Domain Randomization คืออะไร

Policy ที่ฝึกใน simulation เห็นหุ่นที่มีมวล แรงเสียดทาน และ gain ของมอเตอร์ตรงตามโมเดลทุกครั้ง
แต่หุ่นจริงไม่ตรงกับโมเดล เช่น แบตเตอรี่ทำให้หนักขึ้น เฟืองมีแรงเสียดทาน หรือแรงบิดลดลงเมื่อมอเตอร์ร้อน
DR สุ่มค่าเหล่านี้ให้กว้างพอจนครอบคลุมหุ่นจริง policy จึงต้องเรียนรู้ท่าเดินที่ทนต่อความคลาดเคลื่อน
แทนที่จะพึ่งพาค่าของโมเดลตัวเดียว

## Run ที่ใช้อ้างอิง

| Phase | Run (`logs/rsl_rl/g1_23dof_velocity/`) | Iteration | เริ่มจาก | DR |
| --- | --- | --- | --- | --- |
| 1 | `2026-10-20_18-26-38` | 0 → 10000 | ศูนย์ (`resume: false`) | ชุดพื้นฐาน |
| 2 | `2026-10-21_11-41-28` | 10000 → 12999 | `model_10000.pt` ของ Phase 1 | ชุดพื้นฐาน + ชุด Phase 2 |

ทั้งสอง phase ใช้ `num_envs=1024` และ `seed=42`
Phase 2 สั่งรันด้วย `run_phase2_train.sh` ส่วน policy ที่ export แล้วอยู่ที่
`deploy/robots/g1_23dof/config/policy/velocity/v0/exported/` ใช้ชื่อ `policy_phase1_10000.onnx`
และ `policy_phase2_13000.onnx` และในตอนที่เขียนเอกสารนี้ `policy.onnx` คือไฟล์เดียวกับ Phase 2

## ช่วงเวลาที่สุ่ม (mode)

| mode | สุ่มเมื่อไร | ผลต่อการฝึก |
| --- | --- | --- |
| `startup` | ครั้งเดียวตอนสร้าง environment | แต่ละ env ได้หุ่นหนึ่งแบบและใช้ไปตลอด run จึงได้หุ่นที่ต่างกัน 1024 แบบ |
| `reset` | ทุกครั้งที่ episode เริ่มใหม่ | สุ่มท่าและตำแหน่งเริ่มต้น |
| `interval` | ทุก ๆ ช่วงเวลาที่กำหนดระหว่าง episode | ใช้จำลองแรงผลักจากภายนอก |

ถ้าไม่ได้ระบุ `shared_random` จะเป็น `False` หมายความว่าแต่ละข้อต่อหรือแต่ละ body
ได้ค่าสุ่มของตัวเอง ไม่ได้ใช้ค่าเดียวกันทั้งตัว

## 1. DR พื้นฐาน (ใช้ทั้ง Phase 1 และ Phase 2)

กำหนดไว้ใน `src/tasks/velocity/velocity_env_cfg.py` และ `env_cfgs.py` ของ G1-23DoF
เป็นคนกำหนดชื่อ body และ geom

| Event | mode | สุ่มอะไร | ช่วงค่า | ค่า nominal | หมายเหตุ |
| --- | --- | --- | --- | --- | --- |
| `foot_friction` | startup | ค่าแรงเสียดทานของ geom เท้า `left/right_foot[1-7]_collision` | 0.3 – 1.6 (abs) | 0.6 | `shared_random: True` คือเท้าทั้ง 14 geom ใช้ค่าเดียวกัน ครอบคลุมพื้นตั้งแต่ลื่นจนถึงหนืด |
| `encoder_bias` | startup | offset ของมุมข้อต่อที่ policy อ่านได้ ทุกข้อต่อ | ±0.015 rad (≈ ±0.86°) | 0 | จำลองการ calibrate encoder ที่คลาดเคลื่อน |
| `base_com` | startup | ตำแหน่งจุดศูนย์ถ่วงของ `torso_link` บนแกน x, y, z | ±0.05 m ต่อแกน (add) | ตาม XML | จำลองการกระจายมวลที่ไม่ตรงกับโมเดล |
| `push_robot` | interval ทุก 5–6 s | ตั้งความเร็วของ base ทันที | x, y ±0.5 m/s · z ±0.4 m/s · roll, pitch ±0.52 rad/s · yaw ±0.78 rad/s | — | จำลองการถูกผลักหรือชน ปิดในโหมด play |
| `reset_base` | reset | ตำแหน่งและทิศทางเริ่มต้น | x, y ±0.5 m · yaw ±3.14 rad | — | ใช้สุ่มสถานะเริ่มต้น ไม่ได้สุ่มฟิสิกส์ |
| `reset_robot_joints` | reset | offset ของมุมและความเร็วข้อต่อตอนเริ่ม | 0 (ปิดอยู่) | — | ตอนนี้เริ่มจากท่า default ทุกครั้ง |

### Observation noise

ใส่เฉพาะ actor (`enable_corruption: True`) ส่วน critic ไม่มี noise
โดยสุ่มแบบ uniform และบวกเข้าทุก step

| Observation | Noise |
| --- | --- |
| `base_ang_vel` (IMU gyro) | ±0.2 rad/s |
| `projected_gravity` | ±0.05 |
| `joint_pos` | ±0.01 rad |
| `joint_vel` | ±1.5 rad/s |
| `command`, `phase`, `actions` | ไม่มี |

## 2. DR เพิ่มใน Phase 2

กำหนดไว้ใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` ภายใต้หัวข้อ
`# Phase 2 Sim-to-Real Domain Randomization` (commit `233aac8`) ทุกตัวเป็น `startup`

| Event | สุ่มอะไร | Operation | ช่วงค่า | ค่า nominal ในโมเดล train | เหตุผลที่ตั้งช่วงนี้ |
| --- | --- | --- | --- | --- | --- |
| `torso_mass_payload` | มวล `torso_link` | add | −1.0 ถึง +3.5 kg | 7.82 kg (มวลรวมทั้งตัว 32.11 kg) | มวลรวมจึงอยู่ในช่วงประมาณ 31.1–35.6 kg ซึ่งครอบคลุมมวลหุ่นจริงพร้อมแบตเตอรี่ที่ประมาณไว้ 34.1–35.0 kg |
| `limb_mass_scale` | มวลทุก body ยกเว้น `torso_link` (regex `^(?!torso_link$).*`) รวม pelvis, ขา และแขน | scale | ×0.90 – ×1.10 | รวม 24.29 kg | ความคลาดเคลื่อนของมวลแต่ละ link ±10% |
| `joint_friction` | `frictionloss` ของทุกข้อต่อ | abs | 0.0 – 0.30 | 0.0 | ครอบคลุม stiction ของเฟืองและ 4-bar linkage โดยค่าใน `scene_g1_23dof.xml` ที่ใช้ sim2sim คือ 0.2 |
| `joint_damping` | viscous damping ของทุกข้อต่อ | abs | 0.0 – 0.10 | 0.0 | ครอบคลุม damping เชิงกล โดยค่าใน `scene_g1_23dof.xml` คือ 0.05 |
| `motor_pd_gains` | Kp และ Kd ของ actuator ทุกตัว (สุ่ม Kp และ Kd แยกกัน) | scale | Kp ×0.85 – ×1.15 · Kd ×0.85 – ×1.15 | ดูตาราง gain ด้านล่าง | ความต่างของมอเตอร์แต่ละตัว และผลจากแรงดันแบตเตอรี่ |
| `motor_effort_limits` | แรงบิดสูงสุดของ actuator | scale | ×0.85 – ×1.05 | ดูตาราง gain ด้านล่าง | แบตเตอรี่อ่อนหรือมอเตอร์ร้อน ช่วงนี้จึงเอียงไปทางลดแรงบิด |

### ค่า Kp, Kd และ effort limit ที่ใช้เป็นฐาน

คำนวณใน `src/assets/robots/unitree_g1/g1_23dof_constants.py` จาก armature
โดยใช้ ω = 10 Hz และ ζ = 2 แล้วคัดลอกค่าเดียวกันไปไว้ใน `deploy.yaml`

| กลุ่มมอเตอร์ | ข้อต่อ | Kp nominal | Kd nominal | Effort limit (N·m) | ช่วง Kp หลัง DR | ช่วง effort หลัง DR |
| --- | --- | --- | --- | --- | --- | --- |
| 7520-14 | hip_pitch, hip_yaw, waist_yaw | 40.2 | 2.6 | 88 | 34.2 – 46.2 | 74.8 – 92.4 |
| 7520-22 | hip_roll, knee | 99.1 | 6.3 | 139 | 84.2 – 114.0 | 118.2 – 146.0 |
| 5020 × 2 | ankle_pitch, ankle_roll | 28.5 | 1.8 | 50 | 24.2 – 32.8 | 42.5 – 52.5 |
| 5020 | shoulder ×3, elbow, wrist_roll | 14.3 | 0.9 | 25 | 12.2 – 16.4 | 21.3 – 26.3 |

## 3. ภาพรวมแยกตามหมวด

| หมวด | Phase 1 | Phase 2 |
| --- | --- | --- |
| พื้นสัมผัส | foot friction | foot friction |
| มวลและ inertia | COM ของ torso | COM ของ torso + มวล torso + มวล link |
| ข้อต่อ | encoder bias | encoder bias + friction loss + damping |
| มอเตอร์ | — | Kp, Kd + effort limit |
| แรงภายนอก | push ทุก 5–6 s | push ทุก 5–6 s |
| Sensor | observation noise | observation noise |

## 4. สิ่งที่ยังไม่ได้ randomize

ข้อต่อไปนี้ยังไม่มีใน config ปัจจุบัน และควรพิจารณาก่อนนำไปใช้กับหุ่นจริง

- **Action/actuator latency**: ยังไม่ได้ใช้ `DelayedActuator` ทั้งที่หุ่นจริงมี delay จาก DDS และ control loop
- **IMU bias และ drift**: มีเพียง uniform noise รายรอบ ไม่มี bias คงที่ต่อ episode
- **Motor armature**: ใช้ค่าคงที่จากสเปก
- **ท่าเริ่มต้นของข้อต่อ**: `reset_robot_joints` ตั้งช่วงไว้เป็น 0
- **ภูมิประเทศ**: task Flat ใช้พื้นราบ (`plane`) อย่างเดียว

## 5. ข้อควรระวัง

- Event ที่เป็น `startup` จะถูกสุ่มเฉพาะตอนเริ่มฝึกหรือตอน resume เท่านั้น
  ความหลากหลายของหุ่นจึงมีเท่ากับจำนวน env (1024 แบบ) ถ้าลด `num_envs` ความหลากหลายจะลดลงด้วย
- `scripts/play.py` สร้าง environment ด้วย `play=True` ซึ่งปิด push, curriculum และ observation noise
  แต่ **ไม่ได้ปิด DR ของ Phase 2** หุ่นที่เห็นในโหมด play จึงเป็นหุ่นที่ถูกสุ่มค่าแล้วหนึ่งตัว
- Task `Unitree-G1-23Dof-Rough` ได้ DR ชุดเดียวกัน เพราะ Flat สร้างต่อจาก config ของ Rough
- ค่ามวลหุ่นจริง 34.1–35.0 kg มาจาก comment ในโค้ด ยังไม่ได้ชั่งยืนยัน

## ไฟล์ที่เกี่ยวข้อง

- `src/tasks/velocity/velocity_env_cfg.py`: DR พื้นฐานและ observation noise
- `src/tasks/velocity/config/g1_23dof/env_cfgs.py`: DR ของ Phase 2
- `src/assets/robots/unitree_g1/g1_23dof_constants.py`: Kp, Kd และ effort limit
- `logs/rsl_rl/g1_23dof_velocity/<run>/params/env.yaml`: ค่าที่ใช้จริงในแต่ละ run
- `run_phase2_train.sh`: คำสั่ง resume สำหรับ Phase 2
