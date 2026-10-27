# สรุป Domain Randomization แยกตาม Phase: G1-23DoF Velocity

สรุปว่าการฝึก task `Unitree-G1-23Dof-Flat` แต่ละ phase ทำ Domain Randomization (DR) อะไรบ้าง และทำเพื่ออะไร
ค่าทุกตัวตรวจจาก `logs/rsl_rl/g1_23dof_velocity/<run>/params/env.yaml` ซึ่งเป็นค่าที่ใช้ฝึกจริง (ตรวจเมื่อ 27/10/2026)

รายละเอียดเชิงลึกของ Phase 1–2 (ตาราง Kp/Kd ต่อมอเตอร์ และที่มาของมวล) อยู่ใน `doc/domain_randomization_g1_23dof.md`
เอกสารนั้นยังไม่ได้รวม Phase 3

## 1. ภาพรวมแต่ละ Phase

| Phase | Run | Iteration | เริ่มจาก | DR ที่เพิ่ม | เป้าหมาย | ผล | Policy ที่ export |
|---|---|---|---|---|---|---|---|
| 1 | `2026-10-20_18-26-38` | 0 → 10000 | ศูนย์ | ชุดพื้นฐาน | เรียนรู้การเดินที่ทนต่อพื้น, แรงผลัก และ noise ของ sensor | เดินได้ | `policy_phase1_10000.onnx` |
| 2 | `2026-10-21_11-41-28` | 10000 → 12999 | Phase 1 `model_10000.pt` | มวล, ข้อต่อ, มอเตอร์ | ทนต่อความต่างของตัวหุ่นจริงกับโมเดล (sim-to-real) | เดินได้ แต่ทดสอบ sim2sim แล้วพบว่าหุ่นยืนสั่นเมื่อมี delay 20–30 ms และล้มที่ 40 ms | `policy_phase2_13000.onnx` |
| 3a | `2026-10-22_16-01-12` | 13000 → 15998 | Phase 2 `model_12999.pt` | action latency **0–40 ms** | ทนต่อความหน่วงของ DDS และ control loop | ❌ ล้มเหลว: เดินหน้าได้แค่ประมาณ 0.25 m/s ไม่ว่าสั่งเท่าไร | — (tag `phase3a-lag0-40ms-collapsed`) |
| 3b | `2026-10-22_17-02-19` | 13000 → 15998 | Phase 2 `model_12999.pt` | action latency **0–20 ms** | เหมือน 3a แต่แคบลง | ใช้ทดสอบ sim2sim ต่อ | `policy_phase3b_16000.onnx` (= `policy.onnx` ปัจจุบัน) |

ทุก run ใช้ `num_envs=1024`, `seed=42`, physics `dt=0.005 s`, `decimation=4` (policy ทำงานที่ 50 Hz, ทุก 20 ms)
run `2026-10-20_17-54-12` และ `2026-10-20_17-58-43` เป็น run ทดลองที่ไม่มี checkpoint

## 2. ช่วงเวลาที่สุ่ม (mode)

| mode | สุ่มเมื่อไร | ความหมาย |
|---|---|---|
| `startup` | ครั้งเดียวตอนสร้าง env (รวมตอน resume) | แต่ละ env ได้หุ่นหนึ่งแบบไปตลอด run ได้หุ่นต่างกัน 1024 แบบ |
| `reset` | ทุกครั้งที่ episode เริ่มใหม่ | เปลี่ยนค่าทุก episode |
| `interval` | เป็นระยะระหว่าง episode | จำลองเหตุการณ์ที่เกิดขณะเดิน |
| observation noise | ทุก step | สุ่มใหม่ทุกครั้งที่ policy อ่าน sensor |

## 3. ตาราง DR ทั้งหมดแยกตาม Phase

✅ = ใช้, — = ไม่ใช้

| # | Event | หมวด | mode | สุ่มอะไร | ช่วงค่า | ทำเพื่ออะไร | P1 | P2 | P3a | P3b |
|---|---|---|---|---|---|---|:-:|:-:|:-:|:-:|
| 1 | `foot_friction` | พื้นสัมผัส | startup | แรงเสียดทานของ geom เท้า 14 ชิ้น (ใช้ค่าเดียวกันทั้งหมด) | 0.3 – 1.6 (nominal 0.6) | เดินได้ทั้งบนพื้นลื่นและพื้นหนืด | ✅ | ✅ | ✅ | ✅ |
| 2 | `encoder_bias` | Sensor | startup | offset ของมุมข้อต่อที่ policy อ่าน (ทุกข้อต่อ) | ±0.015 rad (≈ ±0.86°) | ทนต่อ encoder ที่ calibrate คลาดเคลื่อน | ✅ | ✅ | ✅ | ✅ |
| 3 | `base_com` | มวล | startup | ตำแหน่ง COM ของ `torso_link` (x, y, z) | ±0.05 m ต่อแกน | ทนต่อการกระจายมวลที่ไม่ตรงโมเดล | ✅ | ✅ | ✅ | ✅ |
| 4 | `push_robot` | แรงภายนอก | interval ทุก 5–6 s | ความเร็วของ base ถูกเปลี่ยนทันที | xy ±0.5 m/s, z ±0.4 m/s, roll/pitch ±0.52 rad/s, yaw ±0.78 rad/s | ทรงตัวกลับได้เมื่อถูกผลักหรือชน | ✅ | ✅ | ✅ | ✅ |
| 5 | `reset_base` | สถานะเริ่มต้น | reset | ตำแหน่งและทิศเริ่มต้น | xy ±0.5 m, yaw ±3.14 rad | ไม่ยึดติดกับทิศเริ่มต้น (สุ่มสถานะ ไม่ได้สุ่มฟิสิกส์) | ✅ | ✅ | ✅ | ✅ |
| 6 | `reset_robot_joints` | สถานะเริ่มต้น | reset | offset มุมและความเร็วข้อต่อตอนเริ่ม | 0 (**ปิดอยู่**) | — | ✅* | ✅* | ✅* | ✅* |
| 7 | `torso_mass_payload` | มวล | startup | มวล `torso_link` (บวกเพิ่ม) | −1.0 ถึง +3.5 kg (nominal 7.82 kg) | มวลรวมประมาณ 31.1–35.6 kg ครอบคลุมหุ่นจริงพร้อมแบตเตอรี่ (ประมาณ 34.1–35.0 kg) | — | ✅ | ✅ | ✅ |
| 8 | `limb_mass_scale` | มวล | startup | มวลทุก body ยกเว้น torso (คูณ) | ×0.90 – ×1.10 | ทนต่อมวลของแต่ละ link ที่คลาดเคลื่อน ±10% | — | ✅ | ✅ | ✅ |
| 9 | `joint_friction` | ข้อต่อ | startup | `frictionloss` ทุกข้อต่อ | 0.0 – 0.30 (nominal 0) | ทนต่อแรงเสียดทานสถิตของเฟืองและ 4-bar linkage (sim2sim ใช้ 0.2) | — | ✅ | ✅ | ✅ |
| 10 | `joint_damping` | ข้อต่อ | startup | viscous damping ทุกข้อต่อ | 0.0 – 0.10 (nominal 0) | ทนต่อ damping เชิงกล (sim2sim ใช้ 0.05) | — | ✅ | ✅ | ✅ |
| 11 | `motor_pd_gains` | มอเตอร์ | startup | Kp และ Kd ทุก actuator (สุ่มแยกกัน) | ×0.85 – ×1.15 | ทนต่อความต่างของมอเตอร์แต่ละตัวและแรงดันแบตเตอรี่ | — | ✅ | ✅ | ✅ |
| 12 | `motor_effort_limits` | มอเตอร์ | startup | แรงบิดสูงสุดของ actuator | ×0.85 – ×1.05 | ทนต่อแบตอ่อนหรือมอเตอร์ร้อน (เอียงไปทางลดแรงบิด) | — | ✅ | ✅ | ✅ |
| 13 | `action_delay` | Latency | reset | ความหน่วงของ position target (ค่าเดียวกันทั้งตัว คงที่ตลอด episode) | P3a: 0–8 physics step (**0–40 ms**) · P3b: 0–4 step (**0–20 ms**) | ทนต่อความหน่วงของ DDS และ control loop บนหุ่นจริง | — | — | ✅ | ✅ |

\* `reset_robot_joints` อยู่ใน config ทุก phase แต่ช่วงค่าเป็น 0 จึงไม่มีผล

### Observation noise (ทุก phase เหมือนกัน)

ใส่เฉพาะ actor (`enable_corruption: True`) ส่วน critic ไม่มี noise สุ่มแบบ uniform ทุก step

| Observation | Noise | ทำเพื่ออะไร |
|---|---|---|
| `base_ang_vel` (IMU gyro) | ±0.2 rad/s | ทนต่อ noise ของ gyro |
| `projected_gravity` | ±0.05 | ทนต่อความคลาดเคลื่อนของการประมาณท่าทาง |
| `joint_pos` | ±0.01 rad | ทนต่อ noise ของ encoder |
| `joint_vel` | ±1.5 rad/s | ทนต่อความเร็วข้อต่อที่ได้จากการหาอนุพันธ์ซึ่งมี noise สูง |
| `command`, `phase`, `actions` | ไม่มี | เป็นค่าภายใน ไม่ได้มาจาก sensor |

## 4. สรุปตามหมวด

| หมวด | Phase 1 | Phase 2 | Phase 3a / 3b |
|---|---|---|---|
| พื้นสัมผัส | foot friction | เหมือน P1 | เหมือน P1 |
| มวล | COM torso | + มวล torso + มวล link | เหมือน P2 |
| ข้อต่อ | encoder bias | + friction loss + damping | เหมือน P2 |
| มอเตอร์ | — | Kp/Kd + effort limit | เหมือน P2 |
| Latency | — | — | **+ action delay** (3a: 0–40 ms, 3b: 0–20 ms) |
| แรงภายนอก | push ทุก 5–6 s | เหมือน P1 | เหมือน P1 |
| Sensor noise | observation noise | เหมือน P1 | เหมือน P1 |
| **จำนวน event DR** | 6 | 12 | 13 |

## 5. Phase 3a กับ 3b ต่างกันอย่างไร

| | Phase 3a | Phase 3b |
|---|---|---|
| Commit / tag | `4f78cb0` / `phase3a-lag0-40ms-collapsed` | `8035771` / `phase3b-lag0-20ms` |
| `lag_range` | (0, 8) physics step | (0, 4) physics step |
| Delay สูงสุด | 40 ms (2 policy step) | 20 ms (1 policy step) |
| ผล | เดินหน้าได้แค่ประมาณ 0.25 m/s ทุกคำสั่ง | ใช้ต่อได้ deploy เป็น `policy.onnx` |
| สาเหตุ (สันนิษฐาน) | delay สูงสุดยาวถึง 2 รอบของ policy policy จึงเลือกเดินช้าเพื่อให้เสถียร | — |

กลไก: actuator ทุกตัวถูกห่อด้วย `DelayedActuatorCfg` (`delay_target="position"`, `delay_hold_prob=1.0`)
และ event `action_delay` (`dr.sync_actuator_delays`) สุ่ม lag **ค่าเดียวต่อ env ใช้กับทุกข้อต่อ** ตอนเริ่มแต่ละ episode

## 6. สิ่งที่ยังไม่ได้ randomize

| รายการ | สถานะ | ผลที่อาจเกิดบนหุ่นจริง |
|---|---|---|
| Delay ของ observation (sensor latency) | ไม่มี มีแค่ action delay | ถ้าหุ่นจริงอ่าน sensor ช้า policy อาจไม่เคยเจอ |
| Delay ที่ต่างกันในแต่ละข้อต่อ หรือเปลี่ยนระหว่าง episode | ไม่มี (ค่าเดียวทั้งตัว คงที่ทั้ง episode) | jitter ของ DDS จริงไม่คงที่ |
| IMU bias / drift คงที่ต่อ episode | ไม่มี มีแค่ noise รายรอบ | |
| Motor armature | ค่าคงที่ | |
| ท่าเริ่มต้นของข้อต่อ | ช่วงค่าเป็น 0 | |
| ภูมิประเทศ | พื้นราบอย่างเดียว (task Flat) | ไม่เคยเจอพื้นต่างระดับ |

## 7. ข้อสังเกตที่พบระหว่างตรวจ

- **comment ไม่ตรงกับโค้ด:** comment ของ `action_delay` ใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` ยังเขียนว่า "0-8 physics steps (0-40 ms)" แต่ค่าจริงคือ `G1_23DOF_MAX_ACTION_LAG = 4` (0–20 ms) ตรงกับ run ของ 3b
- **โหมด play ไม่ได้ปิด DR ของ Phase 2–3:** `play=True` ปิดแค่ push, observation noise และ curriculum หุ่นใน `scripts/play.py` จึงยังถูกสุ่มมวล, gain และ delay อยู่
- **DR แบบ `startup` สุ่มใหม่ตอน resume:** Phase 2 และ 3 resume มาจาก checkpoint หุ่น 1024 แบบจึงถูกสุ่มใหม่ ไม่ใช่ชุดเดิมของ phase ก่อน
- **ความหลากหลายขึ้นกับ `num_envs`:** ถ้าลด `num_envs` จำนวนหุ่นที่ต่างกันจะลดลงตาม
- **Phase 3a และ 3b ใช้ iteration ชุดเดียวกัน** (13000 → 15998) เพราะทั้งคู่ resume จาก Phase 2 `model_12999.pt` แยกกันด้วยชื่อ run เท่านั้น
- มวลหุ่นจริง 34.1–35.0 kg มาจาก comment ในโค้ด ยังไม่ได้ชั่งยืนยัน

## 8. ไฟล์ที่เกี่ยวข้อง

| ไฟล์ | เนื้อหา |
|---|---|
| `src/tasks/velocity/velocity_env_cfg.py` | DR พื้นฐาน (P1) และ observation noise |
| `src/tasks/velocity/config/g1_23dof/env_cfgs.py` | DR ของ Phase 2, 3 และ `G1_23DOF_MAX_ACTION_LAG` |
| `src/tasks/velocity/mdp/events.py` | `delayed_effort_limits` (ใช้กับ `DelayedActuator` ได้) |
| `src/assets/robots/unitree_g1/g1_23dof_constants.py` | Kp, Kd, effort limit ที่เป็นค่าฐาน |
| `run_phase2_train.sh`, `run_phase3_train.sh` | คำสั่ง resume ของแต่ละ phase |
| `logs/rsl_rl/g1_23dof_velocity/<run>/params/env.yaml` | ค่าที่ใช้จริงในแต่ละ run |
| `deploy/robots/g1_23dof/config/policy/velocity/v0/exported/` | policy ที่ export ของแต่ละ phase |
