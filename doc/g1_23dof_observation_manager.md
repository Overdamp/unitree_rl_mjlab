# Observation Manager — G1-23DoF Velocity

เอกสารนี้อธิบายว่า Observation Manager ทำงานอย่างไร ทั้งฝั่ง **training** (Python, mjlab) และฝั่ง **deploy** (C++, `g1_ctrl`) พร้อมตารางค่าที่ policy ของ G1-23DoF สังเกตจริง

> **แหล่งที่ตรวจ (27/10/2026):**
> - config ที่บันทึกตอนเทรน: `logs/rsl_rl/g1_23dof_velocity/2026-10-22_17-02-19/params/env.yaml` (Phase 3b, task `Unitree-G1-23Dof-Flat`)
> - checkpoint `model_15998.pt`
> - `deploy/robots/g1_23dof/config/policy/velocity/v0/exported/policy.onnx` และ `deploy.yaml`
> - mjlab ที่ติดตั้งอยู่: `~/miniconda3/envs/unitree_rl_mjlab/lib/python3.11/site-packages/mjlab/managers/observation_manager.py`

---

## 1. ภาพรวม: Observation Manager คืออะไร

ใน env แบบ manager-based (`ManagerBasedRlEnv`) ทุก step ของ policy (50 Hz) env จะเรียก `observation_manager.compute()` เพื่อสร้าง vector input ให้ neural network

แทนที่จะเขียน obs เป็นก้อนเดียว เราจะประกาศเป็น **term** ย่อยๆ แล้วจัดกลุ่มเป็น **group**:

```
ObservationManager
├── group "actor"   → input ของ policy (ใช้ทั้งตอนเทรนและบนหุ่นจริง)  80 ค่า
│     ├── term base_ang_vel       (3)
│     ├── term projected_gravity  (3)
│     ├── ...
│     └── term actions            (23)
└── group "critic"  → input ของ value function (ใช้เฉพาะตอนเทรน)       95 ค่า
      ├── term ทั้งหมดของ actor
      └── term "privileged" เพิ่ม: base_lin_vel, foot_height, foot_contact ...
```

- **Term** = ฟังก์ชัน `func(env, **params) -> Tensor[num_envs, dim]` บวกกับการตั้งค่า noise/clip/scale/delay/history
- **Group** = ชุดของ term ที่นำมาต่อกัน (concatenate) ตามลำดับที่ประกาศใน dict แล้วได้ tensor เดียว
- **Asymmetric actor-critic:** critic เห็นข้อมูลที่หุ่นจริงวัดไม่ได้ (ความเร็วเชิงเส้นจริง, แรงสัมผัสเท้า) ช่วยให้เทรนเร็วขึ้น แต่ actor ต้องใช้แค่สิ่งที่มี sensor จริงรองรับ

---

## 2. Pipeline การประมวลผลของแต่ละ term (training)

จาก `ObservationManager.compute_group()` (`observation_manager.py:319-386`) แต่ละ term ผ่านขั้นตอนตามลำดับนี้:

```
func(env, **params)          ค่าดิบจาก sim
   │
   ▼  noise   (ถ้า group.enable_corruption=True)   obs = obs + U(n_min, n_max)
   ▼  clip    (ถ้ากำหนด)                          obs = clamp(obs, lo, hi)
   ▼  scale   (ถ้ากำหนด)                          obs = obs * scale
   ▼  NaN check (ถ้า nan_policy != disabled)
   ▼  delay   (ถ้า delay_max_lag > 0)              ส่งค่าจาก k step ก่อน
   ▼  history (ถ้า history_length > 0)             เก็บ N step ย้อนหลังต่อกัน
   │
   ▼
concatenate ทุก term ใน group  →  Tensor[num_envs, group_dim]
```

### 2.1 field ของ `ObservationTermCfg`

| field | ค่า default | ความหมาย |
|---|---|---|
| `func` | — | ฟังก์ชันที่คืน tensor `(num_envs, dim)` |
| `params` | `{}` | argument เพิ่มให้ `func` เช่น `sensor_name`, `command_name` |
| `noise` | `None` | `UniformNoiseCfg(n_min, n_max)` บวก noise แบบ uniform |
| `clip` | `None` | `(min, max)` |
| `scale` | `None` | float หรือ tuple ต่อมิติ |
| `delay_min_lag` / `delay_max_lag` | 0 / 0 | หน่วงข้อมูล (หน่วยเป็น policy step) สุ่ม lag ในช่วงนี้ |
| `delay_per_env`, `delay_hold_prob`, `delay_update_period`, `delay_per_env_phase` | | รายละเอียดการสุ่ม lag |
| `history_length` | 0 | จำนวน step ย้อนหลังที่เก็บ (0 = ไม่มี history) |
| `flatten_history_dim` | `True` | ต่อ history เป็น vector แบน |

### 2.2 field ของ `ObservationGroupCfg`

| field | ความหมาย |
|---|---|
| `terms` | dict ของ term — **ลำดับใน dict = ลำดับใน vector** |
| `concatenate_terms` | `True` = ต่อกันเป็น tensor เดียว |
| `enable_corruption` | `False` จะ**ลบ noise ของทุก term ใน group ทิ้ง** ตอนสร้าง manager (`observation_manager.py:430`) |
| `history_length` | ถ้าไม่ใช่ `None` จะ**ทับ** `history_length` ของทุก term |
| `nan_policy` | `disabled` / `warn` / `sanitize` / `error` |

> **ข้อควรระวัง:** base config ตั้ง `history_length=1` ระดับ group ซึ่งทับค่าของทุก term ให้เป็น 1 = เก็บแค่ step ปัจจุบัน ผลลัพธ์จึงเหมือนไม่มี history (env.yaml ของ run จะยังแสดง `history_length: 0` ที่ระดับ term เพราะบันทึก cfg ก่อนถูกทับ)

---

## 3. ตาราง Observation ของ G1-23DoF

### 3.1 Actor (policy input) — 80 ค่า

ยืนยันจาก: `policy.onnx` input `obs [1, 80]` และ metadata `observation_names = base_ang_vel,projected_gravity,command,phase,joint_pos,joint_vel,actions`, actor `obs_normalizer` shape `(1, 80)`

| # | index | term | dim | ฟังก์ชัน | ค่าคืออะไร | หน่วย | noise ตอนเทรน (uniform ±) | ค่าตอนยืนนิ่ง |
|---|---|---|---:|---|---|---|---|---|
| 1 | 0–2 | `base_ang_vel` | 3 | `mdp.builtin_sensor("robot/imu_ang_vel")` | ความเร็วเชิงมุมของ pelvis จาก gyro ที่ site `imu_in_pelvis` (body frame) | rad/s | 0.2 | ≈ 0 |
| 2 | 3–5 | `projected_gravity` | 3 | `mdp.projected_gravity` | เวกเตอร์แรงโน้มถ่วงใน body frame ของ pelvis บอกความเอียง | unit vector | 0.05 | ≈ (0, 0, -1) |
| 3 | 6–8 | `command` | 3 | `mdp.generated_commands("twist")` | คำสั่ง (vx, vy, ωz) | m/s, m/s, rad/s | — | (0, 0, 0) |
| 4 | 9–10 | `phase` | 2 | `src…mdp.phase(period=0.6)` | (sin, cos) ของ gait clock รอบละ 0.6 s; **เป็น 0 ถ้า ‖command‖ < 0.1** | — | — | (0, 0) |
| 5 | 11–33 | `joint_pos` | 23 | `mdp.joint_pos_rel` | `q − q_default` ของ 23 joint | rad | 0.01 | ≈ 0 |
| 6 | 34–56 | `joint_vel` | 23 | `mdp.joint_vel_rel` | `dq − 0` | rad/s | 1.5 | ≈ 0 |
| 7 | 57–79 | `actions` | 23 | `mdp.last_action` | action ดิบของ step ก่อน (ก่อนคูณ scale และบวก default) | — | — | ≈ 0 |

- ทุก term: `clip = None`, `scale = None`, ไม่มี delay, history = 1
- **Noise ใช้เฉพาะตอนเทรน** (`enable_corruption=True`) ตอน play (`play=True`) และบนหุ่นจริงจะไม่มี noise เพิ่ม
- `height_scan` ใน base config **ถูกลบออก** ในงาน Flat (`env_cfgs.py` `unitree_g1_23dof_flat_env_cfg`) เพราะพื้นเรียบไม่ต้องสแกน

#### ลำดับ joint (ใช้ใน index 11–33, 34–56, 57–79 และ action output)

ลำดับตาม MJCF `g1_23dof.xml` = ลำดับใน ONNX metadata `joint_names` = ลำดับใน `deploy.yaml`

| j | joint | default (rad) | j | joint | default (rad) |
|---:|---|---:|---:|---|---:|
| 0 | left_hip_pitch | -0.10 | 12 | waist_yaw | 0.00 |
| 1 | left_hip_roll | 0.00 | 13 | left_shoulder_pitch | 0.35 |
| 2 | left_hip_yaw | 0.00 | 14 | left_shoulder_roll | 0.18 |
| 3 | left_knee | 0.30 | 15 | left_shoulder_yaw | 0.00 |
| 4 | left_ankle_pitch | -0.20 | 16 | left_elbow | 0.87 |
| 5 | left_ankle_roll | 0.00 | 17 | left_wrist_roll | 0.00 |
| 6 | right_hip_pitch | -0.10 | 18 | right_shoulder_pitch | 0.35 |
| 7 | right_hip_roll | 0.00 | 19 | right_shoulder_roll | -0.18 |
| 8 | right_hip_yaw | 0.00 | 20 | right_shoulder_yaw | 0.00 |
| 9 | right_knee | 0.30 | 21 | right_elbow | 0.87 |
| 10 | right_ankle_pitch | -0.20 | 22 | right_wrist_roll | 0.00 |
| 11 | right_ankle_roll | 0.00 | | | |

ตัวอย่าง: `joint_pos` ของเข่าซ้าย = obs index 11 + 3 = **14**, `joint_vel` ของ waist = 34 + 12 = **46**

### 3.2 Critic (value input) — 95 ค่า

ยืนยันจาก critic `obs_normalizer` shape `(1, 95)` group นี้ `enable_corruption=False` (ไม่มี noise เลย)

| # | term | dim | ฟังก์ชัน | ค่าคืออะไร | มีบนหุ่นจริง? |
|---|---|---:|---|---|---|
| 1–7 | (เหมือน actor ทุก term แต่ไม่มี noise) | 80 | | | ✅ |
| 8 | `base_lin_vel` | 3 | `builtin_sensor("robot/imu_lin_vel")` | ความเร็วเชิงเส้นจริงของ pelvis (velocimeter) | ❌ ไม่มี sensor ตรง |
| 9 | `foot_height` | 2 | `src…mdp.foot_height` | ความสูง z ของ site `left_foot`, `right_foot` ใน world | ❌ |
| 10 | `foot_air_time` | 2 | `src…mdp.foot_air_time` | เวลาที่เท้าลอยอยู่ในอากาศ (s) | ❌ |
| 11 | `foot_contact` | 2 | `src…mdp.foot_contact` | เท้าแตะพื้นหรือไม่ (0/1) | ❌ |
| 12 | `foot_contact_forces` | 6 | `src…mdp.foot_contact_forces` | แรงสัมผัส 3 แกนต่อเท้า แปลงด้วย `sign(f)·log(1+|f|)` | ❌ |

รวม 80 + 3 + 2 + 2 + 2 + 6 = **95**

ข้อมูลเท้ามาจาก contact sensor `feet_ground_contact` (subtree ของ `left/right_ankle_roll_link` กับพื้น) ที่ประกาศใน `config/g1_23dof/env_cfgs.py`

---

## 4. Normalization

`agent.yaml` ตั้ง `obs_normalization: true` ทั้ง actor และ critic → rsl_rl เก็บค่า mean/std แบบ running แล้วทำ `(obs − mean) / std` ก่อนเข้า MLP

**ตอน export ONNX ตัว normalizer ถูกฝังไว้ในกราฟแล้ว** (node แรกคือ `Sub → Div → Gemm …`) ฝั่ง deploy จึงส่ง obs ดิบเข้า ONNX ได้เลย ห้าม normalize ซ้ำ

---

## 5. ฝั่ง Deploy (C++ `g1_ctrl`)

### 5.1 โครงสร้าง

| ส่วน | ไฟล์ | สถานะ |
|---|---|---|
| `ObservationManager` (C++) | `deploy/include/isaaclab/manager/observation_manager.h` | 🔴 shared ทุกรุ่น |
| term cfg + buffer history | `deploy/include/isaaclab/manager/manager_term_cfg.h` | 🔴 shared |
| ฟังก์ชัน term (`REGISTER_OBSERVATION`) | `deploy/include/isaaclab/envs/mdp/observations/observations.h` | 🔴 shared |
| อ่าน IMU/joint จาก LowState | `deploy/include/unitree_articulation.h` | 🔴 shared |
| รายการ term + ลำดับ | `deploy/robots/g1_23dof/config/policy/velocity/v0/params/deploy.yaml` | 🟢 เฉพาะ 23 DoF |
| term คีย์บอร์ด | `deploy/robots/g1_23dof/src/State_RLBase.cpp`, `include/keyboard_*.h` | 🟢 เฉพาะ 23 DoF |

การทำงาน:
1. อ่าน `observations:` ใน `deploy.yaml` **ตามลำดับใน YAML**
2. หาฟังก์ชันจากชื่อ term ใน registry (`REGISTER_OBSERVATION(name)`) ถ้าไม่มีจะ throw `Observation term '...' is not registered.`
3. ทุก step: เรียก `func` → เก็บใน buffer history → (clip/scale) → ต่อกันเป็น vector → ส่งเข้า ONNX
4. ฝั่ง C++ **ไม่มี noise** ไม่มี delay buffer

### 5.2 จับคู่ชื่อ term: training ↔ deploy

| index | training (mjlab) | deploy (`deploy.yaml`) | แหล่งข้อมูลบนหุ่นจริง / sim2sim |
|---|---|---|---|
| 0–2 | `base_ang_vel` | `base_ang_vel` | `LowState.imu_state.gyroscope` (IMU ใน pelvis) |
| 3–5 | `projected_gravity` | `projected_gravity` | คำนวณจาก `imu_state.quaternion`: `q⁻¹ · (0,0,-1)` |
| 6–8 | `command` | `velocity_commands` | จอย: `(ly, −lx, −rx)` clamp ตาม `commands.base_velocity.ranges` / หรือ `keyboard_velocity_commands` |
| 9–10 | `phase` | `gait_phase` (period 0.6) | นาฬิกาภายใน `env->global_phase` / หรือ `keyboard_gait_phase` |
| 11–33 | `joint_pos` | `joint_pos_rel` | `motor_state[joint_ids_map[j]].q − default_joint_pos[j]` |
| 34–56 | `joint_vel` | `joint_vel_rel` | `motor_state[joint_ids_map[j]].dq` |
| 57–79 | `actions` | `last_action` | action ดิบที่ ONNX ให้ในรอบก่อน |

`deploy.yaml` ตั้ง `scale: 1.0` และ `clip: null` ทุก term กับ `history_length: 1` ซึ่งตรงกับตอนเทรน

`joint_ids_map: [0..12, 15..19, 22..26]` แปลงลำดับ policy (0–22) เป็น SDK motor index (ดู `g1_23dof_lowcmd_safety_reference.md` หัวข้อ 2)

### 5.3 โหมดคีย์บอร์ด

เมื่อเปิด keyboard shortcuts `State_RLBase.cpp` จะเปลี่ยนชื่อ term ใน YAML **โดยคงตำแหน่งเดิม** (`keyboard_observation_config.h`):
- `velocity_commands` → `keyboard_velocity_commands`: w 0.4 / s −0.3 / a,d ±0.2 / q,e ±0.3 แล้ว smooth เข้าหาเป้าทีละ 15% ต่อ step
- `gait_phase` → `keyboard_gait_phase`: ใช้คำสั่งตัวเดียวกับข้างบน เพื่อให้เงื่อนไข ‖cmd‖ < 0.1 ตรงกัน

> ห้ามใช้วิธี "ลบ term แล้ว append ใหม่" เพราะจะทำให้ลำดับ obs เพี้ยนทั้ง vector

---

## 6. ความต่างระหว่าง training กับ deploy ที่ควรรู้ (sim2real gap)

| ประเด็น | training | deploy | ผลกระทบ / ข้อแนะนำ |
|---|---|---|---|
| Noise | uniform ตามตาราง 3.1 | ไม่มี (ใช้ noise จริงของ sensor) | ปกติ — noise ตอนเทรนมีไว้ให้ policy ทน noise จริง |
| ช่วง command | `lin_vel_x [-1, 2]`, `lin_vel_y [-1, 1]`, `ang_vel_z [-1, 1]` (ค่าตั้งต้นใน env.yaml; heading control เปิด) | clamp `x [-0.5, 1.0]`, `y [-0.5, 0.5]`, `ωz [-1, 1]` | deploy แคบกว่า = ปลอดภัยกว่า; คีย์บอร์ดใช้ต่ำกว่านั้นอีก |
| Phase reset | `episode_length_buf · dt` เริ่ม 0 ทุก episode | `global_phase` สะสมตลอด (reset ตอน env reset) | phase ไม่ต้องตรงกับเวลาจริง เพราะ policy เห็นแค่ sin/cos |
| Encoder bias DR | obs ใช้ `joint_pos` จริง (`biased=False`) bias ถูกลบจาก **action target** แทน | ไม่มี bias ชดเชย | จำลองว่า "zero ของมอเตอร์คลาดเคลื่อน" ±ค่าใน DR |
| Action lag DR | Phase 3b สุ่ม 0–4 physics step (0–20 ms) ใน **actuator** ไม่ใช่ใน obs | delay จริงของ DDS + compute | obs term เองไม่มี `delay_*` |
| Linear velocity | critic เห็น | ไม่มี | actor ไม่ได้ใช้ จึงไม่ต้องประมาณค่า |

---

## 7. จะแก้ / เพิ่ม observation ต้องแก้ที่ไหน

> ⚠️ **การเปลี่ยน actor obs ทุกอย่าง (เพิ่ม/ลบ/สลับลำดับ/เปลี่ยน scale) = ต้องเทรนใหม่ตั้งแต่ต้น** และต้องแก้ `deploy.yaml` ให้ลำดับตรงกันทุก term เพราะ ONNX รับ vector 80 ค่าตามตำแหน่ง ไม่รู้ชื่อ term

| ต้องการ | แก้ที่ไหน (🟢 เท่านั้น) | ตัวอย่าง |
|---|---|---|
| เปลี่ยน noise ของ term | `config/g1_23dof/env_cfgs.py` | `cfg.observations["actor"].terms["joint_vel"].noise = Unoise(n_min=-1.0, n_max=1.0)` |
| ใส่ delay ให้ IMU | `config/g1_23dof/env_cfgs.py` | `t = cfg.observations["actor"].terms["base_ang_vel"]; t.delay_min_lag = 0; t.delay_max_lag = 1` |
| ใส่ history 4 step | `config/g1_23dof/env_cfgs.py` + `deploy.yaml` | `cfg.observations["actor"].history_length = 4` แล้วใน deploy ตั้ง `history_length: 4` ทุก term (actor จะเป็น 320 ค่า; ตรวจลำดับการเรียง history ให้ตรงกับ `use_gym_history`) |
| ลบ term | `config/g1_23dof/env_cfgs.py` + `deploy.yaml` | `del cfg.observations["actor"].terms["phase"]` และลบ `gait_phase` ใน deploy |
| เพิ่ม term ใหม่ | ฟังก์ชันไว้ใน `config/g1_23dof/` (ไฟล์ใหม่), เพิ่ม term ใน `env_cfgs.py`; ฝั่ง C++ `REGISTER_OBSERVATION` ไว้ใน `deploy/robots/g1_23dof/src/` | ดูตัวอย่าง `keyboard_velocity_commands` ใน `State_RLBase.cpp` |
| ปิด noise ตอน play | มีแล้วใน `env_cfgs.py` (`play=True` → `enable_corruption = False`) | |

**ห้ามแก้** (🔴 shared): `src/tasks/velocity/velocity_env_cfg.py` (actor/critic terms ฐานของทุกรุ่น), `src/tasks/velocity/mdp/observations.py`, `deploy/include/**`, และ mjlab ใน site-packages ให้ override ผ่าน `cfg.observations[...]` ใน `config/g1_23dof/env_cfgs.py` แทน

> หมายเหตุ: commit `233aac8` เคยแก้ `deploy/include/.../observations.h` (shared) โดยแยก `gait_phase_from_command()` ออกมาให้ term คีย์บอร์ดเรียกใช้ การแก้นี้เป็น refactor ที่ไม่เปลี่ยนผลลัพธ์ของ `gait_phase` เดิม จึงไม่กระทบหุ่นรุ่นอื่น

---

## 8. Checklist ตรวจ obs ก่อน sim2sim / sim2real

- [ ] ONNX input = 80 และ `observation_names` ใน metadata ตรงกับลำดับใน `deploy.yaml`
- [ ] `default_joint_pos` ใน `deploy.yaml` = metadata `default_joint_pos` ของ ONNX
- [ ] ยืนนิ่งท่า default: `projected_gravity ≈ (0,0,-1)`, `joint_pos_rel ≈ 0`, `joint_vel ≈ 0`
- [ ] เอียงหุ่นไปข้างหน้า → `projected_gravity[0]` เปลี่ยนเครื่องหมายเดียวกับใน sim
- [ ] กด w → obs[6] ≈ 0.4 และ obs[9..10] ไม่ใช่ (0,0)
- [ ] `joint_ids_map` ตรงกับ motor index ของเครื่องจริง (mode_machine ของรุ่นที่ใช้)
