# Action Manager — G1-23DoF Velocity

เอกสารนี้อธิบายว่า Action Manager แปลง output ของ policy เป็นคำสั่งมอเตอร์อย่างไร ทั้งฝั่ง **training** (Python, mjlab) และ **deploy** (C++, `g1_ctrl`) พร้อมตารางค่า action ของ G1-23DoF ทุก joint

เอกสารคู่กัน: [`g1_23dof_observation_manager.md`](g1_23dof_observation_manager.md), [`g1_23dof_lowcmd_safety_reference.md`](g1_23dof_lowcmd_safety_reference.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - `logs/rsl_rl/g1_23dof_velocity/2026-10-22_17-02-19/params/env.yaml` (Phase 3b)
> - metadata ใน `policy.onnx` (output `actions [1, 23]`)
> - `src/assets/robots/unitree_g1/g1_23dof_constants.py`
> - `src/tasks/velocity/config/g1_23dof/env_cfgs.py`
> - mjlab ที่ติดตั้งอยู่: `managers/action_manager.py`, `envs/mdp/actions/actions.py`
> - `deploy/include/isaaclab/manager/action_manager.h`, `deploy/include/isaaclab/envs/mdp/actions/joint_actions.h`, `deploy/robots/g1_23dof/.../deploy.yaml`

---

## 1. ภาพรวม

Policy ของเราเป็น **joint position policy**: ทุก 20 ms (50 Hz) network ส่งตัวเลข 23 ค่า ซึ่ง**ไม่ใช่มุมโดยตรง** แต่เป็น "ส่วนเบี่ยงจากท่ายืน default ในหน่วยที่ normalize แล้ว" Action Manager แปลงค่านี้เป็น **position target** ของแต่ละ joint จากนั้น PD controller ของมอเตอร์ (ใน sim คือ MuJoCo position actuator, บนหุ่นจริงคือ motor driver) แปลงเป็นแรงบิด

```
policy MLP ──► a (23 ค่า, raw, ไม่ถูก clip)
                 │
     ActionManager.process_action(a)        ← 1 ครั้ง / policy step (50 Hz)
                 │   q_target = a * scale + q_default
                 ▼
     ActionManager.apply_action()           ← ทุก physics step (200 Hz ใน sim)
                 │   (sim) q_target − encoder_bias → actuator (มี delay 0–4 step)
                 ▼
   PD ในมอเตอร์:  τ = Kp (q_target − q) + Kd (0 − dq)   แล้ว clamp ที่ effort limit
```

ใน env ตอนนี้มี action term เดียวชื่อ `joint_pos` (`JointPositionActionCfg`) คุมทั้ง 23 joint

---

## 2. ฝั่ง Training (mjlab)

### 2.1 `ActionManager` (`mjlab/managers/action_manager.py`)

| เมธอด / property | เรียกเมื่อ | ทำอะไร |
|---|---|---|
| `process_action(action)` | 1 ครั้งต่อ policy step | เลื่อน history `prev_prev ← prev ← action ← ค่าใหม่` แล้วตัด vector เป็นช่วงๆ ส่งให้แต่ละ term (`term.process_actions`) |
| `apply_action()` | **ทุก physics substep** (decimation = 4 ครั้ง/step) | ให้แต่ละ term เขียน target ลงใน sim |
| `action` | | action ดิบของ step ปัจจุบัน (ก่อน scale/offset) → obs `actions` ใช้ค่านี้ |
| `prev_action`, `prev_prev_action` | | ใช้ใน reward เช่น `action_rate_l2` |
| `reset(env_ids)` | ตอน episode reset | ตั้ง action และ history เป็น 0 |
| `total_action_dim` | | ผลรวม dim ของทุก term = **23** |

ถ้าขนาด action ไม่ตรง (`total_action_dim != action.shape[1]`) จะ raise `ValueError` ทันที

### 2.2 `JointPositionAction` (`mjlab/envs/mdp/actions/actions.py`)

```python
# process_actions (1 ครั้ง/step)
self._raw_actions[:] = actions
self._processed_actions = self._raw_actions * self._scale + self._offset

# apply_actions (ทุก physics step)
target = self._processed_actions - encoder_bias
self._entity.set_joint_position_target(target, joint_ids=self._target_ids)
```

| field ใน cfg | ค่าที่ G1-23DoF ใช้ | ความหมาย |
|---|---|---|
| `entity_name` | `"robot"` | entity ที่คุม |
| `actuator_names` | `(".*",)` | ทุก actuator → 23 joint เรียงตาม MJCF |
| `scale` | `G1_23DOF_ACTION_SCALE` (dict ต่อ joint) | ตั้งใน `config/g1_23dof/env_cfgs.py`; base config ใช้ 0.25 |
| `use_default_offset` | `True` | offset = `default_joint_pos` (ท่า `HOME_KEYFRAME`) |
| `clip` | `None` | field นี้มีใน `ActionTermCfg` แต่ `JointPositionAction` ของ mjlab ที่ติดตั้งอยู่**ไม่ได้นำไปใช้** (ไม่มีโค้ดอ่าน `cfg.clip`) |
| `preserve_order` | `False` | ใช้ลำดับ joint ตาม MJCF |

การ clip ที่ใช้งานได้จริงตอนเทรนคือ `clip_actions` ใน runner cfg (`RslRlVecEnvWrapper` clamp raw action เป็น ±ค่า) ตอนนี้ `agent.yaml` ตั้งเป็น `clip_actions: null` → **ไม่มีการ clip action เลย**

### 2.3 ที่มาของ action scale

คำนวณใน `g1_23dof_constants.py` (บรรทัดท้ายไฟล์):

```python
G1_23DOF_ACTION_SCALE[n] = 0.25 * effort_limit / stiffness
```

**ความหมาย:** ถ้า action = ±1 หุ่นที่ยังอยู่ท่า default จะได้ position error = scale ซึ่งสร้างแรงบิด `Kp · scale = 0.25 · effort_limit` หรือ 25% ของแรงบิดสูงสุด ถ้า |action| ≈ 4 แรงบิดจาก P-term อย่างเดียวจะเต็ม effort limit

Kp และ Kd มาจากโมเดล actuator ที่ออกแบบให้ natural frequency = 10 Hz และ damping ratio = 2:

```
Kp = armature · ω²            ω = 2π·10
Kd = 2 · ζ · armature · ω     ζ = 2
```

### 2.4 Actuator และ DR ที่กระทบ action (เฉพาะ 23 DoF)

| ขั้นตอน | ตั้งที่ | ค่า | phase |
|---|---|---|---|
| Actuator พื้นฐาน | `g1_23dof_constants.py` | `BuiltinPositionActuatorCfg` 4 กลุ่ม (ตาราง 3) | ทุก phase |
| Encoder bias | `velocity_env_cfg.py` event `encoder_bias` (🔴 shared) | ลบ bias ออกจาก target ใน `apply_actions` จำลองว่าตำแหน่ง zero ของ encoder คลาดเคลื่อน | P1+ |
| PD gains | `env_cfgs.py` `motor_pd_gains` | Kp, Kd × U(0.85, 1.15) | P2+ |
| Effort limit | `env_cfgs.py` `motor_effort_limits` | × U(0.85, 1.05) | P2+ |
| Action delay | `env_cfgs.py` `DelayedActuatorCfg` + event `action_delay` | หน่วง **position target** 0–4 physics step (0–20 ms) โดยใช้ lag เดียวทั้งตัวต่อ episode (`hold_prob=1.0`) | P3b |

ดูรายละเอียดของ DR ได้ใน `domain_randomization_phases_g1_23dof.md`

### 2.5 Reward ที่ควบคุมพฤติกรรมของ action

| reward | weight | ผล |
|---|---:|---|
| `action_rate_l2` | −0.05 | ลงโทษ ‖aₜ − aₜ₋₁‖² ทำให้ action เรียบ ไม่สั่น |
| `joint_pos_limits` | −10.0 | ลงโทษเมื่อเกิน soft limit (90% ของ range ใน XML; `soft_joint_pos_limit_factor=0.9`) |
| `joint_acc_l2` | −2.5e-7 | ลงโทษความเร่งของ joint |

Reward พวกนี้เป็น**แรงจูงใจ** ไม่ใช่ hard limit ถ้า policy ส่ง action สุดโต่ง ระบบก็ยังส่งต่อให้มอเตอร์

---

## 3. ตาราง Action ของ G1-23DoF

Output ของ ONNX: `actions [1, 23]` ลำดับตาม `joint_names` ใน metadata (= ลำดับใน MJCF)

**สูตร:** `q_target[j] = a[j] · scale[j] + default[j]`

| a idx | joint | SDK motor | actuator | default (rad) | scale (rad / unit a) | Kp (Nm/rad) | Kd (Nm·s/rad) | effort limit (Nm) | range ใน XML (rad) |
|---:|---|---:|---|---:|---:|---:|---:|---:|---|
| 0 | left_hip_pitch | 0 | 7520-14 | −0.10 | 0.5475 | 40.18 | 2.558 | 88 | −2.531 … 2.880 |
| 1 | left_hip_roll | 1 | 7520-22 | 0.00 | 0.3507 | 99.10 | 6.309 | 139 | −0.524 … 2.967 |
| 2 | left_hip_yaw | 2 | 7520-14 | 0.00 | 0.5475 | 40.18 | 2.558 | 88 | −2.758 … 2.758 |
| 3 | left_knee | 3 | 7520-22 | 0.30 | 0.3507 | 99.10 | 6.309 | 139 | −0.087 … 2.880 |
| 4 | left_ankle_pitch | 4 | 2×5020 | −0.20 | 0.4386 | 28.50 | 1.814 | 50 | −0.873 … 0.524 |
| 5 | left_ankle_roll | 5 | 2×5020 | 0.00 | 0.4386 | 28.50 | 1.814 | 50 | −0.262 … 0.262 |
| 6 | right_hip_pitch | 6 | 7520-14 | −0.10 | 0.5475 | 40.18 | 2.558 | 88 | −2.531 … 2.880 |
| 7 | right_hip_roll | 7 | 7520-22 | 0.00 | 0.3507 | 99.10 | 6.309 | 139 | −2.967 … 0.524 |
| 8 | right_hip_yaw | 8 | 7520-14 | 0.00 | 0.5475 | 40.18 | 2.558 | 88 | −2.758 … 2.758 |
| 9 | right_knee | 9 | 7520-22 | 0.30 | 0.3507 | 99.10 | 6.309 | 139 | −0.087 … 2.880 |
| 10 | right_ankle_pitch | 10 | 2×5020 | −0.20 | 0.4386 | 28.50 | 1.814 | 50 | −0.873 … 0.524 |
| 11 | right_ankle_roll | 11 | 2×5020 | 0.00 | 0.4386 | 28.50 | 1.814 | 50 | −0.262 … 0.262 |
| 12 | waist_yaw | 12 | 7520-14 | 0.00 | 0.5475 | 40.18 | 2.558 | 88 | −2.618 … 2.618 |
| 13 | left_shoulder_pitch | 15 | 5020 | 0.35 | 0.4386 | 14.25 | 0.907 | 25 | −3.089 … 2.670 |
| 14 | left_shoulder_roll | 16 | 5020 | 0.18 | 0.4386 | 14.25 | 0.907 | 25 | −1.588 … 2.252 |
| 15 | left_shoulder_yaw | 17 | 5020 | 0.00 | 0.4386 | 14.25 | 0.907 | 25 | −2.618 … 2.618 |
| 16 | left_elbow | 18 | 5020 | 0.87 | 0.4386 | 14.25 | 0.907 | 25 | −1.047 … 2.094 |
| 17 | left_wrist_roll | 19 | 5020 | 0.00 | 0.4386 | 14.25 | 0.907 | 25 | −1.972 … 1.972 |
| 18 | right_shoulder_pitch | 22 | 5020 | 0.35 | 0.4386 | 14.25 | 0.907 | 25 | −3.089 … 2.670 |
| 19 | right_shoulder_roll | 23 | 5020 | −0.18 | 0.4386 | 14.25 | 0.907 | 25 | −2.252 … 1.588 |
| 20 | right_shoulder_yaw | 24 | 5020 | 0.00 | 0.4386 | 14.25 | 0.907 | 25 | −2.618 … 2.618 |
| 21 | right_elbow | 25 | 5020 | 0.87 | 0.4386 | 14.25 | 0.907 | 25 | −1.047 … 2.094 |
| 22 | right_wrist_roll | 26 | 5020 | 0.00 | 0.4386 | 14.25 | 0.907 | 25 | −1.972 … 1.972 |

- ค่า Kp, Kd และ scale ตรงกับ metadata ของ `policy.onnx` (`joint_stiffness`, `joint_damping`, `action_scale`)
- Effort limit คือค่าที่ใช้ใน sim (ก่อน DR ×0.85–1.05) ไม่ใช่ limit ที่รับรองสำหรับหุ่นจริง
- ข้อเท้าเป็น 4-bar linkage ขับด้วย 5020 สองตัว sim จึงประมาณเป็น 1:1 และ Kp, Kd, effort ×2
- `ACTUATOR_4010` มีประกาศไว้ในไฟล์แต่ไม่ได้ใช้ เพราะ wrist roll ใช้กลุ่ม 5020

### 3.1 ตัวอย่างการแปลงค่า

| สถานการณ์ | a | q_target |
|---|---|---|
| ยืนท่า default | a = 0 | = default ทุก joint |
| เข่าซ้าย a[3] = +1.0 | 1.0 × 0.3507 + 0.30 | **0.651 rad** |
| สะโพก pitch ขวา a[6] = −2.0 | −2.0 × 0.5475 − 0.10 | **−1.195 rad** |
| ข้อเท้า roll ซ้าย a[5] = +1.0 | 0.4386 | **เกิน range ของ XML (0.262)** → ชน joint limit ใน sim |

ตัวอย่างสุดท้ายแสดงว่า**ไม่มีอะไรป้องกันไม่ให้ target เกิน range** มีแค่ reward `joint_pos_limits` เท่านั้น

---

## 4. ฝั่ง Deploy (C++ `g1_ctrl`)

### 4.1 โครงสร้าง

| ส่วน | ไฟล์ | สถานะ |
|---|---|---|
| `ActionManager`, `ActionTerm`, `REGISTER_ACTION` | `deploy/include/isaaclab/manager/action_manager.h` | 🔴 shared |
| `JointAction` / `JointPositionAction` | `deploy/include/isaaclab/envs/mdp/actions/joint_actions.h` | 🔴 shared |
| เรียก step ของ policy ที่ 50 Hz | `deploy/include/FSM/State_RLBase.h` (thread แยก) | 🔴 shared |
| `env->step()`: obs → ONNX → `process_action` | `deploy/include/isaaclab/envs/manager_based_rl_env.h` | 🔴 shared |
| เขียน `q` ลง LowCmd | `deploy/robots/g1_23dof/src/State_RLBase.cpp` `run()` | 🟢 |
| scale / offset / clip / gains | `deploy/robots/g1_23dof/config/policy/velocity/v0/params/deploy.yaml` | 🟢 |

### 4.2 Data flow บนหุ่น

```
policy thread (50 Hz, step_dt = 0.02)               FSM thread (1 kHz, CtrlFSM dt = 0.001)
──────────────────────────────────                  ─────────────────────────────────────
env->step():                                        State_RLBase::run():
  robot->update()        (อ่าน LowState)              a = action_manager->processed_actions()
  obs = observation_manager->compute()                for i in 0..22:
  a = alg->act(obs)      (ONNX)                         motor_cmd[joint_ids_map[i]].q = a[i]
  action_manager->process_action(a)
     q_target = a*scale + offset  (+clip ถ้ามี)
```

- ตอนเข้าสถานะ (`State_RLBase::enter`): ตั้ง Kp, Kd ของ**ทุก**ช่อง motor เป็น 0 ก่อน แล้วค่อยใส่ `stiffness`/`damping` จาก `deploy.yaml` ให้ 23 ช่องตาม `joint_ids_map` ส่วน `dq` และ `tau` เป็น 0
- FSM thread เขียน target ล่าสุดซ้ำทุก 1 ms (zero-order hold) ไม่มีการ interpolate ระหว่าง policy step ซึ่งตรงกับ sim ที่ใช้ target เดิมตลอด 4 substep
- ตอน `reset()`: `_processed_actions = offset` (ท่า default) ดังนั้นก่อน policy step แรก มอเตอร์จะถูกสั่งไปท่า default ไม่ใช่ 0 rad

### 4.3 ค่าใน `deploy.yaml` เทียบกับตอนเทรน

| field | training | deploy.yaml | ต่างกัน? |
|---|---|---|---|
| term | `joint_pos` (JointPositionAction) | `JointPositionAction` | ชื่อต่างแต่ทำงานเหมือนกัน |
| scale hip pitch/yaw, waist | 0.5475 | 0.55 | ปัดเศษ +0.45% |
| scale hip roll, knee | 0.3507 | 0.35 | −0.2% |
| scale ankle, arm | 0.4386 | 0.44 | +0.3% |
| offset | default_joint_pos | เท่ากัน | ไม่ต่าง |
| clip | None | `null` | ไม่ต่าง (ทั้งสองฝั่งไม่มี clip) |
| Kp / Kd | 40.179/2.558, 99.098/6.309, 28.501/1.814, 14.251/0.907 | 40.2/2.6, 99.1/6.3, 28.5/1.8, 14.3/0.9 | ปัดเศษ < 2% อยู่ในช่วง DR ±15% แล้ว |
| `arm_ema_alpha` | ไม่มี | ไม่ได้ตั้ง → 1.0 | ไม่มี filter |
| action delay | DR 0–20 ms | delay จริงของ DDS + compute | ควรวัดจริงว่าอยู่ในช่วง 0–20 ms |

> ความต่างจากการปัดเศษ scale และ gain เล็กกว่าช่วง DR มาก จึงไม่น่าเป็นปัญหา แต่ถ้าต้องการให้ตรงทุกหลัก ให้คัดลอกค่าจาก metadata ของ `policy.onnx` มาใส่

### 4.4 Option ที่ deploy มีแต่ training ไม่มี

`joint_actions.h` (แก้ใน `233aac8`) รองรับ `arm_ema_alpha`: กรอง low-pass ให้ joint ที่ SDK motor index ≥ 15 (แขน) ตามสูตร

```
q_arm = α · target + (1 − α) · q_arm_prev
```

- ค่า default α = 1.0 คือไม่กรอง ตอนนี้**ไม่มี** `deploy.yaml` ของรุ่นไหนตั้งค่านี้ หุ่นทุกรุ่นจึงทำงานเหมือน upstream
- ถ้าเปิดใช้ (α < 1) แขนจะตามคำสั่งช้ากว่าตอนเทรน ซึ่งเป็น sim2real gap ใหม่ ควรทดสอบใน sim2sim ก่อน
- เกณฑ์ "≥ 15 = แขน" ถูกเฉพาะกับ layout ของ G1 ถ้ารุ่นอื่นตั้ง α < 1 จะกรองผิด joint

---

## 5. ข้อควรระวังด้านความปลอดภัย (action)

| ประเด็น | สถานะตอนนี้ | แนะนำ |
|---|---|---|
| Clip ของ action / target | ไม่มีทั้ง train และ deploy | เพิ่ม `clip` ใน `deploy.yaml` (รูปแบบ `[[lo, hi], …]` 23 คู่ ใช้กับ **q_target** หลัง scale+offset) โดยให้ขอบอยู่ใน soft limit (90% ของ range ใน XML) เป็นอย่างน้อย; ไม่กระทบ policy ถ้าขอบกว้างกว่าที่ policy ใช้จริง |
| NaN / Inf จาก ONNX | ไม่มีการเช็ค | เพิ่มการเช็คใน `State_RLBase.cpp` (🟢) ก่อนเขียน `q` ถ้าพบให้เข้า Passive |
| Torque limit บนหุ่นจริง | ขึ้นกับ firmware | effort limit ของ sim ไม่ได้ถูกส่งไปที่หุ่น |
| Delay จริง | ยังไม่ได้วัด | ถ้าเกิน 20 ms จะออกนอกช่วงที่เทรน (P3a ที่ 40 ms เดินพัง) |

> การเพิ่ม `clip` ใน `deploy.yaml` ไม่ต้องแก้ไฟล์ shared เพราะ `joint_actions.h` อ่าน `clip` อยู่แล้ว

---

## 6. จะแก้ action ต้องแก้ที่ไหน

> ⚠️ การเปลี่ยน scale, offset (ท่า default), Kp/Kd หรือจำนวน joint **ต้องเทรนใหม่** และต้องแก้ `deploy.yaml` ให้ตรงกันทุกค่า

| ต้องการ | แก้ที่ไหน | หมายเหตุ |
|---|---|---|
| เปลี่ยน action scale | `config/g1_23dof/env_cfgs.py`: `joint_pos_action.scale = {...}` | อย่าแก้สูตรใน `g1_23dof_constants.py` (🟡 ใช้ร่วมกับ mimic) |
| เปลี่ยนท่า default | สร้าง `EntityCfg` ใหม่ใน `env_cfgs.py` ด้วย `init_state` อื่น | `HOME_KEYFRAME` อยู่ใน constants (🟡) |
| เปลี่ยน PD gains | override actuator ใน `env_cfgs.py` (แบบเดียวกับที่ห่อ `DelayedActuatorCfg`) | ต้องแก้ `stiffness`/`damping` ใน `deploy.yaml` ด้วย |
| ใส่ clip ตอนเทรน | `clip_actions=<ค่า>` ใน `config/g1_23dof/rl_cfg.py` (clamp raw action ±ค่า) | อย่าใช้ `cfg.actions[...].clip` เพราะ mjlab เวอร์ชันนี้ไม่ได้นำไปใช้ |
| ปรับช่วง action delay | `G1_23DOF_MAX_ACTION_LAG` ใน `env_cfgs.py:25` | 1 step = 5 ms |
| ใส่ clip / filter ตอน deploy | `deploy.yaml` (`clip`, `arm_ema_alpha`) | ไม่ต้องเทรนใหม่ แต่ควรทดสอบ sim2sim |

**ห้ามแก้** (🔴): `velocity_env_cfg.py` (action term ฐาน), mjlab ใน site-packages, `deploy/include/**`

---

## 7. Checklist ก่อน sim2sim / sim2real

- [ ] ONNX output = 23 และ `joint_names` ใน metadata ตรงกับลำดับใน `deploy.yaml`
- [ ] `scale`, `offset`, `stiffness`, `damping` ใน `deploy.yaml` ตรงกับ metadata (ต่างกันแค่การปัดเศษ)
- [ ] `joint_ids_map` = `[0..12, 15..19, 22..26]` ตรงกับ `mode_machine` ของเครื่องจริง
- [ ] ตอนเข้า Velocity ช่อง motor 13, 14, 20, 21, 27, 28 มี Kp = Kd = 0
- [ ] log raw action ช่วงแรก: |a| ส่วนใหญ่ควร < 2–3 ถ้าพุ่งเกิน 4 แปลว่า obs ผิด (ลำดับ, เครื่องหมาย, หน่วย)
- [ ] วัด latency จริง (LowState → LowCmd) ให้อยู่ใน 0–20 ms
