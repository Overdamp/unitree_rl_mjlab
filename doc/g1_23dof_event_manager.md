# Event Manager — G1-23DoF Velocity

เอกสารนี้อธิบายว่า Event Manager ของ mjlab ทำงานอย่างไร (ใครเรียก, เมื่อไร, แบบไหน) และสรุปทุก event ที่ G1-23DoF ใช้ตอนเทรน พร้อมค่าที่ใช้จริง

เอกสารคู่กัน:
- [`domain_randomization_phases_g1_23dof.md`](domain_randomization_phases_g1_23dof.md) — DR แยกตาม phase และเหตุผล
- [`g1_23dof_dr_code_guide.md`](g1_23dof_dr_code_guide.md) — ตำแหน่งโค้ด (file:line) และวิธีปรับ
- [`g1_23dof_observation_manager.md`](g1_23dof_observation_manager.md), [`g1_23dof_action_manager.md`](g1_23dof_action_manager.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - `logs/rsl_rl/g1_23dof_velocity/2026-10-22_17-02-19/params/env.yaml` (Phase 3b, `num_envs: 1024`)
> - `src/tasks/velocity/velocity_env_cfg.py`, `src/tasks/velocity/config/g1_23dof/env_cfgs.py`, `src/tasks/velocity/mdp/events.py`
> - mjlab ที่ติดตั้ง: `managers/event_manager.py`, `envs/manager_based_rl_env.py`, `envs/mdp/events.py`, `envs/mdp/dr/*.py`

---

## 1. ภาพรวม

Event Manager มีหน้าที่**เปลี่ยนสภาพของ simulation** ในจังหวะที่กำหนด แบ่งงานได้ 2 กลุ่ม:

1. **Reset / รบกวนสถานะ**: วางหุ่นใหม่ตอนเริ่ม episode, ผลักหุ่นระหว่างเดิน
2. **Domain Randomization (DR)**: สุ่มค่าฟิสิกส์ (มวล, แรงเสียดทาน, gain, delay …) ให้แต่ละ env ต่างกัน เพื่อให้ policy ทนต่อความต่างระหว่าง sim กับหุ่นจริง

> Event Manager มี**เฉพาะตอนเทรน** ฝั่ง deploy (C++ `g1_ctrl`) และ sim2sim (`unitree_mujoco`) ไม่มี event และไม่มี DR

Event แต่ละตัวเรียกว่า **term** ประกอบด้วย:

```python
EventTermCfg(
  func=dr.body_mass,          # ฟังก์ชัน func(env, env_ids, **params)
  mode="startup",             # เมื่อไรจะเรียก
  params={...},               # argument ของ func
  interval_range_s=None,      # เฉพาะ mode="interval"
  is_global_time=False,       # เฉพาะ mode="interval"
  min_step_count_between_reset=0,  # เฉพาะ mode="reset"
)
```

---

## 2. Mode ทั้ง 4 แบบ

| mode | เรียกเมื่อ | ใครเรียก | env ที่โดน | ตัวอย่างใน 23 DoF |
|---|---|---|---|---|
| `startup` | **ครั้งเดียว**ตอนสร้าง env (`load_managers`) | `ManagerBasedRlEnv.load_managers()` | ทุก env (แต่ละ env สุ่มค่าของตัวเอง) | มวล, แรงเสียดทาน, PD gain, encoder bias |
| `reset` | ทุกครั้งที่ env ถูก reset (ล้ม, หมดเวลา 20 s) | `_reset_idx(env_ids)` | เฉพาะ env ที่ reset | วางตำแหน่งใหม่, สุ่ม action delay |
| `interval` | เป็นระยะตามเวลา สุ่มช่วงจาก `interval_range_s` | `step()` หลังคำนวณ reward | env ที่ตัวจับเวลาหมด | ผลักหุ่น |
| `step` | ทุก policy step | `step()` | ทุก env | (ไม่ได้ใช้) |

### 2.1 ข้อสำคัญของ `startup`

- ค่าที่สุ่มตอน startup **คงที่ตลอดการเทรนครั้งนั้น** และไม่สุ่มใหม่ตอน reset
- run P3b มี `num_envs = 1024` จึงมี "หุ่น 1024 ตัวที่ฟิสิกส์ต่างกันแบบตายตัว" เทรนพร้อมกัน
- ถ้า resume จาก checkpoint (เช่น P2 ต่อจาก P1) env ถูกสร้างใหม่ ค่า startup จึงถูกสุ่มใหม่ทั้งชุด
- ถ้าต้องการให้ค่าเปลี่ยนทุก episode ให้ใช้ `mode="reset"` แต่ term ที่ต้อง recompute (เช่น `body_mass`) จะทำให้ reset ช้าลง (ดู 2.3)

### 2.2 กลไกของ `interval` (เช่น push)

```
ตอน reset env:      time_left[env] = U(5.0, 6.0) s
ทุก step (0.02 s):  time_left -= 0.02
ถ้า time_left < 0:  เรียก func กับ env นั้น แล้วสุ่ม time_left ใหม่
```

- `is_global_time=False` (ค่าที่ใช้): แต่ละ env มีตัวจับเวลาของตัวเอง และนับใหม่ทุก episode
- episode ยาว 20 s → ถูกผลักประมาณ **3 ครั้งต่อ episode** ถ้าไม่ล้มก่อน

### 2.3 `requires_model_fields` และ recompute

ใน MuJoCo-Warp ค่าใน model (เช่น `body_mass`) ปกติใช้ร่วมกันทุก env ฟังก์ชัน DR จึงต้องประกาศด้วย decorator:

```python
@requires_model_fields("body_mass", recompute=RecomputeLevel.set_const)
def body_mass(env, env_ids, ...): ...
```

- **fields**: EventManager รวบรวมไว้ใน `domain_randomization_fields` แล้ว sim จะ**ขยาย field นั้นให้มีค่าแยกต่อ env** ถ้าไม่ประกาศ การเขียนค่าจะไปเปลี่ยนทุก env พร้อมกัน
- **recompute**: หลังเรียก term ใน mode หนึ่ง EventManager จะเรียก `sim.recompute_constants()` ครั้งเดียวตาม level ที่สูงที่สุดที่ถูกเรียก

| level | ใช้เมื่อแก้ | ค่าที่คำนวณใหม่ |
|---|---|---|
| `none` | friction, damping, gain, forcerange | — |
| `set_const_fixed` | `body_gravcomp` | `body_subtreemass` |
| `set_const_0` | armature, inertia, `body_pos/quat`, `qpos0` | `dof_invweight0`, `body_invweight0`, tendon |
| `set_const` | **`body_mass`, `body_ipos` (COM)** | ทุกอย่างข้างบน (แพงที่สุด) |

> ถ้าเขียน DR term เองที่แก้ค่าใน model ต้องใส่ `@requires_model_fields(...)` เสมอ ดูตัวอย่าง `delayed_effort_limits` ใน `src/tasks/velocity/mdp/events.py`

### 2.4 ลำดับใน 1 policy step (`ManagerBasedRlEnv.step`)

```
1. action_manager.process_action(a)
2. วน decimation 4 ครั้ง: apply_action → mj_step   (physics dt 0.005)
3. termination → reward
4. env ที่ต้อง reset → _reset_idx():
      curriculum → sim.reset → scene.reset → event "reset" → obs/action/... reset
5. sim.forward()
6. command_manager.compute
7. event "step" → event "interval"           ← push เกิดตรงนี้
8. observation_manager.compute
```

ผลที่ตามมา: push เกิด**หลัง**คำนวณ reward ของ step นั้น และก่อนคำนวณ obs ดังนั้น obs รอบถัดไป (เช่น `base_ang_vel`) จะเห็นผลของ push ทันที

ในแต่ละ mode event ทำงาน**ตามลำดับใน dict** เช่น `reset_base` ทำก่อน `reset_robot_joints` และก่อน `action_delay`

---

## 3. ตาราง Event ของ G1-23DoF (Phase 3b — policy ที่ deploy)

ทั้งหมด 13 term จาก `env.yaml` ของ run `2026-10-22_17-02-19`
คอลัมน์ "ที่มา": 🔴 = base config ใช้ร่วมทุกรุ่น (`velocity_env_cfg.py`) / 🟢 = เพิ่มเฉพาะ 23 DoF (`config/g1_23dof/env_cfgs.py`)

### 3.1 Reset และรบกวน

| term | mode | func | ค่า | ทำอะไร | ที่มา | phase |
|---|---|---|---|---|---|---|
| `reset_base` | reset | `reset_root_state_uniform` | x, y ∈ U(−0.5, 0.5) m; z = 0; yaw ∈ U(−3.14, 3.14) rad; ความเร็วเริ่ม = 0 | วางหุ่นที่ตำแหน่ง/ทิศสุ่มรอบจุดเริ่มของ env | 🔴 | P1+ |
| `reset_robot_joints` | reset | `reset_joints_by_offset` | position offset 0, velocity offset 0 (ทุก joint) | เริ่มทุก episode จากท่า default พอดี (ไม่สุ่มมุม) | 🔴 | P1+ |
| `push_robot` | interval 5–6 s | `push_by_setting_velocity` | **บวก** Δv ของฐาน: x, y ±0.5 m/s; z ±0.4 m/s; roll, pitch ±0.52 rad/s; yaw ±0.78 rad/s | จำลองโดนชน/สะดุด ให้ policy ฝึกรับแรงกระแทก | 🔴 | P1+ |

### 3.2 Domain Randomization — ตัวหุ่น (startup)

| term | func | target | operation | ช่วง | ค่า nominal → ช่วงจริง | เพื่อ | ที่มา | phase |
|---|---|---|---|---|---|---|---|---|
| `foot_friction` | `dr.geom_friction` | geom `left/right_foot1..7_collision` (14 ชิ้น) | abs, `shared_random=True` | 0.3 – 1.6 | 0.6 → 0.3–1.6 (ทุกชิ้นของ env ใช้ค่าเดียวกัน) | พื้นลื่น/หนืดต่างกัน | 🔴 (geom set 🟢) | P1+ |
| `encoder_bias` | `dr.encoder_bias` | ทุก joint (23) | add (สุ่มแยกต่อ joint) | ±0.015 rad (±0.86°) | ลบจาก action target (ดู action doc) | encoder zero คลาดเคลื่อน | 🔴 | P1+ |
| `base_com` | `dr.body_com_offset` | `torso_link` | add ต่อแกน | x, y, z ±0.05 m | ตำแหน่ง COM ของลำตัว | การกระจายน้ำหนักไม่ตรงโมเดล | 🔴 (body 🟢) | P1+ |
| `torso_mass_payload` | `dr.body_mass` | `torso_link` | add | −1.0 – +3.5 kg | มวลลำตัว ± ของ | แบตเตอรี่/อุปกรณ์เพิ่ม | 🟢 | P2+ |
| `limb_mass_scale` | `dr.body_mass` | ทุก body ยกเว้น `torso_link` | scale | ×0.90 – 1.10 | มวลรวมหุ่นประมาณ 28.7–38.0 kg (รวมกับ payload) | ชิ้นส่วนหนัก/เบากว่าโมเดล | 🟢 | P2+ |
| `joint_friction` | `dr.joint_friction` | ทุก joint | abs | 0.0 – 0.30 | `dof_frictionloss` | stiction ของเกียร์/4-bar | 🟢 | P2+ |
| `joint_damping` | `dr.joint_damping` | ทุก joint | abs | 0.0 – 0.10 | `dof_damping` (viscous) | ความหนืดเชิงกล | 🟢 | P2+ |

### 3.3 Domain Randomization — มอเตอร์

| term | mode | func | operation | ช่วง | ผล | ที่มา | phase |
|---|---|---|---|---|---|---|---|
| `motor_pd_gains` | startup | `dr.pd_gains` | scale (Kp, Kd แยกกัน, ต่อ actuator) | Kp ×0.85–1.15, Kd ×0.85–1.15 | เช่น เข่า Kp 99.1 → 84.2–114.0 | 🟢 | P2+ |
| `motor_effort_limits` | startup | `delayed_effort_limits` (🟢 local) | scale | ×0.85–1.05 | เช่น เข่า 139 → 118–146 Nm | 🟢 | P2+ |
| `action_delay` | **reset** | `dr.sync_actuator_delays` | lag เดียวทั้งตัวต่อ env ต่อ episode | `randint(0, 4)` physics step = 0, 5, 10, 15, 20 ms | หน่วง position target (ใช้คู่กับ `DelayedActuatorCfg`) | 🟢 | P3 |

- `action_delay` ทำงานได้เพราะ actuator ทุกตัวถูกห่อด้วย `DelayedActuatorCfg(delay_target="position", delay_max_lag=4, delay_hold_prob=1.0)` ใน `env_cfgs.py` ส่วน `hold_prob=1.0` ทำให้ lag ไม่เปลี่ยนเองระหว่าง episode และเปลี่ยนเฉพาะตอน event นี้
- `delayed_effort_limits` ต้องเขียนขึ้นเอง เพราะ `dr.effort_limits` ของ mjlab 1.2.0 ไม่รองรับ `DelayedActuator` แต่ทำงานเหมือนกันทุกอย่าง
- P3a ใช้ lag 0–8 (0–40 ms) แล้วเดินพัง (tag `phase3a-lag0-40ms-collapsed`) จึงลดเหลือ 0–4 ใน P3b

### 3.4 สรุป DR ตาม phase

| phase | run | event ที่ทำงาน |
|---|---|---|
| P1 | `2026-10-20_18-26-38` | reset_base, reset_robot_joints, push_robot, foot_friction, encoder_bias, base_com |
| P2 | `2026-10-21_11-41-28` | P1 + torso_mass_payload, limb_mass_scale, joint_friction, joint_damping, motor_pd_gains, motor_effort_limits |
| P3a | `2026-10-22_16-01-12` | P2 + action_delay (0–8 step) |
| P3b ✅ | `2026-10-22_17-02-19` | P2 + action_delay (0–4 step) → `policy.onnx` |

### 3.5 Event ตอน play (`play=True`)

| เปลี่ยน | ผล |
|---|---|
| ลบ `push_robot` | ไม่ผลักหุ่นระหว่างดู |
| เพิ่ม `randomize_terrain` (reset) | สุ่ม tile ของ terrain ตอน reset (งาน Flat เป็นพื้นเรียบ จึงแทบไม่มีผล) |
| DR startup ยังอยู่ทั้งหมด | ตอน play หุ่นแต่ละตัวยังมีมวล/gain/friction สุ่มอยู่ |

---

## 4. ความเชื่อมโยงกับ manager อื่น

| event | กระทบ manager | อย่างไร |
|---|---|---|
| `encoder_bias` | Action | `JointPositionAction.apply_actions` ลบ bias ออกจาก target |
| `action_delay` | Action (actuator) | target ที่ไปถึง PD ช้าไป 0–4 physics step |
| `push_robot` | Observation | `base_ang_vel`, `projected_gravity` ใน obs ถัดไปเปลี่ยนทันที |
| `reset_base` (yaw สุ่ม) | Command | heading command คำนวณจาก yaw ที่สุ่มใหม่ |
| `foot_friction`, mass, gains | Reward / Termination | ทางอ้อม: พฤติกรรมเปลี่ยน reward จึงเปลี่ยน |

---

## 5. จะเพิ่ม / แก้ event ต้องแก้ที่ไหน

แก้ใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` (🟢) เท่านั้น อย่าแก้ `velocity_env_cfg.py` (🔴) ให้ override ค่าของ base แทน

| ต้องการ | วิธี (ใน `unitree_g1_23dof_rough_env_cfg`) |
|---|---|
| ปรับช่วงของ event ใน base | `cfg.events["push_robot"].params["velocity_range"]["x"] = (-0.3, 0.3)` |
| ปรับความถี่ push | `cfg.events["push_robot"].interval_range_s = (8.0, 10.0)` |
| ปิด event | `cfg.events.pop("push_robot", None)` หรือ `cfg.events["encoder_bias"] = None` (manager จะข้าม term ที่เป็น `None`) |
| สุ่มมุม joint ตอนเริ่ม | `cfg.events["reset_robot_joints"].params["position_range"] = (-0.1, 0.1)` |
| ให้มวลเปลี่ยนทุก episode | เปลี่ยน `mode="reset"` (reset จะช้าลงเพราะ `set_const`) |
| เพิ่ม DR ใหม่ (มีใน mjlab) | `cfg.events["joint_armature"] = EventTermCfg(mode="startup", func=dr.joint_armature, params={...})` |
| เขียน DR เอง | สร้างฟังก์ชัน `func(env, env_ids, ...)` พร้อม `@requires_model_fields("<field>", recompute=...)` ไว้ในไฟล์เฉพาะ 23 DoF แล้วอ้างใน `env_cfgs.py` |
| ปรับช่วง action delay | `G1_23DOF_MAX_ACTION_LAG` (`env_cfgs.py:25`) ใช้ทั้งใน actuator wrapper และ event |

> ⚠️ `src/tasks/velocity/mdp/events.py` เป็นไฟล์ shared ของ task velocity (🔴) ตอนนี้มี `delayed_effort_limits` อยู่ ซึ่งเป็นการ**เพิ่ม**ฟังก์ชันใหม่ ไม่ได้แก้ของเดิม จึงไม่กระทบหุ่นรุ่นอื่น ถ้าจะเพิ่มอีกให้ทำแบบเดียวกัน (เพิ่มเท่านั้น ไม่แก้ฟังก์ชันที่มีอยู่)

### ข้อควรระวัง

- การเปลี่ยนช่วง DR **ไม่ต้องเปลี่ยน `deploy.yaml`** เพราะ DR ไม่มีฝั่ง deploy แต่ต้องเทรนต่อหรือเทรนใหม่จึงจะมีผล
- ขยายช่วง DR ทีละน้อย ตัวอย่างคือ P3a ที่ขยายเร็วเกินไปจนเดินพัง ให้ใช้ curriculum หรือ resume ทีละ phase แบบที่ทำอยู่
- `shared_random=True` ของ `foot_friction` ทำให้เท้าทุกชิ้นมีค่าเท่ากัน ถ้าปิด เท้าซ้ายกับขวาจะลื่นไม่เท่ากัน ซึ่งไม่สมจริงบนพื้นเดียวกัน
- ฟังก์ชัน DR ที่ใช้ `operation="scale"` คูณจากค่า **default ของ model** ไม่ได้คูณทบจากค่าที่สุ่มครั้งก่อน การเรียกซ้ำจึงไม่สะสม
