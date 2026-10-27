# Command Manager — G1-23DoF Velocity

เอกสารนี้อธิบายว่า Command Manager ของ mjlab สร้าง "คำสั่งความเร็ว" ให้ policy อย่างไรตอนเทรน, คำสั่งถูกใช้ที่ไหนบ้าง (obs / reward) และฝั่ง deploy แปลงจอยหรือคีย์บอร์ดเป็นคำสั่งเดียวกันอย่างไร พร้อมตารางค่าของ G1-23DoF

เอกสารคู่กัน: [`g1_23dof_observation_manager.md`](g1_23dof_observation_manager.md), [`g1_23dof_reward_manager.md`](g1_23dof_reward_manager.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - `logs/rsl_rl/g1_23dof_velocity/2026-10-22_17-02-19/params/env.yaml` (Phase 3b)
> - `src/tasks/velocity/velocity_env_cfg.py`, `src/tasks/velocity/config/g1_23dof/env_cfgs.py`, `src/tasks/velocity/mdp/curriculums.py`
> - mjlab 1.2.0 ที่ใช้เทรนจริง (conda env `unitree_rl_mjlab`): `~/miniconda3/envs/unitree_rl_mjlab/lib/python3.11/site-packages/mjlab/managers/command_manager.py`, `mjlab/tasks/velocity/mdp/velocity_command.py`
> - deploy: `deploy/include/isaaclab/envs/mdp/observations/observations.h`, `deploy/robots/g1_23dof/src/State_RLBase.cpp`, `deploy/robots/g1_23dof/include/keyboard_velocity_command.h`, `deploy.yaml`

---

## 1. ภาพรวม

Policy เดินของเราเป็น **command-conditioned policy**: นอกจากสถานะของหุ่น policy ยังรับ "คำสั่ง" ว่าต้องการให้เดินอย่างไร แล้วเรียนรู้ที่จะทำตาม

```
คำสั่ง twist = (vx, vy, ωz)     ใน frame ของตัวหุ่น (body frame)
  vx  m/s    เดินหน้า (+) / ถอยหลัง (−)
  vy  m/s    เดินข้างซ้าย (+) / ขวา (−)
  ωz  rad/s  หมุนซ้าย/ทวนเข็ม (+) / หมุนขวา (−)
```

| ช่วงเวลา | ใครสร้างคำสั่ง |
|---|---|
| ตอนเทรน | `CommandManager` สุ่มคำสั่งให้แต่ละ env และเปลี่ยนเป็นระยะ |
| ตอน play (mjlab viewer) | สุ่มเหมือนตอนเทรน หรือใช้ slider "Twist" ใน Viser GUI |
| ตอน deploy (`g1_ctrl`) | จอย Unitree หรือคีย์บอร์ด (โหมด sim) |

---

## 2. การทำงานของ `CommandManager` และ `CommandTerm`

### 2.1 โครงสร้าง (`mjlab/managers/command_manager.py`)

`CommandManager` ถือ term หลายตัวได้ (dict) ใน velocity task มีตัวเดียวชื่อ **`twist`**

| เมธอด | เรียกเมื่อ | ทำอะไร |
|---|---|---|
| `compute(dt)` | ทุก policy step (หลัง reset, ก่อน event interval และ obs) | เรียก `term.compute(dt)` ของทุก term |
| `reset(env_ids)` | ตอน episode reset | log metrics แล้วสุ่มคำสั่งใหม่ทันที |
| `get_command("twist")` | obs, reward, curriculum | คืน tensor `(num_envs, 3)` |

### 2.2 วงจรของ `CommandTerm.compute(dt)`

```python
_update_metrics()                         # สะสม error ระหว่างคำสั่งกับความเร็วจริง
time_left -= dt
env ที่ time_left <= 0  → _resample()       # สุ่ม time_left ใหม่ + _resample_command()
_update_command()                          # ปรับคำสั่งทุก step (heading control, standing)
```

- **ตัวจับเวลาของแต่ละ env แยกกัน** ทุกครั้งที่ resample จะสุ่ม `time_left ~ U(3, 8)` s ใหม่
- **ตอน reset** สุ่มคำสั่งใหม่ทันทีเสมอ
- `command_counter` นับจำนวนครั้งที่ resample ใน episode นั้น

### 2.3 `UniformVelocityCommand` — สุ่มคำสั่งอย่างไร

ตอนเทรนใช้ class จาก **mjlab** (`mjlab.tasks.velocity.mdp.velocity_command`) เพราะ `velocity_env_cfg.py` import `UniformVelocityCommandCfg` มาจาก mjlab

**`_resample_command(env_ids)`** (ทุก 3–8 s และตอน reset):

| ขั้น | ค่า |
|---|---|
| 1 | `vx ~ U(lin_vel_x)`, `vy ~ U(lin_vel_y)`, `ωz ~ U(ang_vel_z)` |
| 2 | `heading_target ~ U(−π, π)` (ทิศที่ต้องการหันไป ใน world frame) |
| 3 | `is_heading_env` = สุ่มด้วยความน่าจะเป็น `rel_heading_envs` = **1.0 → ทุก env** |
| 4 | `is_standing_env` = สุ่มด้วยความน่าจะเป็น `rel_standing_envs` = **0.05 → 5% ของ env** |
| 5 | `init_velocity_prob = 0` → ไม่ตั้งความเร็วเริ่มต้นของหุ่นให้ตรงคำสั่ง |

**`_update_command()`** (ทุก step):

```python
heading_error = wrap_to_pi(heading_target − heading ปัจจุบันของหุ่น)
ωz = clip(0.5 × heading_error, −1.0, 1.0)     # สำหรับ heading env (= ทุก env)
ถ้าเป็น standing env:  (vx, vy, ωz) = (0, 0, 0)
```

> **ข้อสำคัญ:** เพราะ `rel_heading_envs = 1.0` ค่า `ωz` ที่สุ่มในขั้นที่ 1 **ถูกเขียนทับทุก step** ด้วย heading controller ดังนั้น ωz ที่ policy เห็นตอนเทรนเป็น "หมุนเข้าหาทิศเป้าหมาย" เสมอ: เริ่มจากค่าสูงสุด ±1.0 rad/s เมื่อ error มาก แล้วค่อยๆ ลดลงเป็น 0 แบบ exponential (time constant ≈ 1 / 0.5 = 2 s)

### 2.4 Metrics ที่ log

| metric | ความหมาย |
|---|---|
| `error_vel_xy` | ค่าเฉลี่ยของ ‖(cx, cy) − (vx, vy)‖ ต่อ episode (m/s) |
| `error_vel_yaw` | ค่าเฉลี่ยของ |cωz − ωz| ต่อ episode (rad/s) |

ค่าเหล่านี้ใช้ดูว่า policy ตามคำสั่งได้ดีแค่ไหน (ยิ่งต่ำยิ่งดี) metric สะสมโดยหารด้วยจำนวน step ของช่วงคำสั่งที่ยาวที่สุด (8 s) ไม่ใช่ความยาว episode ค่าที่ log จึงใช้เทียบระหว่าง run ได้ แต่ไม่ใช่ค่าเฉลี่ยจริงทางคณิตศาสตร์

---

## 3. ตาราง Command ของ G1-23DoF

### 3.1 ค่าตั้งของ term `twist`

| field | ค่า | ตั้งที่ | ความหมาย |
|---|---|---|---|
| class | `UniformVelocityCommandCfg` (mjlab) | `velocity_env_cfg.py` 🔴 | สุ่มแบบ uniform |
| `entity_name` | `robot` | 🔴 | |
| `resampling_time_range` | (3.0, 8.0) s | 🔴 | เปลี่ยนคำสั่งทุก 3–8 s (≈ 2–6 ครั้งต่อ episode 20 s) |
| `heading_command` | `True` | 🔴 | เปิด heading controller |
| `heading_control_stiffness` | 0.5 | 🔴 | ωz = 0.5 × heading error |
| `rel_heading_envs` | 1.0 | 🔴 (default) | **ทุก env ใช้ heading control** |
| `rel_standing_envs` | 0.05 | 🔴 | 5% ของ env สั่งให้ยืนนิ่ง |
| `init_velocity_prob` | 0.0 | 🔴 (default) | |
| `ranges.heading` | (−π, π) | 🔴 | |
| `viz.z_offset` | 1.15 m | `env_cfgs.py` 🟢 | ความสูงของลูกศรใน viewer |

### 3.2 ช่วงคำสั่งตาม curriculum (`command_vel`)

ช่วงใน `ranges` ถูกเขียนทับโดย curriculum ตาม `common_step_counter` (1 iteration = 24 env step):

| ช่วง | lin_vel_x (m/s) | lin_vel_y (m/s) | ang_vel_z (rad/s) | ใช้ตอน |
|---|---|---|---|---|
| ค่าใน cfg (ก่อน curriculum ทำงาน) | −1.0 … 2.0 | −1.0 … 1.0 | −1.0 … 1.0 | — |
| curriculum stage 0 (step > 0) | −0.5 … 1.0 | −0.5 … 0.5 | −1.0 … 1.0 | P1 iter 0 – 5000 |
| curriculum stage 1 (step > 120,000) | **−1.0 … 2.0** | **−1.0 … 1.0** | −1.0 … 1.0 | P1 iter 5000+, P2, P3a, **P3b** |
| play (Flat, `play=True`) | −0.5 … 1.0 | −0.5 … 0.5 | −0.5 … 0.5 | `scripts/play.py` (ไม่มี curriculum) |

- curriculum ทำงานใน `_reset_idx` (ตอนมี env reset) แล้วเปลี่ยน `cfg.ranges` ของทุก env พร้อมกัน
- `common_step_counter` ถูกเก็บใน checkpoint และกู้คืนตอน resume → P2/P3 อยู่ที่ stage 1 ตั้งแต่เริ่ม
- ช่วง ang_vel_z ใช้เป็น**ขอบของ clip** ใน heading controller (ค่าที่สุ่มเองไม่ได้ถูกใช้ ดู 2.3)

### 3.3 การกระจายของคำสั่งที่ policy เห็นจริง (P3b)

| ประเภท env | สัดส่วน | vx, vy | ωz |
|---|---|---|---|
| standing | ≈ 5% | 0, 0 | 0 |
| moving | ≈ 95% | U(−1, 2), U(−1, 1) | `clip(0.5 × heading_error, ±1)` → อิ่มตัวที่ ±1 เมื่อ error > 2 rad แล้วลดลงเข้าหา 0 |

ผลที่ตามมา:
- เกือบทุกช่วงคำสั่ง หุ่นต้อง "เดินพร้อมหมุน" ไปด้วยในช่วงแรก แล้วค่อยเดินตรง
- คำสั่งเดินตรงล้วน (ωz = 0 ตั้งแต่ต้น) ไม่ค่อยเกิดตอนเทรน แต่เป็นกรณีที่พบบ่อยที่สุดตอน deploy ถึงอย่างนั้นค่า ωz ≈ 0 ยังพบบ่อยในช่วงท้ายของแต่ละคำสั่ง จึงอยู่ในช่วงที่เทรนแล้ว
- ไม่มี deadzone: คำสั่งเล็กๆ เช่น vx = 0.05 ก็เกิดได้

### 3.4 Threshold "ยืน vs เดิน" ที่ใช้กับคำสั่ง

| ใช้ที่ | สูตรวัดขนาดคำสั่ง | threshold | ผล |
|---|---|---:|---|
| obs `phase` (`gait_phase`) | ‖(vx, vy, ωz)‖₂ | < 0.1 → phase = (0, 0) | สัญญาณจังหวะเดินหายไป |
| reward `foot_gait`, `foot_clearance`, `foot_slip`, `soft_landing` | ‖(vx, vy)‖ + |ωz| | > 0.1 → ทำงาน | |
| reward `stand_still` | ‖(vx, vy)‖ + |ωz| | ≤ 0.1 → ทำงาน | ลงโทษการขยับตอนยืน |
| reward `pose` std | ‖(vx, vy)‖ + |ωz| | < 0.1 ยืน / < 1.5 เดิน / ≥ 1.5 วิ่ง | เลือก std |

> สองสูตรต่างกันเล็กน้อย (L2 norm 3 แกน กับ L2 ของ xy บวก |ωz|) ทำให้ช่วงคำสั่งประมาณ 0.07–0.1 ถูกนับเป็น "ยืน" ในบางที่และ "เดิน" ในบางที่ ตอน deploy ควรส่งคำสั่งเป็น 0 พอดี หรือมากกว่า 0.1 ชัดเจน ซึ่งคีย์บอร์ดของ `g1_ctrl` ทำแบบนี้อยู่แล้ว (ค่าต่ำสุด 0.2)

### 3.5 คำสั่งถูกใช้ที่ไหนบ้าง

| ผู้ใช้ | term | ใช้อย่างไร |
|---|---|---|
| Observation | `command` (obs index 6–8) | ป้อนคำสั่งให้ policy ตรงๆ |
| Observation | `phase` (index 9–10) | ปิดเป็น 0 เมื่อคำสั่งเล็ก |
| Reward | `track_linear_velocity`, `track_angular_velocity` | เทียบกับความเร็วจริง |
| Reward | `pose`, `foot_*`, `soft_landing`, `stand_still` | ใช้ขนาดคำสั่งตัดสินโหมด |
| Curriculum | `command_vel` | ขยายช่วงคำสั่ง |

---

## 4. ฝั่ง Deploy: แหล่งคำสั่งของ `g1_ctrl`

ฝั่ง deploy ไม่มี CommandManager คำสั่งถูกสร้างใน**ฟังก์ชัน observation** โดยตรง และไม่มี heading controller

### 4.1 จอย Unitree (`velocity_commands`, 🔴 `observations.h`)

```cpp
obs[0] = clamp( joystick.ly, lin_vel_x)   // คันโยกซ้าย ขึ้น/ลง → vx
obs[1] = clamp(-joystick.lx, lin_vel_y)   // คันโยกซ้าย ซ้าย/ขวา → vy
obs[2] = clamp(-joystick.rx, ang_vel_z)   // คันโยกขวา ซ้าย/ขวา → ωz
```

ค่าจากคันโยกอยู่ในช่วง −1 … 1 แล้ว clamp ตามช่วงใน `deploy.yaml` → `commands.base_velocity.ranges`:

| แกน | input | ช่วงใน deploy.yaml | ค่าสูงสุดที่ได้จริงจากจอย | ช่วงตอนเทรน (P3b) |
|---|---|---|---|---|
| vx | ly | −0.5 … 1.0 m/s | −0.5 … **1.0** | −1.0 … 2.0 |
| vy | −lx | −0.5 … 0.5 m/s | −0.5 … 0.5 | −1.0 … 1.0 |
| ωz | −rx | −1.0 … 1.0 rad/s | −1.0 … 1.0 | −1.0 … 1.0 |

- คันโยกไม่มี scale: ดันสุด = 1.0 m/s ทันที
- ไม่มี deadzone และไม่มี smoothing: คันโยกที่ drift เล็กน้อยจะส่งคำสั่งเล็กๆ ทำให้ phase เป็น 0 (หุ่นยืน) ซึ่งปลอดภัย
- ช่วง deploy อยู่ในช่วงที่เทรนทั้งหมด

### 4.2 คีย์บอร์ด (`keyboard_velocity_commands`, 🟢 `State_RLBase.cpp` + `keyboard_velocity_command.h`)

ใช้เมื่อเปิด keyboard shortcuts (sim2sim)

| ปุ่ม | เป้าหมาย (vx, vy, ωz) |
|---|---|
| `w` | (0.4, 0, 0) |
| `s` | (−0.3, 0, 0) |
| `a` / `d` | (0, ±0.2, 0) |
| `q` / `e` | (0, 0, ±0.3) |
| `x` / space | (0, 0, 0) |

- คำสั่งค้างอยู่จนกดปุ่มอื่น (latch)
- smoothing: แต่ละ step ขยับเข้าหาเป้า 15% (`cur += (target − cur) × 0.15`) → ถึงประมาณ 95% ใน 18 step ≈ 0.37 s
- ตอนหยุด ถ้าค่าน้อยกว่า 0.01 จะ snap เป็น 0
- `keyboard_gait_phase` ใช้คำสั่งตัวเดียวกันเพื่อให้เงื่อนไข < 0.1 ตรงกับ obs คำสั่ง
- ทุกค่าเป้าหมาย ≥ 0.2 ซึ่งเกิน threshold 0.1 ชัดเจน

> **g1pilot `keyboard_teleop` ต่างจากนี้:** ตัวนั้นส่งคำสั่งผ่าน `loco_client` ไปยัง controller ระดับสูงของ Unitree ไม่ได้ป้อนเข้า policy RL นี้

---

## 5. ความต่างระหว่างเทรนกับ deploy

| ประเด็น | เทรน | deploy | ผล / ข้อแนะนำ |
|---|---|---|---|
| ωz | heading controller (ลดลงเองเข้าหา 0) | จอย/คีย์ตรงๆ (ค้างที่ค่าเดิม) | ωz คงที่นานๆ เป็นกรณีที่เทรนน้อยกว่า ให้ทดสอบหมุนต่อเนื่องใน sim2sim |
| การเปลี่ยนคำสั่ง | กระโดดทันทีทุก 3–8 s | จอย: ต่อเนื่อง / คีย์: smooth 15% | deploy นุ่มกว่า ซึ่งดี |
| ช่วง vx | −1 … 2 | −0.5 … 1.0 | deploy แคบกว่า = ปลอดภัย |
| standing | 5% ของ env, คำสั่ง 0 พอดี | ปล่อยคันโยก/กด x | ตรงกัน |
| frame | body frame | body frame | ตรงกัน |

---

## 6. จะแก้ command ต้องแก้ที่ไหน

แก้ใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` (🟢) เท่านั้น อย่าแก้ `velocity_env_cfg.py`, `mdp/curriculums.py`, `mdp/velocity_command.py` (🔴) หรือ mjlab

| ต้องการ | วิธี (ใน `unitree_g1_23dof_rough_env_cfg`) |
|---|---|
| เปลี่ยนช่วงความเร็วตอนเทรน | แก้ `velocity_stages` ของ curriculum: `cfg.curriculum["command_vel"].params["velocity_stages"] = [...]` (แก้แค่ `twist_cmd.ranges` จะถูก curriculum ทับ) |
| ปิด curriculum ความเร็ว | `cfg.curriculum.pop("command_vel")` แล้วตั้ง `twist_cmd.ranges.lin_vel_x = (...)` เอง |
| ให้ ωz สุ่มอิสระด้วย (ไม่ใช่ heading อย่างเดียว) | `twist_cmd.rel_heading_envs = 0.5` (ครึ่งหนึ่งใช้ ωz ที่สุ่ม) |
| เพิ่มสัดส่วนการฝึกยืนนิ่ง | `twist_cmd.rel_standing_envs = 0.1` |
| เปลี่ยนความถี่ในการเปลี่ยนคำสั่ง | `twist_cmd.resampling_time_range = (2.0, 6.0)` |
| เปลี่ยนช่วงตอน play | บล็อก `if play:` ใน `unitree_g1_23dof_flat_env_cfg` |
| เปลี่ยนช่วงตอน deploy | `deploy.yaml` → `commands.base_velocity.ranges` (ไม่ต้องเทรนใหม่ ถ้ายังอยู่ในช่วงที่เทรน) |
| เปลี่ยนความเร็วคีย์บอร์ด | `deploy/robots/g1_23dof/include/keyboard_velocity_command.h` (🟢) |

> ⚠️ `src/tasks/velocity/mdp/velocity_command.py` (สำเนาใน repo) มีบรรทัดที่ตั้งคำสั่งเป็น 0 เมื่อ ‖cmd‖ < 0.1 แต่ **ไม่ได้ถูกใช้ตอนเทรน** เพราะ config import class ของ mjlab ไฟล์นี้มาจาก upstream (commit `867434c`) ถ้าต้องการพฤติกรรมนั้น ให้สร้าง command class ของ 23 DoF เองใน `config/g1_23dof/` แทนการแก้ไฟล์ shared
