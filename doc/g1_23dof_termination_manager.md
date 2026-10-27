# Termination Manager — G1-23DoF Velocity

เอกสารนี้อธิบายว่า Termination Manager ของ mjlab ตัดสินว่า episode จบอย่างไร, ผลที่มีต่อ reward และ PPO, ตาราง termination ที่ G1-23DoF ใช้ตอนเทรน และ**เงื่อนไขหยุดฉุกเฉินฝั่ง deploy** (`g1_ctrl`) ซึ่งทำหน้าที่คล้ายกันบนหุ่นจริง

เอกสารคู่กัน: [`g1_23dof_reward_manager.md`](g1_23dof_reward_manager.md), [`g1_23dof_event_manager.md`](g1_23dof_event_manager.md), [`g1_23dof_lowcmd_safety_reference.md`](g1_23dof_lowcmd_safety_reference.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - `logs/rsl_rl/g1_23dof_velocity/2026-10-22_17-02-19/params/env.yaml` (Phase 3b)
> - `src/tasks/velocity/velocity_env_cfg.py`, `src/tasks/velocity/mdp/terminations.py`
> - mjlab ที่ติดตั้ง: `managers/termination_manager.py`, `envs/mdp/terminations.py`, `rl/vecenv_wrapper.py`
> - deploy: `deploy/include/FSM/FSMState.h`, `State_Passive.h`, `deploy/robots/g1_23dof/src/State_RLBase.cpp`, `deploy/robots/g1_23dof/config/config.yaml`, `unitree_sdk2/.../dds_wrapper/common/Subscription.h`

---

## 1. ภาพรวม

Termination term คือฟังก์ชันที่คืนค่า `bool` ต่อ env: `True` = episode ของ env นี้ต้องจบใน step นี้

```python
TerminationTermCfg(
  func=mdp.bad_orientation,              # func(env, **params) -> BoolTensor[num_envs]
  params={"limit_angle": math.radians(70.0)},
  time_out=False,                        # False = ล้มเหลว, True = หมดเวลา
)
```

term แบ่งเป็น 2 ประเภทตาม `time_out`:

| ประเภท | `time_out` | ความหมาย | ตัวอย่าง |
|---|---|---|---|
| **Truncation** (หมดเวลา) | `True` | episode ถูกตัดเพราะครบเวลา **ไม่ใช่ความผิดของ policy** | `time_out` |
| **Termination** (ล้มเหลว) | `False` | หุ่นอยู่ในสถานะที่ไม่ต้องการ (ล้ม) | `fell_over` |

การแยกสองแบบนี้สำคัญมาก ดูหัวข้อ 3

---

## 2. การทำงานของ `TerminationManager` (`mjlab/managers/termination_manager.py`)

### 2.1 `compute()` — เรียกทุก policy step

```python
truncated = False; terminated = False
for term in terms:
    value = term.func(env, **params)       # BoolTensor[num_envs]
    if term.time_out: truncated  |= value
    else:             terminated |= value
    term_dones[name] = value
return truncated | terminated              # = dones
```

| property | ความหมาย | ใครใช้ |
|---|---|---|
| `terminated` | ล้มเหลว (OR ของ term ที่ `time_out=False`) | reward `is_terminated`, PPO |
| `time_outs` | หมดเวลา (OR ของ term ที่ `time_out=True`) | PPO (bootstrap) |
| `dones` | `terminated \| time_outs` | env เลือกว่าจะ reset env ไหน |

- term ทุกตัวถูกคำนวณทุก step แม้ term ก่อนหน้าจะเป็น `True` แล้ว ดังนั้นถ้าสองเงื่อนไขเกิดพร้อมกันจะถูกนับทั้งคู่
- ตั้ง `cfg.terminations["x"] = None` จะข้าม term นั้น

### 2.2 `reset(env_ids)`

log `Episode_Termination/<term>` = **จำนวน env** (ไม่ใช่สัดส่วน) ที่จบด้วย term นั้นในชุดที่ reset รอบนี้ กราฟนี้ใน TensorBoard ใช้ดูว่าหุ่นล้มบ่อยแค่ไหน (`fell_over`) เทียบกับเดินครบเวลา (`time_out`)

### 2.3 ลำดับใน 1 policy step

```
1. process_action → physics × 4
2. termination_manager.compute()     ← ตรวจสถานะหลัง physics
3. reward_manager.compute()          ← is_terminated อ่านผลจากข้อ 2 (ล้ม = −200 × dt)
4. env ที่ dones → _reset_idx()       ← event "reset", curriculum, log
5. command / event interval → observation
```

env ที่ถูก reset จะส่ง obs ของ **สถานะเริ่มต้นใหม่** กลับไปให้ policy ไม่ใช่ obs ตอนล้ม

---

## 3. ทำไมต้องแยก time_out กับ terminated (สำหรับ PPO)

`RslRlVecEnvWrapper` ส่ง `dones = terminated | truncated` และใส่ `extras["time_outs"] = truncated` ให้ rsl_rl

| กรณี | PPO ทำอะไรกับค่าในอนาคต (value bootstrap) | ผลที่ policy เรียนรู้ |
|---|---|---|
| **terminated** (ล้ม) | ถือว่าอนาคต = 0 | การล้มเสียรางวัลในอนาคตทั้งหมด → **หลีกเลี่ยงการล้ม** |
| **time_out** (ครบ 20 s) | บวก `γ · V(s)` กลับเข้าไปใน reward ของ step สุดท้าย | ไม่ลงโทษการ "ตัดจบ" เพราะหุ่นยังทำงานได้ดีต่อ |

ถ้าตั้ง `time_out=False` ให้ `time_out` ผิด policy จะเข้าใจว่า "ใกล้ 20 s แล้วไม่มีอนาคต" ทำให้พฤติกรรมช่วงท้าย episode เพี้ยน ซึ่งบนหุ่นจริงไม่มีเวลาจำกัด

นอกจากนี้ termination ยังผูกกับ reward `is_terminated` (weight −200) ซึ่งลงโทษทันทีอีกชั้นหนึ่ง (ดู reward doc หัวข้อ 3.2)

---

## 4. ตาราง Termination ของ G1-23DoF (ตอนเทรน)

ทั้งหมด **2 term** กำหนดใน `velocity_env_cfg.py` (🔴 shared) และ 23 DoF ไม่ได้ override ค่าใดๆ ทั้ง P1, P2, P3a และ P3b ใช้ค่าเดียวกัน

| term | func | ประเภท | เงื่อนไข | พารามิเตอร์ | วัดที่ | ผล |
|---|---|---|---|---|---|---|
| `time_out` | `mdp.time_out` | truncation (`time_out=True`) | `episode_length_buf ≥ max_episode_length` | `episode_length_s = 20.0` → 1000 policy step (dt 0.02) | — | reset, ไม่โดน −200, PPO bootstrap |
| `fell_over` | `mdp.bad_orientation` | termination | `|acos(−g_z)| > limit_angle` | `limit_angle = 70°` (1.2217 rad) | `projected_gravity_b` ของ **root body = pelvis** | reset, `is_terminated` −200 × dt, อนาคต = 0 |

### 4.1 `bad_orientation` ละเอียด

```python
projected_gravity = asset.data.projected_gravity_b   # gravity ใน frame ของ pelvis
tilt = acos(-projected_gravity[:, 2])                 # มุมระหว่างแกน z ของ pelvis กับแนวดิ่ง
return tilt > limit_angle
```

- ยืนตรง: g_b ≈ (0, 0, −1) → tilt = 0°
- เอียง 70° ในทิศใดก็ได้ (หน้า/หลัง/ข้าง หรือผสม) → episode จบ
- **ไม่ได้ดูความสูง** ถ้าหุ่นนั่งยองหรือคุกเข่าโดยลำตัวยังตั้งตรง จะไม่ถูกนับว่าล้ม แต่ reward `pose`/tracking จะต่ำ

### 4.2 term ที่มีให้ใช้แต่ยังไม่ได้เปิด

| func | ที่อยู่ | ทำอะไร | ใช้เมื่อ |
|---|---|---|---|
| `root_height_below_minimum(minimum_height)` | mjlab `envs/mdp/terminations.py` | จบเมื่อ pelvis ต่ำกว่า h (world z) | กันการทรุด/คุกเข่า (ระวังบนพื้นขรุขระที่ z ของพื้นเปลี่ยน) |
| `nan_detection()` | mjlab | จบ env ที่ physics state เป็น NaN/Inf | กัน sim ระเบิด (ควรตั้ง `time_out=False`) |
| `illegal_contact(sensor_name, force_threshold)` | `src/tasks/velocity/mdp/terminations.py` (🔴) | จบเมื่อ contact sensor มีแรง > threshold | เช่น เข่า/มือ/ลำตัวแตะพื้น ต้องสร้าง `ContactSensorCfg` เพิ่ม |

---

## 5. ฝั่ง Deploy: เงื่อนไขที่ทำให้ `g1_ctrl` ออกจาก Velocity

บนหุ่นจริงไม่มี "episode" แต่ FSM ของ `g1_ctrl` มี `registered_checks` ซึ่งเป็นรายการ `(เงื่อนไข, state ปลายทาง)` ที่ถูกตรวจทุกรอบของ FSM (1 kHz) ถ้าเงื่อนไขใดเป็นจริง FSM จะเปลี่ยน state ทันที

### 5.1 ตารางเงื่อนไขออกจาก `Velocity` (G1-23DoF)

| # | เงื่อนไข | ไปที่ | ค่า | ตั้งที่ | สถานะไฟล์ |
|---|---|---|---|---|---|
| 1 | **เอียงเกิน** `torso_tilt_exceeds` | Passive | `|acos(−g_z)| > 0.85 rad` (≈ 48.7°) จาก IMU ใน pelvis | `deploy/robots/g1_23dof/src/State_RLBase.cpp` | 🟢 |
| 2 | **LowState ขาดหาย** `lowstate->isTimeout()` | Passive | ไม่ได้รับ LowState นานเกิน **1000 ms** (ค่า default ของ SDK `timeout_ms_{1000}`) | `deploy/include/FSM/FSMState.h` (ทุก state) | 🔴 |
| 3 | จอย `LT + B` | Passive | กดพร้อมกัน | `config.yaml` → `Velocity.transitions` | 🟢 |
| 4 | คีย์บอร์ด `p` | Passive | เฉพาะเมื่อเปิด keyboard shortcuts (sim) | `FSMState.h` | 🔴 |

### 5.2 เทียบกับตอนเทรน

| | ตอนเทรน (`fell_over`) | ตอน deploy (`torso_tilt_exceeds`) |
|---|---|---|
| สูตร | `|acos(−g_z)| > limit` | สูตรเดียวกัน |
| แหล่ง gravity | `projected_gravity_b` ของ pelvis (sim) | quaternion ของ IMU ใน pelvis (`imu_state`) |
| เกณฑ์ | **70°** | **48.7°** (0.85 rad) |
| ผล | reset episode | ตัดไป Passive (ปล่อยแรงมอเตอร์ ใช้แค่ damping) |

- เกณฑ์ตอน deploy **เข้มกว่า** ตอนเทรน ซึ่งเป็นทิศทางที่ปลอดภัย: หุ่นจะถูกตัดก่อนเอียงถึงจุดที่ policy เคยเจอว่าล้ม
- แต่ถ้าโดนผลักแรงจน**เอียงชั่วขณะเกิน 48.7°** แล้ว policy อาจจะกู้กลับได้ (ตอนเทรนเห็นได้ถึง 70°) deploy จะตัดก่อน ถ้าพบว่าตัดบ่อยเกินไปตอนทดสอบ ค่อยพิจารณาขยับเกณฑ์ ไม่ควรเกิน 70°
- ชื่อฟังก์ชันใช้คำว่า "torso" แต่ข้อมูลมาจาก **IMU ใน pelvis** (`unitree_articulation.h` อ่าน `imu_state`) ซึ่งตรงกับตอนเทรน

### 5.3 Passive ทำอะไร (`State_Passive.h`, 🔴)

ตอนเข้า Passive:
- **Kp = 0** ทุกมอเตอร์ (ไม่มีแรงดึงไปที่มุมใดเลย)
- **Kd** = ค่าจาก `config.yaml` (`FSM.Passive.kd`) ให้หุ่น "ทรุดลงช้าๆ" แทนการล้มอิสระ
- ทุกรอบตั้ง `q = q ที่วัดได้` (ไม่มีผลเพราะ Kp = 0)

| กลุ่ม (SDK index) | Kd ใน Passive |
|---|---|
| hip pitch/roll/yaw (0–2, 6–8) | 4.0 |
| knee (3, 9) | 6.0 |
| ankle (4–5, 10–11) | 2.5 |
| waist (12–14) | 4.5 (23 DoF ใช้แค่ 12) |
| แขน (15–28) | 2.5 |

> ⚠️ Passive **ไม่ได้ทำให้หุ่นยืนต่อ** หุ่นจะทรุดลงเพราะไม่มีแรงยึด ตอนทดสอบบนหุ่นจริงต้องมีเครนหรือสายแขวนรับเสมอ

### 5.4 สิ่งที่ฝั่ง deploy ยังไม่มี

ข้อมูลจาก `g1_23dof_lowcmd_safety_reference.md` หัวข้อ 5:
- ไม่เช็ค NaN/Inf ของ obs หรือ action ก่อนส่ง (ตอนเทรนมี `nan_to_num` ใน reward แต่ไม่มี termination)
- ไม่เช็ค joint position/velocity เกิน, อุณหภูมิมอเตอร์ หรือแรงดันแบตเตอรี่
- ไม่เช็คความสูงของลำตัว
- `isTimeout` 1000 ms ถือว่านาน: ถ้า DDS หลุด มอเตอร์จะค้างที่ target สุดท้ายประมาณ 1 วินาที ควรพิจารณาลดค่า (เช่น `lowstate->set_timeout_ms(100)`) ในไฟล์ 🟢 ของ 23 DoF

---

## 6. จะแก้ termination ต้องแก้ที่ไหน

### ตอนเทรน — แก้ใน `config/g1_23dof/env_cfgs.py` (🟢)

| ต้องการ | วิธี |
|---|---|
| เปลี่ยนมุมล้ม | `cfg.terminations["fell_over"].params["limit_angle"] = math.radians(60.0)` |
| เปลี่ยนความยาว episode | `cfg.episode_length_s = 20.0` (กระทบจำนวน push ต่อ episode และ curriculum) |
| เพิ่มเกณฑ์ความสูง | `cfg.terminations["base_too_low"] = TerminationTermCfg(func=envs_mdp.root_height_below_minimum, params={"minimum_height": 0.4})` |
| ใส่ NaN guard | `cfg.terminations["nan"] = TerminationTermCfg(func=envs_mdp.nan_detection)` |
| ปิด term | `cfg.terminations.pop("fell_over")` (ไม่แนะนำ: หุ่นจะนอนกับพื้นจนหมดเวลา) |

> การทำให้เกณฑ์ termination เข้มขึ้นทำให้ episode สั้นลงตอนต้นของการเทรน อาจทำให้เรียนรู้ช้า ให้ปรับทีละน้อยแล้วดู `Episode_Termination/*` ประกอบ

### ตอน deploy — แก้ใน `deploy/robots/g1_23dof/` (🟢) เท่านั้น

| ต้องการ | วิธี |
|---|---|
| เปลี่ยนเกณฑ์เอียง | แก้ `0.85f` ใน `torso_tilt_exceeds(env.get(), 0.85f)` (`State_RLBase.cpp`) |
| เพิ่ม safety check | `this->registered_checks.emplace_back(std::make_pair([&]()->bool{ ... }, FSMStringMap.right.at("Passive")));` ใน constructor ของ `State_RLBase.cpp` |
| เปลี่ยนปุ่มหยุด | `config.yaml` → `FSM.Velocity.transitions.Passive` |
| เปลี่ยน damping ของ Passive | `config.yaml` → `FSM.Passive.kd` |

**ห้ามแก้** (🔴): `deploy/include/isaaclab/envs/mdp/terminations.h` (`bad_orientation` ของ upstream ถูกปิดไว้สำหรับทุกรุ่น จึงต้องเช็คแบบ local แทน), `deploy/include/FSM/*.h`, `velocity_env_cfg.py`, mjlab ใน site-packages
