# Reward Manager — G1-23DoF Velocity

เอกสารนี้อธิบายว่า Reward Manager ของ mjlab คำนวณ reward อย่างไร และสรุปทุก reward term ที่ใช้เทรน policy เดินของ G1-23DoF พร้อมสูตร น้ำหนัก และพารามิเตอร์ที่ใช้จริง

เอกสารคู่กัน: [`g1_23dof_observation_manager.md`](g1_23dof_observation_manager.md), [`g1_23dof_action_manager.md`](g1_23dof_action_manager.md), [`g1_23dof_event_manager.md`](g1_23dof_event_manager.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - `logs/rsl_rl/g1_23dof_velocity/2026-10-22_17-02-19/params/env.yaml` (Phase 3b)
> - `src/tasks/velocity/velocity_env_cfg.py`, `src/tasks/velocity/config/g1_23dof/env_cfgs.py`
> - `src/tasks/velocity/mdp/rewards.py` และ `curriculums.py`
> - mjlab ที่ติดตั้ง: `managers/reward_manager.py`, `envs/mdp/rewards.py`

---

## 1. ภาพรวม

Reward คือสัญญาณที่บอก PPO ว่าพฤติกรรมไหนดีหรือไม่ดี ใน env แบบ manager-based เราไม่ได้เขียน reward เป็นฟังก์ชันก้อนเดียว แต่แยกเป็น **term** ย่อยๆ ที่มีน้ำหนักของตัวเอง:

```python
RewardTermCfg(
  func=mdp.track_linear_velocity,   # func(env, **params) -> Tensor[num_envs]
  weight=1.0,                       # + = รางวัล, − = บทลงโทษ
  params={"command_name": "twist", "std": 0.5},
)
```

- **weight > 0**: term เป็น "รางวัล" ฟังก์ชันส่วนใหญ่คืนค่าในช่วง 0–1 (เช่น `exp(−error)`)
- **weight < 0**: term เป็น "บทลงโทษ" ฟังก์ชันคืนค่า ≥ 0 (เช่น error²) แล้วคูณ weight ติดลบ
- **ฝั่ง deploy ไม่มี reward** ใช้เฉพาะตอนเทรน

---

## 2. การทำงานของ `RewardManager` (`mjlab/managers/reward_manager.py`)

### 2.1 `compute(dt)` — เรียกทุก policy step

```python
reward_buf = 0
for term in terms:
    if term.weight == 0: continue                    # weight 0 = ปิด term (ไม่คำนวณ)
    value = term.func(env, **params) * weight * dt   # scale_by_dt=True (ค่า default)
    value = nan_to_num(value)                        # NaN/Inf → 0 กัน policy พัง
    reward_buf += value
    episode_sums[name] += value
return reward_buf                                     # shape (num_envs,)
```

| ประเด็น | รายละเอียด |
|---|---|
| **คูณด้วย dt** | reward ต่อ step = `raw × weight × 0.02` ทำให้ผลรวมทั้ง episode ไม่ขึ้นกับความถี่ของ policy ดังนั้นให้มองว่า weight คือ "อัตราต่อวินาที" |
| **เรียกเมื่อไร** | ใน `env.step()` หลัง physics และ termination แต่**ก่อน** reset และก่อน event interval (push) |
| **NaN** | ถูกแทนด้วย 0 ทีละ term |
| **ปิด term** | ตั้ง `weight=0.0` (ยังอยู่ใน log) หรือ `cfg.rewards["x"] = None` / `pop` (ไม่สร้างเลย) |
| **Class-based term** | ถ้า `func` เป็น class (เช่น `variable_posture`) จะถูกสร้างครั้งเดียวตอนเริ่ม (precompute ค่าได้) และถูกเรียก `reset()` ตอน episode reset |

### 2.2 `reset(env_ids)` — ตอน episode จบ

- คืนค่า log `Episode_Reward/<term>` = ค่าเฉลี่ยของ `episode_sums` หารด้วยความยาว episode สูงสุด (20 s) แล้วล้างเป็น 0
- ค่านี้คือกราฟ `Episode_Reward/*` ใน TensorBoard มีหน่วยเป็น "reward ต่อวินาที เฉลี่ยตลอด episode" ค่าสูงสุดที่เป็นไปได้ของ term ที่เป็นรางวัลคือ `weight × (เวลาที่อยู่รอด / 20)`

### 2.3 Metrics ที่ term บางตัว log เพิ่ม

| metric | มาจาก | ใช้ดูอะไร |
|---|---|---|
| `Metrics/angular_momentum_mean` | `angular_momentum` | การแกว่งของทั้งตัว |
| `Metrics/slip_velocity_mean` | `foot_slip` | ความเร็วไถลเฉลี่ยของเท้าขณะแตะพื้น (m/s) |
| `Metrics/landing_force_mean` | `soft_landing` | แรงกระแทกเฉลี่ยตอนเท้าลง (N) |

---

## 3. ตาราง Reward ของ G1-23DoF (Phase 3b — policy ที่ deploy)

ทั้งหมด **16 term** ทุก phase (P1–P3b) ใช้ชุด reward เดียวกัน ที่ต่างกันมีแค่ DR

**เงื่อนไข "มีคำสั่ง"** ในตาราง: `‖cmd_xy‖ + |cmd_ωz| > 0.1` เรียกว่า **walking** ถ้า ≤ 0.1 เรียกว่า **standing**

### 3.1 รางวัล (weight > 0)

| # | term | weight | ค่าที่คืน (raw) | สูตร | พารามิเตอร์ | ทำงานเมื่อ | ที่มา |
|---|---|---:|---|---|---|---|---|
| 1 | `track_linear_velocity` | +1.0 | 0–1 | `exp(−[(vx−cx)² + (vy−cy)² + 2·vz²] / σ²)` ความเร็วของ pelvis ใน body frame | σ = 0.5 | ตลอด | 🔴 |
| 2 | `track_angular_velocity` | +1.0 | 0–1 | `exp(−[(ωz−cωz)² + 0.05·(ωx²+ωy²)] / σ²)` | σ = 0.707 | ตลอด | 🔴 |
| 3 | `pose` (`variable_posture`) | +1.0 | 0–1 | `exp(−mean_j[(q_j − q_default_j)² / std_j²])` std เลือกตามความเร็วของคำสั่ง (ตาราง 3.3) | walking ≥ 0.1, running ≥ 1.5 | ตลอด | 🔴 term / 🟢 std |
| 4 | `foot_gait` | +0.5 | 0–1 | สัดส่วนของเท้าที่สถานะ "แตะพื้นจริง" ตรงกับ "ควรแตะตาม clock" | period 0.6 s, offset [0, 0.5], threshold 0.56 | walking | 🔴 |

**`foot_gait` ละเอียด:** แต่ละขามี phase = `(t / 0.6 + offset) mod 1` ถ้า phase < 0.56 ควรอยู่บนพื้น (stance)
- ขาซ้าย offset 0, ขาขวา offset 0.5 ทำให้สลับกันครึ่งรอบ
- stance 56% ของรอบ ทำให้มีช่วงสองเท้าแตะพื้นพร้อมกัน (double support) 2 × 6% = 12% ของรอบ
- 1 รอบ = 0.6 s ทำให้ก้าวละ 0.3 s หรือประมาณ 3.3 ก้าวต่อวินาที
- clock เดียวกันนี้ป้อนเป็น obs `phase` (sin/cos, period 0.6) ให้ policy รู้จังหวะ

### 3.2 บทลงโทษ (weight < 0)

| # | term | weight | ค่าที่คืน (raw ≥ 0) | สูตร | พารามิเตอร์ | ทำงานเมื่อ | ที่มา |
|---|---|---:|---|---|---|---|---|
| 5 | `is_terminated` | **−200** | 0 / 1 | 1 ถ้า episode จบเพราะล้ม (ไม่นับหมดเวลา) | — | ตอนล้ม | 🔴 |
| 6 | `joint_pos_limits` | −10 | rad | Σ ระยะที่เกิน soft limit (90% ของ range ใน XML) | — | ตลอด | 🔴 |
| 7 | `body_orientation_l2` | −1.0 | 0–1 | `gx² + gy²` ของ gravity ใน frame ของ **torso_link** (= sin²(มุมเอียง)) | body = torso_link | ตลอด | 🔴 / 🟢 body |
| 8 | `foot_clearance` | −1.0 | m·m/s | Σ_เท้า `|z_foot − 0.10| × ‖v_foot,xy‖` (ลงโทษเฉพาะเท้าที่กำลังเคลื่อน) | target 0.10 m | walking | 🔴 / 🟢 site |
| 9 | `stand_still` | −1.0 | rad² | Σ_j (q_j − q_default_j)² | threshold 0.1 | **standing** | 🔴 |
| 10 | `self_collisions` | −1.0 | 0–4 | จำนวน substep (จาก history 4) ที่ชิ้นส่วนของหุ่นชนกันเองด้วยแรง > 10 N | force 10 N | ตลอด | 🟢 |
| 11 | `foot_slip` | −0.25 | (m/s)² | Σ_เท้า ‖v_foot,xy‖² × (แตะพื้น) | threshold 0.1 | walking | 🔴 / 🟢 site |
| 12 | `body_ang_vel` | −0.05 | (rad/s)² | ωx² + ωy² ของ torso_link (ไม่นับ yaw) | body = torso_link | ตลอด | 🔴 / 🟢 body |
| 13 | `action_rate_l2` | −0.05 | — | Σ (aₜ − aₜ₋₁)² (raw action) | — | ตลอด | 🔴 |
| 14 | `angular_momentum` | −0.025 | (kg·m²/s)² | ‖L‖² ของทั้งตัว จาก sensor `robot/root_angmom` | — | ตลอด | 🔴 |
| 15 | `soft_landing` | −0.001 | N | Σ ‖F_contact‖ ของเท้าที่**เพิ่งแตะพื้น**ใน step นี้ | threshold 0.1 | walking | 🔴 |
| 16 | `joint_acc_l2` | −2.5e-7 | (rad/s²)² | Σ q̈_j² | — | ตลอด | 🔴 |

🔴 = กำหนดใน `velocity_env_cfg.py` (ใช้ร่วมทุกรุ่น) / 🟢 = กำหนดหรือ override ใน `config/g1_23dof/env_cfgs.py`
"🔴 / 🟢 site/body" = term มาจาก base แต่ 23 DoF ใส่ชื่อ body/site เอง (`torso_link`, `left_foot`/`right_foot`)

### 3.3 `pose` — std ต่อ joint (🟢 `env_cfgs.py`)

ยิ่ง std เล็ก ยิ่งบังคับให้ joint อยู่ใกล้ท่า default

| joint | standing (cmd < 0.1) | walking (0.1 – 1.5) | running (≥ 1.5) |
|---|---:|---:|---:|
| hip_pitch | 0.05 | 0.5 | 0.5 |
| hip_roll | 0.05 | 0.15 | 0.25 |
| hip_yaw | 0.05 | 0.15 | 0.25 |
| knee | 0.05 | 0.5 | 0.5 |
| ankle_pitch | 0.05 | 0.15 | 0.25 |
| ankle_roll | 0.05 | 0.1 | 0.1 |
| waist_yaw | 0.05 | 0.15 | 0.25 |
| shoulder_pitch | 0.05 | 0.15 | 0.25 |
| shoulder_roll | 0.05 | 0.1 | 0.1 |
| shoulder_yaw | 0.05 | 0.1 | 0.1 |
| elbow | 0.05 | 0.1 | 0.1 |
| wrist (roll) | 0.05 | 0.1 | 0.1 |

ตัวอย่าง: ตอนเดิน ถ้าเข่าห่างจาก default 0.5 rad แต่ joint อื่นตรงหมด → `exp(−(1/23)·1) ≈ 0.957`; ตอนยืน ถ้าเข่าเบี้ยว 0.05 rad → `exp(−1/23)` เท่ากัน

> **คำสั่ง (cmd) ในตารางนี้** = `‖(cx, cy)‖ + |cωz|` ไม่ใช่ความเร็วจริงของหุ่น

---

## 4. ขนาดของแต่ละ term เทียบกัน

reward ต่อ step = raw × weight × 0.02 ตารางนี้แสดง **อัตราต่อวินาที (raw × weight)** ในสถานการณ์ทั่วไป เพื่อดูว่า term ไหน "เสียงดัง" กว่ากัน

| term | สถานการณ์ตัวอย่าง | raw | อัตรา/วินาที |
|---|---|---|---:|
| track_linear_velocity | ตามคำสั่งได้ดี error 0.1 m/s | 0.96 | +0.96 |
| track_angular_velocity | ตามคำสั่งได้ดี | ≈ 1 | +1.0 |
| pose | เดินท่าปกติ | ≈ 0.8 | +0.8 |
| foot_gait | ตรงจังหวะ 90% | 0.9 | +0.45 |
| body_orientation_l2 | ลำตัวเอียง 10° | 0.030 | −0.03 |
| foot_slip | เท้าไถล 0.1 m/s หนึ่งข้าง | 0.01 | −0.0025 |
| action_rate_l2 | action เปลี่ยน 0.2 ทุก joint | 0.92 | −0.046 |
| self_collisions | ชนกันทุก substep | 4 | −4.0 |
| **is_terminated** | **ล้ม 1 ครั้ง** | 1 (1 step) | **−200 × 0.02 = −4.0 ต่อครั้ง** |

ข้อสังเกต:
- รางวัลหลัก 4 ตัวรวมกันได้สูงสุดประมาณ **+3.5 ต่อวินาที** ดังนั้นการล้ม (−4) มีค่าเท่ากับเสียรางวัลประมาณ 1 วินาที **และ**เสียรางวัลที่จะได้ในเวลาที่เหลือของ episode ซึ่งมีผลมากกว่า
- `joint_acc_l2` (−2.5e-7) ดูเล็ก แต่ q̈ อาจสูงถึงหลักพัน rad/s² ค่า raw จึงอยู่ระดับ 10⁵–10⁶ ทำให้มีผลจริง

---

## 5. Termination ที่เกี่ยวกับ `is_terminated`

| termination | เงื่อนไข | นับเป็นล้ม? |
|---|---|---|
| `time_out` | ครบ 20 s | ❌ (`time_out=True` จึงไม่โดน −200) |
| `fell_over` | `bad_orientation` > 70° | ✅ โดน −200 × dt |

---

## 6. Curriculum ที่กระทบ reward

`command_vel` (`src/tasks/velocity/mdp/curriculums.py`) ขยายช่วงคำสั่งตามจำนวน env step (`common_step_counter`):

| stage | เริ่มที่ step | ≈ iteration | lin_vel_x | lin_vel_y | ang_vel_z |
|---|---:|---:|---|---|---|
| 0 | 0 | 0 | −0.5 … 1.0 | −0.5 … 0.5 | −1.0 … 1.0 |
| 1 | 120,000 (= 5000 × 24) | 5000 | −1.0 … 2.0 | −1.0 … 1.0 | (คงเดิม) |

- `common_step_counter` ถูกบันทึกใน checkpoint และ**กู้คืนตอน resume** (`mjlab/rl/runner.py`) ดังนั้น P2, P3a และ P3b (iteration 10000+) เทรนด้วย stage 1 ตลอด
- คำสั่งที่เร็วขึ้นทำให้ term `pose` สลับไปใช้ std ของ running (≥ 1.5) บ่อยขึ้น และ tracking ยากขึ้น
- งาน Flat ปิด curriculum `terrain_levels`

---

## 7. จะปรับ reward ต้องแก้ที่ไหน

แก้ใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` (🟢) เท่านั้น ให้ override ค่า base อย่าแก้ `velocity_env_cfg.py` หรือ `mdp/rewards.py` (🔴)

| ต้องการ | วิธี |
|---|---|
| เปลี่ยน weight | `cfg.rewards["foot_slip"].weight = -0.5` |
| เปลี่ยนพารามิเตอร์ | `cfg.rewards["foot_clearance"].params["target_height"] = 0.08` |
| เปลี่ยนจังหวะเดิน | `cfg.rewards["foot_gait"].params["period"] = 0.7` **และ** `cfg.observations["actor"].terms["phase"].params["period"] = 0.7` **และ** `gait_phase.period` ใน `deploy.yaml` ต้องตรงกัน |
| ปิด term | `cfg.rewards["soft_landing"].weight = 0.0` หรือ `cfg.rewards.pop("soft_landing")` |
| ปรับความเข้มของท่า | แก้ dict `std_standing` / `std_walking` / `std_running` ของ `pose` |
| เพิ่ม term ใหม่ | เขียนฟังก์ชันไว้ในไฟล์เฉพาะ 23 DoF แล้ว `cfg.rewards["name"] = RewardTermCfg(func=..., weight=..., params=...)` |

### แนวทางปรับ reward ตามอาการ

| อาการตอน play / sim2sim | term ที่ควรดู |
|---|---|
| ยืนแล้วสั่น / ขยับขาไม่หยุด | `stand_still`, `pose` (std_standing), `action_rate_l2` |
| เท้าไถล | `foot_slip` ↑ (เช่น −0.5), เช็ค `Metrics/slip_velocity_mean` |
| ก้าวเตี้ย สะดุด | `foot_clearance` (target_height), `foot_gait` |
| เท้ากระแทกพื้นแรง | `soft_landing` ↑, ดู `Metrics/landing_force_mean` |
| ลำตัวโยก/เอียง | `body_orientation_l2`, `body_ang_vel` |
| แขนแกว่งมาก/ไม่แกว่ง | `angular_momentum`, std ของ shoulder ใน `pose` |
| เดินไม่ถึงความเร็วที่สั่ง | σ ของ `track_linear_velocity`, ช่วงคำสั่งใน curriculum |
| action กระตุก (แย่บนหุ่นจริง) | `action_rate_l2` ↑, `joint_acc_l2` |

> ⚠️ การเปลี่ยน reward **ไม่ต้องแก้ `deploy.yaml`** (ยกเว้นเปลี่ยน period ของ gait ที่ผูกกับ obs) แต่ต้องเทรนต่อหรือเทรนใหม่จึงจะมีผล ควรปรับครั้งละน้อยแล้วดู `Episode_Reward/*` ใน TensorBoard เทียบกับ run ก่อนหน้า
