# Metrics Manager — G1-23DoF Velocity

เอกสารนี้อธิบาย Metrics Manager ของ mjlab และ**ค่าทั้งหมดที่ log ลง TensorBoard** ระหว่างเทรน G1-23DoF (ทั้งจาก MetricsManager และจาก manager อื่น) ว่ามาจากไหน คำนวณอย่างไร อ่านอย่างไร พร้อมตารางค่าจริงของแต่ละ phase

เอกสารคู่กัน: [`g1_23dof_reward_manager.md`](g1_23dof_reward_manager.md), [`g1_23dof_command_manager.md`](g1_23dof_command_manager.md), [`g1_23dof_termination_manager.md`](g1_23dof_termination_manager.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - TensorBoard event files ของ run P1 `2026-10-20_18-26-38`, P2 `2026-10-21_11-41-28`, P3a `2026-10-22_16-01-12`, P3b `2026-10-22_17-02-19`
> - `params/env.yaml` ของ P3b
> - mjlab 1.2.0 (conda env `unitree_rl_mjlab`): `managers/metrics_manager.py`, `envs/mdp/metrics.py`, `managers/command_manager.py`, `envs/manager_based_rl_env.py`
> - `src/tasks/velocity/mdp/rewards.py`, `src/tasks/velocity/velocity_env_cfg.py`

---

## 1. ภาพรวม

Metrics คือ "ตัวชี้วัด" ที่ log ไว้ดูคุณภาพของ policy **โดยไม่กระทบการเรียนรู้** ต่างจาก reward ตรงที่:

| | Reward | Metric |
|---|---|---|
| เข้า PPO | ✅ | ❌ ใช้ดูอย่างเดียว |
| weight | มี | ไม่มี |
| คูณ dt | ✅ | ❌ |
| ค่าที่ log | ผลรวมทั้ง episode ÷ 20 s | **ค่าเฉลี่ยต่อ step จริง** (sum ÷ จำนวน step) |
| หน่วย | reward/s | หน่วยของปริมาณนั้นเอง |

เพราะไม่มีผลต่อการเทรน จะเพิ่มหรือลบ metric ได้อย่างอิสระโดยไม่ต้องเทรนใหม่ (แต่ค่าจะมีเฉพาะ run ที่มี metric นั้นตั้งแต่เริ่ม)

---

## 2. การทำงานของ `MetricsManager` (`mjlab/managers/metrics_manager.py`)

```python
MetricsTermCfg(func=mdp.mean_action_acc, params={})   # func(env, **params) -> Tensor[num_envs]
```

| เมธอด | เรียกเมื่อ | ทำอะไร |
|---|---|---|
| `compute()` | ทุก policy step หลัง reward (`env.step`) | `step_count += 1`; `episode_sums[term] += func(env)` |
| `reset(env_ids)` | ตอน env reset | log `Episode_Metrics/<term>` = mean ของ (`episode_sums / step_count`) ของ env ที่ reset แล้วล้างค่า |

- ใช้ `clamp(step_count, min=1)` กันหารด้วย 0
- ถ้า `func` เป็น class จะถูกเรียก `reset()` ด้วย (เก็บสถานะข้าม step ได้)
- ถ้า `cfg.metrics` ว่าง env จะใช้ `NullMetricsManager` (ไม่ทำอะไร)

### 2.1 ลำดับใน `env.step`

```
physics → termination.compute → reward.compute → metrics.compute → reset env ที่จบ → command → obs
```

ดังนั้น metric เห็นสถานะเดียวกับ reward ใน step เดียวกัน และ action history (`action`, `prev_action`, `prev_prev_action`) อัปเดตแล้วจาก `process_action` ต้น step

---

## 3. Metric term ของ G1-23DoF (MetricsManager)

กำหนดใน `velocity_env_cfg.py` (🔴) และ 23 DoF ไม่ได้เพิ่ม term

| term | func | สูตร | หน่วย | ดีคือ | log เป็น |
|---|---|---|---|---|---|
| `mean_action_acc` | `mjlab.envs.mdp.metrics.mean_action_acc` | `mean_j |aₜ − 2aₜ₋₁ + aₜ₋₂|` (อนุพันธ์อันดับสองของ raw action เฉลี่ยทุก joint) | หน่วย action / step² | **ต่ำ** = action นุ่ม ไม่กระตุก | `Episode_Metrics/mean_action_acc` |

> metric นี้สำคัญสำหรับ sim2real: action ที่กระตุกมาก (ค่าสูง) ทำให้มอเตอร์จริงร้อนและสั่น ต่างจาก reward `action_rate_l2` ที่ดูแค่อนุพันธ์อันดับหนึ่ง

---

## 4. Metric จากแหล่งอื่น (ไม่ได้ผ่าน MetricsManager)

TensorBoard ยังมีกลุ่ม `Metrics/*` อีกสองแหล่ง:

### 4.1 Command term (`Metrics/twist/*`)

`UniformVelocityCommand._update_metrics()` สะสมทุก step แล้ว `CommandManager.reset()` log เป็น `Metrics/<term>/<name>`

| tag | สูตรสะสมต่อ step | หมายเหตุ |
|---|---|---|
| `Metrics/twist/error_vel_xy` | `‖cmd_xy − v_xy‖ / 400` | 400 = `resampling_time_range[1] / step_dt` = 8 s / 0.02 s |
| `Metrics/twist/error_vel_yaw` | `|cmd_ωz − ωz| / 400` | |

> ⚠️ **ค่านี้ไม่ใช่ค่าเฉลี่ย error โดยตรง** เพราะหารด้วย 400 แต่สะสมทั้ง episode (≈ 1000 step) ค่าที่ log ≈ **2.5 × error เฉลี่ย** ถ้าต้องการ error จริงให้หาร log ด้วย `mean_episode_length / 400` (≈ 2.5) ใช้เทียบระหว่าง run ได้ตราบที่ความยาว episode ใกล้กัน

### 4.2 Reward term ที่ log เพิ่ม (`Metrics/*`)

ฟังก์ชัน reward ใน `src/tasks/velocity/mdp/rewards.py` เขียนลง `env.extras["log"]` ตรงๆ ทุก step:

| tag | มาจาก | ค่า | หน่วย | ดีคือ |
|---|---|---|---|---|
| `Metrics/angular_momentum_mean` | `angular_momentum_penalty` | mean ‖L‖ ของทั้งตัว (ทุก env) | kg·m²/s | ต่ำ (ไม่แกว่ง) |
| `Metrics/slip_velocity_mean` | `feet_slip` | ความเร็ว xy เฉลี่ยของเท้าที่แตะพื้น | m/s | ต่ำ (ไม่ไถล) |
| `Metrics/landing_force_mean` | `soft_landing` | แรงสัมผัสเฉลี่ยตอนเท้าลงครั้งแรก | N | ต่ำ (ลงเบา) — น้ำหนักหุ่น ≈ 330 N |

ค่ากลุ่มนี้เป็น**ค่าเฉลี่ยของทุก env ใน step นั้น** ไม่ใช่ค่าเฉลี่ยต่อ episode และ `env.extras["log"]` ถูกสร้างใหม่ใน `_reset_idx` หลังคำนวณ reward ค่าที่ไปถึง logger จึงมาจาก step ที่ไม่มี env reset ซึ่งเพียงพอสำหรับดูแนวโน้ม

---

## 5. ตาราง tag ทั้งหมดใน TensorBoard (P3b)

| กลุ่ม | tag | มาจาก | ความหมาย |
|---|---|---|---|
| **Episode_Metrics** | `mean_action_acc` | MetricsManager | ความกระตุกของ action |
| **Metrics** | `twist/error_vel_xy`, `twist/error_vel_yaw` | CommandManager | error ของการตามคำสั่ง (ดู 4.1) |
| | `angular_momentum_mean`, `slip_velocity_mean`, `landing_force_mean` | reward funcs | ดู 4.2 |
| **Episode_Reward** | 16 tag (ชื่อ reward term) | RewardManager | ผลรวม reward ทั้ง episode ÷ 20 s (ดู reward doc) |
| **Episode_Termination** | `fell_over`, `time_out` | TerminationManager | **จำนวน env** ที่จบด้วยเหตุนั้นในชุดที่ reset (ไม่ใช่สัดส่วน) |
| **Train** | `mean_reward` | rsl_rl | ผลรวม reward ต่อ episode เฉลี่ย (ไม่หาร 20 s) |
| | `mean_episode_length` | rsl_rl | ความยาว episode เฉลี่ย (step) เต็ม = 1000 |
| | `*/time` | rsl_rl | ค่าเดียวกันแต่แกน x เป็นเวลา |
| **Loss** | `value`, `surrogate`, `entropy`, `learning_rate` | PPO | สถานะการ optimize |
| **Policy** | `mean_std` | PPO | std ของ action noise ตอนเทรน (ยิ่งต่ำ policy ยิ่งมั่นใจ/exploration น้อยลง) |
| **Perf** | `total_fps`, `collection_time`, `learning_time` | rsl_rl | ความเร็วในการเทรน |

ไม่มี tag `Curriculum/*` เพราะ `commands_vel` คืน dict ว่าง (ดู curriculum doc)

---

## 6. ค่าจริงของแต่ละ phase (เฉลี่ย 100 iteration สุดท้ายของแต่ละ run)

| metric | P1 | P2 | P3a (lag 0–40 ms) | **P3b (lag 0–20 ms) ✅** | ทิศที่ดี |
|---|---:|---:|---:|---:|---|
| `Train/mean_episode_length` (step) | 995 | 995 | 996 | **998** | ↑ (เต็ม 1000) |
| `Episode_Termination/fell_over` (env/batch) | 0.022 | 0.018 | 0.016 | **0.008** | ↓ |
| `Train/mean_reward` | 30.2 | 28.8 | 27.7 | **30.3** | ↑ |
| `Metrics/twist/error_vel_xy` (log) | 0.91 | 0.92 | 2.09 | **1.93** | ↓ |
| ≈ error xy จริง (÷ 2.5) m/s | 0.36 | 0.37 | 0.84 | **0.77** | ↓ |
| `Metrics/twist/error_vel_yaw` (log) | 1.41 | 1.49 | 1.24 | **1.19** | ↓ |
| `Episode_Metrics/mean_action_acc` | 0.80 | 0.83 | 0.63 | **0.62** | ↓ |
| `Metrics/slip_velocity_mean` (m/s) | 0.159 | 0.164 | 0.096 | **0.102** | ↓ |
| `Metrics/landing_force_mean` (N) | 287 | 287 | 190 | **219** | ↓ |
| `Metrics/angular_momentum_mean` | 1.12 | 1.14 | 1.03 | **1.07** | ↓ |
| `Policy/mean_std` | 0.39 | 0.40 | 0.30 | **0.30** | — |

### 6.1 การตีความ

- **P3b ล้มน้อยที่สุด, action นุ่มที่สุด, เท้าไถลน้อยและลงเบากว่า P1/P2** ซึ่งดีสำหรับ sim2real
- **แต่ error การตามความเร็ว xy เพิ่มขึ้นประมาณ 2 เท่า** (≈ 0.37 → 0.77 m/s) หลังเพิ่ม action delay: policy เลือกเดินอย่างระมัดระวัง (ช้ากว่าคำสั่ง) เพื่อแลกกับความเสถียร แนวโน้มเดียวกับ P3a ที่เดินพังจนเหลือ ≈ 0.25 m/s แต่ P3b อาการเบากว่า
- ที่ vx ต่ำ (0.4 = ปุ่ม `w` ของคีย์บอร์ด) P3b ยังตามได้ดี แต่ที่ vx ≥ 0.8 ตามไม่ทันชัดเจน ดูผลวัด sim2sim ในหัวข้อ 6.2
- `mean_std` ลดจาก 0.40 → 0.30 ใน P3: exploration ลดลง ถ้าเทรนต่อแล้ว tracking ไม่ดีขึ้น อาจต้องปรับ entropy หรือ reward tracking

> ค่าในตารางเป็น**ค่าตอนเทรน** (มี DR และ noise) ไม่ใช่ค่าบนหุ่นจริง

### 6.2 ความเร็วเดินจริงใน sim2sim (วัด 27/10/2026)

**วิธีวัด (headless):** สคริปต์ Python โหลด `scene_g1_23dof.xml` ตัวเดียวกับ `unitree_mujoco` แล้วสร้าง obs/action ตาม `deploy.yaml` แบบเดียวกับ `g1_ctrl` (policy 50 Hz, PD ที่ physics 500 Hz, ข้อต่อที่ policy ไม่ได้คุมใช้ gain ของ FixStand) ยืน 2 s แล้วเดินตามคำสั่ง 12 s วัดความเร็วเดินหน้าในทิศที่หุ่นหันอยู่ เฉลี่ย 8 s สุดท้าย ไม่ได้จำลอง latency ของ DDS

ONNX ที่ใช้: `deploy/robots/g1_23dof/config/policy/velocity/v0/exported/policy_phase2_13000.onnx` (P2) และ `policy_phase3b_16000.onnx` (P3b, md5 ตรงกับ `policy.onnx` ที่ deploy อยู่)

| คำสั่ง vx (m/s) | P2 vx จริง | P2 % ของคำสั่ง | P3b vx จริง | P3b % ของคำสั่ง | ล้ม |
|---:|---:|---:|---:|---:|---|
| 0.4 | 0.39 ± 0.05 | 97% | 0.39 ± 0.03 | 97% | ไม่ล้มทั้งคู่ |
| 0.8 | 0.89 ± 0.05 | 111% | **0.59** ± 0.03 | **74%** | ไม่ล้มทั้งคู่ |
| 1.0 | 1.13 ± 0.06 | 113% | **0.41** ± 0.02 | **41%** | ไม่ล้มทั้งคู่ |

(± คือ std ของความเร็วระหว่างเดิน ทั้งสอง policy เอียงสูงสุดไม่เกิน 5°)

**ตรวจกับ sim2sim จริง (`unitree_mujoco` + `g1_ctrl`, P3b, ปุ่ม `w` = 0.4):** อ่านตำแหน่งและความเร็วของลำตัวจาก topic `rt/sportmodestate` ที่ sim publish (LowState ไม่มีความเร็วลำตัว) วัด 2 รอบ รอบละ 8 s

| | รอบ 1 | รอบ 2 | headless |
|---|---:|---:|---:|
| ความเร็วเดินหน้า (m/s) | 0.398 | 0.399 | 0.388 |
| std | 0.032 | 0.032 | 0.028 |
| ไถลด้านข้าง (m/s) | 0.08 | 0.07 | 0.02 |

ผลตรงกับ headless จึงใช้ตาราง headless แทน sim2sim ได้

**สรุป**
- **ที่ vx 0.4 ทั้ง P2 และ P3b ตามได้ประมาณ 97–99%**
- **P3b ตามความเร็วสูงไม่ได้:** สั่ง 1.0 กลับเดินช้ากว่าสั่ง 0.8 แปลว่า policy ไม่ได้เรียนท่าเดินเร็วเลย ไม่ใช่แค่ช้ากว่าคำสั่งเล็กน้อย ตรงกับ error xy ตอนเทรนที่เพิ่มเป็น ≈ 0.77 m/s
- P2 เดินเร็วเกินคำสั่งประมาณ 11–13% ที่ vx สูง
- P3b มีความเร็วแกว่งน้อยกว่า P2 (std ประมาณครึ่งหนึ่ง)
- P3b ค่อยๆ เลี้ยวขวาเองประมาณ 2°/s (wz ≈ −0.03 rad/s) ทั้งใน headless และ sim2sim จริง เวลาใช้บนหุ่นจริงต้องคอยแก้ทิศด้วยปุ่ม `q`
- **ผลต่อการใช้งาน:** ถ้าบนหุ่นจริงใช้ vx ≤ 0.4 P3b ใช้ได้ ถ้าต้องการ 0.8–1.0 m/s ต้องเทรนต่อหรือปรับ reward/curriculum ให้ทั้งทนต่อ delay และตามความเร็วได้

---

## 7. จะเพิ่ม metric ต้องแก้ที่ไหน

เพิ่มใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` (🟢) ฟังก์ชันใหม่ให้ไว้ในไฟล์เฉพาะ 23 DoF อย่าแก้ `velocity_env_cfg.py` หรือ `mdp/*.py` (🔴)

```python
from mjlab.managers.metrics_manager import MetricsTermCfg

def forward_speed(env):                       # m/s ความเร็วเดินหน้าจริง
  return env.scene["robot"].data.root_link_lin_vel_b[:, 0]

def max_torque_ratio(env):                    # 0–1 แรงบิดสูงสุด / effort limit
  ...

cfg.metrics["forward_speed"] = MetricsTermCfg(func=forward_speed)
```

ตัวอย่าง metric ที่มีประโยชน์กับงานนี้:

| metric | ตอบคำถาม |
|---|---|
| ความเร็วเดินหน้าจริง / อัตราส่วนกับคำสั่ง | policy เดินช้ากว่าคำสั่งแค่ไหน (ข้อ 6.1) |
| แรงบิดสูงสุดต่อ effort limit | มอเตอร์ทำงานใกล้ขีดจำกัดไหม (ความเสี่ยงบนหุ่นจริง) |
| สัดส่วนเวลาที่ joint เกิน soft limit | policy ใช้ range ของ joint เกินไหม |
| lag ของ action delay ใน env | ผลของ delay ต่อ tracking |

> metric ไม่ต้องแก้ `deploy.yaml` และไม่กระทบ policy แต่ทุก metric ถูกคำนวณทุก step ทุก env ควรใช้ tensor operation ล้วน ไม่ใช้ `.item()` หรือ loop Python
