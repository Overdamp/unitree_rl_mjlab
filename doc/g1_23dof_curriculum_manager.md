# Curriculum Manager — G1-23DoF Velocity

เอกสารนี้อธิบายว่า Curriculum Manager ของ mjlab ปรับความยากของงานระหว่างเทรนอย่างไร, ทำงานเมื่อไร, เก็บสถานะข้าม phase (resume) อย่างไร และสรุป curriculum ที่ G1-23DoF ใช้จริงในแต่ละ phase

เอกสารคู่กัน: [`g1_23dof_command_manager.md`](g1_23dof_command_manager.md), [`g1_23dof_reward_manager.md`](g1_23dof_reward_manager.md), [`domain_randomization_phases_g1_23dof.md`](domain_randomization_phases_g1_23dof.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - `params/env.yaml` และ `params/agent.yaml` ของ run P1 `2026-10-20_18-26-38`, P2 `2026-10-21_11-41-28`, P3b `2026-10-22_17-02-19`
> - `infos.env_state.common_step_counter` ใน checkpoint `model_10000.pt`, `model_12999.pt`, `model_15998.pt`
> - `src/tasks/velocity/velocity_env_cfg.py`, `src/tasks/velocity/config/g1_23dof/env_cfgs.py`, `src/tasks/velocity/mdp/curriculums.py`
> - mjlab 1.2.0 (conda env `unitree_rl_mjlab`): `managers/curriculum_manager.py`, `envs/manager_based_rl_env.py`, `rl/runner.py`

---

## 1. ภาพรวม

**Curriculum learning** คือการเริ่มเทรนจากงานง่ายแล้วค่อยๆ เพิ่มความยาก เช่น เริ่มด้วยคำสั่งความเร็วต่ำก่อน แล้วค่อยขยายเป็นความเร็วสูง ถ้าให้งานยากตั้งแต่แรก policy มักจะล้มตลอดและไม่ได้เรียนรู้อะไร

ใน mjlab แต่ละ curriculum เป็น **term**:

```python
CurriculumTermCfg(
  func=mdp.commands_vel,        # func(env, env_ids, **params) -> state (สำหรับ log)
  params={...},
)
```

ฟังก์ชันของ curriculum **แก้ค่าของ manager อื่นโดยตรง** เช่น `cfg.ranges` ของ command, `weight` ของ reward หรือระดับ terrain ของแต่ละ env

---

## 2. การทำงานของ `CurriculumManager` (`mjlab/managers/curriculum_manager.py`)

### 2.1 เมธอด

| เมธอด | เรียกเมื่อ | ทำอะไร |
|---|---|---|
| `compute(env_ids)` | **ตอน env reset เท่านั้น** (`_reset_idx`) เป็นขั้นแรกสุด | เรียกทุก term แล้วเก็บค่าที่คืนไว้ใน `_curriculum_state` |
| `reset(env_ids)` | ตอน reset (หลัง compute) | log `Curriculum/<term>` (หรือ `Curriculum/<term>/<key>` ถ้าคืนเป็น dict) |

ลำดับใน `ManagerBasedRlEnv._reset_idx(env_ids)`:

```
1. curriculum_manager.compute(env_ids)   ← ปรับความยากก่อน
2. sim.reset / scene.reset
3. event "reset"                          ← วางหุ่นใหม่ (ใช้ terrain level ใหม่)
4. observation/action/reward/command/... reset
      └ command reset → สุ่มคำสั่งใหม่ด้วยช่วงที่ curriculum เพิ่งปรับ
```

ผลที่ตามมา:
- curriculum **ไม่ได้ทำงานทุก step** แต่ทำงานทุกครั้งที่มี env ใด env หนึ่ง reset (ล้ม/หมดเวลา) ซึ่งใน 1024 env เกิดเกือบทุก step อยู่แล้ว
- term ที่แก้ค่าแบบ global (เช่น `cfg.ranges`) จะมีผลกับ**ทุก env** ทันที แต่ env อื่นจะเห็นช่วงใหม่ตอน resample คำสั่งรอบถัดไป (3–8 s)
- term ที่แก้แบบต่อ env (เช่น `terrain_levels_vel`) ใช้ `env_ids` เพื่อปรับเฉพาะ env ที่ reset

### 2.2 ตัวนับเวลา: `common_step_counter`

curriculum แบบ "ตามเวลา" ของ task นี้ใช้ `env.common_step_counter` ซึ่ง:
- เพิ่ม 1 ทุก **env step** (policy step) ไม่ใช่ทุก iteration
- 1 iteration ของ PPO = `num_steps_per_env` = **24 step** → `step = iteration × 24`
- **ถูกบันทึกใน checkpoint** (`infos.env_state.common_step_counter`, `mjlab/rl/runner.py: save`) และ**กู้คืนตอน resume** (`load`) ทำให้ curriculum ต่อเนื่องข้าม phase

ยืนยันจาก checkpoint จริง:

| checkpoint | iteration | `common_step_counter` | = iteration × 24 |
|---|---:|---:|---|
| P1 `model_10000.pt` | 10000 | 240,024 | 10001 × 24 ✓ |
| P2 `model_12999.pt` | 12999 | 312,024 | 240,024 + 3000 × 24 ✓ (ต่อจาก P1) |
| P3b `model_15998.pt` | 15998 | 384,024 | 312,024 + 3000 × 24 ✓ (ต่อจาก P2) |

> ⚠️ ถ้าเปลี่ยน `num_steps_per_env` ใน `rl_cfg.py` จังหวะของ stage (ที่นับเป็น step) จะเลื่อนเมื่อเทียบกับ iteration

### 2.3 Play mode

`unitree_g1_23dof_rough_env_cfg(play=True)` ตั้ง `cfg.curriculum = {}` → `scripts/play.py` **ไม่มี curriculum** ช่วงคำสั่งใช้ค่าที่ตั้งในบล็อก `if play:` แทน

---

## 3. ตาราง Curriculum ของ G1-23DoF

### 3.1 term ที่ประกาศใน base config (`velocity_env_cfg.py`, 🔴)

| term | func | แก้อะไร | งาน Rough | งาน Flat (ที่ใช้เทรน) |
|---|---|---|---|---|
| `terrain_levels` | `terrain_levels_vel` | ระดับความยากของ terrain ต่อ env | ✅ | ❌ ถูก `pop` ใน `unitree_g1_23dof_flat_env_cfg` |
| `command_vel` | `commands_vel` | ช่วงคำสั่งความเร็ว (`twist.cfg.ranges`) | ✅ | ✅ **ตัวเดียวที่ทำงาน** |

ทุก run (P1, P2, P3a, P3b) ใช้งาน `Unitree-G1-23Dof-Flat` จึงมี curriculum เพียง **`command_vel`** และ G1-23DoF ไม่ได้ override ค่าใดๆ

### 3.2 `command_vel` — stage ของช่วงคำสั่ง

```python
for stage in velocity_stages:
    if env.common_step_counter > stage["step"]:
        cfg.ranges.<axis> = stage[<axis>]      # เฉพาะแกนที่ระบุใน stage
```

| stage | เงื่อนไข (step) | ≈ iteration | lin_vel_x (m/s) | lin_vel_y (m/s) | ang_vel_z (rad/s) |
|---|---|---:|---|---|---|
| (ก่อน stage แรก) | step = 0 | 0 | −1.0 … 2.0 (ค่าใน cfg) | −1.0 … 1.0 | −1.0 … 1.0 |
| **0** | > 0 | 0 | **−0.5 … 1.0** | **−0.5 … 0.5** | −1.0 … 1.0 |
| **1** | > 120,000 (= 5000 × 24) | 5000 | **−1.0 … 2.0** | **−1.0 … 1.0** | (ไม่ระบุ → คงเดิม −1.0 … 1.0) |

- stage ถูกตรวจทุกตัวตามลำดับ stage หลังจึงทับ stage ก่อน
- ช่วงที่ไม่ถูกระบุใน stage จะ**คงค่าล่าสุด**ไว้
- `ang_vel_z` ใช้เป็นขอบของ heading controller (ดู command doc หัวข้อ 2.3)
- ฟังก์ชันคืน dict ว่าง (บรรทัด log ถูก comment ไว้ใน `curriculums.py`) จึง**ไม่มีกราฟ `Curriculum/command_vel`** ใน TensorBoard ต้องคำนวณ stage จาก iteration เอง

### 3.3 Stage ที่ใช้จริงในแต่ละ phase

| phase | run | iteration | `common_step_counter` | stage ของ command | ช่วง vx |
|---|---|---|---|---|---|
| P1 | `2026-10-20_18-26-38` | 0 → 5000 | 0 → 120,000 | 0 | −0.5 … 1.0 |
| P1 | 〃 | 5000 → 10000 | 120,000 → 240,024 | **1** | −1.0 … 2.0 |
| P2 | `2026-10-21_11-41-28` | 10001 → 12999 | 240,024 → 312,024 | 1 | −1.0 … 2.0 |
| P3a | `2026-10-22_16-01-12` | ต่อจาก 12999 | > 312,024 | 1 | −1.0 … 2.0 |
| P3b ✅ | `2026-10-22_17-02-19` | 12999 → 15998 | 312,024 → 384,024 | 1 | −1.0 … 2.0 |

สรุป: **phase ที่เพิ่ม DR (P2, P3) ทั้งหมดเทรนด้วยช่วงคำสั่งกว้างสุด (stage 1)** การเพิ่มความยากระหว่าง phase ทำผ่าน DR (Event Manager) ไม่ใช่ curriculum

### 3.4 `terrain_levels_vel` (มีอยู่แต่ไม่ได้ใช้ในงาน Flat)

ทำงานเฉพาะเมื่อ terrain เป็น generator และ `terrain_generator.curriculum = True` (งาน Rough ของ 23 DoF ตั้งไว้):

| เงื่อนไข (ตอน env reset) | ผล |
|---|---|
| เดินได้ไกลจากจุดเริ่ม > ครึ่งหนึ่งของขนาด tile | **ขึ้น** ระดับ terrain (ยากขึ้น) |
| เดินได้ < ครึ่งหนึ่งของระยะที่ควรได้ตามคำสั่ง (`‖cmd_xy‖ × 20 s × 0.5`) และไม่ได้ขึ้น | **ลง** ระดับ terrain |
| อื่นๆ | คงระดับเดิม |

คืนค่าเฉลี่ยของระดับ terrain → log เป็น `Curriculum/terrain_levels`; ระดับเริ่มต้นสุ่มไม่เกิน `max_init_terrain_level = 5`

### 3.5 ฟังก์ชันที่มีให้แต่ยังไม่ได้ใช้: `reward_weight`

`src/tasks/velocity/mdp/curriculums.py` มี `reward_weight(reward_name, weight_stages)` สำหรับเปลี่ยน weight ของ reward ตาม step ทำงานแบบเดียวกับ `commands_vel` (stage หลังทับ stage ก่อน) และคืน weight ปัจจุบัน ซึ่งจะ log เป็น `Curriculum/<term>`

---

## 4. ความสัมพันธ์กับ manager อื่น

| curriculum | แก้ที่ | กระทบ |
|---|---|---|
| `command_vel` | `CommandManager` → `twist.cfg.ranges` | obs `command`, reward tracking, std ของ `pose` (ความเร็ว ≥ 1.5 ใช้ std แบบวิ่ง) |
| `terrain_levels` | `scene.terrain` env origins | event `reset_base` วางหุ่นบน tile ใหม่ |
| `reward_weight` (ถ้าเปิด) | `RewardManager` term weight | สมดุลของ reward |

---

## 5. จะแก้ curriculum ต้องแก้ที่ไหน

แก้ใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` (🟢) เท่านั้น อย่าแก้ `velocity_env_cfg.py` หรือ `mdp/curriculums.py` (🔴 ใช้ร่วมกับทุกหุ่น)

| ต้องการ | วิธี (ใน `unitree_g1_23dof_rough_env_cfg` หรือ `_flat_env_cfg`) |
|---|---|
| เปลี่ยน stage ความเร็ว | `cfg.curriculum["command_vel"].params["velocity_stages"] = [{"step": 0, "lin_vel_x": (-0.5, 0.8), ...}, {"step": 3000 * 24, ...}]` |
| เพิ่ม stage ให้ ωz | ใส่ `"ang_vel_z": (-1.5, 1.5)` ใน stage ที่ต้องการ |
| ปิด curriculum ความเร็ว | `cfg.curriculum.pop("command_vel")` แล้วตั้ง `cfg.commands["twist"].ranges` เอง |
| ค่อยๆ เพิ่ม reward | `cfg.curriculum["slip_weight"] = CurriculumTermCfg(func=mdp.reward_weight, params={"reward_name": "foot_slip", "weight_stages": [{"step": 0, "weight": -0.1}, {"step": 2000 * 24, "weight": -0.5}]})` |
| เขียน curriculum ใหม่ | ฟังก์ชัน `func(env, env_ids, ...)` ไว้ในไฟล์เฉพาะ 23 DoF; ถ้าแก้ค่า model ต่อ env ต้องใช้กลไกของ Event Manager แทน |

### ข้อควรระวังเมื่อ resume

- เพราะ `common_step_counter` ถูกกู้คืน ถ้า**ลด** stage threshold ลง stage ใหม่จะมีผล**ทันที**ตอน resume ไม่ได้เริ่มนับใหม่
- ถ้าต้องการให้ curriculum เริ่มใหม่ตอน resume ต้องตั้ง step ของ stage ให้อิงจาก counter ปัจจุบัน (เช่น 312,024 + n × 24) หรือรีเซ็ต counter เอง (ต้องแก้ใน runner ของ 23 DoF)
- ถ้าเพิ่ม DR ที่ยากขึ้นใน phase ใหม่ แนะนำให้ใช้ `reward_weight` หรือ stage ที่อิงจาก counter ปัจจุบัน เพื่อเพิ่มความยากทีละน้อย แทนการเปิดเต็มทันที (บทเรียนจาก P3a ที่เปิด delay 0–40 ms ทันทีแล้วเดินพัง)
