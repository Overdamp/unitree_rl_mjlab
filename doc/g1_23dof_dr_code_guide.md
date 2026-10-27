# คู่มือโค้ด Domain Randomization และโครงสร้างโปรเจกต์: G1-23DoF

เอกสารนี้อธิบาย:
1. โครงสร้างโค้ดของ `unitree_rl_mjlab` (เน้นส่วนที่เกี่ยวกับ G1-23DoF)
2. DR แต่ละตัวเขียนไว้ที่ไหน ปรับค่าตรงไหน และปรับอย่างไร
3. รายละเอียดเชิงลึกของ DR แต่ละตัว

ตรวจกับโค้ดที่ commit `f96f209` (branch `phase3-delay-dr`) และ mjlab 1.2.0 ที่ติดตั้งไว้ที่
`~/.local/lib/python3.10/site-packages/mjlab/` (27/10/2026) เลขบรรทัดอาจเลื่อนเมื่อแก้โค้ด

เอกสารที่เกี่ยวข้อง:
- `doc/domain_randomization_phases_g1_23dof.md`: DR แยกตาม phase และผลของแต่ละ phase
- `doc/domain_randomization_g1_23dof.md`: รายละเอียด Phase 1–2 และตาราง Kp/Kd ต่อมอเตอร์

---

## 1. โครงสร้างโปรเจกต์

```text
unitree_rl_mjlab/
├── scripts/                         # จุดเริ่มต้นฝั่ง Python
│   ├── train.py                     # ฝึก policy: python scripts/train.py <task_id> ...
│   ├── play.py                      # รัน policy ใน sim และ export ONNX (ใช้ play_env_cfg)
│   ├── list_envs.py                 # แสดง task ที่ลงทะเบียนไว้
│   ├── csv_to_npz.py                # แปลง motion สำหรับ task mimic/tracking
│   └── visualize_terrain.py
│
├── src/
│   ├── assets/
│   │   ├── robots/
│   │   │   └── unitree_g1/
│   │   │       ├── g1_23dof_constants.py   # ★ armature, Kp/Kd, effort limit, action scale, robot cfg
│   │   │       └── g1_constants.py         # รุ่น 29 DoF
│   │   └── motions/                         # ไฟล์ motion สำหรับ mimic
│   │
│   └── tasks/
│       ├── velocity/                        # task เดินตามคำสั่งความเร็ว
│       │   ├── velocity_env_cfg.py          # ★ config กลาง (ใช้ร่วมทุกหุ่น): obs, noise, DR พื้นฐาน, reward, sim dt
│       │   ├── config/
│       │   │   ├── g1_23dof/
│       │   │   │   ├── __init__.py          # ลงทะเบียน task Unitree-G1-23Dof-Rough / -Flat
│       │   │   │   ├── env_cfgs.py          # ★ ปรับเฉพาะ G1-23DoF: DR Phase 2–3, delay, reward std, play mode
│       │   │   │   └── rl_cfg.py            # PPO: experiment_name, max_iterations, save_interval
│       │   │   └── g1/ go2/ h1_2/ h2/ a2/ as2/ r1/   # หุ่นรุ่นอื่น โครงเดียวกัน
│       │   ├── mdp/
│       │   │   ├── events.py                # ★ delayed_effort_limits (DR ที่เขียนเพิ่มเอง)
│       │   │   ├── rewards.py               # ฟังก์ชัน reward
│       │   │   ├── observations.py          # observation เพิ่มเติม (phase, foot_*)
│       │   │   ├── curriculums.py           # curriculum ของ terrain และคำสั่งความเร็ว
│       │   │   ├── terminations.py
│       │   │   └── velocity_command.py      # ตัวสุ่มคำสั่งความเร็ว
│       │   └── rl/runner.py                 # VelocityOnPolicyRunner (export ONNX)
│       └── tracking/                        # task mimic (เต้น ฯลฯ)
│
├── deploy/                          # ฝั่ง C++ สำหรับรัน policy (sim2sim / หุ่นจริง)
│   ├── include/                     # FSM กลาง, isaaclab-style observation/action manager
│   └── robots/g1_23dof/
│       ├── main.cpp                 # g1_ctrl: เลือก network, คีย์บอร์ด (u/v/p, wasd)
│       ├── src/State_RLBase.cpp     # state Velocity: รัน ONNX, คีย์บอร์ดสั่งความเร็ว, safety bad_orientation
│       ├── include/keyboard_velocity_command.h   # ความเร็วของแต่ละปุ่ม และการ smooth
│       └── config/
│           ├── config.yaml          # FSM transitions (ปุ่มรีโมต)
│           └── policy/velocity/v0/
│               ├── params/deploy.yaml   # ★ Kp/Kd, action scale, default pose ที่ใช้ตอน deploy
│               └── exported/            # policy_phase*.onnx, policy.onnx (ตัวที่ g1_ctrl โหลด)
│
├── simulate/                        # unitree_mujoco (C++): sim ที่คุยผ่าน Unitree DDS
│   └── config.yaml                  # robot, scene, domain_id, interface
│
├── logs/rsl_rl/g1_23dof_velocity/<run>/
│   ├── params/env.yaml              # ★ ค่า config ที่ใช้จริงใน run นั้น (รวม DR)
│   ├── params/agent.yaml            # ค่า PPO และ resume
│   └── model_<iter>.pt              # checkpoint
│
├── run_phase2_train.sh, run_phase3_train.sh   # คำสั่ง resume ของแต่ละ phase
└── doc/                             # เอกสาร
```

★ = ไฟล์ที่เกี่ยวกับ DR โดยตรง

### ลำดับการประกอบ config

```text
scripts/train.py Unitree-G1-23Dof-Flat
  └─ src/tasks/velocity/config/g1_23dof/__init__.py      (register_mjlab_task)
       └─ unitree_g1_23dof_flat_env_cfg()                 env_cfgs.py
            └─ unitree_g1_23dof_rough_env_cfg()           env_cfgs.py
                 ├─ make_velocity_env_cfg()               velocity_env_cfg.py   ← DR พื้นฐาน + noise
                 ├─ get_g1_23dof_robot_cfg()              g1_23dof_constants.py ← Kp/Kd/effort ฐาน
                 ├─ ห่อ actuator ด้วย DelayedActuatorCfg                       ← Phase 3
                 └─ cfg.events[...] = EventTermCfg(...)                        ← Phase 2, 3
            └─ เปลี่ยนเป็นพื้นราบ, ลบ height_scan
```

ฟังก์ชัน DR (`dr.body_mass`, `dr.pd_gains` ฯลฯ) อยู่ใน mjlab ไม่ได้อยู่ใน repo นี้:
`~/.local/lib/python3.10/site-packages/mjlab/envs/mdp/dr/` (`body.py`, `joint.py`, `geom.py`, `actuator.py`)
และ `mjlab/envs/mdp/events.py` (`push_by_setting_velocity`, `reset_*`)

### ไฟล์ไหนแก้ได้ ไฟล์ไหนไม่ควรแก้

ตรวจด้วยการ `grep` หาว่าแต่ละไฟล์ถูก import หรือใช้จากหุ่นรุ่นไหน

#### 🔴 ใช้ร่วมกับหุ่นทุกรุ่น: ไม่ควรแก้

| ไฟล์ / โฟลเดอร์ | ใช้ร่วมกับ | ถ้าแก้จะเกิดอะไร |
|---|---|---|
| `src/tasks/velocity/velocity_env_cfg.py` | velocity ของทั้ง 8 รุ่น (g1, g1_23dof, go2, h1_2, h2, a2, as2, r1) | DR พื้นฐาน, noise, reward, `dt` และความยาว episode เปลี่ยนไปทุกรุ่น |
| `src/tasks/velocity/mdp/*.py` (`rewards.py`, `observations.py`, `curriculums.py`, `terminations.py`, `velocity_command.py`) | ทั้ง 8 รุ่น | สูตร reward และ observation เปลี่ยนทุกรุ่น policy เก่าของทุกรุ่นอาจใช้ไม่ได้ |
| `src/tasks/velocity/rl/runner.py` | ทุก task velocity | การ train และการ export ONNX ของทุกรุ่น |
| `scripts/train.py`, `scripts/play.py` | ทุก task | |
| `deploy/include/` (FSM, observation/action manager, `param.h`) | `g1_ctrl` ของทุกรุ่น (CMakeLists 6 รุ่น) | ต้อง build ใหม่ทุกรุ่น และพฤติกรรม FSM เปลี่ยนทุกรุ่น |
| `simulate/src/` | `unitree_mujoco` ของทุกรุ่น | |
| mjlab ใน `~/.local/lib/python3.10/site-packages/mjlab/` | ทุก task และทุกโปรเจกต์ที่ใช้ mjlab | หายเมื่อ `pip install` ใหม่ และไม่ถูกเก็บใน git |

#### 🟡 ใช้ร่วมกันเฉพาะ G1-23DoF: แก้ได้ แต่ต้องรู้ผลกระทบ

| ไฟล์ | ใช้ร่วมกับ | ข้อควรระวัง |
|---|---|---|
| `src/assets/robots/unitree_g1/g1_23dof_constants.py` | task **velocity** และ **tracking (mimic/เต้น)** ของ G1-23DoF | ถ้าเปลี่ยน Kp/Kd/effort ต้องแก้ `deploy.yaml` ทั้งของ `policy/velocity/` และ `policy/mimic/` ให้ตรงกัน |
| `src/assets/robots/__init__.py` | export ค่าคงที่ของทุกรุ่น | แก้ได้แค่การเพิ่ม import อย่าลบหรือเปลี่ยนชื่อของรุ่นอื่น |
| `simulate/config.yaml` | sim ตัวเดียวที่ใช้กับทุกรุ่น | ตอนนี้ตั้งเป็น `g1_23dof` ถ้าเปลี่ยนรุ่นจะกระทบการทดสอบ sim2sim |

#### 🟢 เฉพาะ G1-23DoF velocity: แก้ได้อย่างปลอดภัย

| ไฟล์ | ใช้ปรับอะไร |
|---|---|
| `src/tasks/velocity/config/g1_23dof/env_cfgs.py` | **ที่หลักสำหรับปรับ DR** (override ค่าจาก BASE ได้ที่นี่) |
| `src/tasks/velocity/config/g1_23dof/rl_cfg.py` | ค่า PPO, จำนวน iteration |
| `deploy/robots/g1_23dof/` (`main.cpp`, `src/`, `include/`, `config/`) | `g1_ctrl` ของ 23 DoF |
| `run_phase*_train.sh` | คำสั่ง train ของแต่ละ phase |

#### กรณีพิเศษ: `src/tasks/velocity/mdp/events.py`

ไฟล์นี้อยู่ในโฟลเดอร์ที่ใช้ร่วมกัน แต่ตอนนี้มีแค่ `delayed_effort_limits` ซึ่งเขียนเพิ่มเองและมีแค่ G1-23DoF ที่เรียกใช้

- **เพิ่มฟังก์ชันใหม่ได้** รุ่นอื่นไม่ได้เรียก จึงไม่กระทบ
- **อย่าเปลี่ยนการทำงานของฟังก์ชันที่มีอยู่** ถ้ารุ่นอื่นเริ่มเรียกใช้ในอนาคตจะกระทบไปด้วย

> **หลักง่าย ๆ:** ถ้าต้องการเปลี่ยนค่าที่อยู่ใน `velocity_env_cfg.py` อย่าแก้ไฟล์นั้น
> ให้ override ใน `config/g1_23dof/env_cfgs.py` แทน (ดูตัวอย่างในหัวข้อ 2)

---

## 2. DR แต่ละตัว: โค้ดอยู่ที่ไหนและปรับอย่างไร

ตัวย่อไฟล์:
- **BASE** = `src/tasks/velocity/velocity_env_cfg.py` ⚠️ ใช้ร่วมกับหุ่นทุกรุ่น
- **G1** = `src/tasks/velocity/config/g1_23dof/env_cfgs.py` เฉพาะ G1-23DoF
- **CONST** = `src/assets/robots/unitree_g1/g1_23dof_constants.py`

| # | DR | ไฟล์:บรรทัด | ฟังก์ชัน | พารามิเตอร์ที่ปรับ | วิธีปรับ |
|---|---|---|---|---|---|
| 1 | `foot_friction` | BASE:224 (ช่วงค่า) · G1:110 (เลือก geom เท้า) | `dr.geom_friction` | `ranges=(0.3, 1.6)`, `operation="abs"`, `shared_random=True` | เปลี่ยน `ranges` ถ้าตั้ง `shared_random=False` geom แต่ละชิ้นจะได้ค่าต่างกัน |
| 2 | `encoder_bias` | BASE:234 | `dr.encoder_bias` | `bias_range=(-0.015, 0.015)` rad | เปลี่ยน `bias_range` ถ้าจะกำหนดเฉพาะบางข้อต่อ ให้ใส่ `joint_names` ใน `asset_cfg` |
| 3 | `base_com` | BASE:242 (ช่วงค่า) · G1:111 (เลือก `torso_link`) | `dr.body_com_offset` | `ranges={0: x, 1: y, 2: z}`, `operation="add"` | ปรับช่วงแยกแต่ละแกน (เมตร) |
| 4 | `push_robot` | BASE:209 | `mdp.push_by_setting_velocity` | `interval_range_s=(5.0, 6.0)`, `velocity_range={x, y, z, roll, pitch, yaw}` | ผลักแรงขึ้น: ขยาย `velocity_range` · ผลักบ่อยขึ้น: ลด `interval_range_s` · ปิด: `cfg.events.pop("push_robot")` |
| 5 | `reset_base` | BASE:187 | `mdp.reset_root_state_uniform` | `pose_range`, `velocity_range` | ใส่ `velocity_range` เพื่อให้เริ่ม episode ขณะที่มีความเร็วอยู่แล้ว |
| 6 | `reset_robot_joints` | BASE:200 | `mdp.reset_joints_by_offset` | `position_range=(-0.0, 0.0)`, `velocity_range` | ตอนนี้ปิดอยู่ เปิดได้โดยตั้ง เช่น `(-0.1, 0.1)` rad |
| 7 | `torso_mass_payload` | G1:115 | `dr.body_mass` | `ranges=(-1.0, 3.5)` kg, `operation="add"` | ถ้าชั่งหุ่นจริงแล้ว ให้ปรับช่วงให้ครอบคลุมมวลจริง |
| 8 | `limb_mass_scale` | G1:125 | `dr.body_mass` | `ranges=(0.90, 1.10)`, `operation="scale"`, `body_names=(r"^(?!torso_link$).*",)` | regex เลือกทุก body ยกเว้น torso |
| 9 | `joint_friction` | G1:135 | `dr.joint_friction` | `ranges=(0.0, 0.30)`, `operation="abs"` | ปรับตาม frictionloss ของหุ่นจริง |
| 10 | `joint_damping` | G1:145 | `dr.joint_damping` | `ranges=(0.0, 0.10)`, `operation="abs"` | |
| 11 | `motor_pd_gains` | G1:155 · ค่าฐาน CONST:121–132 | `dr.pd_gains` | `kp_range=(0.85, 1.15)`, `kd_range=(0.85, 1.15)`, `operation="scale"` | ปรับช่วงสุ่มที่นี่ ถ้าจะเปลี่ยนค่าฐาน ให้แก้ `NATURAL_FREQ`/`DAMPING_RATIO` ใน CONST **และต้องแก้ `deploy.yaml` ให้ตรงกัน** |
| 12 | `motor_effort_limits` | G1:165 · ค่าฐาน CONST:103–118 · ฟังก์ชัน `src/tasks/velocity/mdp/events.py:20` | `delayed_effort_limits` | `effort_limit_range=(0.85, 1.05)`, `operation="scale"` | ต้องใช้ฟังก์ชันนี้แทน `dr.effort_limits` เพราะ actuator ถูกห่อด้วย DelayedActuator |
| 13 | `action_delay` | G1:25 (`G1_23DOF_MAX_ACTION_LAG = 4`) · G1:43 (ห่อ actuator) · G1:180 (event) | `dr.sync_actuator_delays` | `G1_23DOF_MAX_ACTION_LAG` (หน่วยเป็น physics step ขนาด 5 ms) | แก้ค่าคงที่ตัวเดียว ใช้ทั้งตอนห่อ actuator และใน `lag_range` · 4 = 0–20 ms, 8 = 0–40 ms (แบบ Phase 3a ซึ่งล้มเหลว) |
| — | Observation noise | BASE:62, 66, 78, 82 (actor) · BASE:127 `enable_corruption=True` | `UniformNoiseCfg` (`Unoise`) | `n_min`, `n_max` ของแต่ละ term | `base_ang_vel`:62, `projected_gravity`:66, `joint_pos`:78, `joint_vel`:82 |

### ⚠️ ปรับใน BASE หรือใน G1 ดี?

`velocity_env_cfg.py` (BASE) ใช้ร่วมกับหุ่น**ทุกรุ่น** (g1, go2, h1_2, h2, a2, as2, r1) ถ้าแก้ในไฟล์นี้ หุ่นทุกรุ่นจะเปลี่ยนตามไปด้วย
ถ้าต้องการแก้เฉพาะ G1-23DoF ให้ override ใน `env_cfgs.py` หลังบรรทัด `cfg = make_velocity_env_cfg()` (G1:30) เช่น:

```python
# env_cfgs.py ใน unitree_g1_23dof_rough_env_cfg() หลัง G1:111
cfg.events["foot_friction"].params["ranges"] = (0.2, 1.8)
cfg.events["push_robot"].params["velocity_range"]["x"] = (-0.8, 0.8)
cfg.events["push_robot"].interval_range_s = (3.0, 5.0)
cfg.observations["actor"].terms["joint_vel"].noise.n_min = -2.0
cfg.observations["actor"].terms["joint_vel"].noise.n_max = 2.0
```

### สูตรการปรับที่ใช้บ่อย

**ปิด DR ตัวใดตัวหนึ่ง** (เช่น เพื่อทดลอง ablation):
```python
cfg.events.pop("joint_damping", None)
```

**เพิ่ม DR ใหม่** (ตัวอย่าง: สุ่ม armature ของมอเตอร์ ซึ่งยังไม่ได้ทำ):
```python
cfg.events["motor_armature"] = EventTermCfg(
  mode="startup",
  func=dr.joint_armature,   # mjlab/envs/mdp/dr/joint.py:57
  params={
    "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
    "operation": "scale",
    "ranges": (0.9, 1.1),
  },
)
```

**เลือก mode ให้เหมาะ:**
| ต้องการ | ใช้ mode |
|---|---|
| คุณสมบัติของหุ่นที่ไม่เปลี่ยน (มวล, gain) | `startup`: ได้หุ่น `num_envs` แบบ |
| ให้เปลี่ยนทุก episode (หลากหลายกว่า) | `reset`: ใช้เวลาคำนวณมากขึ้นเล็กน้อย |
| เหตุการณ์ระหว่างเดิน | `interval` + `interval_range_s` |

### หลังแก้ DR ต้องทำอะไรบ้าง

1. **ฝึกต่อจาก checkpoint** (แบบเดียวกับ Phase 2/3): แก้ `--agent.load-run` และ `--agent.load-checkpoint` ในสคริปต์ `run_phase*_train.sh`
   ถ้าการเปลี่ยนแปลงทำให้ขนาด observation หรือ action เปลี่ยน จะ resume ไม่ได้ ต้องฝึกใหม่ตั้งแต่ต้น
2. **ตรวจค่าที่ใช้จริง:** ดูใน `logs/rsl_rl/g1_23dof_velocity/<run ใหม่>/params/env.yaml` ใต้หัวข้อ `events:`
3. **Export และ deploy:** `scripts/play.py` export ไฟล์ ONNX แล้วคัดลอกไปที่ `deploy/robots/g1_23dof/config/policy/velocity/v0/exported/policy.onnx`
4. **ถ้าเปลี่ยนค่าฐาน** (Kp/Kd, action scale, default pose) ต้องแก้ `deploy/.../params/deploy.yaml` ให้ตรงกัน ไม่อย่างนั้น policy บนหุ่นจะทำงานไม่ตรงกับตอนฝึก
5. **ทดสอบ sim2sim** ด้วย `unitree_mujoco` + `g1_ctrl` ก่อนนำไปใช้กับหุ่นจริง
6. **ตั้ง git tag** ให้ checkpoint ของแต่ละ phase (เช่น `phase3b-lag0-20ms`)

---

## 3. รายละเอียดเชิงลึกของ DR แต่ละตัว

ตรวจกลไกการสุ่มจากซอร์สของ mjlab (`envs/mdp/dr/*.py`, `envs/mdp/events.py`)

| # | DR | จำลองอะไรในโลกจริง | วิธีสุ่มใน sim | ช่วงค่าและผลเป็นตัวเลข | ถ้าไม่มี DR นี้ |
|---|---|---|---|---|---|
| 1 | `foot_friction` | พื้นแต่ละแบบ (กระเบื้อง, พรม, ยาง, พื้นเปียก) และพื้นรองเท้าที่สึก | ตอน startup สุ่ม 1 ค่าต่อ env ใช้กับ geom เท้าทั้ง 14 ชิ้น | μ = 0.3–1.6 (nominal 0.6) | เดินได้แค่บนพื้นที่ μ ≈ 0.6 พื้นลื่นจะทำให้เท้าไถลแล้วล้ม |
| 2 | `encoder_bias` | encoder calibrate ไม่ตรงศูนย์ หรือข้อต่อประกอบเหลื่อม | ตอน startup สุ่ม offset แยกแต่ละข้อต่อ บวกเข้ากับมุมที่ policy อ่าน (ฟิสิกส์ไม่เปลี่ยน) | ±0.015 rad = ±0.86° | หุ่นจริงยืนเอียงหรือเดินเบี้ยว |
| 3 | `base_com` | COM ไม่ตรงโมเดล (แบตเตอรี่, สายไฟ, ของที่ติดเพิ่ม) | ตอน startup บวก offset ให้ COM ของ `torso_link` แยกสามแกน | ±5 cm ต่อแกน | ถ้า COM จริงเยื้องไป หุ่นจะล้มไปทางนั้น |
| 4 | `push_robot` | ถูกผลัก, ชนสิ่งของ, สะดุด | ทุก 5–6 s **บวก**ความเร็วสุ่มเข้ากับความเร็วของ base ที่มีอยู่ (`vel_w += sample`) | xy ±0.5 m/s, z ±0.4 m/s, roll/pitch ±0.52 rad/s (≈30°/s), yaw ±0.78 rad/s (≈45°/s) | แรงกระแทกเล็กน้อยก็ทำให้ล้มได้ |
| 5 | `reset_base` | เริ่มจากตำแหน่งและทิศไหนก็ได้ | ทุก episode สุ่ม xy และ yaw | xy ±0.5 m, yaw ±180° | policy อาจจำทิศเริ่มต้น (เป็นการสุ่มสถานะ ไม่ใช่ฟิสิกส์) |
| 6 | `reset_robot_joints` | ท่าเริ่มที่ไม่ตรง default | ทุก episode | **0 (ปิดอยู่)** | ไม่มีผล |
| 7 | `torso_mass_payload` | แบตเตอรี่, LiDAR, คอมพิวเตอร์, ของที่ถือ | ตอน startup **บวก**มวลที่ `torso_link` | −1.0 ถึง +3.5 kg → torso 6.8–11.3 kg | หุ่นจริงหนักกว่าโมเดล ขาย่อ หรือมอเตอร์ออกแรงเกิน |
| 8 | `limb_mass_scale` | มวลแต่ละชิ้นไม่ตรงกับ CAD | ตอน startup **คูณ**มวลทุก body ยกเว้น torso แยกสุ่มแต่ละ body | ×0.9–×1.1 → รวม 21.9–26.7 kg | ยกขาและวางเท้าคลาดเคลื่อน |
| 9 | `joint_friction` | แรงเสียดทานสถิตของเฟืองดาวเคราะห์และ linkage | ตอน startup ตั้ง `frictionloss` ทุกข้อต่อ (ค่าสัมบูรณ์) | 0.0–0.30 N·m (sim2sim ใช้ 0.2) | ข้อต่อติดและกระตุก ยืนนิ่งไม่ได้ |
| 10 | `joint_damping` | แรงต้านที่แปรผันตามความเร็ว | ตอน startup ตั้ง damping ทุกข้อต่อ (ค่าสัมบูรณ์) | 0.0–0.10 N·m·s/rad (sim2sim ใช้ 0.05) | ก้าวขาไม่ทัน |
| 11 | `motor_pd_gains` | มอเตอร์ตอบสนองไม่เท่ากัน, แรงดันแบต | ตอน startup **คูณ** Kp และ Kd แยกกัน แยกสุ่มแต่ละมอเตอร์ | ×0.85–×1.15 เช่น Kp เข่า 99.1 → 84–114 | ข้อต่อแข็งหรืออ่อนกว่าที่คาด อาจสั่นหรือตอบสนองช้า |
| 12 | `motor_effort_limits` | แบตอ่อนหรือมอเตอร์ร้อน | ตอน startup **คูณ**แรงบิดสูงสุด | ×0.85–×1.05 เช่น เข่า 139 → 118–146 N·m | หุ่นทรุดหรือลุกไม่ขึ้นเมื่อแรงบิดจริงไม่ถึง |
| 13 | `action_delay` | ความหน่วงของ DDS, network และ control loop | ทุก episode สุ่ม lag เป็นจำนวนเต็ม 1 ค่าต่อ env ใช้ทุกข้อต่อ คงที่ตลอด episode | 3b: 0, 5, 10, 15, 20 ms (สูงสุด = 1 รอบของ policy ที่ 50 Hz) | Phase 2 (ไม่มี delay) ยืนสั่นที่ 20–30 ms และล้มที่ 40 ms |

### Observation noise (สุ่มใหม่ทุก step ใส่เฉพาะ actor)

| Observation | จำลองอะไร | Noise | บรรทัด |
|---|---|---|---|
| `base_ang_vel` | noise ของ IMU gyro | ±0.2 rad/s (≈±11°/s) | BASE:62 |
| `projected_gravity` | ความคลาดเคลื่อนในการประมาณท่าทาง | ±0.05 (≈±3°) | BASE:66 |
| `joint_pos` | noise ของ encoder | ±0.01 rad (≈±0.57°) | BASE:78 |
| `joint_vel` | ความเร็วที่ได้จากการหาอนุพันธ์ซึ่งมี noise สูง | ±1.5 rad/s | BASE:82 |

`height_scan` (BASE:88) มี noise ±0.1 แต่ task Flat ลบ term นี้ออก ส่วน `base_lin_vel` (BASE:98) อยู่ใน critic ซึ่งไม่ได้เปิด noise

### มวลรวมเมื่อรวม #7 และ #8

| กรณี | Torso | ส่วนอื่น | รวม |
|---|---|---|---|
| โมเดล (nominal) | 7.82 kg | 24.29 kg | 32.11 kg |
| เบาสุด | 6.82 kg | 21.86 kg | **28.7 kg** |
| หนักสุด | 11.32 kg | 26.72 kg | **38.0 kg** |
| หุ่นจริง (ประมาณการใน comment, ยังไม่ได้ชั่ง) | | | 34.1–35.0 kg |

### ข้อเท็จจริงของ mjlab ที่ควรรู้

- DR แบบ `scale` คูณกับ**ค่าเริ่มต้นของโมเดล**เสมอ (`get_default_field`) ไม่ได้คูณทบกับค่าที่สุ่มรอบก่อน การ resume หลายครั้งจึงไม่ทำให้ค่าเลื่อนไปเรื่อย ๆ
- ถ้าไม่ระบุ `distribution` จะสุ่มแบบ `uniform`
- ถ้าไม่ระบุ `shared_random` จะเป็น `False` คือแต่ละ body, joint หรือ geom ได้ค่าของตัวเอง
- `play=True` (G1:244) ปิดแค่ push, observation noise และ curriculum **DR ของ Phase 2–3 ยังทำงานอยู่**
