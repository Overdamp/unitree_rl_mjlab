# Scene (SceneCfg / Scene) — G1-23DoF Velocity

เอกสารนี้อธิบายส่วน **Scene** ของ mjlab ซึ่งเป็นตัวประกอบโลกจำลอง (พื้น, หุ่น, sensor) ก่อนที่ manager อื่นจะทำงาน พร้อมตารางค่าที่ G1-23DoF ใช้จริง

> **หมายเหตุชื่อ:** mjlab ไม่มี class ชื่อ `SceneManager` ส่วนนี้คือ `mjlab.scene.Scene` กับ `SceneCfg` ซึ่ง env เข้าถึงผ่าน `env.scene` (แบบเดียวกับ `InteractiveScene` ใน Isaac Lab) และไม่ได้สืบทอดจาก `ManagerBase` เหมือน manager ตัวอื่น

เอกสารคู่กัน: [`g1_23dof_observation_manager.md`](g1_23dof_observation_manager.md), [`g1_23dof_action_manager.md`](g1_23dof_action_manager.md), [`g1_23dof_event_manager.md`](g1_23dof_event_manager.md), [`g1_23dof_reward_manager.md`](g1_23dof_reward_manager.md)

> **แหล่งที่ตรวจ (27/10/2026):**
> - `logs/rsl_rl/g1_23dof_velocity/2026-10-22_17-02-19/params/env.yaml` (Phase 3b, `scene:` และ `sim:`)
> - `src/tasks/velocity/velocity_env_cfg.py`, `src/tasks/velocity/config/g1_23dof/env_cfgs.py`
> - `src/assets/robots/unitree_g1/g1_23dof_constants.py`, `src/assets/robots/unitree_g1/xmls/g1_23dof.xml`
> - mjlab 1.2.0 (conda env `unitree_rl_mjlab`): `scene/scene.py`, `scene/scene.xml`, `terrains/terrain_entity.py`, `envs/manager_based_rl_env.py`

---

## 1. ภาพรวม

Scene คือ "สิ่งของในโลก" ทั้งหมดของ env และเป็นสะพานระหว่าง config กับ MuJoCo:

```
SceneCfg (Python dataclass)
  ├── terrain   : TerrainEntityCfg       → พื้น (plane / generator)
  ├── entities  : {"robot": EntityCfg}   → หุ่น + actuator + collision
  ├── sensors   : (SensorCfg, ...)       → contact / raycast / camera
  ├── num_envs, env_spacing, extent
  └── spec_fn   : callback แก้ MjSpec ก่อน compile
          │
          ▼  Scene.__init__  (ประกอบ MjSpec)
      MjSpec ──compile()──► MjModel ──► Simulation (mujoco_warp, num_envs worlds)
          │
          ▼  Scene.initialize(...)
      entity.data / sensor.data  (torch tensor ต่อ env บน GPU)
```

Manager อื่นทุกตัวอ่านหรือเขียนข้อมูลผ่าน scene:

```python
robot  = env.scene["robot"]                 # Entity
q      = robot.data.joint_pos               # (num_envs, 23)
feet   = env.scene["feet_ground_contact"]   # ContactSensor
gyro   = env.scene["robot/imu_ang_vel"]     # BuiltinSensor
```

`scene[key]` ค้นหาใน sensor ก่อน แล้วจึงค้นใน entity ถ้าไม่พบจะ raise `KeyError` พร้อมรายชื่อที่มี

---

## 2. การทำงานของ `Scene` (`mjlab/scene/scene.py`)

### 2.1 ขั้นตอนสร้าง (`Scene.__init__`)

| ขั้น | เมธอด | ทำอะไร |
|---|---|---|
| 1 | `MjSpec.from_file(scene.xml)` | เริ่มจาก XML พื้นฐานของ mjlab (แสง, haze, ขนาดเงา, `meansize`) ไม่มี body ใดๆ |
| 2 | `extent` | ถ้าตั้งไว้ จะทับ `stat.extent` (ใช้กับกล้อง/viewer) |
| 3 | `_add_terrain()` | สร้าง `TerrainEntity` ส่ง `num_envs`/`env_spacing` ให้ แล้ว attach เข้า worldbody (ไม่มี prefix) และลงทะเบียนเป็น entity ชื่อ `"terrain"` |
| 4 | `_add_entities()` | `EntityCfg.build()` ทีละตัว → ดึง keyframe แรกออก → `attach(prefix="<name>/")` → รวม keyframe ทุก entity เป็น keyframe เดียวชื่อ `init_state` |
| 5 | `_add_sensors()` | `SensorCfg.build()` → `edit_spec()` เพิ่ม sensor ลงใน spec; sensor ที่มีอยู่แล้วใน XML ของ entity จะถูกลงทะเบียนเป็น `BuiltinSensor` อัตโนมัติ |
| 6 | `spec_fn(spec)` | callback สุดท้ายก่อน compile |

ผลของ prefix: ชื่อทุกอย่างของหุ่นใน MuJoCo จะขึ้นต้นด้วย `robot/` เช่น `robot/imu_ang_vel` (นี่คือเหตุผลที่ obs ใช้ `sensor_name="robot/imu_ang_vel"`)

### 2.2 วงจรชีวิตใน env

| จังหวะ | เรียก | ผล |
|---|---|---|
| สร้าง env | `Scene(cfg.scene)` → `scene.compile()` → `Simulation(...)` → `scene.initialize(mj_model, model, data)` | สร้าง buffer ต่อ env, `env_origins`, `SensorContext` (ถ้ามี camera/raycast) |
| ทุก physics substep (×4) | `scene.write_data_to_sim()` → `sim.step()` → `scene.update(physics_dt)` | entity เขียน target/แรงลง sim แล้วอัปเดต sensor (เช่น air time) |
| หลัง reset | `scene.write_data_to_sim()` → `sim.forward()` | ทำให้สถานะที่เพิ่งเขียนตอน reset มีผล |
| ก่อนคำนวณ obs | `sim.sense()` | คำนวณ sensor ที่ต้องใช้ context (raycast/camera) |
| env reset | `sim.reset(env_ids)` → `scene.reset(env_ids)` | ล้างสถานะภายในของ entity และ sensor (เช่น air time, history) แล้ว event `reset` จึงวางหุ่นใหม่ |

### 2.3 Env origins

| terrain | วิธีจัด `env_origins` |
|---|---|
| `plane` (ที่ใช้) | ตาราง grid ห่างกัน `env_spacing` = 2.0 m |
| `generator` (Rough) | ตาม tile ของ terrain และระดับ curriculum (`update_env_origins`) |

> ใน mujoco_warp แต่ละ env เป็น **world แยกกัน** (batch) หุ่นต่างกันไม่ชนกัน ไม่ว่าจะอยู่ใกล้แค่ไหน `env_spacing` จึงมีผลแค่ตำแหน่งเริ่มต้นและการแสดงผลใน viewer ส่วน event `reset_base` สุ่มตำแหน่ง ±0.5 m รอบ origin ของ env นั้น

---

## 3. ค่าของ G1-23DoF (Phase 3b, งาน Flat)

### 3.1 `SceneCfg`

| field | ค่า | ตั้งที่ | หมายเหตุ |
|---|---|---|---|
| `num_envs` | **1024** | CLI `--env.scene.num-envs=1024` (`run_phase*_train.sh`) | base = 1 |
| `env_spacing` | 2.0 m | 🔴 base (default) | |
| `extent` | 2.0 | 🔴 `velocity_env_cfg.py` | |
| `terrain.terrain_type` | **`plane`** | 🟢 `unitree_g1_23dof_flat_env_cfg` | Rough ใช้ `generator` (`ROUGH_TERRAINS_CFG`) |
| `terrain.terrain_generator` | `None` | 🟢 Flat | |
| `terrain.max_init_terrain_level` | 5 | 🔴 | ไม่มีผลกับ plane |
| `entities` | `{"robot": get_g1_23dof_robot_cfg()}` (ห่อ actuator ด้วย `DelayedActuatorCfg`) | 🟢 `env_cfgs.py` | |
| `sensors` | `feet_ground_contact`, `self_collision` | 🟢 `env_cfgs.py` | `terrain_scan` ของ base ถูกลบในงาน Flat |
| `spec_fn` | `None` | | |

### 3.2 Terrain (plane)

| รายการ | ค่า |
|---|---|
| geom | ระนาบชื่อ `terrain` |
| texture | checker `groundplane` (0.2, 0.3, 0.4) / (0.1, 0.2, 0.3) |
| แสง | `sun` แบบ directional จากด้านบน ทอดเงา |
| แรงเสียดทาน | ตั้งที่ geom ของเท้า (0.6 แล้วสุ่ม 0.3–1.6 ด้วย DR) — MuJoCo ใช้ค่าจาก geom ที่ `priority` สูงกว่า และเท้ามี `priority=1` |

### 3.3 Entity `robot`

| รายการ | ค่า | ที่มา |
|---|---|---|
| MJCF | `src/assets/robots/unitree_g1/xmls/g1_23dof.xml` (`g1_23dof_rev_1_0`) | 🟡 `g1_23dof_constants.py` |
| ท่าเริ่มต้น (`HOME_KEYFRAME`) | pelvis z = 0.8 m; hip_pitch −0.1, knee 0.3, ankle_pitch −0.2, shoulder_pitch 0.35, shoulder_roll ±0.18, elbow 0.87 | 🟡 |
| Actuator | 4 กลุ่ม (5020, 7520-14, 7520-22, ankle 2×5020) ห่อด้วย `DelayedActuatorCfg` (0–4 step) | 🟡 + 🟢 (ดู action doc) |
| `soft_joint_pos_limit_factor` | 0.9 | 🟡 ใช้กับ reward `joint_pos_limits` |
| Collision (`FULL_COLLISION`) | ทุก geom `*_collision`; เท้า `left/right_foot1..7_collision`: `condim=3`, `priority=1`, friction 0.6; ชิ้นอื่น `condim=1` | 🟡 |
| Self-collision | **เปิด** (`FULL_COLLISION` ไม่ปิด contype) และถูกลงโทษด้วย reward `self_collisions` | 🟡 |

> `condim=1` = สัมผัสแบบไม่มีแรงเสียดทาน ใช้กับชิ้นส่วนที่ไม่ควรรับน้ำหนัก (ถูกกว่า) เท้า `condim=3` มีแรงเสียดทานในแนวราบ

### 3.4 Sensors

**Sensor ที่ประกาศใน config (🟢 `env_cfgs.py`):**

| ชื่อ | ชนิด | primary | secondary | fields | reduce | อื่นๆ | ใช้ที่ |
|---|---|---|---|---|---|---|---|
| `feet_ground_contact` | `ContactSensorCfg` | subtree ของ `left_ankle_roll_link`, `right_ankle_roll_link` (2 เท้า) | body `terrain` | found, force | netforce | `num_slots=1`, `track_air_time=True` | critic obs (foot_air_time/contact/forces), reward gait/slip/landing |
| `self_collision` | `ContactSensorCfg` | subtree `pelvis` (ทั้งตัว) | subtree `pelvis` | found, force | none | `history_length=4` (เก็บ 4 substep) | reward `self_collisions` (> 10 N) |

**Sensor ที่มากับ XML ของหุ่น (ลงทะเบียนอัตโนมัติเป็น `BuiltinSensor`):**

| ชื่อใน scene | ชนิด MuJoCo | ติดที่ | ใช้ที่ |
|---|---|---|---|
| `robot/imu_ang_vel` | gyro | site `imu_in_pelvis` | actor obs `base_ang_vel` |
| `robot/imu_lin_vel` | velocimeter | site `imu_in_pelvis` | critic obs `base_lin_vel` |
| `robot/imu_lin_acc` | accelerometer | site `imu_in_pelvis` | (ไม่ได้ใช้) |
| `robot/root_angmom` | subtreeangmom | body `pelvis` | reward `angular_momentum` |

**Sensor ใน base ที่ถูกลบ (งาน Flat):** `terrain_scan` (`RayCastSensorCfg`, grid 1.6 × 1.0 m ละเอียด 0.1 m, ระยะ 5 m) ใช้กับ obs `height_scan` ในงาน Rough

### 3.5 ค่า Simulation ที่ผูกกับ scene (`cfg.sim`)

| field | Flat (ใช้จริง) | Rough | หมายเหตุ |
|---|---|---|---|
| `mujoco.timestep` | 0.005 s | 0.005 | physics 200 Hz; `decimation=4` → policy 50 Hz |
| `mujoco.integrator` | implicitfast | | |
| `mujoco.solver` / iterations / ls_iterations | newton / 10 / 20 | | |
| `mujoco.cone` / `impratio` | pyramidal / 1.0 | | |
| `mujoco.ccd_iterations` | **50** | 500 | ตั้งใน `env_cfgs.py` 🟢 |
| `njmax` | **300** | (base) | จำนวน constraint สูงสุดต่อ world |
| `nconmax` | **None** (อัตโนมัติ) | 48 | จำนวน contact สูงสุดต่อ world |
| `contact_sensor_maxmatch` | **64** | 500 | จำนวน contact ที่ contact sensor จับคู่ได้ |
| `nan_guard.enabled` | False | | |

> ถ้าเพิ่ม sensor หรือ collision มากขึ้นแล้วเห็น warning เรื่อง contact เต็ม ให้เพิ่ม `njmax` / `nconmax` / `contact_sensor_maxmatch` ใน `env_cfgs.py`

---

## 4. Scene กับ manager อื่น

| manager | ใช้อะไรจาก scene |
|---|---|
| Observation | `robot.data` (projected_gravity, joint_pos/vel), `robot/imu_*`, `feet_ground_contact` |
| Action | `robot` → `set_joint_position_target` ผ่าน actuator |
| Event | `robot` (reset pose, push, mass/COM/friction DR ผ่าน `sim.model`), `scene.env_origins` |
| Reward | `robot.data` (body/site), `feet_ground_contact`, `self_collision`, `robot/root_angmom` |
| Termination | `robot.data.projected_gravity_b` |
| Command | `robot.data` (heading, velocity) |
| Curriculum | `scene.terrain` (เฉพาะ Rough) |

---

## 5. ความต่างจาก sim2sim (`unitree_mujoco`) และหุ่นจริง

| ประเด็น | mjlab scene (เทรน) | `unitree_mujoco` (sim2sim) / หุ่นจริง |
|---|---|---|
| โมเดล | `g1_23dof.xml` ของ repo นี้ + actuator จาก Python | XML ของ unitree_mujoco (ตรวจว่าเป็น rev เดียวกัน) |
| Actuator | position actuator ของ MuJoCo (PD ใน sim) + delay | sim2sim: PD คำนวณจาก LowCmd ใน simulator / จริง: motor driver |
| Self-collision | เปิด | ขึ้นกับ XML ของ simulator |
| พื้น | plane, friction สุ่ม | plane ค่าคงที่ / พื้นจริง |
| จำนวนหุ่น | 1024 worlds | 1 |
| Sensor | ค่าจาก MuJoCo ไม่มี noise เอง (noise ใส่ที่ obs) | LowState (IMU + encoder จริง) |

---

## 6. จะแก้ scene ต้องแก้ที่ไหน

แก้ใน `src/tasks/velocity/config/g1_23dof/env_cfgs.py` (🟢) เป็นหลัก

| ต้องการ | วิธี |
|---|---|
| เปลี่ยนจำนวน env | CLI `--env.scene.num-envs=N` (ไม่ต้องแก้โค้ด) |
| เปลี่ยนเป็นพื้นขรุขระ | ใช้ task `Unitree-G1-23Dof-Rough` (มี `terrain_scan` + `height_scan` → obs เปลี่ยน ต้องเทรนใหม่และแก้ `deploy.yaml`) |
| เพิ่ม contact sensor (เช่น เข่า/มือแตะพื้น) | `cfg.scene.sensors += (ContactSensorCfg(name="knee_ground", primary=ContactMatch(mode="subtree", pattern=r"^(left|right)_knee_link$", entity="robot"), secondary=ContactMatch(mode="body", pattern="terrain"), fields=("found","force"), reduce="netforce", num_slots=1),)` แล้วใช้ใน reward/termination |
| ปิด self-collision | ใช้ `FULL_COLLISION_WITHOUT_SELF` ใน robot cfg ที่สร้างใน `env_cfgs.py` (อย่าแก้ค่า default ใน constants) |
| เปลี่ยนท่าเริ่มต้น | สร้าง `EntityCfg` ใหม่ใน `env_cfgs.py` ด้วย `init_state` อื่น (ต้องแก้ `default_joint_pos` ใน deploy ด้วย เพราะ action offset = ท่านี้) |
| ปรับ solver/contact budget | `cfg.sim.njmax`, `cfg.sim.nconmax`, `cfg.sim.contact_sensor_maxmatch`, `cfg.sim.mujoco.ccd_iterations` |
| แก้ MjSpec ก่อน compile | `cfg.scene.spec_fn = lambda spec: ...` |
| export scene ไปดูใน MuJoCo viewer | `env.scene.write(Path("out_dir"))` → `out_dir/scene.xml` + `assets/` |

**ห้ามแก้ / ระวัง:**
- 🔴 `velocity_env_cfg.py` (scene ฐานของทุกหุ่น), mjlab ใน site-packages
- 🟡 `g1_23dof_constants.py` และ `g1_23dof.xml` ใช้ร่วมกับงาน mimic (tracking) ของ 23 DoF ถ้าต้องเปลี่ยนให้ override ใน `env_cfgs.py` แทน
- การเปลี่ยน XML, collision หรือ actuator ทำให้ฟิสิกส์ต่างจากที่ policy เคยเห็น ต้องเทรนต่อหรือเทรนใหม่ และทดสอบ sim2sim อีกครั้ง
