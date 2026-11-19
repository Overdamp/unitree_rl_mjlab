# ProtoMotions ศึกษา 01: G1 Whole-Body Tracker (From Data to Real Robot)

> วันที่ศึกษา: 2026-11-18 · สถานะ: อ่าน docs อย่างเดียว ยังไม่ได้ติดตั้งหรือรันอะไร
> แหล่งที่มา (อ่านวันนี้):
> - https://nvlabs.github.io/ProtoMotions/tutorials/workflows/g1_deployment.html
> - https://nvlabs.github.io/ProtoMotions/getting_started/installation.html
> - https://nvlabs.github.io/ProtoMotions/robots/g1.html
> - https://nvlabs.github.io/ProtoMotions/getting_started/seed_g1_csv_preparation.html
> - https://huggingface.co/datasets/bones-studio/seed
>
> คำถามที่ต้องการตอบ: จะใช้ ProtoMotions ช่วยงานบันได G1 23-DoF (mjlab) ได้อย่างไร
> และเครื่องที่มีอยู่ (luke-mini RTX 3060 6 GB / DGX Spark aarch64) ใช้ได้ไหม

---

## 1. ภาพรวม pipeline ของ tutorial

```
BONES-SEED G1 CSV (120 fps, retarget มาแล้ว)
   └─ convert_g1_csv_to_proto.py  → .motion (30 fps)
       └─ motion_lib.py           → MotionLib .pt (รวมหลายคลิป)
           └─ train_agent.py (IsaacLab, 4096 envs) → tracker checkpoint
               └─ export_to_onnx.py → unified_pipeline.onnx + .yaml
                   ├─ test_tracker_mujoco.py   (sim2sim, CPU)
                   └─ RoboJuDo (sim / real G1, 50 Hz)
```

สิ่งที่เทรนคือ **motion tracker** (BeyondMimic-style) คือหุ่นเล่นตามคลิปทีละเฟรม
**ไม่ใช่** velocity policy แบบของเรา และไม่ใช่ AMP

## 2. ขั้นตอนโดยละเอียด

### 2.1 ข้อมูล
- **BONES-SEED**: ~142K motions (ประมาณ 10 เท่าของ AMASS) retarget เป็น G1 แล้ว
  - ดาวน์โหลด: https://huggingface.co/datasets/bones-studio/seed
  - **gated**: ต้อง login HF + ยอมรับ BONES-SEED License (ผู้ใช้ต้องทำเอง)
  - license: `bones-seed-license` (ข้อความเต็มที่ bones.studio/info/seed-license ยังไม่ได้อ่าน)
  - มี metadata 51 คอลัมน์ ต่อคลิป: คำบรรยายภาษาธรรมชาติสูงสุด 6 แบบ, tag
    (`content_type_of_movement`, `content_body_position`, `content_props`), temporal labels
  - หมวด: Locomotion 74,488 คลิป ("Walking, jogging, jumping, climbing, crawling, turning,
    and transitions"), Environments 614 คลิป (ไม่มีคำอธิบาย) ฯลฯ
  - **หน้า dataset ไม่มีคำว่า stairs/steps** → ต้องค้นใน metadata เอง (ศึกษา 02)
- คลิปตัวอย่างใน repo: `data/g1-kimodo-generated/proto/output_walk.motion` ฯลฯ
  และ `data/motion_for_trackers/g1_bones_seed_mini.pt`

**รูปแบบ CSV ของ BONES-SEED (ต่างจาก LAFAN1 ที่เราใช้!)**

| | BONES-SEED G1 | LAFAN1 G1 (ตัวแปลงของเรา) |
|---|---|---|
| fps | 120 | 30 |
| header | มี (`Frame`, `root_translateX`...) | ไม่มี |
| root pos | เซนติเมตร | เมตร |
| root rot | Euler XYZ (extrinsic) หน่วยองศา | quaternion xyzw |
| joints | องศา, `<joint>_dof`, ลำดับตาม MJCF | เรเดียน, ลำดับ LAFAN |

→ ถ้าจะใช้กับ `scripts/g1_23dof_motion_csv_to_npz.py` ต้องเพิ่มขั้นแปลง BONES → LAFAN-format ก่อน
(cm→m, euler→quat xyzw, deg→rad, เรียงคอลัมน์ตามชื่อ joint) — ต้องเช็กชื่อ joint จริงจาก header

### 2.2 แปลงและรวมคลิป
```bash
python data/scripts/convert_g1_csv_to_proto.py \
    --input-dir /path/to/bones-seed/g1/csv --output-dir /path/to/output/motions \
    --input-fps 120 --output-fps 30
python protomotions/components/motion_lib.py \
    --motion-path /path/to/output/motions/ --output-file /path/to/seed_g1_motions.pt
```
มี `--apply-motion-filter` สำหรับกรองคลิปเสีย (ใช้ในการแปลงทั้ง dataset)

### 2.3 เทรน tracker
มี pretrained ให้แล้ว: `data/pretrained_models/motion_tracker/g1-bones-deploy/` (แนะนำ ข้ามการเทรนได้)
```bash
python protomotions/train_agent.py --robot-name g1 --simulator isaaclab \
    --experiment-path data/pretrained_models/motion_tracker/g1-bones-deploy/experiment_config.py \
    --experiment-name g1_bm_tracker --motion-file /path/to/bones_seed_g1_motions.pt \
    --num-envs 4096 --batch-size 16384
```
- pretrained เทรนด้วย **24 × A100** บน BONES-SEED เต็มชุด (ไม่บอกเวลา)
- docs บอกว่า GPU น้อยกว่า + คลิปน้อยกว่า ยังได้ tracker ที่ใช้ได้กับท่าทั่วไป (เดิน เลี้ยว ท่ามือ)
- obs: proprio มี noise, target pose หลาย horizon [1, 2, 4, 8], action ก่อนหน้า, + สำเนาไม่มี noise สำหรับ L2C2
- reward: body tracking แบบ heading-invariant (pos/rot/lin vel/ang vel, ถ่วงตามส่วนของร่างกาย),
  anchor orientation, action rate, soft joint limit
- DR: obs noise, friction, CoM, push

### 2.4 ทดสอบใน MuJoCo (CPU, ไม่ต้องใช้ GPU)
```bash
python deployment/test_tracker_mujoco.py \
    --onnx data/pretrained_models/motion_tracker/g1-bones-deploy/compiled_models/unified_pipeline.onnx \
    --motion data/g1-kimodo-generated/proto/output_walk.motion --cache-motion --render
```
script นี้ถือเป็น "สัญญา" ของการ deploy (MJCF, PD, heading align, accel clamp, 50 Hz)

### 2.5 Export ONNX
```bash
python deployment/export_to_onnx.py --checkpoint <ckpt> --output /path/to/export_dir
```
ได้ `unified_pipeline.onnx` + `.yaml` (ถ้าไม่ใส่ `--output` จะเขียนทับไฟล์ที่มากับ repo)

### 2.6 Deploy (RoboJuDo)
- repo: https://github.com/HansZ8/RoboJuDo branch `release`
- **caveat ใน docs**: branch `release` ไม่ป้อน input `processed_action_history` → ONNX ที่มากับ repo error
- ขั้นของหุ่นจริง: ramp-up (ห้อย gantry) → blend-in → tracking → blend-out, e-stop = ปุ่ม A
- control 50 Hz, ONNX output = PD position targets (tanh + offset/scale อยู่ในกราฟแล้ว)

## 3. G1 ใน ProtoMotions
- `--robot-name g1` = **29 DoF, 33 bodies**, asset `mjcf/g1_holo_compat.xml`, config `protomotions/robot_configs/g1.py`
- anchor = `torso_link`, root = `pelvis`
- PD gains เป็นค่า sim (10 Hz natural freq, damping ratio 2) "ไม่ใช่ค่า hardware"
- **ไม่มี G1 23-DoF** ต้องใช้ tutorial "Adding a Custom Robot" ถ้าจะทำเอง

## 4. ข้อกำหนดการติดตั้ง

| Simulator | Python | Hardware | หมายเหตุ |
|---|---|---|---|
| IsaacLab | 3.12 | **Linux x86_64** + NVIDIA GPU | pin commit `4ecd0b03…`, ใช้ `uv pip install` ไม่ใช่ `uv sync` |
| Newton 1.0.0 | 3.10+ | NVIDIA GPU (CC ≥ 5.0, driver 545+) | ติดตั้งจาก PyPI ได้ |
| IsaacGym | 3.8 | NVIDIA GPU | ดาวน์โหลดจาก NVIDIA, x86 เท่านั้น |
| MuJoCo | 3.10+ | CPU, `num_envs=1` เท่านั้น | ใช้ดู/ทดสอบได้ เทรนไม่ได้ |

- ทุกแบบต้อง `git lfs install && git lfs pull` (checkpoint, motion, mesh อยู่ใน LFS)
- docs ไม่ระบุ VRAM บอกแค่ให้ลด `--num-envs` ถ้าหน่วยความจำไม่พอ

## 5. ประเมินกับเครื่องของเรา

| เครื่อง | ทำอะไรได้ | ข้อจำกัด |
|---|---|---|
| luke-mini (x86_64, 3060 6 GB) | ติดตั้ง, export, `test_tracker_mujoco.py` กับ pretrained, แปลงข้อมูล, ดูคลิป | เทรน 4096 envs ไม่ไหวแน่ ต้องลดมาก (คาดว่าหลักร้อย env) |
| DGX Spark (aarch64, GB10) | น่าจะเทรนด้วย **Newton** backend ได้ (ต้องลอง) | **IsaacLab ตาม docs ต้อง x86_64** และ IsaacGym ก็ x86 → ใช้ 2 ตัวนี้บน DGX ไม่ได้ตามที่เขียน |

ข้อควรระวัง:
- IsaacLab ของ ProtoMotions pin คนละ commit กับ `~/nvidia/IsaacLab-unitree` และ `~/nvidia/IsaacLab`
  → ถ้าติดตั้งต้องเป็น **venv ใหม่แยก** ห้ามติดตั้งทับ env เดิม (ดู isaac env map)
- DGX มีการแก้ ARM MuJoCo/CMake แบบ local ห้าม force checkout (doc/dgx_aarch64_local_git_setup.md)

## 6. ข้อสรุปสำหรับงานบันได

1. **ไม่ควรย้ายทั้งงานไป ProtoMotions** — tracker เล่นตามคลิป ต้องมีคลิปอ้างอิงตลอดเวลาตอนใช้งาน
   ไม่รับคำสั่งความเร็ว และไม่เห็น terrain (ไม่มี height scan / depth) จึงไม่ตอบโจทย์ "เดินขึ้นบันไดที่ไม่รู้จัก"
   อีกทั้งเป็น 29 DoF
2. **ของที่มีค่าที่สุดคือ BONES-SEED** — คลิป G1 retarget แล้ว 142K + มีคำบรรยาย ค้นหาได้
   ถ้ามีคลิปขึ้นลงบันได จะได้คลิปสำหรับ AMP โดยไม่ต้อง retarget จาก AMASS เอง
   (ข้ามงานใหญ่ที่สุดในแผนเดิมไปได้)
3. **tracker ใช้เป็นตัวกรองคลิปได้ (ไม่บังคับ)** — เล่นคลิปใน physics เพื่อดูว่าหุ่นทำได้จริง
   แต่คลิปบันไดต้องมีบันไดใน sim ด้วย ไม่งั้นเท้าจะเหยียบอากาศ → ต้องสร้าง terrain ให้ตรงคลิป (ยาก)
   สำหรับ AMP อาจข้ามขั้นนี้ได้ เพราะ AMP ใช้แค่สถิติของท่า ไม่ได้เล่นตามทีละเฟรม
4. **ข้อจำกัดเดิมยังอยู่**: 29→23 ตัด waist roll/pitch, คลิปไม่มีเรขาคณิตบันได

### เส้นทางที่เสนอ
```
BONES-SEED metadata (ค้น "stair", "step up", "climb") ──► เลือกคลิป
  ──► ตัวแปลง BONES CSV → LAFAN-format CSV (ต้องเขียนใหม่, ไฟล์เฉพาะ g1_23dof)
  ──► scripts/g1_23dof_motion_csv_to_npz.py (มีแล้ว) ──► AMP ใน mjlab
```
ไม่ต้องติดตั้ง ProtoMotions เลยสำหรับเส้นทางนี้

## 7. งานถัดไป
- [ ] (ผู้ใช้) login HF + ยอมรับ license BONES-SEED, อ่าน license ว่าใช้กับงานนี้ได้ไหม
- [ ] (ศึกษา 02) โหลดเฉพาะ `seed_metadata_v003.csv` (ไม่ต้องโหลด tar) ค้นคลิปบันได นับจำนวน ดูคำบรรยาย
- [ ] ถ้ามีคลิปบันไดพอ: โหลด g1 tar, ดู header จริง, เขียนตัวแปลง BONES → 23-DoF
- [ ] (ไม่บังคับ) ติดตั้ง ProtoMotions MuJoCo backend (CPU) บน luke-mini ใน env ใหม่ เพื่อดูคลิปและลอง pretrained tracker
- [ ] เทียบกับ AMASS (ศึกษา 03) เฉพาะถ้า BONES-SEED ไม่มีคลิปบันได
