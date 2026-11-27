# แผน Phase 4: G1-23DoF ขึ้นบันไดโปร่ง (open-riser)

เริ่ม 6/11/2026 · branch `phase4-stairs` · worktree `~/nvidia/unitree/unitree_rl_mjlab-stairs` (แตกจาก `phase3-delay-dr` @ cba2434)

## เป้าหมาย

หุ่นต้องขึ้นและลงบันไดที่**ไม่มีลูกตั้ง** (ข้างล่างกลวง) ได้ งานนี้ทำใน sim ทั้งหมด ยังไม่มีแผนทดสอบบนหุ่นจริงจนกว่า policy เดินพื้นเรียบจะผ่านการทดสอบบนหุ่นจริงก่อน

## ข้อจำกัด

- เซนเซอร์ที่มี: กล้อง Intel RealSense **D455f** บนหัว (`realsense_gst.service` บน .164) ไม่มี 3D lidar แล้ว (MID-360 ถูกถอดออก)
- D455 วัด depth ได้ดีที่ระยะ ≳0.6 m และขั้นที่อยู่ใต้เท้าอยู่นอกภาพ policy จึงต้องจำขั้นที่เห็นเมื่อครู่ก่อน (GRU หรือ history)
- ภาพ depth มองทะลุช่องว่างระหว่างขั้น และแผ่นขั้นที่บางทำให้ภาพมี noise สูง
- policy แบบ blind อาศัยให้ปลายเท้าชนลูกตั้งเพื่อรู้ว่าถึงขั้นแล้ว บันไดโปร่งไม่มีลูกตั้ง ปลายเท้าจึงสอดเข้าไปใต้ขั้นถัดไปได้
- แก้เฉพาะไฟล์ของ g1_23dof เท่านั้น (ห้ามแก้ `velocity_env_cfg.py`, `mdp/*`, `deploy/include/*`)

## ขั้นตอน

| ขั้น | งาน | ใช้หุ่นไหม | สถานะ |
|---|---|---|---|
| 0 | วัดบันไดเป้าหมาย (สูงขั้น ลึกขั้น ความหนาแผ่น จมูกยื่น ช่องว่าง) และตำแหน่ง/มุมกล้อง | ไม่ (ใช้ตลับเมตร) | ยังไม่ทำ |
| 1 | สร้าง terrain บันไดโปร่งใน `src/tasks/velocity/config/g1_23dof/` | ไม่ | เสร็จ 6/11 (task `Unitree-G1-23Dof-OpenStairs-Blind`) |
| 2 | Baseline: วัดว่า policy blind เดินบนบันไดโปร่งได้สูงสุดกี่ cm | ไม่ | เสร็จ 11/11 สำหรับ P3b (ยังไม่มี `Rough-Blind` ที่เทรนเสร็จให้วัด) |
| 2b | Fine-tune แบบ blind บนบันไดโปร่ง (resume P3b, `run_stairs_blind_train.sh`, 4000 iteration) แล้ววัดด้วย `eval_open_stairs.py` · บอกว่า blind ไปได้ไกลแค่ไหน และเป็น baseline ที่ teacher ต้องชนะ | ไม่ | เสร็จ 11/11: ขึ้นได้ถึง ~5 cm (44% ที่ 5 cm, 0% ตั้งแต่ 8 cm) และเดินช้าลง ไม่ใช้แทน P3b · ผลดิบ `step2b_stairs_blind_vx0.5.jsonl` |
| 3 | Teacher: เทรนด้วย `height_scan` แบบ privileged | ไม่ | 18/11 `stairs_teacher5` model_22996 (heading hold): **ลง ≤ 10 cm 98–100%, ลง 12 cm 36%** · ขึ้น 3/5/8/10 cm 97/97/56/8% (ขึ้นแย่กว่า teacher4 ที่ 8 cm) · ล้ม ≤ 5% ทุกเคส = หยุดที่ขอบ ไม่ล้ม · ส่ง teacher6 (`a59b530`: `is_terminated` −100 + progress reward) เทรนต่อจาก teacher5 (ดูขั้น 3d) |
| 4 | Student: depth จาก `PinholeCameraPatternCfg` ตั้งค่าตาม D455 (noise, latency, ตัดระยะใกล้) + GRU แล้ว distill จาก teacher | ไม่ | ยังไม่ทำ |
| 5 | อ่านภาพ D455 จริงแบบอ่านอย่างเดียว เทียบกับภาพจำลอง วัด latency | อ่านอย่างเดียว | ยังไม่ทำ |
| 6 | ทดสอบบนหุ่น | ใช่ | รอให้เดินพื้นเรียบผ่านก่อน |

## ไฟล์ที่จะเพิ่ม

- `src/tasks/velocity/config/g1_23dof/terrains_stairs.py`: `OpenRiserStairsTerrainCfg` (มีแล้ว)
- `src/tasks/velocity/config/g1_23dof/env_cfgs_stairs.py`: task blind (มีแล้ว), teacher, student
- `src/tasks/velocity/config/g1_23dof/__init__.py`: ลงทะเบียน task ใหม่
- `view_open_stairs.sh`: ดู policy เดินบนบันไดโปร่ง (มีแล้ว)
- `eval_open_stairs.py`: วัดอัตราผ่าน/ล้มทีละความสูงขั้น ขึ้นและลง (มีแล้ว)
- สคริปต์ทุกตัวใช้ path แบบ relative (`cd "$(dirname "$0")"`) และตั้ง `PYTHONPATH="$PWD"`

> [!warning] ต้องตั้ง PYTHONPATH ใน worktree
> conda env `unitree_rl_mjlab` ติดตั้งโฟลเดอร์หลักไว้แบบ editable ถ้ารัน `python scripts/train.py` ใน worktree เฉยๆ `import src` จะโหลดโค้ดของ**โฟลเดอร์หลัก** และ task ใหม่จะไม่มีให้เลือก ใช้สคริปต์ `run_*.sh` / `view_*.sh` หรือรันด้วย `PYTHONPATH=$PWD python scripts/...`

## Terrain บันไดโปร่ง (ขั้น 1)

`OpenRiserStairsTerrainCfg` วางขั้นเป็นวงสี่เหลี่ยมรอบ platform กลาง แบบเดียวกับ pyramid stairs ของ mjlab เดินออกทางไหนก็เจอบันได

| ค่า | ช่วง (ชั่วคราว รอวัดบันไดจริง) | หมายเหตุ |
|---|---|---|
| ความสูงขั้น | 0.02 → 0.18 m | เพิ่มตาม difficulty (แถวของ curriculum) · เริ่มที่ 2 cm เพราะ P3b ล้ม 86% ที่ขาขึ้น 3 cm |
| ความลึกขั้น | 0.25–0.32 m | สุ่มทีละ patch |
| ความหนาแผ่น | 0.03–0.06 m | สุ่ม · จำกัดไม่เกิน 60% ของความสูงขั้น เพื่อให้มีช่องว่างใต้ขั้นเสมอ |
| จมูกยื่น | 0–0.03 m | ขั้นบนยื่นเลยขั้นล่าง |

- `open_stairs_down` (pyramid): เกิดบน platform ยอด เดินออก = **ลง**
- `open_stairs_up` (bowl): เกิดที่ก้นหลุม เดินออก = **ขึ้น**
- มีพื้นใต้บันไดและผนังรอบหลุม กล้อง depth ที่มองทะลุช่องว่างจะเห็นพื้น ไม่เห็นความว่างเปล่า
- สัดส่วน: flat 10%, ลง 45%, ขึ้น 45%
- ตรวจแล้ว: ยิง ray แนวนอนเหนือขั้นล่าง 3 cm ผ่านใต้ขั้นถัดไปได้จนชนผนัง (ยืนยันว่าข้างล่างกลวงจริง) · เทรน 64 envs × 3 iteration ผ่าน · `view_open_stairs.sh` โหลด checkpoint P3b ได้ (obs เท่ากับ Rough-Blind)

## บันทึกผล

### ขั้น 2: baseline ของ P3b (เทรนบนพื้นเรียบ) บนบันไดโปร่ง (11/11/2026)
`python eval_open_stairs.py` · checkpoint `model_phase3b_15998.pt` · สั่ง vx 0.5 m/s เดินตรง · 64 ตัวต่อกรณี · 14 วินาที · ลูกนอน 0.28 m, แผ่นหนา 0.04 m, จมูก 0.02 m (ค่าสมมติจนกว่าจะวัดบันไดจริง) · ผลดิบ: `doc/g1_23dof_stairs_results/step2_p3b_vx0.5.jsonl`

| สูงขั้น | ลง: ผ่านครบ 8 ขั้น | ลง: ล้ม | ขึ้น: ผ่านครบ 7 ขั้น | ขึ้น: ล้ม | ขึ้น: ไปได้กี่ขั้น (median) |
|---|---|---|---|---|---|
| 0 cm | 89% | 2% | 94% | 0% | 7 |
| 3 cm | 95% | 5% | 12% | 86% | 4 |
| 5 cm | 92% | 8% | 0% | 100% | 3 |
| 8 cm | 86% | 14% | 0% | 100% | 2 |
| 10 cm | 62% | 38% | 0% | 100% | 2 |
| 12 cm | 58% | 42% | 0% | 98% | 2 |
| 15 cm | 50% | 50% | 0% | 91% | 2 |
| 18 cm | 33% | 67% | 0% | 69% | 1 |

อ่านผล:
- **ขึ้นไม่ได้เลย:** แค่ขั้นสูง 3 cm ก็ล้ม 86% และตั้งแต่ 5 cm ไม่มีตัวไหนขึ้นครบ สาเหตุที่คาดไว้คือ policy ที่เทรนบนพื้นเรียบยกเท้าไม่พ้นขอบแผ่น (ยังไม่ได้เปิดดูด้วยตา ใช้ `./view_open_stairs.sh` ยืนยัน)
- **ลงได้บางส่วน:** ถึง 8 cm ยังผ่าน 86% แล้วตกเหลือราวครึ่งที่ 12–15 cm
- แถว 0 cm (พื้นเรียบ) ล้ม 0–2% ยืนยันว่าตัววัดไม่ได้ทำให้ล้มเอง · ตัวที่ไม่ล้มแต่ไม่ผ่าน น่าจะเดินไม่ถึงปลายทางใน 14 วินาที (P3b เดินได้ช้ากว่าที่สั่ง)
- ที่ 18 cm ขาขึ้นล้มน้อยลง (69%) เพราะหุ่นติดอยู่หน้าขั้นแรกแทนที่จะล้ม ไม่ได้แปลว่าดีขึ้น
- ยังไม่ได้เทียบกับบันไดทึบที่ความสูงเท่ากัน จึงยังแยกไม่ได้ว่าส่วนไหนเป็นผลของ "โปร่ง" และส่วนไหนเป็นผลของ "บันได"

### ขั้น 3: teacher (height scan) บนบันไดโปร่ง (13/11/2026)
`python eval_open_stairs.py --task Unitree-G1-23Dof-OpenStairs-Teacher` · checkpoint `stairs_teacher2/model_7999.pt` (รอบแรก 2428 iteration จากศูนย์ + resume ด้วย config ที่แก้ใน `f395750` จนครบ 8000) · เงื่อนไขเดียวกับขั้น 2/2b · ผลดิบ `doc/g1_23dof_stairs_results/step3_teacher_vx0.5.jsonl`

| สูงขั้น | ลง ผ่าน: teacher / blind 2b | ลง ล้ม: teacher / blind | ขึ้น ผ่าน: teacher / blind | ขึ้น ล้ม: teacher / blind |
|---|---|---|---|---|
| 0 cm | 70% / 59% | 0% / 0% | 78% / 66% | 3% / 0% |
| 3 cm | **100%** / 81% | 0% / 0% | 33% / 23% | 6% / 14% |
| 5 cm | **94%** / 69% | 0% / 0% | 45% / 44% | 5% / 25% |
| 8 cm | 0% / 62% | 0% / 20% | 9% / 0% | 14% / 94% |
| 10 cm | 0% / 55% | 0% / 28% | 0% / 0% | 6% / 86% |
| 12–18 cm | 0% / 38–50% | 0% / 31–61% | 0% / 0% | 0% / 30–81% |

ตอนเทรน: `terrain_levels` 1.5 → 2.2 แล้วนิ่งตั้งแต่ ~iteration 6600 (≈ ขั้นสูง 5–6 cm) · error_vel_xy ~0.73 m/s · ล้ม ~20%

อ่านผล:
- **ตั้งแต่ 8 cm teacher หยุดยืนหน้าขอบ** ทั้งขึ้นและลง (ไปได้ 0 ขั้น, x เฉลี่ย 0.4–0.8 m คือยังอยู่บน platform, ไม่ล้ม) ส่วน blind ไม่เห็นขอบจึงเดินลงไปและรอด 38–62% · teacher เห็นด้วย height scan ว่าเป็นพื้นแบบที่ไม่เคยฝึก (curriculum ค้างที่ ~5 cm) แล้วเลือกไม่เสี่ยง เพราะล้ม = −200
- **ที่ ≤ 5 cm teacher ดีกว่า blind ชัดเจน** โดยเฉพาะลง (94–100% vs 69–81%) และล้มน้อยกว่ามาก
- **สาเหตุที่น่าจะเป็น (ยังไม่ยืนยัน):** reward `foot_clearance` (`src/tasks/velocity/mdp/rewards.py`) เทียบความสูงเท้ากับ **z = 0.10 m ของโลก** ไม่ใช่กับพื้นใต้เท้า บนบันไดสูง เท้าที่ขยับอยู่ห่าง 0.10 m มาก (pyramid 8 cm: platform สูง 0.64 m · bowl: ก้นลึก −0.56 m) จึงโดนลงโทษตลอดเวลาที่ก้าว ยิ่งด่านสูงยิ่งคุ้มที่จะยืนเฉย → ไม่ผ่านด่าน → curriculum ค้าง
- แก้แล้ว 13/11: `rewards_stairs.py` `feet_clearance_terrain` ยิง ray ลงจาก ankle roll link ของแต่ละเท้า (เฉพาะ geom ของ terrain) วัดความสูงพื้นรองเท้าเหนือพื้นใต้เท้า น้ำหนักและเป้า 0.10 m เท่าเดิม · ตรวจ: หุ่น 40 ตัวยืนบนบันไดที่สูงถึง 1.5 m วัดได้ −0.003 ถึง 0.097 m · เทรนต่อด้วย `run_stairs_teacher_resume.sh` (resume `stairs_teacher2/model_7999` อีก 5000 iteration เป็น `stairs_teacher3`) · ยังไม่ทำ student จนกว่า teacher จะผ่าน ≥ 80% ที่ 8–10 cm

### ขั้น 3b: teacher3 (foot clearance เทียบพื้นใต้เท้า) (16/11/2026)

checkpoint `stairs_teacher3/model_12998.pt` (resume teacher2 + `4d163d8` อีก 5000 iteration) · ตอนเทรน `terrain_levels` 2.19 → 2.76, ล้ม ~16%, error_vel_yaw ~2.5 ไม่ลด

**eval แบบเดิม (สั่ง wz = 0) วัดผิดจุด:** หุ่นเดินเร็วตามสั่ง (vx_b ≈ 0.45–0.5) แต่หันตัวเองช้าๆ จน |yaw| ถึง 2.3 rad แล้วเดินอ้อมบันได (ขึ้น 2–3 ขั้นแล้วเลี้ยวลง) · ตอนเทรนคำสั่ง heading ส่ง wz แก้ทิศให้ตลอดจึงไม่เห็น · ใน eval `ang_vel_z = (0, 0)` ทำให้ wz ของ heading ถูก clip เป็น 0 → เพิ่ม `--heading-hold` (ให้แก้ทิศได้ ±1 rad/s แบบคนถือจอย) · ผลดิบ `step3b_teacher3_vx0.5.jsonl` (wz = 0), `*_hold.jsonl` (heading hold)

| สูงขั้น (heading hold) | ลง: teacher2 / teacher3 | ขึ้น: teacher2 / teacher3 | ล้ม teacher3 ขึ้น |
|---|---|---|---|
| 0 cm | 92% / 95% | 95% / 97% | 0% |
| 3 cm | 98% / 100% | 80% / 69% | 11% |
| 5 cm | 94% / 98% | **92% / 64%** | 2% |
| 8 cm | **2% / 95%** | **31% / 5%** | 0% |
| 10 cm | 0% / 11% | 2% / 0% | 0% |
| 12–18 cm | 0% / 0% | 0% / 0% | 0–2% |

อ่านผล:
- cost เดิม `|h − 0.10| × ความเร็วเท้า` ลงโทษทั้งต่ำและ**สูง**กว่าเป้า · teacher2 (h เทียบ world z): ใน bowl พื้นติดลบ ยกเท้าสูง = ใกล้ 0.10 = จ่ายน้อย → ช่วยขึ้นโดยบังเอิญ แต่ pyramid ลงโทษทุกก้าว · teacher3 (h เทียบพื้น): ลงหายติด แต่ก้าวขึ้นต้องยกเท้า rise + 10 cm เหนือขั้นล่างก่อนพื้นใต้เท้าจะสูงขึ้น → โดนลงโทษ → ยกต่ำ ขึ้นแย่ลง
- ≥ 10 cm ทั้งขึ้นและลงยังหยุดที่ขอบ (ไม่ล้ม): curriculum เพิ่งถึง ~7 cm

แก้ 16/11 → `stairs_teacher4` (resume teacher3 อีก 5000 iteration):
1. `feet_clearance_terrain(below_only=True)`: ลงโทษเฉพาะ `max(0, 0.10 − h)` ยกสูงข้ามขั้นไม่เสีย
2. `track_angular_velocity` weight 1.0 → 2.0 (เท่า linear) แก้หันตัวเอง
- เป้า: ≥ 80% ที่ 8–10 cm ทั้งขึ้นและลง ทั้ง eval แบบ wz = 0 และ heading hold

### ขั้น 3c: teacher4 (clearance ข้างเดียว + yaw tracking ×2) (17/11/2026)

checkpoint `stairs_teacher4/model_17997.pt` (resume teacher3 + `7e2c328` อีก 5000 iteration) · ผลดิบ `step3c_teacher4_vx0.5.jsonl` (wz = 0), `step3c_teacher4_vx0.5_hold.jsonl` (heading hold)

ตอนเทรน: `terrain_levels` 2.94 → 2.96 (นิ่งตั้งแต่ ~1000 iteration แรก) · error_vel_yaw 2.5 → **1.7** · error_vel_xy 0.72 · ล้ม ~18%

| สูงขั้น | ลง wz=0: t3 / **t4** | ลง hold: t3 / **t4** | ขึ้น wz=0: t3 / **t4** | ขึ้น hold: t2 / t3 / **t4** |
|---|---|---|---|---|
| 0 cm | 47% / 89% | 95% / 98% | 63% / 92% | 95% / 97% / 100% |
| 3 cm | 97% / 98% | 100% / 100% | 9% / **77%** | 80% / 69% / **95%** |
| 5 cm | 89% / 94% | 98% / 100% | 8% / **69%** | 92% / 64% / **97%** |
| 8 cm | 70% / **92%** | 95% / **100%** | 2% / 27% | 31% / 5% / **70%** |
| 10 cm | 16% / 11% | 11% / 11% | 0% / 3% | 2% / 0% / 9% |
| 12–18 cm | 0% | 0% | 0% | 0% |

ล้ม ≤ 5% ทุกเคส · ≥ 10 cm เดินถึงขอบ platform (x 0.6–1.2 m) แล้วยืนนิ่ง

อ่านผล:
- clearance ข้างเดียวได้ผล: ขึ้น 8 cm 5% → 70% โดยลงยัง 100% · teacher ตัวแรกที่ดีทั้งขึ้นและลง
- yaw ×2 ช่วยแต่ยังไม่หมด: wz = 0 ดีขึ้นมาก (ขึ้น 3 cm 9% → 77%) แต่ยังต่ำกว่า heading hold 20–40 pp
- เพดาน ~7–8 cm มาจาก curriculum ค้าง (หุ่นแทบไม่ได้ฝึกขั้นสูงกว่า) สาเหตุที่น่าจะเป็น:
  1. reward `pose` ตอนเดิน std knee / hip_pitch = 0.5 rad แต่ก้าวขึ้น 12–15 cm ต้องงอเข่าเพิ่ม ~0.9 rad → reward นี้หายเกือบหมด
  2. `is_terminated` −200 เทียบ tracking ~2 ต่อ step → ยืนนิ่งที่ขอบคุ้มกว่าเสี่ยง (หยุด ไม่ล้ม)
  3. `terrain_levels_vel` (mdp ที่ใช้ร่วม แก้ไม่ได้): ต้องไปไกล > 4 m ไม่งั้นลดแถว ขณะที่คำสั่งสุ่มใหม่ทุก 3–8 s → ระดับนิ่งที่จุดสำเร็จ ~50%
- เสนอ teacher5: ผ่อน `pose` std ตอนเดินของ knee/hip_pitch 0.5 → 1.0 (เฉพาะงานบันได) ยังไม่แตะ `is_terminated` เพื่อแยกผล · ยังรอขนาดบันไดจริง (ขั้น 0) เพื่อกำหนดเป้าความสูง

### ขั้น 3d: teacher5 (pose std knee/hip_pitch 1.0) (18/11/2026)

checkpoint `stairs_teacher5/model_22996.pt` (resume teacher4 + `9319338` อีก 5000 iteration) · ผลดิบ `step3d_teacher5_vx0.5.jsonl` (wz = 0), `step3d_teacher5_vx0.5_hold.jsonl` (heading hold)

ตอนเทรน: `terrain_levels` 2.96 → 3.04 (iteration 20286) → **2.74** ตอนจบ · error_vel_yaw 1.63 → 1.81 · ล้มลดลงครึ่งหนึ่ง (fell_over 0.75 → 0.375 ต่อรอบ log)

| สูงขั้น | ลง wz=0: t4 / **t5** | ลง hold: t4 / **t5** | ขึ้น wz=0: t4 / **t5** | ขึ้น hold: t4 / **t5** |
|---|---|---|---|---|
| 0 cm | 89% / 88% | 98% / 100% | 92% / 89% | 100% / 98% |
| 3 cm | 98% / 95% | 100% / 100% | 77% / 72% | 95% / 97% |
| 5 cm | 94% / 98% | 100% / 100% | 69% / 58% | 97% / 97% |
| 8 cm | 92% / 98% | 100% / 100% | 27% / 9% | 70% / 56% |
| 10 cm | 11% / **83%** | 11% / **98%** | 3% / 3% | 9% / 8% |
| 12 cm | 0% / **34%** | 0% / **36%** | 0% / 2% | 0% / 0% |
| 15–18 cm | 0% | 0% | 0% | 0% |

ล้ม ≤ 5% ทุกเคส

อ่านผล:
- pose std ช่วย **ขาลง** ชัดเจน (10 cm 11% → 98%, 12 cm 0% → 36%): ลงขั้นสูงต้องงอเข่าขาที่ยืนมาก ซึ่ง std 0.5 เคยลงโทษ
- **ขาขึ้นไม่ดีขึ้น** (8 cm แย่ลง 70% → 56%, wz = 0 27% → 9%) และ 10 cm ยัง ~8% · หุ่นยังหยุดที่ขอบ ไม่ได้ล้ม
- curriculum ถอยกลับ (3.04 → 2.74) ทั้งที่ล้มน้อยลง = เลือกเล่นปลอดภัย ตรงกับสาเหตุข้อ 2 ของขั้น 3c (`is_terminated` −200 แพงกว่าการยืนรอ)
- teacher6 (`a59b530`): `is_terminated` −200 → −100 และ reward `progress_along_command` (w 1.0, เชิงเส้น, ยืนนิ่งได้ 0; tracking แบบ exp ยังให้ 37% ตอนยืนนิ่งที่ cmd 0.5 m/s) · resume จาก **teacher5** (ขาลง 10–12 cm ได้แล้ว ขาขึ้นใกล้ teacher4; ตอนแรกตั้งเป็น teacher4 ตาม `terrain_levels` แต่ eval บอกว่า teacher5 ดีกว่า) · คง pose std 1.0 ไว้เพราะช่วยขาลง

### ขั้น 3e: teacher6 (cheaper falls + progress reward) (19/11/2026)

run `2026-11-18_11-54-57_stairs_teacher6` (resume teacher5 model_22996, 5000 iteration) · ผลดิบ `step3e_teacher6_<ckpt>_vx0.5{,_hold}.jsonl`

ตอนเทรน: `terrain_levels` 1.7 → **3.81** (iteration ~24300–24500) แล้วถอยเหลือ 3.09 ตอนจบ · ตั้งแต่ iteration 24957 `action_rate_l2` ระเบิดเป็นช่วงๆ (แย่สุด −794,913 จากปกติ −1.1) ทำให้ `Loss/value` ถึง 4.7e13 แล้ว policy เสื่อม (std 0.57 → 0.69, ล้มเพิ่ม) · เริ่มเกิดตั้งแต่ teacher5 (iteration 19601, 14 ครั้ง) teacher2–4 ไม่เคยเกิด · action ไม่ได้ clip (`clip_actions` = None)

heading hold (success %):

| สูงขั้น | ลง t5 / 24500 / 24900 / 27995 | ขึ้น t5 / 24500 / 24900 / 27995 |
|---|---|---|
| 5 cm | 100 / 100 / 100 / 100 | 97 / **100** / 95 / 86 |
| 8 cm | 100 / 100 / 100 / 100 | 56 / **98** / 88 / 58 |
| 10 cm | 98 / 100 / 100 / 100 | 8 / **75** / 27 / 17 |
| 12 cm | 36 / **100** / 98 / 100 | 0 / 12 / 2 / 3 |
| 15 cm | 0 / **91** / 92 / 86 | 0 |
| 18 cm | 0 / **84** / 69 / 58 | 0 |

ล้ม ≤ 9% ยกเว้นลง 18 cm (8–12%) · โหมด wz = 0 ขาขึ้นยังแย่ (5 cm 23%, 8 cm 5% ที่ 24500) = หุ่นหันเบี่ยงเองเมื่อไม่มีการแก้ heading

อ่านผล:
- **model_24500 ดีที่สุด** (ก่อน spike แรก): ขึ้น 8 cm 98%, ขึ้น 10 cm 75% (เป้า 80%), ลงได้ถึง 18 cm 84% → progress reward + โทษล้มที่ถูกลงแก้ปัญหาหยุดที่ขอบได้
- checkpoint หลัง spike แย่ลงตามลำดับ → ปัญหาคือความเสถียรเชิงตัวเลข ไม่ใช่ reward
- ขนาด action ของ 24500 (512 env บน terrain เทรน): |a| p50 0.27, p99 3.06, p99.9 4.46, max 10.7 (เข่า)
- teacher7: resume 24500 + `--agent.clip-actions=10` (ไม่ตัด action ปกติ แต่กัน `action_rate` ระเบิด) · deploy ต้อง clip ค่าเดียวกัน


### ขั้น 3f: หุ่นวางเท้าตรงไหนบนบันได + teacher7 (19/11/2026)

`foothold_open_stairs.py` (setup เดียวกับ eval) บันทึกตำแหน่งฝ่าเท้า (ส้น x −0.064 ถึงปลาย x 0.142 ใน ankle_roll_link) ทุกครั้งที่เท้าแตะพื้น เทียบกับลูกนอนที่เหยียบ · model_24500, heading hold, 64 env

ablation height scan ก่อน (`scan_ablation`): scan แบน = ไม่ขึ้นขั้นเลย (0%) และเดินตกขอบตอนลง (ล้ม 48–92%) · noise ±10 cm แทบไม่เปลี่ยนผล → policy **ใช้ scan จริง**

| กรณี | สำเร็จ | กลางเท้า − กลางลูกนอน (สำเร็จ / ไม่สำเร็จ) | ส้นล้นขอบหลัง > 2 cm | support < 70% | step เท้าโดนแรงแนวนอน / ตัว |
|---|---|---|---|---|---|
| ขึ้น 5 cm | 100% | −6 cm | 40% | 34% | 6 |
| ขึ้น 8 cm | 97% | −10 / −17 cm | 47 / 70% | 43 / 64% | 8 / 31 |
| ขึ้น 10 cm | 72% | −13 / −16 cm | 52 / 68% | 48 / 63% | 10 / 34 |
| ลง 10 cm | 100% | +10 cm | (ปลายล้นหน้า 62%) | 47% | 2 |
| ลง 15 cm | ~87% | +9 / +9 cm | (ปลายล้นหน้า 58%) | 41 / 45% | 3 / 97 |

อ่านผล: หุ่นรู้ว่ามีขั้นและยกเท้าพ้น แต่**ไม่ปรับระยะก้าว** · ขาขึ้นเท้าลงชิดขอบหลัง ส้นลอยเหนือช่องว่าง ยิ่งสูงยิ่งเยื้อง ตัวที่ล้มเหยียบแค่จมูกบันได · ปลายเท้ามุดใต้ขั้นถัดไปแทบไม่มี (1–2%) = ที่ด้านหน้ายังเหลือ · ไม่มี reward ไหนสอนเรื่องนี้

teacher7 (`rewards_stairs.py`, `env_cfgs_stairs.py` ข้อ 10–11, `run_stairs_teacher_resume.sh`):
- `foothold` (w −1.0): ray 6 เส้นต่อเท้า (ส้น/กลาง/ปลาย × ซ้าย/ขวา) ยิงลงจากความสูงข้อเท้า หมุนตาม yaw ของเท้า · เท้าที่แตะพื้น: สัดส่วนจุดที่พื้นใต้ต่ำกว่าพื้นสูงสุดใต้เท้าเดียวกัน > 1.5 cm · เทียบกับพื้นใต้เท้าเอง (ไม่ใช่ความสูงฝ่าเท้า) เพราะแบบแรกโดนโทษ ~0.18/step บนพื้นเรียบตอนเปิดส้น · ตรวจกับ 24500: พื้นเรียบ 0.000, ขึ้น 10 cm เฉลี่ย 0.16, ลง 15 cm 0.16
- `stumble` (w −0.5): จำนวนเท้าที่แรงสัมผัสแนวนอน > แนวตั้ง และ > 20 N
- `clip_actions` 10 ตั้งใน `rl_cfg.py` (`STAIRS_CLIP_ACTIONS`, runner cfg ของ Teacher / Depth / AMP; Flat / Rough / Blind ไม่ clip เหมือนเดิม) · eval checkpoint เก่า (t4–t6) จะ clip 10 ด้วย ผลแทบไม่ต่างเพราะ action ไม่ค่อยถึง 10
- เป้า: ขึ้น 10 cm ≥ 80% และวัดซ้ำด้วย `foothold_open_stairs.py`: กลางเท้าเยื้อง ≤ ±5 cm, ส้นล้นขอบ < 15%
- รัน (DGX): `./run_stairs_teacher_resume.sh` (ค่าเริ่มต้น = teacher6 model_24500, 3000 iteration, clip 10)

#### ผล teacher7 แบบ clip อย่างเดียว (20/11/2026)

run `2026-11-19_11-02-54_stairs_teacher7` เริ่มก่อน commit reward foothold: resume teacher6 model_24500 + `--agent.clip-actions=10` อย่างเดียว 3000 iteration · ผลดิบ `step3f_teacher7clip_27499_vx0.5{,_hold}.jsonl`

ตอนเทรน (เทียบ teacher6 ช่วง iteration เดียวกัน): `action_rate_l2` แย่สุด −0.94 (t6 −794,913) · `Loss/value` สูงสุด 0.28 (t6 4.7e13) · `terrain_levels` 3.76–3.85 นิ่ง (t6 ถอยเหลือ 3.0–3.5) · std 0.57 คงที่ → **clip แก้ปัญหาค่าระเบิดได้ทั้งหมด**

heading hold, success % (t6 24500 / t7 27499):

| สูงขั้น | ลง | ขึ้น |
|---|---|---|
| 0–8 cm | 98–100 / 100 | 98–100 / 98–100 |
| 10 cm | 100 / 100 | 75 / 62 (128 env: 75.0 / **75.8**) |
| 12 cm | 100 / 100 | 12 / **38** (128 env: 16.4 / **34.4**) |
| 15 cm | 91 / 98 | 0 / 0 |
| 18 cm | 84 / 75 | 0 / 0 |

wz = 0 ขาขึ้นดีขึ้นเล็กน้อย (3 cm 45 → 59%, 5 cm 23 → 33%) · ล้ม ≤ 6% ทุกกรณี

ตำแหน่งเท้า (ขึ้น 10 cm, 128 env, ตัวที่สำเร็จ): กลางเท้า −14.1 cm (t6 −12.0), ส้นล้นขอบหลัง 55% (t6 51%) = ไม่เปลี่ยน ตามคาดเพราะยังไม่มี reward foothold

อ่านผล: clip ทำให้เทรนนิ่งและไม่เสื่อม ขึ้น 12 cm ดีขึ้น ~2 เท่า แต่ขึ้น 10 cm ยังประมาณ 75% (เป้า 80%) · run นี้ใช้เป็นตัวเทียบของ `stairs_teacher7_foothold` (เงื่อนไขเดียวกัน ต่างแค่ reward foothold + stumble)

### ขั้น 3g: teacher7_foothold (foothold + stumble) (20/11/2026)

run `2026-11-20_10-47-00_stairs_teacher7_foothold` (DGX): resume teacher6 model_24500 อีก 3000 iteration · clip 10 + `foothold` w −1.0 + `stumble` w −0.5 (setup ขั้น 3f) · ต่างจาก teacher7 clip แค่ 2 reward นี้ · ผลดิบ `step3g_teacher7foothold_27499_vx0.5{,_hold}.jsonl`

ตอนเทรน (iteration 25100 → 27499, teacher7 clip ในวงเล็บ): `foothold` −0.074 → −0.072 (แทบไม่ลด) · `stumble` −0.019 → −0.020 · `terrain_levels` 3.67 → 3.80 (3.81) · `progress` 0.59 (0.59) · `fell_over` 0.57 (0.57) · เทรนนิ่ง ไม่ระเบิด

heading hold, success % (t7 clip / **t7 foothold**):

| สูงขั้น | ลง | ขึ้น |
|---|---|---|
| 0–5 cm | 100 / 98–100 | 98–100 / 94–98 |
| 8 cm | 100 / 100 | 98 / 91 |
| 10 cm | 100 / 100 | 62 / 41 (128 env: 75.8 / **50.8**) |
| 12 cm | 100 / 97 | 38 / 6 (128 env: 34.4 / **8.6**) |
| 15 cm | 98 / 97 | 0 / 0 |
| 18 cm | 75 / **86** | 0 / 0 |

wz = 0 แย่ลงทั้งขึ้นและลง: ขึ้น 3 cm 59 → 52%, 5 cm 33 → 14% · ลง 15 cm 94 → 83%, 18 cm 80 → 56% (ล้ม 9%)

ตำแหน่งเท้า (ขึ้น 10 cm, 64 env, ตัวที่สำเร็จ; t7 clip / t7 foothold): สำเร็จ 73 / 58% · กลางเท้า −13.3 / −10.4 cm · ส้นล้นขอบหลัง 54 / 45% · support median 0.75 / 1.0 · step เท้าโดนแรงแนวนอน / ตัว 6.7 / 8.0 → **ดีขึ้นนิดเดียว ยังห่างเป้า** (±5 cm, < 15%)

อ่านผล:
- ขาขึ้น 10 cm ล้มแค่ 0.8% แต่สำเร็จ 51%, median ขึ้นได้ 4/7 ขั้น = **หุ่นติดค้างหน้าขั้น ไม่ใช่ล้ม**
- สาเหตุที่น่าจะเป็น: ตอนก้าวขึ้น ปลายเท้าแตะจมูกบันไดหรือเหยียบไม่เต็มก่อนเสมอ โดนทั้ง foothold และ stumble → policy เลือกยืนเท้าเต็มฝ่าบนขั้นเดิมแทนการก้าว
- ลง 18 cm แบบ heading hold ดีขึ้น (75 → 86%) แต่แบบ wz = 0 แย่ลง → ไม่นับเป็นข้อดีที่ชัด
- reward แบบนี้ไม่ได้สอนให้ "ก้าวลึกขึ้น" แต่สอนให้ "เลี่ยงขอบ" ซึ่งทางที่ง่ายที่สุดคือไม่ก้าว

ตัดสินใจ:
- **teacher หลัก = teacher7 clip (`2026-11-19_11-02-54_stairs_teacher7/model_27499.pt`)** · พัก foothold / stumble รูปแบบนี้
- ถัดไป: resume teacher7 clip เข้า `Unitree-G1-23Dof-OpenStairs-Teacher-AMP` ใช้ `walk1_straight.npz` (LAFAN1 walk1 200.5–208.5 s, เดินตรง 0.72 m/s) เป็นต้นแบบท่า แล้วเทียบกับ teacher7 clip ด้วย eval ชุดเดียวกัน
- ถ้าลอง foothold อีก: หักเฉพาะเท้าที่วางนิ่งแล้ว (ไม่หักช่วงแตะพื้นแรก), ตัด stumble, w −0.3

### ขั้น 3h: teacher8 = teacher7 clip + stair token (23/11/2026)

อ้างอิง Zhang et al., "Explicit Stair Geometry Conditioning for Robust Humanoid Locomotion" (arXiv 2605.09944): ป้อนเรขาคณิตบันไดเป็นตัวเลขไม่กี่ค่าให้ policy ตรงๆ (เขา: ประเภทพื้น, h, d, yaw → ซิม 96% เทียบ HeightMap 88%, หุ่นจริงขึ้น 33 ขั้น) · เหตุผลที่ลองกับเรา: ขั้น 3f พบว่าหุ่น**ไม่ปรับระยะก้าว** และ foothold penalty (3g) ทำให้หยุดก้าว → ลองบอกความลึกขั้นตรงๆ แทนการลงโทษ

`stair_token.py`: ray 3 เส้นตาม heading ของ pelvis (y −0.15 / 0 / +0.15 m, x −0.2 ถึง 1.2 m ทุก 2 cm, terrain เท่านั้น) หาขอบขั้นจากความสูงที่กระโดด > 1 cm · token 4 ค่า:
- `h` ความสูงขั้นของขอบแรกข้างหน้า (+ ขึ้น / − ลง, 0 = ไม่มีขอบ) ใช้แทน "ประเภทพื้น" ของเปเปอร์
- `d` ความลึกขั้น (ขอบแรก → ขอบสอง แก้ด้วย cos yaw)
- `yaw` มุมหุ่นเทียบแนวขอบ (จากจุดที่เส้นซ้าย/ขวาตัดขอบเดียวกับเส้นกลาง)
- `dist` ระยะ pelvis ถึงขอบแรก (ช่องที่ 4 แทนประเภทพื้น เพราะเกี่ยวกับการวางเท้าโดยตรง)

วัดจาก ray ไม่ได้อ่านจาก terrain generator → ใช้ได้ทั้ง pyramid ตอนเทรน บันไดตรงของ eval และให้ student เรียนประมาณจากกล้องได้ภายหลัง · ตรวจกับค่าจริงบนบันได eval (16 env, 6 s): h ผิด 0 ทุกกรณี · d median ผิด 2.5 mm (p95 1.9 cm) · yaw median ผิด 0.019 rad (ความละเอียด ~0.067 rad จาก ray 2 cm; ผิดมากเฉพาะตอนหุ่นหันเกิน ~0.5 rad)

resume แบบไม่เทรนใหม่: `expand_checkpoint_obs.py` ขยายชั้นแรก actor 267 → 271, critic 282 → 286 (น้ำหนักใหม่ = 0, normalizer mean 0 / std 1, Adam state pad 0) · ตรวจแล้ว action ของ teacher8 ก่อนเทรนเท่า teacher7 (ต่างสูงสุด 1.4e-6 ตลอด 300 step) · env = teacher7 (ไม่มี foothold / stumble) + token ต่อท้าย actor และ critic

- รัน (DGX): `./run_stairs_teacher8.sh` (teacher7 model_27499 → 3000 iteration, run `stairs_teacher8`)
- ความเสี่ยง: policy อาจไม่ใช้ token เพราะ teacher7 พึ่ง height scan อยู่แล้ว · ตรวจด้วย ablation (ใส่ token ผิด แล้วดูว่าการวางเท้าเปลี่ยนไหม) · ถ้าไม่ใช้: `NOISE=0.01` หรือเทรนนานขึ้น
- เป้าเดิม: ขึ้น 10 cm ≥ 80%, กลางเท้าเยื้อง ≤ ±5 cm, ส้นล้นขอบ < 15% · eval: `--task Unitree-G1-23Dof-OpenStairs-Teacher8`

ผล teacher8 (24/11, run `2026-11-24_10-58-19_stairs_teacher8`, model_30498; ตัวเทียบ = teacher7_amp ที่เริ่มจุดเดียวกันและเทรนนานเท่ากัน): heading hold ขึ้น 10 / 12 cm (128 env) **96.9 / 92.2%** (amp 81.2 / 46.9) · ขึ้น 15 cm 52% (amp 3%) · ลง 18 cm 97% · wz = 0 ขึ้น 10 cm 23% · กลางเท้า −9.4 cm, ส้นล้นขอบ 44% (amp −14.1 / 54%) · ablation (`STAIR_TOKEN_ABLATE=flat`) ขึ้น 10 / 12 cm ยังได้ 94.5 / 91.4% → actor แทบไม่พึ่ง token ตอนเดิน · ผลดิบ `step3h_teacher8_30498_vx0.5{,_hold}.jsonl`

### ขั้น 3i: teacher9 / teacher9b (24–25/11/2026) — ไม่ผ่าน

- **teacher9** (resume teacher8): foothold −3 (ไม่มี stumble) + token noise / dropout + บันไดตรงมีผนัง 50% + curriculum แบบ Chebyshev · ผล: หยุดยืนหน้าขั้นแรก ขึ้น 10 cm hold 97 → **4%** · `terrain_levels` ค้าง ~2.5 (~6 cm) ตลอด run · สาเหตุ: (1) foothold ปรับเฉพาะตอนยืนบนขั้น → ยืนหน้าบันไดคุ้มกว่า (เหมือน teacher7_foothold จึงไม่ใช่ความผิดของ stumble) (2) เกณฑ์ลดระดับของ mjlab (เดินไม่ถึงครึ่งของระยะที่สั่ง ~5 m) ลดระดับหุ่นบนบันไดตรงที่ผนังขวาง
- **teacher9b** (resume teacher8): teacher9 ไม่มี foothold + เพดานลดระดับ 2 m · `terrain_levels` 4.36 (แก้ curriculum ได้) · hold ขึ้น 10 / 12 / 15 cm 96 / 76 / 31% (แย่กว่า teacher8 ที่ขั้นสูง) · **wz = 0 ขึ้น 10 cm 22% (ไม่ดีขึ้น)** · ผลดิบ `step3i_teacher9{,b}_33497_vx0.5{,_hold}.jsonl`
- **สาเหตุจริงที่ขึ้นตรงไม่ได้** (วัด yaw ตอน wz = 0 ขึ้น 10 cm, ขอบแรก x = 1.04 m): ที่ 2 s |yaw| median 15° → ที่ 4 s (ถึงขอบ) **91°** (98% ของหุ่นหันเกิน 29°) ความเร็วเดินหน้ายัง 0.5 m/s = หุ่น**หันหนีขั้นแล้วเดินเลียบ** ไม่ได้ล้มหรือหยุด · teacher8 อาการเดียวกัน · เพราะ track_linear_velocity และ progress วัดใน body frame → "เดินหน้า" ได้คะแนนเต็มไม่ว่าหันไปทางไหน · วัดกับ teacher8: progress ตอนหันหนี 0.91 **มากกว่า**ตอนหันเข้าบันได 0.82 · การไปขึ้นที่มุมบันไดวงเป็นอาการเดียวกัน (หาทางง่ายแล้วยังได้ reward เต็ม)
- บนพื้นราบ wz = 0 หุ่นก็ค่อยๆ หันเอง ~56° ใน 14 s → deploy ต้องมีตัวคุม heading (คนบังคับ หรือ heading hold จาก IMU yaw ในโค้ด deploy ที่เป็นไฟล์ร่วม)

ตัดสินใจ: หยุดสาย resume (teacher3 → 9b ต่อกันมา นิสัยหันหนีฝังอยู่ และ curriculum เริ่มใหม่ทุกครั้งที่ resume) → เทรนใหม่ตั้งแต่ศูนย์ด้วยกฎที่เรียนมา (ขั้น 4)

## ขั้น 4: teacher-v2 เทรนใหม่ตั้งแต่ศูนย์ (25/11/2026)

ไฟล์: `env_cfgs_stairs_v2.py` (กฎทั้งหมดในที่เดียว เริ่มจาก rough env ไม่ต่อจาก teacher env เดิม) · `heading_lock.py` · task `Unitree-G1-23Dof-OpenStairs-TeacherV2` · obs = actor 271 / critic 286 (เรียงเหมือน teacher8: proprio + height scan + stair token) ตั้งแต่ต้น ไม่ต้องผ่าตัด checkpoint

กฎ (รหัสตรงกับ docstring ของ `env_cfgs_stairs_v2.py`):

| รหัส | กฎ | มาจาก |
|---|---|---|
| R1 | เริ่มที่ terrain แถวง่ายสุด (`max_init_terrain_level = 1`) | teacher1 |
| R2 | คำสั่งช่วงเดียวตามที่ deploy ส่งได้ (vx −0.5..1.0, vy ±0.5, wz ±1) | teacher1 |
| R3 | foot clearance วัดจากพื้นใต้เท้า ปรับเฉพาะยกต่ำ | teacher2/3 |
| R4 | track_angular_velocity w 2.0 | teacher3 |
| R5 | pose std hip pitch / knee = 1.0 ตอนเดิน | teacher4 |
| R6 | is_terminated −100 | teacher5 |
| R7 | progress แบบ linear (ยืนเฉยได้ 0) | teacher6 |
| R8 | stair token ใน obs; ฝั่ง actor มี noise (h 1 cm, d 1.1 cm, yaw 4°, dist 2 cm) + dropout 10% เป็น "ไม่มีขอบ" | teacher8/9b |
| R9 | terrain: บันไดวง 60% + บันไดตรงมีผนัง 30% + พื้นราบ 10% · เลื่อนระดับด้วย Chebyshev distance · ลดระดับเพดาน 2 m | teacher9/9b |
| **N1** | **track_linear_velocity และ progress วัดใน world frame เทียบทิศอ้างอิงที่หุ่นขยับเองไม่ได้** (heading env: heading_target; wz env: yaw ตอน resample + อินทิกรัล wz ที่สั่ง) · ไม่ปรับความเร็วแนวดิ่ง (ขึ้นขั้นต้องมี) | วิเคราะห์ 25/11 |
| **N2** | heading drift `1 − cos(yaw − ref)` w −0.5 (ต่ำกว่า progress เพื่อไม่ให้ยืนนิ่งคุ้มกว่า) | วิเคราะห์ 25/11 |
| **N3** | heading-command envs 80% (mjlab 30%) → หันหนีแล้วคำสั่ง wz ดึงกลับตลอด | วิเคราะห์ 25/11 |
| ตัดออก | foothold / stumble แบบลงโทษ (หยุดหน้าขั้น 2 ครั้ง) · AMP กับคลิปพื้นราบ (discriminator พัง) | 3g, 3i |

ตรวจ N1 ก่อนเทรน (teacher8 บน env v2, wz = 0 ขึ้น 10 cm): progress body / **locked** ตอนหันเข้าบันได 0.82 / 0.81 · ตอนหันหนีเกิน 60° 0.91 / **0.08** · tracking body / **locked** 0.81 / **0.11** → reward ใหม่ไม่ให้คะแนนการหันหนีแล้ว · smoke test (64 env) และ `RESUME=1` ผ่าน

- รัน (DGX): `./run_stairs_teacher_v2.sh` (30000 iteration, run `stairs_teacher_v2`, ~2 วัน) · ถ้าเครื่องล่ม: `RESUME=1 ./run_stairs_teacher_v2.sh`
- เช็กระหว่างทางทุก ~5000 iteration: `./eval_stairs_quick.sh Unitree-G1-23Dof-OpenStairs-TeacherV2 <run>/model_N.pt` (ขึ้น 8/10/12 cm แบบ wz = 0 และ hold, ~5 นาที; teacher8 ได้ wz0 31 / 20 / 17%, hold 97 / 98 / 92%)
- **เป้า (ตั้งไว้ก่อนดูผล):** wz = 0 ขึ้น 10 cm ≥ 80% (ตัวหลัก) · hold ขึ้น 10 / 12 cm ≥ teacher8 (97 / 92%) · กลางเท้า ±5 cm, ส้นล้นขอบ < 15% (รอง)
- ดูระหว่างเทรน: `Metrics/twist/heading_drift` ควรลดลง · `terrain_levels` ควรไต่ขึ้นต่อเนื่อง · ถ้า `progress` ต่ำและ `stand_still` สูงนาน = หยุดหน้าขั้น → ลด `V2_HEADING_DRIFT_W`
- ความเสี่ยง: ขนาดบันไดจริงยังไม่ได้วัด (tread 0.25–0.32 m, rise 2–18 cm เป็นค่าสมมติ) · เรื่องวางเท้ายังไม่มีกฎ (ถ้า v2 ขึ้นตรงได้แล้ว ค่อยลอง reward บวกตามระยะจากกลางขั้น)

### ขั้น 4 run 1: หยุดที่ 10k (26/11) และแก้ N1

run `stairs_teacher_v2` แรก (DGX) ที่ iteration 10131: ล้ม : หมดเวลา = **6.25 : 1.54** · episode 522 step · pose reward 0.001 · action std 0.76 (teacher2/3 ช่วงเดียวกัน: 0.75 : 3.8 · ~930 step · pose ~0.45) → หยุด run

สาเหตุ: N1 เวอร์ชันแรกตั้ง ref = heading_target **ทันทีที่สุ่มคำสั่ง** → เป้าอยู่ข้างหลัง คำสั่ง "เดินหน้า" ถูกวัดเป็น "เดินถอยหลัง" ตลอดช่วงหมุนตัว (สูงสุด ~3 s) ไปทางที่ policy มองไม่เห็น · ทดสอบกับ teacher8 (เป้า 180° ข้างหลัง, vx 0.5): progress แบบเดิม = 0.00 ตลอด 2 s แรก ทั้งที่หุ่นเดินและหมุนตามคำสั่งถูกต้อง (body frame 0.97) · ตอนทดสอบก่อนรัน ตรวจแค่กรณีหันหนีบนบันได eval ที่ heading คงที่ จึงไม่เจอ

แก้ (`heading_lock.py`):
- heading env: ref = ทิศหุ่นระหว่างหมุนไปหาเป้า → **ล็อก ref = เป้า เมื่อหันเข้าใกล้ < 15°** จนกว่าจะสุ่มคำสั่งใหม่
- wz env / standing: ref = yaw ตอนสุ่ม + อินทิกรัล wz ที่สั่ง แล้วไหลเข้าหาทิศหุ่นด้วย time constant 2 s (กัน error สะสมที่มองไม่เห็น)

ตรวจ (teacher8 บน env v2):

| กรณี | progress body | progress ใหม่ | progress v2 run 1 |
|---|---|---|---|
| เป้า 180° ข้างหลัง ระหว่างหมุน (0–2 s) | 0.84–0.98 | **0.84–0.98** | 0.00 |
| หันเข้าบันได (wz = 0) | 0.82 | 0.81 | 0.81 |
| หันหนีเกิน 60° (wz = 0) | 0.89 | **0.22** | 0.08 |

หันหนียังได้ 0.22 เพราะ wz env 20% ที่ ref ไหลตามหุ่น (env ที่ล็อกแล้ว 80% ได้ ~0) · smoke test ผ่าน · เริ่มเทรนใหม่จากศูนย์ (`./run_stairs_teacher_v2.sh`)

เช็กช่วงแรก: ~2000 iteration ล้ม : หมดเวลา ควรใกล้ teacher2/3 (หมดเวลามากกว่าล้ม) · episode > 800 step · pose > 0.3 · ถ้ายังเหมือน run 1 ให้หยุดเร็ว

### Step 4 run 2 (teacher-v2, from scratch after the lock fix; power cut at ~2150, resumed from model_2100)
Quick eval of model_15000 (64 envs, 7 steps up, success %):

| | 8 cm | 10 cm | 12 cm |
|---|---|---|---|
| wz0 teacher-v2 | **81.2** | **84.4** | **78.1** |
| wz0 teacher8 | 31 | 20 | 17 |
| hold teacher-v2 | 85.9 | 70.3 | 84.4 |
| hold teacher8 | 97 | 98 | 92 |

Straight climbing at wz = 0 reaches the step-4 target (>= 80% at 10 cm): the heading-locked
rewards fixed the turn-away. **But the posture is unusable**: walking flat at vx 0.5 it twists
the waist +1.78 rad, raises both arms (shoulder pitch +1.2), bends the knees 1.1 rad (pelvis
0.66 m instead of 0.77). model_2100 was still normal. The shared pose reward
exp(-mean(err^2/std^2)) is ~exp(-6) there, so there is no gradient back (log: pose 0.0015).

### Step 4b: teacher-v2b (posture.py)
Two penalties that grow with the error. Measured at vx 0.5, flat / up 12 cm:

| | model_2100 | model_15000 |
|---|---|---|
| upper-body sum dq^2 (waist + arms) | 0.14 / 0.28 | 11.9 / 12.2 |
| pelvis above mean foot z, m | 0.755 / 0.734 | 0.640 / 0.636 |

- N4 `upper_body_deviation` -0.5 x sum dq^2 (normal -0.07, twisted -6)
- N5 `pelvis_height` -20 x max(0, 0.70 - h)
- Fine-tune model_15000 for 5000 iterations, terrain starting at level <= 3
  (`run_stairs_teacher_v2b.sh`). If posture returns but wz0 drops a lot: from scratch with N4/N5.
