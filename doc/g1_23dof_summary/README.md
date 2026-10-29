# G1-23DoF: สรุปงานทั้งหมด (สถานะ 29/10/2026)

สรุปงาน velocity policy ของ G1-23DoF ตั้งแต่การเทรน, ทดสอบ sim2sim, ตรวจหุ่นจริง, kp/kd, ไปจนถึงแผน deploy รอบแรก
แยกเป็นไฟล์ตามเรื่อง ไฟล์นี้เป็นสารบัญ, สถานะล่าสุด และงานที่ค้าง

## สารบัญ

| ไฟล์ | เรื่อง |
|---|---|
| [01_training.md](01_training.md) | สาย policy ทั้งหมด (P1 → P3b, Phase 4, Hist5, yaw_ft, yaw_stand), วิธีเทรนแบบ best practice, คำสั่งรันบน DGX |
| [02_sim2sim_eval.md](02_sim2sim_eval.md) | วิธีวัด sim2sim (headless และ `unitree_mujoco` + `g1_ctrl`), ผลเทียบทุก policy, ปัญหาที่เจอ |
| [03_kp_kd.md](03_kp_kd.md) | kp/kd คืออะไร มาจากไหน เทียบกับที่อื่น และวิธีตรวจกับหุ่นจริง 4 ชั้น |
| [04_real_robot_tools.md](04_real_robot_tools.md) | ผลตรวจหุ่นจริงแบบอ่านอย่างเดียว (28/10) และเครื่องมือทุกตัว |
| [05_deploy_safety.md](05_deploy_safety.md) | แผน deploy รอบแรก (P3b), ผลตรวจความปลอดภัย, กลไกหยุด, FixStand, soft start |
| [06_cross_sim_envs.md](06_cross_sim_envs.md) | unitree_rl_lab (Isaac Lab) กับ G1-23dof, แผนที่ environment ของ Isaac ทุกตัว |

เอกสารเดิมที่ยังใช้อ้างอิง:

| ไฟล์ | เรื่อง |
|---|---|
| [`../g1_23dof_real_robot_setup_checklist.md`](../g1_23dof_real_robot_setup_checklist.md) | checklist หน้างาน: setup (หัวข้อ 1–8) และ deploy รอบแรก (หัวข้อ 9) |
| [`../domain_randomization_phases_g1_23dof.md`](../domain_randomization_phases_g1_23dof.md) | DR ของแต่ละ phase |
| [`../g1_23dof_metrics_manager.md`](../g1_23dof_metrics_manager.md) | metric ตอนเทรน และผล sim2sim ของ P2/P3b (27/10) |
| `../g1_23dof_*_manager.md` | อธิบาย manager แต่ละตัวของ mjlab |

## สถานะล่าสุด

| เรื่อง | สถานะ |
|---|---|
| ตรวจหุ่นจริงแบบอ่านอย่างเดียว | ✅ ผ่าน 28/10: joint map 23 ตัว, ทิศ, IMU, LowState 1044 Hz |
| policy สำหรับ deploy รอบแรก | **P3b** (`config/policy/velocity/v0`, md5 `727da8a2…`) |
| ตรวจ kp/kd ในไฟล์ | ✅ `gain_check.py` ผ่านทั้ง P3b, yaw_ft และ yaw_stand |
| ตรวจ kp/kd กับหุ่นจริง | ⏳ 30/10 checklist หัวข้อ 9.1 (แขวน, Passive + FixStand) |
| yaw_ft (Hist5 + fine-tune การหมุน) | หมุนได้ ~78% แต่ยืนส่าย → แก้ด้วย yaw_stand |
| yaw_stand (`hist5_yawstand`) | ✅ ยืนนิ่ง, vx 0.8/1.0 ≥ 94%, หมุน 72–80% → ตัวเลือก policy ตัวที่สอง (ยังไม่ทดสอบกับ `g1_ctrl` จริง) |
| B: soft start 0.5 s ตอนเข้า Velocity | ✅ ทดสอบใน sim แล้ว (P3b entry 0.28 → 0.015 rad) |
| policy สาย Hist5 ตอนแขวน | ❌ เตะขาแรงเมื่อเท้าไม่แตะพื้น ห้ามเข้า Velocity ตอนแขวน (P3b นิ่ง) |
| กล้อง / lidar | ยังไม่ใช้ policy ทุกตัวเป็น blind |

## งานที่ค้าง

1. หาทางให้ policy สาย Hist5 ไม่เตะขาตอนแขวน หรือกำหนดขั้นตอน deploy ที่เข้า Velocity ตอนเท้าแตะพื้นเท่านั้น
2. ให้ `g1_ctrl` ปิดตัวเมื่อมี controller อื่นส่ง `rt/lowcmd` อยู่ (ตอนนี้แค่เตือน)
3. ทดสอบ yaw_stand กับ `g1_ctrl` + `unitree_mujoco` (`sim2sim_policy.sh hist5_yawstand`, ต้องกด Load key ใน GUI)
4. หุ่นจริง 30/10: checklist หัวข้อ 9 ด้วย P3b (แขวน: Passive → FixStand ตรวจ kp/kd → ลงพื้น → Velocity → เดินช้า)
5. ที่เสนอไว้แต่ยังไม่ได้ทำ: ใส่ `clip` ใน `deploy.yaml`
6. unitree_rl_lab: smoke test task G1-23dof (แผนขั้น 2.1)

## Branch และ commit ที่เกี่ยวข้อง

| repo / branch | commit | เนื้อหา |
|---|---|---|
| unitree_rl_mjlab `phase3-delay-dr` | `843a27a` | task Phase 4 Rough-Blind |
| | `67353a7` | เครื่องมือตรวจหุ่นจริงแบบอ่านอย่างเดียว + ผล 28/10 |
| | `54df935` | `lowcmd_monitor`, `pd_check`, `vel_eval`, `tau_est` |
| | `32a9f5e` | soft start, checklist หัวข้อ 9, `gain_check`, `vel_eval --hang`, สคริปต์ sim2sim, โฟลเดอร์ `hist5_yawft` / `hist5_yawstand`, เอกสารชุดนี้ |
| unitree_rl_mjlab `hist5-from-scratch` | `c779814`, `0100001` | task Hist5 จากศูนย์ + แก้ PYTHONPATH ของ worktree |
| | `81416d4` | task yaw fine-tune + `vel_eval` |
| | `3901e56` | task YawStand |
| g1pilot `g1-23dof-real` | `f8fc4a1` | `validate_viewer.py`, `validate_byobu.sh` |
