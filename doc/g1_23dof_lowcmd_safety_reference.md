# G1-23DoF LowCmd, motor map, PD gains และ safety limits

เอกสารนี้รวบรวมข้อมูลสำหรับเตรียม deploy policy ของ G1-23DoF ผ่าน Unitree SDK2 / DDS โดยแยกข้อมูลที่ยืนยันได้จาก SDK และ repository ออกจากค่าที่เป็นเพียงค่าของ simulation หรือยังต้องยืนยันกับหุ่นยนต์จริง

## ขอบเขตและสถานะของข้อมูล

- โค้ดโปรเจกต์ที่ตรวจ: `deploy/robots/g1_23dof/` และ `deploy/robots/g1_23dof/config/policy/velocity/v0/params/deploy.yaml` ใน checkout ปัจจุบัน
- SDK2 ที่มีใน workspace: `unitree/unitree_sdk2` revision `63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36`; Python SDK2 revision `814556d15970dd2ecf1c9984e845ca02ab07e206`.
- ข้อมูล motor index และ LowCmd อ้างอิงตัวอย่าง low-level ของ SDK2 และ IDL ที่มากับ SDK
- ค่า joint range/effort/velocity ใน URDF/XML เป็นค่าของ robot description/model เท่านั้น ไม่ใช่ safety limit ที่รับรองว่าใช้กับหุ่นยนต์และ firmware ทุกตัวได้
- ยังไม่มีข้อมูล serial number, firmware, รุ่นย่อย และเอกสาร safety limit ที่ผูกกับเครื่องจริง จึงยังระบุ threshold สำหรับ deploy ไม่ได้

## 1. LowCmd และ LowState ที่เกี่ยวข้อง

SDK2 ส่ง `unitree_hg::msg::dds_::LowCmd_` และรับ `LowState_` ผ่าน DDS. LowCmd มีข้อมูลส่วนหัว เช่น `mode_pr`, `mode_machine`, motor command array, reserve และ CRC; แต่ละ `motor_cmd` มี `mode`, `q`, `dq`, `tau`, `kp`, `kd` และ reserve fields. LowState มี motor state array, `mode_machine` และข้อมูลสถานะหุ่นยนต์.

ข้อควรแยกให้ชัด:

1. ตัวอย่าง G1 ใน SDK ใช้ 29 motor index สำหรับรุ่นที่มีข้อมือครบ
2. IDL กำหนด array ได้ 35 ช่อง; จำนวนช่องใน message ไม่ได้แปลว่าหุ่นยนต์มีมอเตอร์ใช้งานจริง 35 ตัว
3. G1-23DoF ในโปรเจกต์นี้ใช้เฉพาะ 23 index ตามตารางด้านล่าง
4. `Mode.PR = 0` และ `Mode.AB = 1` ในตัวอย่าง SDK เป็นโหมดการตั้งค่า motor (โดย ankle ใช้การจัดแบบ A/B); `motor_cmd.mode = 1` คือเปิด command ในตัวอย่าง ส่วน `0` คือปิด
5. ตัวอย่าง SDK อ่าน `mode_machine` จาก LowState แล้วใส่กลับใน LowCmd และคำนวณ CRC ก่อน publish

**เหตุผลที่โค้ดนี้ใช้ `mode_machine = 4`:** XML ที่โปรเจกต์ใช้ชื่อ `g1_23dof_rev_1_0` (`src/assets/robots/unitree_g1/xmls/g1_23dof.xml`) ซึ่งเป็น G1-23DoF revision 1.0; ตาราง robot description ของ Unitree แยก `g1_23dof_rev_1_0` เป็น mode 4 ออกจาก `g1_23dof` รุ่นเก่าซึ่งเป็น mode 1. ดังนั้น 4 ไม่ได้หมายถึง 23DoF ทุกรุ่น แต่หมายถึง hardware/model profile รุ่นย่อยนั้นโดยเฉพาะ.

หน้า [Joint Motor Sequence ของ Unitree](https://support.unitree.com/home/en/G1_developer/joint_motor_sequence#heading-3) ที่ระบุ “23DOF Version (`mode_machine == 1`)” ใช้กับ G1-23DoF รุ่นเดิม/legacy. ตัวเลข 1 กับ 4 จึงไม่ขัดกันเมื่อแยก revision; ต้องใช้ mode ของเครื่องจริง ไม่ใช่เลือกจากจำนวน DoF อย่างเดียว. SDK `check_mode_machine()` เพียงปฏิเสธเมื่อ mode ที่ส่งไม่ตรงกับ LowState (ยกเว้น simulation mode 0); มันไม่ได้ค้นหา mode ที่ถูกให้เอง. ก่อน hardware deployment ต้องดู Machine Type ใน app หรืออ่าน `LowState.mode_machine` แล้วตรวจให้ตรงกับรุ่น/profile.

## 2. ลำดับ joint และ SDK motor index

ลำดับ policy/model ใน `deploy.yaml` เทียบกับช่อง `LowCmd.motor_cmd[]` ของ SDK ได้ดังนี้. คอลัมน์ policy index เป็นลำดับที่ใช้กับ joint vectors และ gain vectors ใน config นี้. **ข้อจำกัดหลักฐาน:** ตาราง mode 4 แบบแยกต่างหากยังไม่พบในหน้า Unitree Support ที่ตรวจ; ตารางด้านล่างจับคู่ชื่อจากโมเดล `g1_23dof_rev_1_0` กับ IDL indices จาก SDK2 G1 low-level example (ซึ่งระบุลำดับ 29 ช่องและช่องที่ invalid สำหรับ 23DoF) และ mapping ใน deploy config. จึงเป็น mapping ที่สอดคล้องกันใน source code แต่ควรยืนยันกับตาราง mode 4/เครื่องจริงก่อนสั่งมอเตอร์.

| Policy index | Joint | SDK motor index |
|---:|---|---:|
| 0 | left_hip_pitch | 0 |
| 1 | left_hip_roll | 1 |
| 2 | left_hip_yaw | 2 |
| 3 | left_knee | 3 |
| 4 | left_ankle_pitch (ankle B) | 4 |
| 5 | left_ankle_roll (ankle A) | 5 |
| 6 | right_hip_pitch | 6 |
| 7 | right_hip_roll | 7 |
| 8 | right_hip_yaw | 8 |
| 9 | right_knee | 9 |
| 10 | right_ankle_pitch (ankle B) | 10 |
| 11 | right_ankle_roll (ankle A) | 11 |
| 12 | waist_yaw | 12 |
| 13 | left_shoulder_pitch | 15 |
| 14 | left_shoulder_roll | 16 |
| 15 | left_shoulder_yaw | 17 |
| 16 | left_elbow | 18 |
| 17 | left_wrist_roll | 19 |
| 18 | right_shoulder_pitch | 22 |
| 19 | right_shoulder_roll | 23 |
| 20 | right_shoulder_yaw | 24 |
| 21 | right_elbow | 25 |
| 22 | right_wrist_roll | 26 |

ช่องที่ข้ามสำหรับ mapping 23DoF นี้คือ SDK index `13,14` (waist เพิ่มเติม), `20,21` (ข้อมือซ้ายเพิ่ม) และ `27,28` (ข้อมือขวาเพิ่ม). อย่าเขียน policy vector 23 ค่าเข้า LowCmd 23 ช่องแรกโดยตรง เพราะแขนเริ่มที่ SDK index 15 และ 22.

## 3. PD gains: ค่าในโปรเจกต์และตัวอย่าง SDK

`deploy.yaml` ปัจจุบันกำหนด `step_dt = 0.02` วินาที และใช้ค่า gains ตามลำดับ policy index:

| กลุ่ม joint | Kp | Kd |
|---|---:|---:|
| left/right hip pitch, hip yaw | 40.2 | 2.6 |
| left/right hip roll, knee | 99.1 | 6.3 |
| ankle pitch/roll ทั้งสองข้าง | 28.5 | 1.8 |
| waist yaw | 40.2 | 2.6 |
| ไหล่ ข้อศอก และ wrist roll ทั้ง 10 joints | 14.3 | 0.9 |

ค่าของโปรเจกต์มาจาก actuator model ใน `g1_23dof_constants.py` โดยตั้งสมมติฐาน inertia, natural frequency 10 Hz และ damping ratio 2 เพื่อใช้กับ simulation/policy. จึงไม่ใช่ค่าจากโรงงานหรือค่าที่รับรองว่าปลอดภัยบน hardware.

ตัวอย่าง low-level Python ของ Unitree SDK2 ใช้ Kp/Kd สาธิตสำหรับ G1 29 motor ดังนี้:

- Kp: `[60,60,60,100,40,40, 60,60,60,100,40,40, 60,40,40, 40 x 14]`
- Kd: `[1,1,1,2,1,1, 1,1,1,2,1,1, 1,1,1, 1 x 14]`

ค่านี้เป็น demo ในตัวอย่างการควบคุม ไม่ใช่ค่าตั้งต้นที่ Unitree รับรองให้ใช้กับ policy นี้ และไม่ควรนำมาแทน gain ใน policy โดยไม่ทดสอบ/วิเคราะห์ closed-loop. ตัวอย่าง SDK ทำ low-level loop ประมาณ 2 ms ขณะที่ policy config นี้มี step 20 ms (50 Hz); ต้องตรวจ cadence, transport watchdog และผลของ delay/jitter ให้เข้ากับ implementation จริง.

## 4. Joint, velocity และ effort limits

ใน deploy config ปัจจุบัน `action.clip` เป็น `null` และไม่พบชุด hardware limit สำหรับ clipping target, joint velocity หรือ torque ในเส้นทาง deploy ที่ตรวจ. XML/URDF ของ G1 ให้ค่าขอบเขตเชิงกลของโมเดล และ actuator config ของ simulation มี effort/velocity assumptions (เช่น actuator types 5020, 7520-14, 7520-22) แต่ห้ามนำค่าเหล่านั้นไปใช้เป็น software safety thresholds ของเครื่องจริงโดยตรง.

ก่อน deploy ต้องได้ limit ที่ยืนยันตรงกับรุ่นย่อยและ firmware จากเอกสาร Unitree หรือการยืนยันของผู้ผลิต/ผู้ดูแลหุ่นยนต์ สำหรับอย่างน้อย:

- มุม joint และช่วงที่อนุญาตขณะควบคุม (รวม margin จาก hard stop)
- ความเร็วและ torque/current สูงสุดต่อ joint หรือ motor
- อุณหภูมิ, แรงดันแบตเตอรี่ และเงื่อนไขลดกำลัง/หยุด
- ค่า timeout/watchdog และวิธีเข้า safe/idle state ที่รองรับใน firmware

อย่าใช้ค่า URDF/XML เป็น hard stop software โดยไม่ตรวจว่าตรงกับเครื่องจริง และอย่าสมมติว่าการตั้ง torque เป็นศูนย์หรือหยุด publish เป็นวิธี emergency stop ที่ปลอดภัย.

## 5. จุดเสี่ยงใน deploy code ปัจจุบัน

จากการตรวจโค้ด G1-23DoF ใน checkout นี้ พบรายการที่ควรปิดก่อน hardware trial:

1. **Machine mode hardcode:** `main.cpp` กำหนด `mode_machine() = 4`. ควรตรวจค่าจาก LowState และยืนยัน hardware profile ก่อนส่งคำสั่ง
2. **ไม่มี joint/action clipping ใน config:** `action.clip: null`; ยังไม่เห็น guard ที่ครอบคลุม finite values และขอบเขต hardware ของ target
3. **Safety checks ยังไม่ครบ:** มีการตรวจ orientation และ LowState timeout ใน FSM แต่ยังไม่พบการตรวจครบวงจรของ NaN/Inf, joint position/velocity, torque estimate, temperature, battery voltage และ deadline misses ก่อนส่งทุก command
4. **การหยุด/เปลี่ยน state ต้องตรวจจริง:** มี publisher ownership check ใน init FSM แต่เส้นทางเมื่อพบ publisher อื่นควรตรวจ เพราะ shutdown/exit ถูกคอมเมนต์ไว้
5. **ความถี่ต่างกัน:** policy step 20 ms; DDS low-level control ต้องมี cadence, freshness check และ behavior เมื่อพลาด deadline ที่กำหนดชัด

นี่เป็นผลจาก static review ของโค้ด ไม่ได้ยืนยันพฤติกรรมบนหุ่นยนต์จริง.

## 6. Safety gates ก่อนเริ่มขยับหุ่นยนต์

- ระบุรุ่น G1, DoF, firmware, network/interface และค่าจาก live LowState ให้ชัด
- ตรวจ mapping ทุก joint ด้วย motor index ที่อ่านได้จริง และยืนยันว่ามี 23 ช่องตาม profile
- ตรวจ `mode_pr`, `mode_machine`, motor command enable mode และ CRC ตาม SDK version ที่ใช้
- เพิ่ม finite check ก่อนสร้างและก่อน publish command; reject NaN/Inf และ stale state
- กำหนด target/velocity/torque bounds จากข้อมูลที่ยืนยันกับ hardware โดยมี margin; ไม่ดึงค่า simulation มาใช้แทน
- ตรวจสถานะ joint, torque estimate, อุณหภูมิ, แบตเตอรี่ และ LowState age ทุก cycle; ระบุพฤติกรรมเมื่อข้อมูลผิดปกติหรือ DDS หลุด
- ทดสอบทางเข้าสู่ safe state และ physical emergency stop โดยไม่มีคน/สิ่งของอยู่ในระยะอันตราย และเริ่มจากยกตัว/จำกัดการเคลื่อนไหวตามขั้นตอนของผู้ดูแลหุ่นยนต์
- ทดสอบ policy ด้วย gain และ cadence จริงแบบค่อยเป็นค่อยไป พร้อมบันทึก LowState/command และตรวจ saturation ก่อนเพิ่มช่วงการเคลื่อนไหว

รายการเหล่านี้เป็นเกณฑ์เตรียมความพร้อม ไม่ได้แทนขั้นตอน safety ที่ Unitree ระบุสำหรับ hardware รุ่นที่ใช้งาน.

## 7. แหล่งอ้างอิง

### เอกสารผู้ผลิต

- Unitree Support: [G1 SDK Development Guide](https://support.unitree.com/home/en/G1_developer). หน้าเอกสารนี้เป็น portal ทางการและมีหัวข้อย่อย เช่น [Quick Development](https://support.unitree.com/home/en/G1_developer/quick_development) และ [Basic Motion Development](https://support.unitree.com/home/en/G1_developer/basic_motion_development). ในการตรวจครั้งนี้หน้าเว็บเปิดได้แต่เนื้อหาบทความไม่ถูกส่งออกมาใน text view จึงยังใช้ยืนยัน motor index, gain หรือ safety threshold เฉพาะรุ่นไม่ได้; ค่าทางเทคนิคในเอกสารนี้จึงอ้างอิง SDK source และระบุข้อจำกัดไว้ตามเดิม.
- Unitree Support: [Joint Motor Sequence — 23DOF Version (`mode_machine == 1`)](https://support.unitree.com/home/en/G1_developer/joint_motor_sequence#heading-3). ระบุ joint index/ลำดับตาม machine mode และ `mode_pr`; mode 1 เป็นตารางของ 23DoF profile รุ่นเก่า จึงต้องเทียบกับ robot revision ก่อนนำ mapping ไปใช้.
- Unitree `unitree_ros` G1 model catalog: https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/README.md — แยก `g1_23dof` mode 1 (deprecated) กับ `g1_23dof_rev_1_0` mode 4.

### SDK และ interface

- Unitree SDK2 Python low-level G1 example (motor indices, PR/AB, demo gains, mode readback, CRC): https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/example/g1/low_level/g1_low_level_example.py
- Unitree SDK2 C++ G1 low-level example: https://github.com/unitreerobotics/unitree_sdk2/blob/63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36/example/g1/low_level/g1_dual_arm_example.cpp
- SDK2 Python HG DDS IDL directory: https://github.com/unitreerobotics/unitree_sdk2_python/tree/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/idl/unitree_hg/msg/dds_

### Robot model และ simulation config

- Unitree `unitree_rl_mjlab` G1-23DoF actuator constants: https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/src/assets/robots/unitree_g1/g1_23dof_constants.py
- Unitree `unitree_ros` G1-23DoF URDF: https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_23dof.urdf
- G1-23DoF deploy config ที่ตรวจใน checkout ปัจจุบัน: `deploy/robots/g1_23dof/config/policy/velocity/v0/params/deploy.yaml`
- G1-23DoF deploy implementation ที่ตรวจ: `deploy/robots/g1_23dof/main.cpp`, `deploy/robots/g1_23dof/`

> เอกสารนี้สรุปข้อมูลที่หาได้จาก SDK/repository และตรวจโค้ดใน workspace; ค่าที่เป็น safety limit สำหรับ hardware ยังต้องยืนยันตามรุ่นและ firmware ก่อนใช้งานจริง.
