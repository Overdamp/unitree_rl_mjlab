# G1 keyboard walking: diagnosis and simulation checks

## สิ่งที่แก้

โค้ด keyboard integration เดิมลบ `velocity_commands` แล้วเพิ่ม
`keyboard_velocity_commands` ที่ท้าย YAML แต่ ObservationManager ต่อข้อมูลตามลำดับ
YAML จึงย้ายคำสั่งความเร็วจาก input index 6–8 ไปเป็น 95–97
จำนวน input ยังคง 98 ทำให้ ONNX ยอมรับข้อมูลที่มีความหมายผิดตำแหน่ง

แก้โดยเปลี่ยนชื่อ observation ในตำแหน่งเดิม นอกจากนี้ `gait_phase` เดิมอ่านคำสั่ง
จาก joystick โดยตรง จึงเพิ่ม `keyboard_gait_phase` ที่ใช้คำสั่งเดียวกับ keyboard
และล้างคำสั่งเดินค้างเมื่อ reset policy

แก้ค่า slide/turn เพิ่มเติม: เดิม `a/d/q/e` ใช้ขนาดคำสั่ง `0.05` แต่ gait phase
ปิดเมื่อ command norm ต่ำกว่า `0.1` ทำให้ไม่เข้าสู่จังหวะก้าวตามที่ต้องการ
ตอนนี้ `a/d` ใช้ ±0.2 m/s และ `q/e` ใช้ ±0.3 rad/s โดยคงเงื่อนไข gait เดิมไว้
ค่าทั้งหมดอยู่ใน command ranges ของ deploy.yaml
controller และการทดสอบใช้ key mapping กับ smoothing จาก
`include/keyboard_velocity_command.h` ร่วมกัน

`g1_ctrl` พิมพ์ `ONNX policy loaded successfully: ...` หลังสร้าง ONNX session
สำเร็จแล้ว การเข้า `Velocity` จึงเป็นการเริ่ม inference loop

FixStand เป็น PD ไปยังท่าข้อต่อเป้าหมาย ไม่มี feedback สำหรับการทรงตัวทั้งตัว
การทดสอบพบว่าปล่อยฟิสิกส์ใน FixStand นาน 2 วินาทีแล้วล้มได้ แม้เริ่มจากท่า
nominal จึงใช้ท่า `stand_ready` และเข้า Velocity ขณะที่ฟิสิกส์ยัง Pause

## วิธีเริ่มใหม่บน desktop — simulation เท่านั้น

ปิด `g1_ctrl` และ simulator รุ่นที่กำลังรันก่อน เพื่อโหลด binary และ XML ใหม่

Terminal 1:

```bash
cd /home/luke/nvidia/unitree/unitree_rl_mjlab
env LD_LIBRARY_PATH=/opt/unitree_robotics/lib ./simulate/build/unitree_mujoco
```

ใน MuJoCo:

1. เลือก **Pause** ในหัวข้อ Simulation
2. เลือก **Key = 0** แล้วกด **Load key** ซึ่งเป็นท่า `stand_ready`
3. คงสถานะ Pause ไว้ระหว่างเปิด controller

ท่า `stand_ready` เพิ่มไว้ใน `scene_g1.xml` เป็นมุมข้อต่อ nominal จาก deploy.yaml
พร้อม pelvis height 0.793 m และ quaternion identity ไม่มีแรงพยุงหรือการปิด gravity
ปุ่ม **Reset** ธรรมดายังกลับไปท่าเริ่มต้นเดิม ต้องใช้ **Load key**
ถ้ายังไม่มี key ให้ปิด/เปิด simulator เพื่ออ่าน XML ใหม่

Terminal 2:

```bash
cd /home/luke/nvidia/unitree/unitree_rl_mjlab/deploy/robots/g1/build
env -u LD_LIBRARY_PATH ./g1_ctrl --network=lo
```

1. รอข้อความโหลด ONNX สำเร็จ โดย path ของ Velocity ลงท้ายด้วย
   `config/policy/velocity/v0/exported/policy.onnx` และ `FSM: Start Passive`
2. ใช้ keyboard layout ภาษาอังกฤษ คลิก terminal นี้ กด `u` หนึ่งครั้ง
   ต้องเห็น `FSM: Change state from Passive to FixStand`
3. **ขณะที่ MuJoCo ยัง Pause** กด `v` หนึ่งครั้ง
   ต้องเห็น `FSM: Change state from FixStand to Velocity`
4. กลับไป MuJoCo แล้วกด **Run**
5. ปล่อยให้ policy รับคำสั่งความเร็วศูนย์สัก 5 วินาที แล้วกลับมา terminal controller
   กด `w` ทดสอบเดินหน้า จากนั้นกด `x` เพื่อสั่งความเร็วศูนย์

ไม่ต้องรอให้หุ่นยืนด้วย FixStand 2 วินาทีก่อนกด `v` สำหรับขั้นตอนนี้
หากเคยล้ม ให้เริ่มใหม่จาก Pause + Load key พร้อม controller ที่กลับ Passive
อย่าคาดหวังว่า locomotion policy จะลุกจากพื้นเอง

| ปุ่มใน terminal controller | การทำงาน |
| --- | --- |
| `u` | Passive → FixStand |
| `v` | FixStand → Velocity |
| `w` / `s` | สั่งหน้า +0.4 / หลัง -0.3 m/s |
| `a` / `d` | สั่ง slide ซ้าย +0.2 / ขวา -0.2 m/s ตามแกน y ของหุ่น |
| `q` / `e` | สั่งหมุนซ้าย +0.3 / ขวา -0.3 rad/s |
| `x` / Space | สั่งความเร็วศูนย์ โดยยังใช้ policy ประคองตัว |
| `p` | กลับ Passive; ไม่มีการรักษาท่ายืนและอาจล้ม |

คำสั่งทิศทางค้างจนกดปุ่มอื่นหรือ `x`/Space ไม่ใช่ปล่อยปุ่มแล้วหยุด
หากกด Space ตอน focus อยู่ที่หน้าต่าง MuJoCo จะเป็นคำสั่งของ viewer
การทดสอบตัวเลขครอบคลุมทุกปุ่มทิศทางทีละปุ่ม แต่ไม่ได้ทดสอบปุ่มหลายตัวพร้อมกัน
แต่ละปุ่มจะเลือกคำสั่งใหม่แทนทิศทางเดิม คีย์ทั้งหมดต้องกดใน terminal controller

## ทดสอบซ้ำโดยไม่เปิดหน้าต่าง

```bash
cd /home/luke/nvidia/unitree/unitree_rl_mjlab
bash deploy/robots/g1/tests/run_headless_checks.sh
```

ต้องมี g++, yaml-cpp, fmt, Unitree headers และไลบรารี MuJoCo/ONNX ที่มากับ checkout
สคริปต์สำหรับ x86_64 นี้ compile โปรแกรมชั่วคราวใน `/tmp` และลบทิ้งเมื่อจบ
ไม่ได้เปิด DDS publisher และไม่ได้ส่งคำสั่งไปหุ่นจริง

การตรวจครอบคลุม:

- ส่ง input ที่แยกแต่ละตำแหน่งได้ผ่าน ObservationManager ตัวจริง
  ยืนยัน input 98 ค่าและ ONNX output 29 ค่าเท่ากับ config ต้นฉบับ
- ใช้ MuJoCo scene, ONNX, observation/action manager และสูตร PD เดียวกับ bridge
  เริ่มจาก `stand_ready` และรัน policy ขณะ physics ไม่เคลื่อนไหวก่อนเริ่ม
  ทดสอบทุกทิศทางแยกกัน: ยืน 5 วินาที เคลื่อนที่ 10 วินาที หยุด 5 วินาทีตามเวลาจำลอง
  ตรวจคำสั่งไม่เกินช่วง config, gait ทำงานจริง, เคลื่อนถูกทิศ, ไม่ล้ม และหยุดได้
- จำลองบั๊ก remove/append เดิม เพื่อแสดงผลการป้อน observation ผิดลำดับ
- ตรวจ FixStand 2 วินาทีทั้งจาก Reset และ stand_ready เพื่ออธิบายปัญหาการเริ่มต้น

ข้อจำกัด: เป็นการคำนวณใน process เดียว ไม่ครอบคลุม DDS latency,
FSM threads, terminal keyboard events, GUI หรือ hardware จริง
การทดสอบนี้เป็นหลักฐานว่า policy ใช้กับ scene นี้ได้ภายใต้เงื่อนไขที่ระบุ
ไม่ใช่การยืนยัน end-to-end ของ `g1_ctrl` ผ่าน DDS

ผลที่รันได้ใน checkout นี้วันที่ 2026-10-20:

| กรณี | ผล |
| --- | --- |
| ลำดับ observation หลังแก้ | input 98 ค่าและ output 29 ค่าตรงกับ config ต้นฉบับ |
| stand_ready → policy ขณะ pause → เริ่ม physics | ยืน 5 s, เดินหน้า 3.881 m ใน 10 s, หยุด 5 s |
| ตลอด 20 s ของกรณีข้างต้น | pelvis ต่ำสุด 0.7736 m, เอียงสูงสุด 0.0449 rad, vx สุดท้าย -0.00058 m/s |
| ลำดับ observation แบบผิด | ตกเกณฑ์ความสูงขั้นต่ำที่ 1.128 s |
| Reset → FixStand 2 s | pelvis เหลือประมาณ 0.0964 m; ล้มก่อน policy |
| stand_ready → FixStand 2 s | pelvis เหลือประมาณ 0.0951 m; ยังต้องเข้า policy ก่อน Run |

ผลทดสอบคำสั่งทิศทางหลังแก้ slide/turn (20 s ต่อกรณี):

| ปุ่ม | ระยะตามทิศหัวตอนเริ่มเคลื่อนที่ในช่วง 10 s | การเปลี่ยน yaw ในช่วงเดียวกัน |
| --- | --- | --- |
| `w` | x = +3.885 m | +0.244 rad |
| `s` | x = -2.413 m | +0.214 rad |
| `a` | y = +1.302 m | +0.424 rad |
| `d` | y = -1.314 m | +0.618 rad |
| `q` | — | +3.165 rad |
| `e` | — | -2.032 rad |

ทุกกรณีไม่ล้มและหยุดได้ตามเกณฑ์ (ความเร็วแนวราบและ yaw rate ต่ำกว่า 0.2 หลังหยุด 5 s)
นับ yaw แบบต่อเนื่องข้าม ±π จึงไม่สูญเสียจำนวนรอบการหมุน
ยังมี heading drift ระหว่าง slide โดยเฉพาะ `d` หัวเบนประมาณ 35° ใน 10 s
และเคลื่อนตาม x ประมาณ 0.662 m จึงยังไม่ใช่การ slide แบบรักษาทิศหัวให้คงที่
การผ่านนี้ยืนยันการเคลื่อนถูกทิศและหยุดได้ ไม่ใช่ความแม่นยำในการตามความเร็วทุกแกน

## Policy ทางเลือก

- โปรเจกต์ [unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab)
  มี workflow สำหรับ G1; checkout นี้ใช้ velocity/v0 ที่มี input 98 และ action 29 ค่า
- [unitree_rl_gym G1 config](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_mujoco/configs/g1.yaml)
  อ้างถึง `deploy/pre_train/g1/motion.pt` แต่ใช้ input 47 และ action 12 ค่า
  จึงต้องใช้ model/config/deployment ที่เข้าคู่กัน เอามาแทน ONNX ปัจจุบันตรง ๆ ไม่ได้
- [SDK2 G1 LocoClient example](https://github.com/unitreerobotics/unitree_sdk2/blob/master/example/g1/high_level/g1_loco_client_example.cpp)
  เป็นตัวอย่างเรียกบริการ locomotion ไม่ใช่ไฟล์ weights สำหรับโหลดเข้า g1_ctrl
  bridge ใน checkout นี้รับ LowCmd แล้วคำนวณ PD ใน MuJoCo ไม่ได้ให้บริการ
  locomotion ของหุ่นจริงโดยอัตโนมัติ
