# เปรียบเทียบปุ่มรีโมต Unitree G1 แต่ละเวอร์ชัน

สรุปจาก PDF ในโฟลเดอร์นี้ (อ่านปุ่มจากรูปในหน้าภาษาอังกฤษ, 30/10/2026) ปุ่มทิศ (D-pad) อ่านจากไอคอน ถ้าจะใช้งานจริงให้ดู PDF ประกอบ

## สารบัญ

- [ไฟล์และเวอร์ชัน](#ไฟล์และเวอร์ชัน)
- [สัญลักษณ์](#สัญลักษณ์)
- [1. สลับโหมด (Mode)](#1-สลับโหมด-mode)
- [2. สลับการเดิน (Motion Control Switching)](#2-สลับการเดิน-motion-control-switching)
- [3. ท่าทางโต้ตอบ (Interactional)](#3-ท่าทางโต้ตอบ-interactional)
- [4. ท่าเต้น (เฉพาะ 23dof key description)](#4-ท่าเต้น-เฉพาะ-23dof-key-description)
- [5. โหมดต่อสู้ (เฉพาะ Combat / Boxing)](#5-โหมดต่อสู้-เฉพาะ-combat--boxing)
- [6. ลำดับเปิด-ปิดเครื่อง](#6-ลำดับเปิด-ปิดเครื่อง)
- [7. สรุปความเหมือนและความต่าง](#7-สรุปความเหมือนและความต่าง)
- [8. เทียบกับ g1_ctrl ของเรา](#8-เทียบกับ-g1_ctrl-ของเรา)

## ไฟล์และเวอร์ชัน

| ย่อ | ไฟล์ | เวอร์ชัน / หุ่น |
|---|---|---|
| **A** | `Motion Control Version < 8.2.0.0.pdf` | firmware เก่ากว่า 8.2.0.0 (ปุ่มหลักคือ L1) |
| **B** | `Motion Control Version >= 8.2.0.0.pdf` | ≥ 8.2.0.0 (ปุ่มหลักเปลี่ยนเป็น L2), แยกเอว 1-DoF / 3-DoF |
| **C** | `Motion Control Version >= 8.4.2.222，G1 23-Dof .pdf` | ≥ 8.4.2.222 สำหรับ G1 23-DoF |
| **D** | `23dofRemote control key description.pdf` | G1 23-DoF รุ่นที่มีท่าเต้น, ลุกขึ้นเอง และโหมดเสียง (ไม่ระบุเลขเวอร์ชัน) |
| **E** | `Combat Version.pdf` = `Boxing Controller.pdf` | เวอร์ชันต่อสู้ (สองไฟล์เป็นไฟล์เดียวกัน md5 `192152a4…`) |

## สัญลักษณ์

| สัญลักษณ์ | ความหมาย |
|---|---|
| ★ | กดสั้นครั้งเดียว |
| ▲ | กดค้าง 2 วินาที |
| `X + Y` | กด X ค้างไว้แล้วกด Y |
| ↑ ↓ ← → | ปุ่มทิศ (D-pad) |
| — | ไม่มีในเวอร์ชันนั้น |

ไฟล์ A ไม่มีสัญลักษณ์ ★ / ▲

## 1. สลับโหมด (Mode)

| โหมด | A (< 8.2) | B (≥ 8.2) | C (≥ 8.4.2.222, 23dof) | D (23dof key desc.) | E (Combat) |
|---|---|---|---|---|---|
| Zero Moment (ปล่อยแรงบิด) | L1 + B | L2 + Y ★ | L2 + Y ★ | L2 + Y ★ | L2 + Y ★ |
| ① Damping (หน่วง) | L1 + A | L2 + B ★ | L2 + B ★ | L2 + B ★ | L2 + B ★ |
| ② Locked Standing (ล็อกยืน) | L1 + ↑ | L2 + ↑ ★ | L2 + ↑ ★ | L2 + ↑ ★ | L2 + ↑ ★ |
| ④ Seated Mode (นั่ง) | L1 + ← | L2 + ← ▲ | L2 + ← ▲ | L2 + ← ▲ | L2 + ← ▲ |
| ⑤ Lie → Stand (นอน → ยืน) | L1 + X | L2 + X ★ | L2 + X ★ | L2 + X ★ | L2 + X ★ |
| ⑥ Squat ↔ Stand (ย่อ ↔ ยืน) | L1 + Y | L2 + A ▲ | L2 + A ▲ | L2 + A ▲ | L2 + A ▲ |
| ⑩ Adapt Rise (ลุกขึ้นเอง) | — | — | — | R1 + R2 + X ★ | R1 + R2 + X ★ |

ทุกเวอร์ชันตั้งแต่ B: อยู่ในโหมดเดินแล้วกด ① ค้าง 5 วินาที → เข้า damping protection

## 2. สลับการเดิน (Motion Control Switching)

| โหมด | A (< 8.2) | B (≥ 8.2) | C (≥ 8.4.2.222, 23dof) | D (23dof key desc.) | E (Combat) |
|---|---|---|---|---|---|
| ③ Running / เอว 1-DoF | R1 + X (1-DoF Waist) | R2 + A ▲ (Running) | R2 + A ▲ | R2 + A ▲ | R2 + A ▲ |
| └ Low / High Speed (Running) | ดับเบิลคลิก L2 / L1 | R2 + ↓ / ↑ ★ | R2 + ↓ / ↑ ★ | R2 + ↓ / ↑ ★ | — |
| เอว 3-DoF | R1 + Y | (ใน Regular) | — | — | — |
| Regular Mode (เดินปกติ) | — | R1 + X (1-DoF) / R1 + Y (3-DoF) ▲ | R1 + X ▲ | R1 + X ▲ | **R1 + Y ▲** (⑦) |
| └ Low / High Speed (Regular) | — | ดับเบิลคลิก L2 / L1 ★ | ดับเบิลคลิก L2 / L1 ★ | ดับเบิลคลิก L2 / L1 ★ | ดับเบิลคลิก L2 / L1 ★ |
| Keep Stepping (ย่ำเท้า) | ดับเบิลคลิก START | — | — | — | — |
| Standing (หยุดย่ำ) | คลิก START | — | — | — | — |
| ⑧ Climb Mode (ข้ามสิ่งกีดขวาง) | — | — | R2 + B ▲ | R2 + B ▲ | — |
| └ Stepping / Standing | — | — | ดับเบิลคลิก START ★ | ดับเบิลคลิก START ★ | — |
| Switch to Voice Mode: Normal / Welcome | — | — | — | R1 + R2 + Y ★ | — |
| Running: เชียร์มือซ้าย / ขวา / สองมือ / ปล่อยมือ | — | — | — | — | ดับเบิลคลิก ← / → / ↑ / ↓ ★ |

- A ใช้ R1 + X / R1 + Y เลือกเอว 1-DoF / 3-DoF แล้วเปลี่ยนความเร็วด้วยดับเบิลคลิก L2 (ต่ำ) / L1 (สูง)
- ตั้งแต่ B ปุ่ม R2 + A เข้าโหมด Running และ R1 + X/Y เข้า Regular
- ใน E ปุ่ม Regular เปลี่ยนเป็น **R1 + Y** (ใน B–D คือ R1 + X)

## 3. ท่าทางโต้ตอบ (Interactional)

| ท่า | A | B | C | D | E |
|---|---|---|---|---|---|
| Handshake (จับมือ) | SELECT + A | SELECT + A ★ | SELECT + A ★ | SELECT + A ★ | — |
| Wave Hand (โบกมือ) | SELECT + Y | SELECT + Y ★ | SELECT + Y ★ | SELECT + Y ★ | — |
| Turn Around and Wave Hand | SELECT + X | SELECT + X ★ | — | — | — |
| Reject (ไขว้มือเป็น X) | — | — | SELECT + X ★ | — | — |
| Face Wave (โบกมือหน้าอก) | — | — | ดับเบิลคลิก Y ★ | ดับเบิลคลิก Y ★ | — |
| Right Kiss (ส่งจูบ) | — | — | ดับเบิลคลิก X ★ | ดับเบิลคลิก X ★ | — |
| Clap (ปรบมือ) | — | — | ดับเบิลคลิก A ★ | ดับเบิลคลิก A ★ | — |
| Right Raise (ยกมือขวา) | — | — | ดับเบิลคลิก B ★ | — | — |

## 4. ท่าเต้น (เฉพาะ 23dof key description)

| ท่า | ปุ่ม (D) | พื้นที่ที่ต้องใช้ |
|---|---|---|
| ⑨ เข้า Dance Mode | R1 + B | |
| Charleston dance 1 | R1 + ↑ ★ | → 1 m |
| Charleston dance 2 | R1 + ← ★ | ↑ 1 m, ← 1 m |
| Charleston dance 3 | R1 + → ★ | ↗ 3 m |
| Jeet Kune Do | R2 + ↑ ★ | 3 m × 3 m |
| Twist | R2 + ← ★ | 3 m × 3 m |

ระหว่างเต้นหรือทำท่าแขน ดับเบิลคลิก START = หยุดฉุกเฉิน

## 5. โหมดต่อสู้ (เฉพาะ Combat / Boxing)

เข้า ⑧ Boxing ด้วย **R1 + A ▲** ทุกท่ากดสั้น ★

| หมวด | ท่า | ปุ่ม | หมวด | ท่า | ปุ่ม |
|---|---|---|---|---|---|
| Upper Body | Straight Punch | R1 + ↑ | Legs | Front Kick | R2 + ↑ |
| | Left Hook | R1 + ← | | Left Side Kick | R2 + ← |
| | Right Hook | R1 + → | | Right Side Kick | R2 + → |
| | Uppercut | R1 + ↓ | | Knee Kick | R2 + ↓ |
| Evasion | Turn Around 180° | ดับเบิลคลิก ↑ | Taunt | Ready Stance | ดับเบิลคลิก Y |
| | Left Dodge | ดับเบิลคลิก ← | | Block | ดับเบิลคลิก X |
| | Right Dodge | ดับเบิลคลิก → | | Bring it on | ดับเบิลคลิก B |
| | Back Dodge | ดับเบิลคลิก ↓ | | Slap Own Butt | ดับเบิลคลิก A |
| Stance | Lying → Standing | L2 + ↑ | Stance | Standing → Lying | L2 + ↓ |
| Combo | Combo 1 / 2 / 3 / 4 | L1 + ↑ / ← / → / ↓ | Combo | Combo Punch / Ram | R1 + SELECT / R2 + SELECT |

## 6. ลำดับเปิด-ปิดเครื่อง

| วิธี | A | B, C | D | E |
|---|---|---|---|---|
| เปิดตอนนอน | ① → ⑤ → Show → ⑥ → ปิด | ① → ⑤ → Demo → ⑥ → ปิด | ① → **⑩** → Demo → ⑥ → ปิด | ① → **⑩** → Demo → ⑥ → ปิด |
| เปิดตอนย่อ | ① → ⑥ → Show → ⑥ → ปิด | ① → ⑥ → Demo → ⑥ → ปิด | เหมือน B | เหมือน B |
| เปิดตอนนั่ง (แนะนำ) | ① → ② → ③ → Show → ④ → ปิด | เหมือน A | เหมือน A | เหมือน A |
| เปิดแบบต่อสู้ | — | — | — | ① → ② → ⑧ → ② → Demo → ปิด |

ทุกเวอร์ชัน: ช่วง ② → ③ ตอนเปิดแบบนั่ง ต้องมีคนประคองให้หุ่นยืนตรง

## 7. สรุปความเหมือนและความต่าง

**เหมือนกันทุกเวอร์ชันตั้งแต่ B (≥ 8.2):**

- ปุ่มสลับโหมดชุดเดียวกัน: L2 + Y / B / ↑ / ← / X / A = Zero Moment / Damping / Locked Standing / Seated / Lie → Stand / Squat ↔ Stand
- R2 + A ▲ = Running Mode
- ดับเบิลคลิก L2 / L1 = ความเร็วต่ำ / สูงของ Regular Mode
- กด ① ค้าง 5 วินาที = damping protection
- SELECT + A / Y = Handshake / Wave Hand (ยกเว้น E ที่ไม่มีท่าโต้ตอบ)

**ความต่างที่สำคัญ:**

| เรื่อง | รายละเอียด |
|---|---|
| A → B: ปุ่มหลักเปลี่ยน | A ใช้ **L1** + ปุ่ม, ตั้งแต่ B ใช้ **L2** + ปุ่ม และความหมายของปุ่มสลับกัน (A: L1 + A = Damping, B: L2 + B = Damping, A: L1 + B = Zero Moment) **ต้องรู้ firmware ของหุ่นก่อนกด** |
| เอว 1-DoF / 3-DoF | A แยกเป็นสองโหมด (R1 + X / R1 + Y) พร้อม Keep Stepping ด้วย START, B รวมไว้ใน Regular, C/D (23dof) มีแค่ R1 + X เพราะเอวมีแกนเดียว |
| Climb Mode | มีเฉพาะ C และ D (R2 + B ▲, ดับเบิลคลิก START = ย่ำ/ยืน) |
| Adapt Rise | มีเฉพาะ D และ E (R1 + R2 + X) และใช้แทน ⑤ ในลำดับเปิดตอนนอน |
| ท่าโต้ตอบ | B: 3 ท่า (มี Turn Around and Wave), C: 7 ท่า (Reject, ดับเบิลคลิก Y/X/A/B), D: 5 ท่า (ไม่มี Reject, Right Raise), E: ไม่มี |
| ท่าเต้น / โหมดเสียง | มีเฉพาะ D (R1 + B เข้าโหมดเต้น, R1 + R2 + Y สลับเสียง) |
| ต่อสู้ | มีเฉพาะ E (R1 + A ▲) ปุ่ม R1/R2/L1 + D-pad กลายเป็นท่าต่อย/เตะ/combo และ Regular เปลี่ยนเป็น R1 + Y |
| SELECT | B–D: ใช้คู่กับ A/Y/X เป็นท่าโต้ตอบ, E: ใช้คู่กับ R1/R2 เป็น Combo Punch / Ram |
| START | A: ย่ำ/หยุด, C/D: ย่ำ/ยืนใน Climb, D: ดับเบิลคลิกหยุดท่าเต้นฉุกเฉิน |

## 8. เทียบกับ g1_ctrl ของเรา

ปุ่มเหล่านี้เป็นของ controller ในตัว Unitree ตอนรัน `g1_ctrl` ต้องเข้า **debug mode** ก่อน (L2 + R2 ตามคู่มือ Unitree, PDF ชุดนี้ไม่ได้ระบุ) controller ในตัวจะหยุด แล้ว `g1_ctrl` อ่านรีโมตจาก `rt/lowstate` เอง

| `g1_ctrl` (`deploy/robots/g1_23dof/config/config.yaml`) | ปุ่ม | ปุ่มเดียวกันใน firmware ≥ 8.2 | ความหมายใกล้กันไหม |
|---|---|---|---|
| Passive → FixStand | LT (L2) + ↑ | ② Locked Standing | ใกล้กัน (ยืนค้างท่า) |
| FixStand → Velocity (policy) | RT (R2) + A | ③ Running Mode | ใกล้กัน (เข้าโหมดเดิน) |
| ทุกโหมด → Passive (หยุดฉุกเฉิน) | LT (L2) + B | ① Damping | ใกล้กัน (หน่วง kp = 0) |
| สั่งความเร็ว | stick ซ้าย = vx / vy, stick ขวาแกนนอน = wz | – | – |

- `g1_ctrl` เลือกปุ่มให้ตรงกับความหมายของ firmware ≥ 8.2 คนที่เคยใช้รีโมตจึงจำง่าย แต่ **ถ้าหุ่นเป็น firmware < 8.2 (A) ปุ่ม L2 + B ใน firmware เดิมไม่ใช่ Damping** ตอนอยู่ใน debug mode ไม่มีปัญหา เพราะ `g1_ctrl` เป็นคนอ่านปุ่ม
- `g1_ctrl` ไม่มีโหมดอื่น (นั่ง, เต้น, ต่อสู้) ปุ่มอื่นใช้ไม่ได้ระหว่างรัน `g1_ctrl`
- ห้ามออกจาก debug mode ขณะ `g1_ctrl` ยังรันอยู่ controller ในตัวน่าจะกลับมาทำงานและสั่งมอเตอร์แย่งกับ `g1_ctrl` (ยังไม่ได้ทดสอบ)
