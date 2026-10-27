# DGX (aarch64): ซ่อนไฟล์ build เฉพาะเครื่องจาก git

บันทึกเมื่อ 27/10/2026 · เครื่อง `edgexpert-cc54` (DGX, ARM/aarch64) · branch `phase3-delay-dr`

## ปัญหา

repo นี้ใช้ MuJoCo 3.3.6 สำหรับ **x86** ที่ commit อยู่ใน `simulate/mujoco/` (`lib/libmujoco.so` เป็น symlink ไปที่ `libmujoco.so.3.3.6`) บน DGX ต้องเปลี่ยนเป็น MuJoCo ที่ build สำหรับ aarch64 และแก้ CMake ให้ตรง ทำให้ `git status` ขึ้นไฟล์ที่เปลี่ยนจำนวนมาก:

| กลุ่ม | ไฟล์ | สถานะบน DGX |
|---|---|---|
| header ที่ track อยู่ | `simulate/mujoco/include/mujoco/**` | ถูกลบ (`deleted`) |
| library | `simulate/mujoco/lib/libmujoco.so` | symlink ถูกแทนด้วยไฟล์จริง (`typechange`) |
| CMake | `simulate/CMakeLists.txt`, `deploy/robots/g1_23dof/CMakeLists.txt` | ถูกแก้ path (`modified`) |
| ไฟล์ใหม่ | `simulate/mujoco/include/*.h`, `include/experimental/`, `include/render/`, `include_bak/`, `lib/libmujoco.so.bak`, `mujoco-3.3.6/` | `untracked` |

> ⚠️ **ห้าม commit ไฟล์พวกนี้** ถ้าขึ้น repo ไป sim บนเครื่อง x86 จะ build ไม่ได้

## วิธีที่ใช้ (รันบน DGX เท่านั้น)

`.git/info/exclude` ใช้ซ่อนไฟล์ untracked และ `skip-worktree` ใช้ซ่อนไฟล์ที่ track อยู่แล้ว ทั้งสองอย่างมีผลแค่ในเครื่องนั้น ไม่ถูก push

```bash
cd ~/nvidia/unitree/unitree_rl_mjlab

# 1) ซ่อนไฟล์ untracked ของ MuJoCo ARM
cat >> .git/info/exclude <<'EOF'

# DGX (aarch64) local MuJoCo build, not for the repo
/simulate/mujoco/include/*.h
/simulate/mujoco/include/experimental/
/simulate/mujoco/include/render/
/simulate/mujoco/include_bak/
/simulate/mujoco/lib/libmujoco.so.bak
/simulate/mujoco/mujoco-3.3.6/
EOF

# 2) ให้ git ไม่สนใจไฟล์ที่ track อยู่แล้วแต่ถูกแก้หรือลบบน DGX
git ls-files -z simulate/mujoco/include/mujoco simulate/mujoco/lib/libmujoco.so \
    deploy/robots/g1_23dof/CMakeLists.txt simulate/CMakeLists.txt \
  | xargs -0 git update-index --skip-worktree

git status   # ต้องขึ้น: nothing to commit, working tree clean
```

`/simulate/mujoco/include/*.h` จะซ่อนเฉพาะไฟล์ `.h` ที่วางอยู่ใน `include/` โดยตรง ไม่รวม `include/mujoco/` ที่ repo track อยู่

## ข้อควรระวัง

- **การแก้ `CMakeLists.txt` บน DGX จะไม่ขึ้นใน `git status`** ถ้าตั้งใจแก้ไฟล์นี้เพื่อ commit ต้องยกเลิก skip-worktree ก่อน
- **ถ้า remote แก้ไฟล์ที่ซ่อนไว้** `git pull` หรือ `git checkout` จะขึ้น error ประมาณ `Your local changes ... would be overwritten` ให้ยกเลิก skip-worktree แล้ว merge ด้วยมือ **อย่าใช้** `git checkout -f`, `git restore` หรือ `git reset --hard` เพราะไฟล์ build สำหรับ ARM จะหาย
- clone ใหม่บน DGX ต้องรันคำสั่งข้างบนซ้ำ เพราะการตั้งค่าทั้งสองแบบไม่ได้อยู่ใน repo

## คำสั่งตรวจและยกเลิก

```bash
# ดูไฟล์ที่ถูกซ่อนด้วย skip-worktree (ขึ้นต้นด้วย S)
git ls-files -v | grep '^S'

# ยกเลิก skip-worktree ทั้งหมด
git ls-files -v | grep '^S' | cut -c3- | tr '\n' '\0' | xargs -0 git update-index --no-skip-worktree

# ยกเลิก exclude: ลบบล็อก "# DGX (aarch64) ..." ออกจาก .git/info/exclude
```
