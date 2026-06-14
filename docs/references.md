# Curated External References for TP2

This document collects online resources useful for implementing the path-finder robot agent.
Links are grouped by topic and annotated with their relevance to the MR32 / PIC32 platform.

> **Note**: Web links can rot over time. If a link is dead, use the title + author to search for a mirror.

---

## 1. Hardware & Platform Specific

| Resource | URL | Why It's Useful |
|----------|-----|-----------------|
| **MR32 Resources (J.L. Azevedo, DETI-UA)** | https://sweet.ua.pt/jla/ | Home page for the MR32 robot author. May contain updated examples, datasheets, or errata. |
| **Programming the DETI Robot in WSL** | https://sweet.ua.pt/pf/2022/06/25/Programming_DETI_Robot_from_WSL.html | DETI-specific tutorial for setting up the programming toolchain under Windows Subsystem for Linux (also relevant for understanding USB/serial connectivity). |
| **PIC32MX Family Reference Manual** | https://hades.mech.northwestern.edu/images/2/21/61132B_PIC32ReferenceManual.pdf | Core hardware manual for the PIC32MX series (CPU, memory model, peripherals, interrupts). Useful if you need to understand timer ISR behaviour or ADC details that `rm-mr32.c` abstracts. |
| **PIC32M Family Reference (Microchip)** | https://developerhelp.microchip.com/xwiki/bin/view/products/mcu-mpu/32bit-mcu/PIC32/family-reference-pages/manual/ | Official Microchip reference covering core, memory, and peripherals not fully detailed in individual device datasheets. |
| **MPLAB C32 Release Notes (UA-LARS)** | http://lars.mec.ua.pt/public/LAR%20Projects/Humanoid/2011_WilliamLage/Guias%20e%20Tutoriais/MPLAB_IDE_8_56_Release_Notes/Readme%20for%20MPLAB%20C32.htm | Notes on the PIC32 GCC toolchain (case-sensitive filenames, shell command behaviour). Relevant if you hit build issues with `pcompile`. |

### Local Header Files (inside the Nix shell)
- `detpic32.h` — Register definitions for the DETI PIC32 board. Located in the toolchain include path (usually under `/opt/pic32mx` or the Nix store path patched by the flake).
- `rm-mr32.h` — Hardware API header provided with this assignment (see `src/rm-mr32.h`).

---

## 2. Line Following & PID Control

| Resource | URL | Why It's Useful |
|----------|-----|-----------------|
| **Pololu Advanced PID Line Following** | https://www.pololu.com/docs/0J21/7.c | Industry-standard tutorial on PID-based line following. Walks through weighted sensor error, PID calculation, and motor differential updates. Translates well from Pololu 3pi to MR32 5-bit sensors. |
| **TeachMeMicro PID Implementation** | https://www.teachmemicro.com/implementing-pid-for-a-line-follower-robot/ | Beginner-friendly breakdown of `pid_calc()` and adjusting left/right motor speeds based on the correction term. Good starting point before tuning. |
| **Industrial Monitor Direct (Multi-Sensor C)** | https://industrialmonitordirect.com/blogs/knowledgebase/implementing-pid-line-follower-with-multi-sensor-array-in-c | Specifically covers multi-sensor arrays in **C** (not Arduino C++). Discusses symmetric weight arrays (e.g., `{-3,-1,+1,+3}`) for error calculation — adaptable to the MR32's 5-bit digital pattern. |
| **ThinkRobotics PID Tuning Guide** | https://thinkrobotics.com/blogs/learn/pid-tuning-for-line-follower-robot-complete-how-to-guide | Practical tuning workflow: start with Kp until oscillation, add Kd to dampen, introduce small Ki if steady-state error persists. Typical starting values: Kp≈1.0, Ki=0.0, Kd≈0.5. |
| **RobotShop PID Tutorials** | https://www.robotshop.com/community/forum/t/pid-tutorials-for-line-following/13164 | Classic tutorial with downloadable PDF and sample video. Good conceptual refresher on P, I, D terms. |

### Key Takeaways for MR32
- Your sensor is a **5-bit digital pattern** (`readLineSensors(0)`), not an analog reflectance array. You can approximate an "error" value by assigning weights to each bit (e.g., bit4 = -2, bit3 = -1, bit2 = 0, bit1 = +1, bit0 = +2) and summing the active bits.
- The control loop **must run inside `waitTick40ms()`**: compute error → PID → `setVel2()` once per tick. Avoid floating-point computation inside ISRs (the FPU is slow on PIC32).

---

## 3. Odometry & Dead Reckoning

| Resource | URL | Why It's Useful |
|----------|-----|-----------------|
| **CMU Kinematics & Dead Reckoning** | https://web2.qatar.cmu.edu/~gdicaro/16311-Fall17/slides/16311-8-Kinematics-DeadReckoning.pdf | Academic slides covering forward kinematics equations for differential drive. Explains how to integrate wheel velocities into pose updates over `Δt`. |
| **Caltech DiffDrive Odometry** | http://robotics.caltech.edu/wiki/images/d/d5/DiffDriveOdometry.pdf | Rigorous mathematical derivation of odometry equations. Useful for understanding the assumptions behind `getRobotPos()` / `setRobotPos()` in the MR32 library. |
| **Seattle Robotics Dead Reckoning** | https://archive.seattlerobotics.org/encoder/200010/dead_reckoning_article.html | Practical C implementation of dead reckoning on a small robot. Shows how encoder counts translate to `x, y, theta` updates in a background thread — conceptually similar to what `rm-mr32.c` does internally. |
| **Rossum Project DiffSteer** | https://rossum.sourceforge.net/papers/DiffSteer/ | Tutorial and elementary trajectory model for differential steering. Contains simplified formulas and a Java applet to visualise arc-based motion. |
| **Robotics StackExchange (Arduino Odometry)** | https://robotics.stackexchange.com/questions/8693/how-to-perform-odometry-on-an-arduino-for-a-differential-wheeled-robot | Highly voted answer with the exact update formulas you can adapt: `meanDistance = (SL + SR)/2`, then `x += meanDistance * cos(theta)`, `y += meanDistance * sin(theta)`. Directly applicable if you ever need to implement your own odometry helper. |
| **Automatic Addison (Wheel Odometry)** | https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/ | Step-by-step numerical example of updating pose from encoder ticks. Good for sanity-checking your own math against a worked example. |

### MR32 Notes
- The MR32 library already provides `getRobotPos(&x, &y, &h)` and `setRobotPos(x, y, h)`. You generally do **not** need to re-implement the kinematics unless debugging drift.
- Use `normalizeAngle()` from `rm-mr32.h` when comparing headings to keep values in `]-π, +π]`.
- Phase 3 "straight-line return" using odometry alone is risky over long distances due to wheel-slip drift. The recommended strategy is **parent-pointer DFS** (Strategy A) rather than pure odometry (Strategy B).

---

## 4. PIC32 / Embedded C Programming

| Resource | URL | Why It's Useful |
|----------|-----|-----------------|
| **PlatformIO PIC32 Docs** | https://docs.platformio.org/en/latest/platforms/microchippic32.html | Overview of the PIC32 development ecosystem ( toolchains, upload methods, framework support). |
| **ChipKIT Max32 Robot Example** | https://chipkit.net/my-first-robot-controlled-by-a-chipkit-max32-using-a-microchip-pic32-microcontroller | Project log of a PIC32-based robot with IR sensors, encoders, and SPI. Good for seeing how sensors and actuators are wired on a similar MIPS-based MCU. |
| **PIC Microcontroller Robotics Projects** | https://pic-microcontroller.com/projects/robotics-automation-projects/ | Curated list of PIC-based robotics projects (many use line following, obstacle avoidance, etc.). Can provide alternative design ideas. |

### Embedded Constraints Checklist
- **No `malloc` / `free`**: All data structures must be statically allocated.
- **No `printf` in the 40 ms loop**: Use LEDs or accumulate debug info to print outside the real-time path.
- **C only**: PIC32 GCC 3.4.4. No C++ classes, templates, or STL.

---

## 5. Videos (Supplementary)

| Title | URL | Length | Notes |
|-------|-----|--------|-------|
| Fast Line Follower Robot using PID | https://www.youtube.com/watch?v=ST8KdWPMzp4 | ~10 min | Demonstrates aggressive PID tuning and high-speed turning behaviour. |
| Kinematics of Differential Drive Robots | https://www.youtube.com/watch?v=RZlZcDxQ8P4 | ~50 min | Deep dive into differential drive kinematics and odometry derivation. |
| Mark 01 — Odometry and PID Controller | https://www.youtube.com/watch?v=337Sp3PtVDc | ~11 min | Practical build + tuning of a differential drive robot with encoder feedback. |

---

## Quick Search Queries (for finding alternatives)

If a link above dies, use these queries in your search engine of choice:

- `Pololu PID line follower tutorial`
- `PIC32MX family reference manual PDF`
- `differential drive robot odometry C implementation`
- `MR32 DETI UA robot programming`
- `5-bit line sensor PID error calculation embedded C`
