# TP2: Path Finder Robot (PIC32)

## Assignment Summary

Build a C agent for a differential-drive robot (PIC32 MCU) to:

1. **Phase 1**: Follow black lines on white floor, turning left at intersections
2. **Phase 2**: Detect the target area (wide black line) and stop
3. **Phase 3**: Return to the start position via the shortest path, minimizing time

**Key constraint**: The black-line graph is acyclic (a tree). Left-turn priority
at intersections guarantees complete exploration without explicit backtracking
logic during Phase 1.

## Project Structure

```
TP2/
├── src/                        # Robot agent source (*.c, *.h)
│   ├── rm-mr32.c               # DETI hardware library (READ-ONLY)
│   ├── rm-mr32.h               # Hardware API header (READ-ONLY)
│   └── robot-agent.c           # Your implementation (create this)
├── flake.nix                   # Nix dev shell with PIC32 toolchain
├── flake.lock                  # Pinned nixpkgs
├── .envrc                      # direnv: auto-enters `nix develop --impure`
├── pic32-64-2017_09_15.tgz     # Compiler tarball (untracked, required locally)
└── assignment.md               # Full assignment text
```

## Dev Environment

The project uses a Nix flake with `autoPatchelfHook` to make the precompiled
PIC32 GCC toolchain work on NixOS. The tarball must be present in the project
root but is `.gitignore`d.

```bash
# Enter the dev shell (handled automatically by direnv)
nix develop --impure
```

## Build & Deploy Commands

All commands run inside the `nix develop --impure` shell:

```bash
cd src

# Compile agent + hardware library
pcompile robot-agent.c rm-mr32.c
# Output: robot-agent.hex

# Flash to robot (connect USB first)
ldpic32 -w robot-agent.hex

# Open serial terminal for sensor output
pterm
```

## Hardware API (rm-mr32.h)

| Function | Purpose |
|----------|---------|
| `initPIC32()` | Initialize robot hardware and timers |
| `closedLoopControl(true)` | Enable wheel speed PID control |
| `setVel2(left, right)` | Wheel velocities [-100, 100]; 100 ≈ 0.7 m/s |
| `readAnalogSensors()` | Fill `analogSensors` struct with ADC readings |
| `readLineSensors(0)` | Return 5-bit ground IR pattern (bit4=left, bit0=right) |
| `getRobotPos(&x, &y, &h)` | Dead reckoning: x, y (m), heading (rad) |
| `setRobotPos(x, y, h)` | Reset odometry to given pose |
| `waitTick40ms()` | Block until next 40 ms control tick |
| `startButton()` / `stopButton()` | Read physical control buttons |
| `leds(state)` / `led(n, val)` | Set status LEDs |
| `enableObstSens()` / `disableObstSens()` | Obstacle sensors (not used in this assignment) |
| `normalizeAngle(angle)` | Wrap angle to ]-π, +π] |

### Global Variables
- `MR32_analogSensors analogSensors` — struct with last ADC readings
- `volatile bool tick40ms` — set every 40 ms by timer ISR

## Code Style & Constraints

- **Language**: C (not C++). PIC32 GCC 3.4.4. No `class`, templates, or STL.
- **No dynamic allocation**: Never use `malloc` / `free` on embedded.
- **No floating-point in ISRs**: PIC32 FPU is slow; keep float math in the main loop only.
- **40 ms tick discipline**: Use `waitTick40ms()` as the sole timing source for the control loop. Avoid `delay()`, `wait()`, or busy-waiting.
- **Sensor read order**: Always call `readAnalogSensors()` before `readLineSensors(0)`.
- **Velocity scale**: `setVel2()` accepts [-100, 100]. Use moderate speeds (e.g., 30-70) for stability.

## Example: Line Follower Tick

```c
void lineFollowerTick(void)
{
    waitTick40ms();
    readAnalogSensors();
    unsigned int ground = readLineSensors(0);  // 5-bit sensor pattern

    if (ground == 0b00100) {          // Centered on line
        setVel2(60, 60);
    } else if (ground & 0b11000) {    // Sensors to the left see line → drift right
        setVel2(30, 60);
    } else if (ground & 0b00011) {    // Sensors to the right see line → drift left
        setVel2(60, 30);
    } else if (ground == 0b00000) {   // Lost line
        setVel2(0, 0);                // Stop or enter search behaviour
    }
}
```

## Example: Odometry-Based Straight-Line Drive

```c
void driveDistance(double distance, int speed)
{
    double x0, y0, h0;
    getRobotPos(&x0, &y0, &h0);

    double traveled = 0.0;
    while (traveled < distance) {
        waitTick40ms();
        double x, y, h;
        getRobotPos(&x, &y, &h);
        traveled = hypot(x - x0, y - y0);
        setVel2(speed, speed);
    }
    setVel2(0, 0);
}
```

## Algorithm Guidance

### Phase 1 — Left-Turn Exploration
At intersections the ground sensor shows a wide black pattern (e.g., `0b11111`
or sustained multi-bit detection). The required behaviour is **always turn left**.
Because the graph is an acyclic tree, this explores every reachable edge exactly
once without needing an explicit stack or visited set.

Suggested implementation:
- Detect intersection (`ground == 0b11111` or similar wide pattern).
- Execute a 90° left turn using `rotateRel()` or open-loop `setVel2()` with encoder/odometry check.
- Resume line following on the new branch.

### Phase 2 — Target Detection
The target is a **wide black line** (wider than normal path lines). Detection
criteria:
- All 5 ground sensors report black simultaneously (`ground == 0b11111`).
- The wide pattern persists for a longer distance than a normal intersection.

On detection:
1. Stop the robot (`setVel2(0, 0)`).
2. Record the target pose with `getRobotPos(&targetX, &targetY, &targetH)`.
3. Transition to Phase 3.

### Phase 3 — Shortest-Path Return
Since the graph is a tree, the shortest path from target back to start is the
unique simple path. Recommended strategies:

| Strategy | How it works | Pros | Cons |
|----------|--------------|------|------|
| **A. Parent-pointer DFS** | During Phase 1, record each intersection as a node and the direction taken. Store parent pointer. Return by following parent links. | Optimal, guaranteed correct | Requires graph data structures |
| **B. Odometry straight-line** | Compute bearing from target pose to (0,0), drive straight. | Simple | Dead-reckoning drift makes this unreliable over long paths |
| **C. Learned graph + Dijkstra** | Build a node/edge map during exploration (coordinates + connectivity). Run Dijkstra/A* on the learned tree. | Optimal, reusable | More code complexity |

**Recommended**: **Strategy A** for reliability, or **C** if the assignment
explicitly rewards shortest-path optimality. Ensure the robot stops exactly at
the start position (compare pose to `(0, 0)` with a small tolerance).

## Robot Calibration

The `rm-mr32.c` library contains per-robot servo calibration values selected by
the `ROBOT` macro:

```c
#ifndef ROBOT
    #define ROBOT 1
#endif
```

If you know your physical robot's ID, define it at compile time:

```bash
pcompile -DROBOT=5 robot-agent.c rm-mr32.c
```

Default (`ROBOT 1`) is safe for initial testing; adjust if the robot's heading
or servo behaviour is visibly off.

## Assignment-Specific Notes

- The PDF contains a **path map image** showing the black-line layout (tree
topology, start position, target position). This image is not included in the
textual assignment. Consult it when designing the exploration/return strategy.
- **Obstacle sensors** are mentioned in the example code but are **not used**
in this assignment. Focus entirely on ground sensors and odometry.
- The `pcompile` script hardcodes `PICDIR=/opt/pic32mx` but has been patched
by the Nix derivation to use the Nix store path automatically.

## Boundaries

- ✅ **Always do**: Use `waitTick40ms()` for control-loop timing; include
  `rm-mr32.c` when compiling; call `initPIC32()` and `closedLoopControl(true)`
  in `main()` before the control loop.
- ⚠️ **Ask first**: Before modifying `rm-mr32.c` / `rm-mr32.h`; before changing
  build commands in `flake.nix`; before adding new external libraries.
- 🚫 **Never do**: Use `printf` inside the 40 ms control loop (too slow for
  real-time control); use `malloc` / `free`; modify servo calibration `#defines`
  in `rm-mr32.c`; commit `pic32-64-2017_09_15.tgz` to git.

## External References

See `docs/references.md` for curated online documentation on line-following
algorithms, PID control, differential-drive odometry, and PIC32 hardware details.

## Deliverables Checklist

- [ ] `src/robot-agent.c` — Working robot agent implementing all 3 phases
- [ ] Report (PDF, Springer LNCS format)
- [ ] Live demo (3 runs)
