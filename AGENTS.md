# TP2: Path Finder Robot (PIC32)

Embedded C agent for a differential-drive PIC32 robot. Follows black lines, turns left at intersections, detects a wide target line, and returns to start via parent-pointer DFS.

## Project Structure

```
src/
├── robot-agent.c      # Main loop: init, 40 ms tick, sensor read, stateMachine_tick()
├── state_machine.c/h  # Core state machine (9 states: IDLE → FOLLOW_LINE → VERIFY_TARGET/PREPARE_TURN → TURN_LEFT → AT_TARGET → RETURN_TURN → RETURN_FOLLOW → DONE)
├── line_follower.c/h  # Bang-bang line following (not PID)
├── rotation.c/h       # PI heading controller for in-place turns
├── nav_stack.c/h      # Static pose stack for parent-pointer DFS return
├── leds.c/h           # Phase indication via 4 on-board LEDs
├── logging.c/h        # printf-based state transition logging
├── config.h           # Compile-time constants (speeds, thresholds, sensor bitmasks)
├── rm-mr32.c/h        # DETI hardware library (READ-ONLY)
└── pcompile           # Wrapper script with HARDCODED Nix store path
```

## Dev Environment

Requires Nix with flakes. The compiler tarball `pic32-64-2017_09_15.tgz` must exist in the project root (it is `.gitignore`d, not tracked).

```bash
# Enter shell (direnv handles this automatically)
nix develop --impure
```

**Critical**: `src/pcompile` contains a hardcoded Nix store path (`PICDIR=/nix/store/...`). If the flake is ever rebuilt on a new machine, this path will be stale and compilation will fail. The Makefile works around this by using the local `src/pcompile`.

## Build & Deploy

Use the Makefile — do not call `pcompile` directly unless you understand the Nix store path issue.

```bash
# Build (default ROBOT=1)
make

# Build for a specific physical robot
make ROBOT=5

# Build + flash to robot (USB must be connected)
make flash

# Build + flash + open serial terminal
make run

# Just the terminal
make term

# Clean build artifacts
make clean
```

Output: `build/robot-agent.hex`

## Control Loop Discipline

The main loop in `robot-agent.c` is the only place that should call `waitTick40ms()`:

```c
while (1) {
    waitTick40ms();          // Macro: blocks until tick40ms==1, then clears it
    readAnalogSensors();     // MUST call this BEFORE readLineSensors(0)
    unsigned int ground = readLineSensors(0);
    // ... state machine tick ...
}
```

- **Never** use `delay()`, `wait()`, or busy-waiting for timing.
- **Avoid** calling `printf` on every 40 ms tick — it is slow and can destabilize the control loop. `logging.c` demonstrates the right pattern: print only on state transitions (infrequent events). For sensor debugging, throttle prints (e.g., every N ticks or only when values change significantly).
- **No floating-point in ISRs**: PIC32 FPU is slow; keep `double` math in the main loop.
- **No dynamic allocation**: No `malloc`/`free`. All structures are static (see `nav_stack.c`).

## Architecture Notes

### State Machine
`state_machine.c` implements a table-driven state machine. States have optional `onEnter`/`onTick`/`onExit` handlers. The `changeState()` helper logs transitions and updates the `Phase` enum for LED mapping.

Key states:
- **FOLLOW_LINE**: Calls `lineFollower_centerOnLine()`. On `SENSOR_ALL`, transitions to `VERIFY_TARGET`. On left-path detection (`ground & SENSOR_LEFT`), skips verification and goes to `PREPARE_TURN`.
- **VERIFY_TARGET**: Drives straight while `SENSOR_ALL` persists. If traveled distance ≥ `TARGET_DIST_THRESHOLD`, it's the target → `AT_TARGET`. Otherwise it was just an intersection → `PREPARE_TURN`.
- **PREPARE_TURN**: Drives forward `INTERSECTION_CENTER_DIST` mm into the intersection, then pushes current pose onto `nav_stack` and transitions to `TURN_LEFT`.
- **TURN_LEFT**: Uses `rotation.c` PI controller to rotate +90°.
- **AT_TARGET**: Records target pose, then pops the stack and transitions to `RETURN_TURN`.
- **RETURN_TURN**: Computes bearing to parent node, uses `rotation.c` to turn toward it.
- **RETURN_FOLLOW**: Follows line while checking odometry distance to parent node. When within `ORIGIN_TOLERANCE`, pops next parent or finishes.

### Line Follower
Bang-bang (not PID). `lineFollower_centerOnLine()` sets motor speeds based on discrete sensor bit patterns. Returns `*pLost = true` when `ground == SENSOR_NONE` so the state machine can handle recovery (spin search, then timeout).

### Rotation Controller
`rotation.c` uses a PI controller (`KP_ROT`, `KI_ROT` from `config.h`) to rotate in place. `rotation_start()` sets target angle; `rotation_tick()` computes error via `normalizeAngle()` and calls `setVel2(-cmdVel, cmdVel)`.

### Navigation Stack
`nav_stack.c` is a static array stack (`MAX_STACK_DEPTH = 32`). During exploration, each intersection pose is `nav_push()`ed. During return, poses are `nav_pop()`ed to follow parent pointers back to start. This is Strategy A (parent-pointer DFS) from the assignment.

## Hardware API (rm-mr32.h)

| Function | Purpose |
|----------|---------|
| `initPIC32()` | Initialize hardware, timers, ISRs |
| `closedLoopControl(true)` | Enable wheel speed PID |
| `setVel2(left, right)` | Wheel velocities [-100, 100]; 100 ≈ 0.7 m/s |
| `readAnalogSensors()` | Fill `analogSensors` struct (call before `readLineSensors`) |
| `readLineSensors(0)` | Return 5-bit ground IR pattern (bit4=left, bit0=right) |
| `getRobotPos(&x, &y, &h)` | Dead reckoning: x, y (m), heading (rad) |
| `setRobotPos(x, y, h)` | Reset odometry |
| `waitTick40ms()` | Block until next 40 ms control tick (macro) |
| `startButton()` / `stopButton()` | Physical control buttons |
| `leds(state)` / `led(n, val)` | Set status LEDs |
| `normalizeAngle(angle)` | Wrap angle to ]-π, +π] |

### Global Variables
- `MR32_analogSensors analogSensors` — last ADC readings
- `volatile bool tick40ms` — set every 40 ms by timer ISR

## Sensor Bit Patterns (config.h)

```c
#define SENSOR_NONE   0b00000
#define SENSOR_CENTER 0b00100
#define SENSOR_LEFT   0b11000   // bits 4 and 3
#define SENSOR_RIGHT  0b00011   // bits 1 and 0
#define SENSOR_ALL    0b11111
```

## Robot Calibration

`rm-mr32.c` selects servo calibration values via the `ROBOT` macro (default: 1). Pass at compile time:

```bash
make ROBOT=5
```

Supported IDs: 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12.

## Boundaries

- ✅ **Always do**: Use `make` (not raw `pcompile`); include `rm-mr32.c` in builds; call `initPIC32()` and `closedLoopControl(true)` before the loop; call `readAnalogSensors()` before `readLineSensors(0)`.
- ⚠️ **Ask first**: Before modifying `rm-mr32.c/h`; before changing `flake.nix`; before adding libraries.
- 🚫 **Never do**: Use `malloc`/`free`; modify servo calibration `#defines` in `rm-mr32.c`; commit `pic32-64-2017_09_15.tgz` to git.

## References

- [`README.md`](README.md) — Primary project documentation: quick-start guide, state-machine diagrams, and build instructions
- [`docs/references.md`](docs/references.md) — Curated external links on line following, PID, odometry, and PIC32 hardware
