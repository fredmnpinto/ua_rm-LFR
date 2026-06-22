# TP2: Path Finder Robot (PIC32)

Embedded C agent for a differential-drive PIC32 robot. Follows black lines, turns left at intersections, detects a wide target line, and returns to start via Dijkstra shortest-path on a navigation graph.

## Project Structure

```
src/
├── robot-agent.c      # Main loop: init, 40 ms tick, sensor read, stateMachine_tick()
├── state_machine.c/h  # Core state machine (11 states: IDLE → FOLLOW_LINE → VERIFY_TARGET → AT_TARGET → APPROACH_CENTER → CHOOSE_NEXT → TURN_TO_EDGE → RETURN_TURN → RETURN_FOLLOW → RETURN_AT_NODE → DONE)
├── line_follower.c/h  # Bang-bang line following (not PID)
├── rotation.c/h       # PI heading controller for in-place turns
├── nav_graph.c/h      # Static navigation graph: nodes, edges, Dijkstra shortest path
├── leds.c/h           # Phase indication via 4 on-board LEDs
├── logging.c/h        # printf-based state transition logging
├── config.h           # Compile-time constants (speeds, thresholds, sensor bitmasks)
├── rm-mr32.c/h        # DETI hardware library (READ-ONLY)
└── pcompile           # Wrapper script with HARDCODED Nix store path
tests/
├── test_nav_graph.c   # Host-runnable unit tests for nav_graph logic
├── rm-mr32.h          # Mock hardware header for host compilation
└── Makefile           # Test build runner
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
- **No dynamic allocation**: No `malloc`/`free`. All structures are static (see `nav_graph.c`).

## Architecture Notes

### State Machine
`state_machine.c` implements a table-driven state machine. States have optional `onEnter`/`onTick`/`onExit` handlers. The `changeState()` helper logs transitions and updates the `Phase` enum for LED mapping.

Key states:
- **FOLLOW_LINE**: Calls `lineFollower_centerOnLine()`. On `SENSOR_ALL`, transitions to `VERIFY_TARGET`. On left-path detection (`ground & SENSOR_LEFT`), skips verification and goes to `APPROACH_CENTER`. Also detects left/right corners and deadends.
- **VERIFY_TARGET**: Drives straight while `SENSOR_ALL` persists. If traveled distance ≥ `TARGET_DIST_THRESHOLD`, it's the target → `AT_TARGET`. Otherwise it was just an intersection → `APPROACH_CENTER`.
- **AT_TARGET**: Records target pose, sets target flag on graph node, then transitions to `CHOOSE_NEXT` to continue exploration.
- **APPROACH_CENTER**: Drives forward `INTERSECTION_CENTER_DIST` into the intersection, then creates/merges a graph node and links an edge to the previous node.
- **CHOOSE_NEXT**: Checks all edges of the current node for unexplored status regardless of node type. Uses left-hand rule (left, straight, right, back) to choose the next direction. If all edges are explored and at origin, starts Dijkstra return navigation.
- **TURN_TO_EDGE**: Uses `rotation.c` PI controller to turn toward the chosen edge direction.
- **RETURN_TURN**: Computes bearing to next node on the Dijkstra path, uses `rotation.c` to turn toward it.
- **RETURN_FOLLOW**: Follows line while checking odometry distance to next node. When within `ORIGIN_TOLERANCE`, transitions to `RETURN_AT_NODE`.
- **RETURN_AT_NODE**: Advances the path index. If at the end of leg 1 (origin→target), computes Dijkstra path for leg 2 (target→origin). If at the end of leg 2, mission is complete.

### Line Follower
Bang-bang (not PID). `lineFollower_centerOnLine()` sets motor speeds based on discrete sensor bit patterns. Returns `*pLost = true` when `ground == SENSOR_NONE` so the state machine can handle recovery (spin search, then timeout).

### Rotation Controller
`rotation.c` uses a PI controller (`KP_ROT`, `KI_ROT` from `config.h`) to rotate in place. `rotation_start()` sets target angle; `rotation_tick()` computes error via `normalizeAngle()` and calls `setVel2(-cmdVel, cmdVel)`.

### Navigation Graph
`nav_graph.c` maintains a static graph of nodes and edges (`MAX_NODES = 32`, `MAX_EDGES = 64`). During exploration, each intersection/corner/deadend becomes a node, and line segments become weighted edges. During return, `navGraph_dijkstra()` computes the shortest path between any two nodes. The return mission consists of two legs: origin→target, then target→origin.

### Corner and Deadend Detection
- **Left corner**: `SENSOR_LEFT` persists for `CORNER_TICKS` consecutive ticks.
- **Right corner**: `SENSOR_RIGHT` persists for `CORNER_TICKS` consecutive ticks.
- **Deadend**: Line lost for `LOST_TICKS` consecutive ticks (spin search timeout).

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

## Unit Tests

Host-runnable tests for pure logic (no hardware dependencies) are in `tests/`:

```bash
cd tests && make
```

Tests cover nav_graph node creation, merging, edge linking, Dijkstra pathfinding, target flagging, and edge exploration. The mock `tests/rm-mr32.h` shadows the real hardware header during host compilation.

## Boundaries

- ✅ **Always do**: Use `make` (not raw `pcompile`); include `rm-mr32.c` in builds; call `initPIC32()` and `closedLoopControl(true)` before the loop; call `readAnalogSensors()` before `readLineSensors(0)`.
- ⚠️ **Ask first**: Before modifying `rm-mr32.c/h`; before changing `flake.nix`; before adding libraries.
- 🚫 **Never do**: Use `malloc`/`free`; modify servo calibration `#defines` in `rm-mr32.c`; commit `pic32-64-2017_09_15.tgz` to git.

## References

- [`README.md`](README.md) — Primary project documentation: quick-start guide, state-machine diagrams, and build instructions
- [`docs/references.md`](docs/references.md) — Curated external links on line following, PID, odometry, and PIC32 hardware
