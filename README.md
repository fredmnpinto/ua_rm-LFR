# TP2 Path Finder Robot

Embedded C agent for a differential-drive PIC32 robot (DETI MR32 platform).  
Follows black lines, turns left at intersections, detects a wide target line, and returns to the start position via Dijkstra shortest-path on a navigation graph.

## Mission Overview

- Phase 1 — **Explore**: Follow lines and always turn left at intersections (acyclic tree exploration)
- Phase 2 — **Detect Target**: Distinguish the wide target line from normal intersections using a distance threshold
- Phase 3 — **Return**: Navigate back to the start via the shortest path using Dijkstra on a graph recorded during exploration (two-leg return: origin→target, then target→origin)

## Quick Start

1. Enter the Nix development shell: `nix develop --impure`
2. Ensure `pic32-64-2017_09_15.tgz` exists in the project root (`.gitignore`d, not tracked)
3. Build: `make` (default `ROBOT=1`) or `make ROBOT=5` for specific calibration
4. Flash: `make flash` (robot USB must be connected)
5. Run: `make run` (build + flash + serial terminal)
6. Press the robot's **start button** to begin the mission

> **Note**: `src/pcompile` contains a hardcoded Nix store path. If the flake is rebuilt on a new machine, this path may become stale. Always use `make` (which uses the local `src/pcompile`) rather than calling `pcompile` directly.

## Repository Structure

```
.
├── Makefile              # Build system (supports ROBOT=N calibration)
├── src/
│   ├── robot-agent.c     # Main loop: init, 40 ms tick, sensor dispatch
│   ├── state_machine.c/h # 11-state table-driven FSM
│   ├── line_follower.c/h # Bang-bang line following
│   ├── rotation.c/h      # PI heading controller for in-place turns
│   ├── nav_graph.c/h     # Static navigation graph (nodes, edges, Dijkstra)
│   ├── leds.c/h          # Phase indication via 4 on-board LEDs
│   ├── logging.c/h       # printf-based state transition logging
│   ├── config.h          # Compile-time constants, sensor masks, Pose typedef
│   └── rm-mr32.c/h       # DETI hardware library (READ-ONLY)
├── tests/
│   ├── test_nav_graph.c  # Host-runnable unit tests for nav_graph logic
│   ├── rm-mr32.h         # Mock hardware header for host compilation
│   └── Makefile          # Test build runner
├── docs/
│   ├── specs/
│   │   └── functional-spec.md  # Full FR-XXX requirements
│   └── references.md     # Curated links on PID, odometry, PIC32
├── assignment.md         # Official course assignment
└── AGENTS.md             # Project coding guidelines & architecture notes
```

## Hardware Platform

- **Platform**: DETI MR32 differential-drive robot
- **MCU**: PIC32MX (MIPS M4K core, no hardware FPU)
- **Sensors**: 5-bit ground IR sensor array, wheel encoders
- **Actuators**: 2 DC motors with closed-loop PID velocity control
- **I/O**: Physical start/stop buttons, 4 status LEDs
- **Control frequency**: 25 Hz (40 ms periodic tick)

## Build & Deploy

| Command | Description |
|---------|-------------|
| `make` | Build with default `ROBOT=1` |
| `make ROBOT=5` | Build with robot 5 servo calibration |
| `make flash` | Build and flash to robot via USB |
| `make run` | Build + flash + open serial terminal |
| `make term` | Open serial terminal only |
| `make clean` | Remove all build artifacts |

Supported robot IDs: 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12.

Build artifacts are placed in `build/`:
- `build/robot-agent.hex` — Flashable binary
- `build/robot-agent.elf` — Linked ELF
- `build/*.o` — Object files
- `build/robot-agent.map` — Linker map

## State Machine

**State Descriptions**:

| State | Phase | Description |
|-------|-------|-------------|
| `STATE_IDLE` | — | Motors off, waiting for `startButton()` |
| `STATE_FOLLOW_LINE` | 1 | Bang-bang line following; detects intersections, corners, and left-path shortcuts |
| `STATE_VERIFY_TARGET` | 2 | Validates whether wide pattern is target (persistent + distance) |
| `STATE_AT_TARGET` | 2 | Target confirmed; record pose; continue exploration or start return |
| `STATE_APPROACH_CENTER` | 1 | Drive forward into intersection; create/merge graph node and link edge |
| `STATE_CHOOSE_NEXT` | 1 | Select next unexplored edge using left-hand rule; fall back to backtracking |
| `STATE_TURN_TO_EDGE` | 1 | Non-blocking turn toward chosen edge direction |
| `STATE_RETURN_TURN` | 3 | Turn toward next node on Dijkstra return path |
| `STATE_RETURN_FOLLOW` | 3 | Follow line to next node on return path |
| `STATE_RETURN_AT_NODE` | 3 | Arrived at node; advance path index or switch return leg |
| `STATE_DONE` | — | Mission complete; motors off; wait for restart |

**State Transitions**:

| # | From | To | Condition |
|---|------|----|-----------|
| 1 | `IDLE` | `FOLLOW_LINE` | `startButton()` pressed |
| 2 | `FOLLOW_LINE` | `VERIFY_TARGET` | `ground == SENSOR_ALL` (intersection or target candidate) |
| 3 | `FOLLOW_LINE` | `APPROACH_CENTER` | `(ground & SENSOR_LEFT) == SENSOR_LEFT` (path to left, skip verification) |
| 4 | `FOLLOW_LINE` | `APPROACH_CENTER` | Left corner or right corner detected for `CORNER_TICKS` consecutive ticks |
| 5 | `FOLLOW_LINE` | `APPROACH_CENTER` | Lost line for ≥ `LOST_TICKS` (deadend) |
| 6 | `VERIFY_TARGET` | `AT_TARGET` | `ground == SENSOR_ALL` persists AND `dist >= TARGET_DIST_THRESHOLD` |
| 7 | `VERIFY_TARGET` | `APPROACH_CENTER` | `ground != SENSOR_ALL` before threshold (normal intersection) |
| 8 | `AT_TARGET` | `CHOOSE_NEXT` | Continue exploring after recording target |
| 9 | `APPROACH_CENTER` | `DONE` | `navGraph_addNode()` fails (graph full) |
| 10 | `APPROACH_CENTER` | `CHOOSE_NEXT` | Reached center; node created/merged and edge linked |
| 11 | `CHOOSE_NEXT` | `TURN_TO_EDGE` | Unexplored edge chosen (left / straight / right / back) |
| 12 | `CHOOSE_NEXT` | `RETURN_TURN` | All edges explored and back at origin; start Dijkstra return |
| 13 | `TURN_TO_EDGE` | `FOLLOW_LINE` | `rotation_done()` (turn complete) |
| 14 | `RETURN_TURN` | `RETURN_FOLLOW` | `rotation_done()` (facing next node) |
| 15 | `RETURN_FOLLOW` | `RETURN_AT_NODE` | `dist < ORIGIN_TOLERANCE` (reached next node) |
| 16 | `RETURN_AT_NODE` | `RETURN_TURN` | More nodes on current return leg |
| 17 | `RETURN_AT_NODE` | `RETURN_TURN` | Finished leg 1 (origin→target); start leg 2 (target→origin) |
| 18 | `RETURN_AT_NODE` | `DONE` | Finished leg 2 (target→origin); mission complete |
| 19 | `DONE` | `FOLLOW_LINE` | `startButton()` pressed (restart mission) |
| 20 | *Any* | `IDLE` | `stopButton()` pressed (emergency stop) |

**Mermaid State Diagram**:

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> FOLLOW_LINE : startButton()
    FOLLOW_LINE --> VERIFY_TARGET : ground == SENSOR_ALL
    FOLLOW_LINE --> APPROACH_CENTER : (ground & SENSOR_LEFT) == SENSOR_LEFT
    FOLLOW_LINE --> APPROACH_CENTER : corner detected
    FOLLOW_LINE --> APPROACH_CENTER : lost >= LOST_TICKS (deadend)
    VERIFY_TARGET --> AT_TARGET : ground == SENSOR_ALL<br/>AND dist >= TARGET_DIST_THRESHOLD
    VERIFY_TARGET --> APPROACH_CENTER : ground != SENSOR_ALL
    AT_TARGET --> CHOOSE_NEXT : CONTINUE_EXPLORE
    APPROACH_CENTER --> CHOOSE_NEXT : reached center
    APPROACH_CENTER --> DONE : GRAPH_FULL
    CHOOSE_NEXT --> TURN_TO_EDGE : unexplored edge
    CHOOSE_NEXT --> RETURN_TURN : all explored<br/>AND at origin
    TURN_TO_EDGE --> FOLLOW_LINE : rotation_done()
    RETURN_TURN --> RETURN_FOLLOW : rotation_done()
    RETURN_FOLLOW --> RETURN_AT_NODE : dist < ORIGIN_TOLERANCE
    RETURN_AT_NODE --> RETURN_TURN : next node on leg
    RETURN_AT_NODE --> RETURN_TURN : LEG2_START
    RETURN_AT_NODE --> DONE : mission complete
    DONE --> FOLLOW_LINE : startButton()
    DONE --> [*]

    note right of IDLE
        Emergency stop: stopButton()
        transitions ANY state -> IDLE
    end note
```

**ASCII Fallback Diagram**:

```
                               +------------------+
                               |   STATE_IDLE     |
                               | (wait for start) |
                               +--------+---------+
                                        | startButton()
                                        v
+-------------------+        +------------------+
| STATE_TURN_TO_EDGE|<-------| STATE_FOLLOW_LINE|<-----------+
| (turn to edge)    |        | (bang-bang follow)|            |
+--------+----------+        +--------+---------+            |
         | rotation_done()            |                      |
         v                            | SENSOR_ALL            |
+-------------------+                 v                      |
|  STATE_FOLLOW_LINE|        +------------------+             |
|  (resume follow)  |        |STATE_VERIFY_TARGET|            |
+-------------------+        +--------+---------+             |
                                      | pattern breaks        |
                    target confirmed  v                     |
                    +------------------+        +------------------+
                    |  STATE_AT_TARGET |        |STATE_APPROACH_CENTER|
                    | (record, continue)|       | (drive, node+edge) |
                    +--------+---------+        +--------+---------+
                             |                          | reached center
                             v                          v
                    +------------------+        +------------------+
                    | STATE_CHOOSE_NEXT |       | STATE_CHOOSE_NEXT |
                    | (pick next edge)  |       | (pick next edge)  |
                    +--------+---------+        +--------+---------+
                             | unexplored                | all explored
                             v                           v
                    +------------------+        +------------------+
                    | STATE_TURN_TO_EDGE|       | STATE_RETURN_TURN |
                    +--------+---------+        +--------+---------+
                             |                           |
                             v                           v
                    +------------------+        +------------------+
                    | STATE_FOLLOW_LINE |       |STATE_RETURN_FOLLOW|
                    +-------------------+       +--------+---------+
                                                         |
                                                         v
                                                +------------------+
                                                |STATE_RETURN_AT_NODE|
                                                +--------+---------+
                                                         |
                                    +--------------------+--------------------+
                                    |                                         |
                                    v                                         v
                           +------------------+                      +------------------+
                           | STATE_RETURN_TURN |                      |    STATE_DONE    |
                           | (next node/leg2)  |                      | (mission complete)|
                           +-------------------+                      +------------------+
```

## Control Loop Discipline

The main loop in `robot-agent.c` is the only place that calls `waitTick40ms()`.  
**Never** use `delay()`, `wait()`, or busy-waiting for timing inside the real-time loop.

```c
while (1) {
    waitTick40ms();          // Macro: blocks until tick40ms==1, then clears it
    readAnalogSensors();     // MUST call this BEFORE readLineSensors(0)
    unsigned int ground = readLineSensors(0);
    // ... state machine tick ...
}
```

- **Avoid** calling `printf` on every 40 ms tick — it is slow and can destabilize the control loop. Print only on state transitions (infrequent events).
- **No floating-point in ISRs**: PIC32 FPU is slow; keep `double` math in the main loop.
- **No dynamic allocation**: No `malloc`/`free`. All structures are static.

## Sensor Interpretation

The 5-bit ground IR sensor array returns patterns from `readLineSensors(0)` (bit 4 = leftmost, bit 0 = rightmost):

| Pattern | Binary | Meaning |
|---------|--------|---------|
| `SENSOR_NONE` | `0b00000` | Lost line (white floor) |
| `SENSOR_CENTER` | `0b00100` | Centered on line |
| `SENSOR_LEFT` | `0b11000` | Line on left side → drift right |
| `SENSOR_RIGHT` | `0b00011` | Line on right side → drift left |
| `SENSOR_ALL` | `0b11111` | Wide black area (intersection or target) |

**Target vs. Intersection**: The target is confirmed only if `SENSOR_ALL` persists while the robot travels at least `TARGET_DIST_THRESHOLD` (0.06 m). If the pattern breaks before that distance, it was a normal intersection.

**Corner Detection**: Left and right corners are detected when `SENSOR_LEFT` or `SENSOR_RIGHT` persists for `CORNER_TICKS` consecutive ticks (3 ticks ≈ 120 ms).

## LED Indicators

The 4 on-board LEDs indicate the current mission phase:

| LED | State(s) | Meaning |
|-----|----------|---------|
| LED 0 | `IDLE`, `FOLLOW_LINE`, `VERIFY_TARGET`, `APPROACH_CENTER`, `CHOOSE_NEXT`, `TURN_TO_EDGE` | Phase 1: Exploring |
| LED 1 | `AT_TARGET` | Phase 2: At target |
| LED 2 | `RETURN_TURN`, `RETURN_FOLLOW`, `RETURN_AT_NODE` | Phase 3: Returning |
| LED 3 (solid) | `DONE` + success | Mission complete |
| LED 3 (fast blink) | `DONE` + error | Mission failed (lost, graph full, etc.) |

## Navigation Graph

The robot builds a graph during exploration:
- **Nodes**: Intersections, corners, deadends, and the target
- **Edges**: Line segments between nodes, with distance weights
- **Exploration**: Left-hand rule with backtracking when no unexplored edges remain
- **Return**: Dijkstra shortest-path algorithm computes the optimal route from origin to target (leg 1) and target back to origin (leg 2)

Graph data structures are statically allocated (`MAX_NODES = 32`, `MAX_EDGES = 64`).

## Unit Tests

Host-runnable tests for the navigation graph logic are in `tests/`:

```bash
cd tests && make
```

Tests cover node creation/merging, edge linking, Dijkstra pathfinding, target flagging, and edge exploration tracking. The tests use a mock `rm-mr32.h` to compile on the host without PIC32 dependencies.

## References

- [`docs/specs/functional-spec.md`](docs/specs/functional-spec.md) — Full functional requirements (FR-001 through FR-011)
- [`docs/references.md`](docs/references.md) — External resources on PID, odometry, and PIC32 programming
- [`assignment.md`](assignment.md) — Official course assignment description
- [`AGENTS.md`](AGENTS.md) — Project coding guidelines and architecture notes
