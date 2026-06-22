# Functional Specification: TP2 Path Finder Robot

> **Version**: 1.3.0 | **Date**: 2026-06-22 | **Author**: Documenter Agent | **Status**: Implemented

## Change Log

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.3.0 | 2026-06-22 | Documenter | Refactored from 9-state stack-based architecture to 11-state graph-based architecture with Dijkstra shortest-path navigation; replaced `nav_stack` with `nav_graph`; added corner/deadend detection; updated state machine, data structures, functional requirements, constants, and testing strategy |
| 1.2.0 | 2026-06-22 | Documenter | Refactored from monolithic `robot-agent.c` to modular architecture with Makefile build system; updated component table, data structures, references, and build commands |
| 1.1.0 | 2026-06-21 | Documenter | Updated to match actual table-driven state machine implementation; replaced `STATE_DETECT_INTERSECTION` with `STATE_PREPARE_TURN`; logging is now unconditional; removed `TARGET_TICK_THRESHOLD`; updated constants and sensor interpretation |
| 1.0.0 | 2026-06-14 | Documenter | Initial specification based on implemented `robot-agent.c` |

---

## 1. Introduction

### 1.1 Purpose

This document is the single source of truth for the **TP2 Path Finder Robot** — a C-based embedded agent for the DETI MR32 differential-drive robot (PIC32 MCU). It defines the robot's behaviour across three mission phases (exploration, target detection, shortest-path return), the constraints of the embedded platform, and the acceptance criteria used to validate correctness.

This spec serves as:
- A reference for lab debugging (sensor patterns, state transitions, LED codes)
- A source of truth for the Springer LNCS report
- A baseline for any future modifications (e.g., PID tuning, algorithm changes)

### 1.2 Scope

**In Scope**:
- Line-following behaviour on black-line / white-floor topology
- Left-hand-rule exploration at intersections (acyclic tree exploration)
- Corner detection (left and right corners) and deadend detection
- Target area detection (wide black line) vs. normal intersection discrimination
- Graph-based navigation: node creation/merging, edge linking, exploration tracking
- Shortest-path return to start using Dijkstra's algorithm on the built navigation graph
- Two-leg return mission: origin → target (leg 1), then target → origin (leg 2)
- Emergency stop via physical stop button
- Unconditional logging subsystem for mission traceability
- Host-runnable unit tests for pure navigation logic

**Out of Scope (Non-Goals)**:
- Obstacle avoidance (obstacle sensors are not used in this assignment)
- Graph cycle detection or handling (the environment is guaranteed acyclic)
- Multi-robot coordination
- Wireless telemetry or remote control
- Path planning algorithms beyond Dijkstra on the built graph (e.g., A*, dynamic replanning)
- Dynamic memory allocation strategies
- Floating-point computation inside interrupt service routines

### 1.3 Audience

- **Developers**: Implementing or modifying the agent source files
- **Testers / Lab Operators**: Debugging robot behaviour during demo runs
- **Report Authors**: Extracting design decisions and algorithm descriptions for the Springer LNCS report
- **Course Evaluators**: Verifying compliance with assignment requirements

### 1.4 References

- `assignment.md` — Official assignment description (Robotica Movel - TP2)
- `AGENTS.md` — Project structure, build instructions, and coding guidelines
- `src/rm-mr32.h` / `src/rm-mr32.c` — DETI hardware library (READ-ONLY)
- `docs/references.md` — External resources on PID, odometry, and PIC32 programming
- `src/robot-agent.c` — Main entry point: hardware init and control loop dispatch
- `src/config.h` — Compile-time constants, sensor masks, and `Pose` typedef
- `src/state_machine.c` / `src/state_machine.h` — 11-state table-driven FSM and mission logic
- `src/nav_graph.c` / `src/nav_graph.h` — Static navigation graph: nodes, edges, Dijkstra shortest path
- `src/line_follower.c` / `src/line_follower.h` — Bang-bang line following and lost-line recovery
- `src/rotation.c` / `src/rotation.h` — Non-blocking PI rotation controller
- `src/logging.c` / `src/logging.h` — Unconditional serial trace subsystem
- `src/leds.c` / `src/leds.h` — LED phase indicators and error blink patterns
- `tests/test_nav_graph.c` — Host-runnable unit tests for nav_graph logic
- `tests/rm-mr32.h` — Mock hardware header for host compilation
- `tests/Makefile` — Test build runner
- `Makefile` — Build system with per-robot calibration support

---

## 2. System Description

### 2.1 Current State (As-Is)

The MR32 robot platform provides:
- Differential drive with wheel encoders and closed-loop PID velocity control
- 5-bit ground IR sensor array (`readLineSensors(0)`)
- Dead-reckoning odometry (`getRobotPos` / `setRobotPos`)
- 40 ms periodic timer tick (`waitTick40ms()`)
- Physical start/stop buttons and 4 status LEDs

The agent is implemented as a modular C codebase. `robot-agent.c` contains a thin `main()` that initializes hardware and dispatches to the current state's tick handler. Mission logic is split across dedicated modules: an 11-state table-driven finite-state machine (`state_machine`), non-blocking PI rotation (`rotation`), a static navigation graph for node/edge management and Dijkstra pathfinding (`nav_graph`), bang-bang line following (`line_follower`), unconditional serial logging (`logging`), and LED phase indicators (`leds`). All compile-time constants and the `Pose` typedef are centralized in `config.h`. Build is orchestrated by a `Makefile` at the project root. Host-runnable unit tests validate the navigation graph logic independently of hardware.

### 2.2 Target State (To-Be)

The robot shall, upon pressing the start button:
1. **Explore** the entire acyclic black-line tree by following lines, detecting intersections, corners, and deadends, and building a navigation graph (nodes and weighted edges)
2. **Detect** the target area (wide black line) and distinguish it from normal intersections
3. **Continue exploring** after target detection until all edges are explored and the robot returns to the origin
4. **Return** to the target via the shortest path using Dijkstra's algorithm (leg 1)
5. **Return** from the target to the start position via the shortest path using Dijkstra's algorithm (leg 2)
6. **Stop** exactly at the origin `(0, 0)` within a defined tolerance

### 2.3 Project Goals

| Goal | Metric |
|------|--------|
| Complete exploration | All reachable edges visited and marked as explored |
| Target detection | Zero false positives (intersections misclassified as target) and zero false negatives |
| Return path optimality | Shortest path computed by Dijkstra on the built graph |
| Start position accuracy | Stop within `ORIGIN_TOLERANCE` (0.05 m) of `(0, 0)` |
| Real-time safety | 40 ms control loop deadline never missed; emergency stop latency < 40 ms |
| Test coverage | nav_graph unit tests pass on host (node creation, merging, edges, Dijkstra, target flagging) |

---

## 3. Functional Requirements

### Summary Table

| ID | Title | Priority | Status |
|----|-------|----------|--------|
| FR-001 | Hardware Initialization | Must | Implemented |
| FR-002 | 40 ms Tick Discipline | Must | Implemented |
| FR-003 | Sensor Read Order | Must | Implemented |
| FR-004 | Line Following | Must | Implemented |
| FR-005 | Intersection/Corner/Deadend Detection & Graph Building | Must | Implemented |
| FR-006 | Target Detection | Must | Implemented |
| FR-007 | Shortest-Path Return via Dijkstra | Must | Implemented |
| FR-008 | Stop at Start Position | Must | Implemented |
| FR-009 | Static Allocation Only | Must | Implemented |
| FR-010 | Emergency Stop | Must | Implemented |
| FR-011 | Logging Subsystem | Could | Implemented |
| FR-012 | Corner and Deadend Detection | Must | Implemented |
| FR-013 | Graph Node Merging | Must | Implemented |

---

#### FR-001: Hardware Initialization
**Description**: The system shall initialize the PIC32 hardware and enable closed-loop wheel speed control before entering the main control loop.
**Priority**: Must
**Source**: `AGENTS.md` § "Always do"; `rm-mr32.h` API contract
**Dependencies**: None
**Acceptance Criteria**:
- [ ] `initPIC32()` is called exactly once in `main()` before any sensor or actuator use
- [ ] `closedLoopControl(true)` is called immediately after `initPIC32()`
- [ ] `setVel2(0, 0)` is set to ensure motors are stationary at startup
- [ ] `setRobotPos(0.0, 0.0, 0.0)` resets odometry to the origin
**Status**: Implemented

---

#### FR-002: 40 ms Tick Discipline
**Description**: The system shall use `waitTick40ms()` as the sole timing source for the control loop. No `delay()`, `wait()`, or busy-waiting shall be used inside the real-time loop.
**Priority**: Must
**Source**: `AGENTS.md` § "Code Style & Constraints"
**Dependencies**: FR-001
**Acceptance Criteria**:
- [ ] Exactly one `waitTick40ms()` call per control-loop iteration
- [ ] No `delay()` or `wait()` calls inside the main `while(1)` control loop
- [ ] `delay()` is permitted only in idle states (e.g., startup button-poll blink)
- [ ] Control loop frequency is strictly 25 Hz (40 ms period)
**Status**: Implemented

---

#### FR-003: Sensor Read Order
**Description**: The system shall always call `readAnalogSensors()` before `readLineSensors(0)` on every tick.
**Priority**: Must
**Source**: `AGENTS.md` § "Code Style & Constraints"
**Dependencies**: FR-001
**Acceptance Criteria**:
- [ ] `readAnalogSensors()` precedes `readLineSensors(0)` in every control-loop iteration
- [ ] Ground sensor variable (`ground`) is refreshed every tick
**Status**: Implemented

---

#### FR-004: Line Following
**Description**: The system shall follow a black line on a white floor using bang-bang control with moderate wheel speeds.
**Priority**: Must
**Source**: Assignment § "Objective"; `AGENTS.md` § "Example: Line Follower Tick"
**Dependencies**: FR-002, FR-003
**Acceptance Criteria**:
- [ ] Centered pattern (`0b00100`) drives straight at `BASE_SPEED` (40)
- [ ] Left-side detection (`0b11000`) reduces left wheel to `TURN_SPEED` (20), keeps right at `BASE_SPEED`
- [ ] Right-side detection (`0b00011`) reduces right wheel to `TURN_SPEED` (20), keeps left at `BASE_SPEED`
- [ ] Speeds remain within the moderate range [20, 40] for stability
- [ ] Lost line (`0b00000`) triggers recovery behaviour (spin search) rather than immediate stop
**Status**: Implemented

---

#### FR-005: Intersection/Corner/Deadend Detection & Graph Building
**Description**: The system shall detect intersections (`ground == 0b11111`), left corners (`SENSOR_LEFT` persists for `CORNER_TICKS`), right corners (`SENSOR_RIGHT` persists for `CORNER_TICKS`), and deadends (line lost for `LOST_TICKS`). Upon detection, the robot shall drive forward `INTERSECTION_CENTER_DIST` (0.15 m) into the feature, create or merge a graph node at that location, link an undirected weighted edge to the previous node, and then use the left-hand rule to choose the next unexplored direction.
**Priority**: Must
**Source**: Assignment § "Objective"; `AGENTS.md` § "Algorithm Guidance / Phase 1"
**Dependencies**: FR-004
**Acceptance Criteria**:
- [ ] Intersection detected when `ground == SENSOR_ALL` (`0b11111`)
- [ ] Left corner detected when `SENSOR_LEFT` persists for `CORNER_TICKS` (3) consecutive ticks
- [ ] Right corner detected when `SENSOR_RIGHT` persists for `CORNER_TICKS` (3) consecutive ticks
- [ ] Deadend detected when line is lost for `LOST_TICKS` (25) consecutive ticks
- [ ] On detection, robot drives forward `INTERSECTION_CENTER_DIST` (0.15 m) before graph operations
- [ ] Node is created via `navGraph_addNode()` if no existing node is within `NODE_MERGE_TOLERANCE` (0.10 m)
- [ ] Existing node within tolerance is reused via `navGraph_findNodeAt()`
- [ ] An undirected edge is linked between the previous node and the current node with weight = Euclidean distance
- [ ] The edge is marked as explored via `navGraph_setEdgeExplored()`
- [ ] Graph full condition (`MAX_NODES` or `MAX_EDGES` exceeded) transitions to `STATE_DONE` with error
- [ ] Left-hand rule chooses next direction: left, straight, right, back (in that priority)
- [ ] Turn to chosen direction uses non-blocking PI rotation (`rotation_tick()` / `rotation_done()`)
**Status**: Implemented

---

#### FR-006: Target Detection
**Description**: The system shall distinguish the target area (wide black line) from a normal intersection by requiring the `0b11111` pattern to persist while the robot travels at least `TARGET_DIST_THRESHOLD` (0.06 m).
**Priority**: Must
**Source**: Assignment § "Objective"; `AGENTS.md` § "Algorithm Guidance / Phase 2"
**Dependencies**: FR-004, FR-005
**Acceptance Criteria**:
- [ ] Target verification starts when `ground == 0b11111` is first seen
- [ ] Verification state (`STATE_VERIFY_TARGET`) monitors `ground == 0b11111`
- [ ] Distance traveled during verification is computed via `hypot(x - verifyStartX, y - verifyStartY)`
- [ ] Target confirmed only if `distance >= TARGET_DIST_THRESHOLD` (0.06 m)
- [ ] If pattern breaks before threshold, state transitions to `STATE_APPROACH_CENTER` (normal intersection)
- [ ] On confirmation, robot stops and records target pose `(targetX, targetY, targetH)`
- [ ] Target node is flagged in the graph via `navGraph_setNodeTarget()`
- [ ] After target recording, exploration continues (`STATE_CHOOSE_NEXT`)
**Status**: Implemented

---

#### FR-007: Shortest-Path Return via Dijkstra
**Description**: The system shall return from the origin to the target and then from the target back to the origin via the shortest path computed by Dijkstra's algorithm on the built navigation graph. During return, the robot follows the line using `lineFollower_centerOnLine()` while checking Euclidean distance to the next node on the path.
**Priority**: Must
**Source**: Assignment § "Objective"; `AGENTS.md` § "Algorithm Guidance / Phase 3"
**Dependencies**: FR-005, FR-006
**Acceptance Criteria**:
- [ ] Return navigation starts only when all edges are explored and robot is at origin (node 0)
- [ ] Leg 1: `navGraph_dijkstra(0, targetNode)` computes shortest path from origin to target
- [ ] Leg 2: `navGraph_dijkstra(targetNode, 0)` computes shortest path from target to origin
- [ ] Robot turns toward the next node on the path using `atan2(dy, dx)` and non-blocking PI rotation
- [ ] Robot follows the line toward the next node until within `ORIGIN_TOLERANCE` (0.05 m)
- [ ] Upon reaching a node, `s_returnPathIndex` is advanced
- [ ] At the end of leg 1, Dijkstra is recomputed for leg 2 and return continues
- [ ] At the end of leg 2, state transitions to `STATE_DONE`
- [ ] If Dijkstra fails to find a path, transition to `STATE_DONE` with error reason
**Status**: Implemented

---

#### FR-008: Stop at Start Position
**Description**: The system shall stop the robot exactly at the start position `(0, 0)` within a defined tolerance.
**Priority**: Must
**Source**: Assignment § "Objective"
**Dependencies**: FR-007
**Acceptance Criteria**:
- [ ] Final stop condition: `hypot(returnTargetX - x, returnTargetY - y) < ORIGIN_TOLERANCE` (0.05 m)
- [ ] Motors are set to `setVel2(0, 0)` upon reaching start
- [ ] State transitions to `STATE_DONE`
- [ ] If target is at start (origin node is target), transition directly to `STATE_DONE` after exploration
**Status**: Implemented

---

#### FR-009: Static Allocation Only
**Description**: The system shall use only static memory allocation. No `malloc` / `free` shall be used.
**Priority**: Must
**Source**: `AGENTS.md` § "Code Style & Constraints"
**Dependencies**: None
**Acceptance Criteria**:
- [ ] All data structures (graph nodes, edges, state variables, pose records, return path) are declared as `static` global variables or local static arrays
- [ ] No calls to `malloc`, `calloc`, `realloc`, or `free` anywhere in the agent source files
- [ ] Graph bounds are compile-time constants (`MAX_NODES = 32`, `MAX_EDGES = 64`, `MAX_PATH_LENGTH = 32`)
**Status**: Implemented

---

#### FR-010: Emergency Stop
**Description**: The system shall immediately halt the motors when the physical stop button is pressed.
**Priority**: Must
**Source**: `AGENTS.md` § "Hardware API"
**Dependencies**: FR-001
**Acceptance Criteria**:
- [ ] `stopButton()` is polled at the beginning of every control-loop iteration
- [ ] On activation, `setVel2(0, 0)` is executed immediately
- [ ] State transitions to `STATE_IDLE`
- [ ] Stop latency is bounded by one tick period (≤ 40 ms)
- [ ] Robot remains in `STATE_IDLE` until `startButton()` is pressed again
**Status**: Implemented

---

#### FR-011: Logging Subsystem
**Description**: The system shall provide an unconditional logging subsystem that emits structured event messages over the serial port without violating real-time constraints.
**Priority**: Could
**Source**: Implementation design decision
**Dependencies**: FR-002
**Acceptance Criteria**:
- [ ] Logging is always active; no compile-time flag is required
- [ ] Logged events include: state transitions, node additions, edge additions, edge explored, target detection, turn start/done, lost timeout, mission done, ground detection, corner detection, Dijkstra path, exploration done
- [ ] Logging functions: `log_transition()`, `log_target()`, `log_nodeAdd()`, `log_edgeAdd()`, `log_edgeExplored()`, `log_turnStart()`, `log_turnDone()`, `log_lostTimeout()`, `log_groundDetection()`, `log_cornerDetect()`, `log_dijkstraPath()`, `log_explorationDone()`, `log_missionDone()`
- [ ] No `printf` is called inside motor control or sensor read paths (only in state-transition logic)
- [ ] Standard compile command produces a binary with logging enabled: `make`
**Status**: Implemented

---

#### FR-012: Corner and Deadend Detection
**Description**: The system shall detect left corners, right corners, and deadends during line following and treat them as graph nodes with appropriate `NodeType`.
**Priority**: Must
**Source**: `AGENTS.md` § "Architecture Notes / Corner and Deadend Detection"
**Dependencies**: FR-004, FR-005
**Acceptance Criteria**:
- [ ] Left corner: `SENSOR_LEFT` persists for `CORNER_TICKS` (3) consecutive ticks
- [ ] Right corner: `SENSOR_RIGHT` persists for `CORNER_TICKS` (3) consecutive ticks
- [ ] Deadend: line lost for `LOST_TICKS` (25) consecutive ticks (spin search timeout)
- [ ] Each detected feature transitions to `STATE_APPROACH_CENTER` with the correct `NodeType`
- [ ] Corner/deadend nodes are created/merged and linked to the previous node exactly like intersections
**Status**: Implemented

---

#### FR-013: Graph Node Merging
**Description**: The system shall merge newly detected features with existing graph nodes when they are within `NODE_MERGE_TOLERANCE` (0.10 m) to avoid duplicate nodes at the same physical location.
**Priority**: Must
**Source**: Implementation design decision
**Dependencies**: FR-005
**Acceptance Criteria**:
- [ ] `navGraph_findNodeAt(pose, tolerance)` returns the existing node ID if a node is within tolerance
- [ ] If a node is found within tolerance, it is reused instead of creating a new node
- [ ] If no node is found within tolerance, a new node is created
- [ ] Edge linking still occurs between the previous node and the (possibly merged) current node
**Status**: Implemented

---

## 4. Non-Functional Requirements

### Summary Table

| ID | Category | Description | Metric | Target |
|----|----------|-------------|--------|--------|
| NFR-001 | Language | Source language | C standard | C (PIC32 GCC 3.4.4); no C++ |
| NFR-002 | Performance | Floating-point in ISRs | Prohibited | All `float`/`double` math in main loop only |
| NFR-003 | Memory | Dynamic allocation | Prohibited | Zero `malloc`/`free` calls |
| NFR-004 | Timing | Control loop deadline | Period | Exactly 40 ms per iteration |
| NFR-005 | Portability | Logging | Compile-time | Clean compile with standard `make` command |
| NFR-006 | Testing | Unit test coverage | Host-runnable | All `tests/` cases pass with `cd tests && make` |

---

#### NFR-001: Language Constraint
**Description**: The implementation shall be written in C only. C++ features (classes, templates, STL) are prohibited.
**Priority**: Must
**Acceptance Criteria**:
- [ ] Source files compile with `make` (PIC32 GCC 3.4.4) without `-x c++` or similar
- [ ] No `class`, `template`, `namespace`, or C++ standard library headers
**Status**: Implemented

---

#### NFR-002: No Floating-Point in ISRs
**Description**: All floating-point arithmetic shall be confined to the main loop. No `float` or `double` operations shall occur inside interrupt service routines.
**Priority**: Must
**Acceptance Criteria**:
- [ ] `rm-mr32.c` ISRs do not perform floating-point math (verified by library design)
- [ ] `robot-agent.c` does not define or trigger ISRs that use floating-point
- [ ] All trigonometric (`atan2`, `hypot`, `normalizeAngle`) and PI calculations occur in the main loop
**Status**: Implemented

---

#### NFR-003: No Dynamic Allocation
**Description**: The system shall not use dynamic memory allocation at runtime.
**Priority**: Must
**Acceptance Criteria**:
- [ ] No heap allocation functions linked or called
- [ ] Graph size is bounded by `MAX_NODES * sizeof(GraphNode) + MAX_EDGES * sizeof(Edge)` (32 × 40 + 64 × 12 = 2048 bytes)
- [ ] Return path is bounded by `MAX_PATH_LENGTH` (32) bytes
- [ ] Total static RAM usage is deterministic at compile time
**Status**: Implemented

---

#### NFR-004: 40 ms Control Loop Deadline
**Description**: The control loop shall complete all processing within one 40 ms tick period.
**Priority**: Must
**Acceptance Criteria**:
- [ ] Worst-case execution time (WCET) of one loop iteration is < 40 ms
- [ ] `waitTick40ms()` is the first statement in the loop body, ensuring synchronization to the timer
- [ ] No blocking I/O (e.g., unbounded `printf` flush) inside the loop
**Status**: Implemented

---

#### NFR-005: Compile-Time Portability
**Description**: The codebase shall compile cleanly with the standard `make` command.
**Priority**: Should
**Acceptance Criteria**:
- [ ] `make` succeeds from the project root
- [ ] No linker errors or undefined references
- [ ] Code size is acceptable for PIC32 flash memory
**Status**: Implemented

---

#### NFR-006: Unit Test Portability
**Description**: The navigation graph logic shall be testable on the host without hardware dependencies.
**Priority**: Should
**Acceptance Criteria**:
- [ ] `cd tests && make` compiles and runs `test_nav_graph` on the host
- [ ] All test cases pass (node creation, merging, edge linking, Dijkstra simple/branches/max-length, target node, edge explored)
- [ ] Mock `tests/rm-mr32.h` shadows the real hardware header during host compilation
**Status**: Implemented

---

## 5. Architecture Overview

### 5.1 Components

| Component | Files | Purpose | Responsibilities |
|-----------|-------|---------|-----------------|
| **Hardware Abstraction Layer** | `rm-mr32.c` / `rm-mr32.h` | Timer ISRs, ADC, encoder reading, motor PWM, odometry integration |
| **Main Control Loop** | `robot-agent.c` | 40 ms tick synchronization, sensor polling, emergency stop, dispatches to current state's `onTick` |
| **Configuration** | `config.h` | Centralized compile-time constants, sensor bit masks, and shared type definitions (`Pose`) |
| **State Machine** | `state_machine.c` / `state_machine.h` | Table-driven `State` struct with `onEnter`/`onTick`/`onExit` callbacks | 11-state finite-state machine governing mission phases |
| **Line Follower** | `line_follower.c` / `line_follower.h` | Bang-bang control based on 5-bit ground sensor pattern; returns `pLost` flag |
| **Rotation Controller** | `rotation.c` / `rotation.h` | Non-blocking PI heading control using odometry feedback |
| **Navigation Graph** | `nav_graph.c` / `nav_graph.h` | Static graph of nodes and edges; Dijkstra shortest-path computation |
| **LED Indicator** | `leds.c` / `src/leds.h` | Visual feedback of current mission phase; includes blink pattern for error states (`g_doneReason != NULL`) |
| **Logging Subsystem** | `logging.c` / `logging.h` | Unconditional serial trace of mission events |
| **Unit Tests** | `tests/test_nav_graph.c` / `tests/Makefile` | Host-runnable validation of nav_graph logic |

### 5.2 State Machine

The robot behaviour is governed by an 11-state table-driven finite-state machine. Each state is a global `State` instance containing function pointers for `onEnter`, `onTick`, and `onExit`, plus a human-readable name.

```
STATE_IDLE
    │ startButton() pressed
    ▼
STATE_FOLLOW_LINE ◄────────────────────────────────────────────┐
    │ intersection / corner / deadend detected                 │
    ▼                                                          │
STATE_VERIFY_TARGET ──(pattern breaks)──► STATE_APPROACH_CENTER
    │ (target confirmed)          │ drive 150mm, create/merge
    ▼                             │ node, link edge
STATE_AT_TARGET ◄───────────────┘
    │ (record target, continue exploring)
    ▼
STATE_CHOOSE_NEXT
    │ (left-hand rule chooses next direction)
    ▼
STATE_TURN_TO_EDGE
    │ (rotation done)
    └────────────────► STATE_FOLLOW_LINE

Return path (triggered when all edges explored at origin):
    ▼
STATE_RETURN_TURN
    │ (rotation done)
    ▼
STATE_RETURN_FOLLOW ──(reached next node)──► STATE_RETURN_AT_NODE
    │                                          │
    │ (more nodes on path)                     │ (end of leg 1)
    └────────────────◄─────────────────────────┘
                                               │ (run Dijkstra for leg 2)
                                               ▼
                                     STATE_RETURN_TURN (leg 2)
                                               │
                                               │ (end of leg 2)
                                               ▼
                                          STATE_DONE
```

**State Descriptions**:

| State | Phase | Description |
|-------|-------|-------------|
| `STATE_IDLE` | — | Motors off, waiting for `startButton()` |
| `STATE_FOLLOW_LINE` | 1 | Normal bang-bang line following; detects intersections, left/right corners, and deadends |
| `STATE_VERIFY_TARGET` | 2 | Validates whether wide pattern is target (persistent + distance ≥ 0.06 m) |
| `STATE_AT_TARGET` | 2 | Target confirmed; record pose and flag target node in graph; continue exploration |
| `STATE_APPROACH_CENTER` | 1 | Drive forward 0.15 m into the feature, then create/merge graph node and link edge |
| `STATE_CHOOSE_NEXT` | 1 | Use left-hand rule (left, straight, right, back) to choose next unexplored direction |
| `STATE_TURN_TO_EDGE` | 1 | Non-blocking PI rotation toward the chosen edge direction |
| `STATE_RETURN_TURN` | 3 | Turn toward the next node on the Dijkstra-computed return path |
| `STATE_RETURN_FOLLOW` | 3 | Follow line to the next node on the path until within `ORIGIN_TOLERANCE` |
| `STATE_RETURN_AT_NODE` | 3 | Advance path index; at end of leg 1, compute Dijkstra for leg 2; at end of leg 2, mission complete |
| `STATE_DONE` | — | Mission complete; motors off; wait for restart |

**State Transitions**:

Transitions are performed by `changeState(State *newState, const char *detail)`, which:
1. Logs the transition via `log_transition()`
2. Calls `onExit()` of the old state (if defined)
3. Updates `g_currentState` to the new state
4. Calls `onEnter()` of the new state (if defined)
5. Updates the `Phase` enum for LED mapping

### 5.3 Data Flow

```
+---------------+     +-------------------+     +------------------+
|  Hardware     |     |  Main Control Loop |     |  State Machine   |
|  (rm-mr32)    |────►|  (40 ms tick)      |────►|  (g_currentState)|
+---------------+     +-------------------+     +------------------+
       │                                              │
       │ 1. Timer ISR sets tick40ms                   │ 2. Dispatch to state's onTick
       │ 2. ADC / encoder values updated              │ 3. Call lineFollower / rotationTick
       │ 3. Odometry integrated                       │ 4. Update nav_graph
       │                                              │ 5. Update LEDs
       ▼                                              ▼
+---------------+     +-------------------+     +------------------+
|  Sensors      |◄────|  Sensor Read Order |◄────|  Actuators       |
|  (ground,     |     |  analog → line     |     |  (setVel2, leds) |
|   pose)       |     |                    |     |                  |
+---------------+     +-------------------+     +------------------+
```

### 5.4 Data Structures

#### Pose
```c
typedef struct {
    double x;   // X coordinate [m]
    double y;   // Y coordinate [m]
    double h;   // Heading [rad]
} Pose;
```

**Usage**: Defined in `config.h` and used throughout the codebase (state machine, navigation graph, rotation controller, and logging) to represent robot poses and node positions uniformly.

#### State
```c
typedef struct State {
    void (*onEnter)(void);
    void (*onTick)(void);
    void (*onExit)(void);
    const char *name;
} State;
```

**Usage**: Each state is a global instance (`g_stateIdle`, `g_stateFollowLine`, etc.). The state machine dispatches to the current state's callbacks via `g_currentState->onTick()` every 40 ms. State transitions are handled by `changeState()`, which manages the `onExit`/`onEnter` lifecycle.

#### GraphNode
```c
typedef struct {
    Pose pose;
    unsigned char edges[4];
    unsigned char edgeCount;
    unsigned char flags;
    NodeType type;
} GraphNode;
```

**Usage**: Each intersection, corner, or deadend becomes a `GraphNode` in the static array `g_nodes[MAX_NODES]` (32 entries). Nodes store their pose, up to 4 incident edge IDs, a type (`NODE_TYPE_INTERSECTION`, `NODE_TYPE_LEFT_CORNER`, `NODE_TYPE_RIGHT_CORNER`, `NODE_TYPE_DEADEND`), and flags (`NODE_FLAG_IS_TARGET`). During exploration, new nodes are created or merged with existing nodes within `NODE_MERGE_TOLERANCE`.

#### Edge
```c
typedef struct {
    float distance;
    unsigned char nodeA;
    unsigned char nodeB;
    unsigned char flags;
} Edge;
```

**Usage**: Each line segment between two nodes becomes an `Edge` in the static array `g_edges[MAX_EDGES]` (64 entries). Edges store the Euclidean distance between nodes, the two connected node IDs, and flags (`EDGE_FLAG_EXPLORED`). During return navigation, `navGraph_dijkstra()` uses edge distances as weights to compute the shortest path.

#### Global Cached Pose
```c
static double g_robotX = 0.0;
static double g_robotY = 0.0;
static double g_robotH = 0.0;
```

**Usage**: Updated once per tick via `getRobotPos()` in `main()`, then used by state handlers and the rotation controller to avoid redundant odometry queries.

#### Mission Tick Counter
```c
static unsigned int g_missionTick = 0;
```

**Usage**: Incremented once per control-loop iteration. Used by all logging functions to timestamp events.

#### Done Reason
```c
static const char *g_doneReason = NULL;
```

**Usage**: Set to a string when entering `STATE_DONE` due to an error (e.g., `"LOST"`, `"GRAPH_FULL"`, `"LOST_RETURN"`, `"NO_PATH_TO_TARGET"`, `"NO_PATH_HOME"`). `NULL` indicates successful completion. Used by `updateLEDs()` to select the LED blink pattern.

#### Return Path
```c
static unsigned char s_returnPath[MAX_PATH_LENGTH];
static unsigned char s_returnPathLen = 0;
static unsigned char s_returnPathIndex = 0;
static bool s_returnToTarget = true;
```

**Usage**: Stores the Dijkstra-computed shortest path as an array of node IDs. `s_returnPathIndex` tracks progress along the path. `s_returnToTarget` is `true` during leg 1 (origin → target) and `false` during leg 2 (target → origin).

---

## 6. Interfaces

### 6.1 Hardware API

The agent interacts with the MR32 platform through the following API (from `rm-mr32.h`):

| Function | Signature | Purpose | Called By |
|----------|-----------|---------|-----------|
| `initPIC32` | `void initPIC32(void)` | Initialize hardware and timers | `main()` (once) |
| `closedLoopControl` | `void closedLoopControl(bool flag)` | Enable wheel PID | `main()` (once) |
| `setVel2` | `void setVel2(int velLeft, int velRight)` | Set wheel speeds [-100, 100] | State handlers, line follower, rotation |
| `readAnalogSensors` | `void readAnalogSensors(void)` | Update ADC readings | Main loop (every tick) |
| `readLineSensors` | `unsigned int readLineSensors(int gain)` | Return 5-bit ground pattern | Main loop (every tick, after analog) |
| `getRobotPos` | `void getRobotPos(double *xp, double *yp, double *hp)` | Dead reckoning pose | Main loop (cache), state handlers, rotation, return |
| `setRobotPos` | `void setRobotPos(double xx, double yy, double tt)` | Reset odometry | `main()`, `restartMission()` |
| `waitTick40ms` | Macro: `while(!tick40ms); tick40ms = 0;` | Synchronize to 40 ms tick | Main loop (every iteration) |
| `startButton` | Macro: `(!PORTBbits.RB3)` | Read start button | `main()` (idle, done), `restartMission()` |
| `stopButton` | Macro: `(!PORTBbits.RB4)` | Read stop button | Main loop (every tick) |
| `leds` / `led` | `void leds(int state)` / `void led(int ledNr, int value)` | Set status LEDs | `updateLEDs()` |
| `normalizeAngle` | `double normalizeAngle(double angle)` | Wrap to ]-π, +π] | `rotationTick()`, `startRotation()` |

### 6.2 Sensor Interpretation

**Ground Sensor Bit Patterns** (bit 4 = leftmost, bit 0 = rightmost):

| Pattern | Binary | Meaning |
|---------|--------|---------|
| `SENSOR_NONE` | `0b00000` | Lost line (white floor) |
| `SENSOR_CENTER` | `0b00100` | Centered on line |
| `SENSOR_LEFT` | `0b11000` | Line on left side → drift right |
| `SENSOR_RIGHT` | `0b00011` | Line on right side → drift left |
| `SENSOR_ALL` | `0b11111` | Wide black area (intersection or target) |

**Intersection Detection Logic**:
- Intersection or target candidate detected **only** when `ground == SENSOR_ALL` (`0b11111`)
- Left corner: `SENSOR_LEFT` (`0b11000`) persists for `CORNER_TICKS` (3) consecutive ticks
- Right corner: `SENSOR_RIGHT` (`0b00011`) persists for `CORNER_TICKS` (3) consecutive ticks
- Deadend: line lost (`SENSOR_NONE`) for `LOST_TICKS` (25) consecutive ticks
- Target vs. intersection is distinguished via persistence and distance threshold (FR-006)

### 6.3 Motor Commands

**Velocity Constants**:

| Constant | Value | Usage |
|----------|-------|-------|
| `BASE_SPEED` | 40 | Straight-line driving |
| `TURN_SPEED` | 20 (`BASE_SPEED / 2`) | Reduced wheel speed during bang-bang correction and rotation saturation |
| `RECOVERY_SPEED` | 40 (`BASE_SPEED`) | Spin-search speed when line is lost |

**Rotation PI Gains**:

| Gain | Value | Purpose |
|------|-------|---------|
| `KP_ROT` | 40.0 | Proportional heading error correction |
| `KI_ROT` | 5.0 | Integral term to eliminate steady-state error |

**Output Saturation**: `cmdVel` is clamped to `[-TURN_SPEED, +TURN_SPEED]` before applying to motors.

**Motor Mapping for Rotation**:
```c
setVel2(-cmdVel, cmdVel);   // Positive cmdVel = left turn (counter-clockwise)
```

### 6.4 LED Indicators

| LED | State(s) | Meaning |
|-----|----------|---------|
| LED 0 | `STATE_IDLE` through `STATE_TURN_TO_EDGE` | Phase 1: Exploring |
| LED 1 | `STATE_AT_TARGET` | Phase 2: At target |
| LED 2 | `STATE_RETURN_TURN`, `STATE_RETURN_FOLLOW`, `STATE_RETURN_AT_NODE` (leg 2) | Phase 3: Returning to origin |
| LED 2 | `STATE_RETURN_TURN`, `STATE_RETURN_FOLLOW`, `STATE_RETURN_AT_NODE` (leg 1) | Phase 3: Returning to target |
| LED 3 | `STATE_DONE` + `g_doneReason == NULL` | Mission complete (success — solid) |
| LED 3 | `STATE_DONE` + `g_doneReason != NULL` | Mission complete (error — fast blink) |

---

## 7. Testing Strategy

### 7.1 Unit-Level Validation (Host-Runnable)
- `cd tests && make` compiles and runs `test_nav_graph`
- Tests cover: node creation, node merging within tolerance, edge linking (including duplicate prevention), Dijkstra simple path, Dijkstra branch selection, Dijkstra max-length path, target node flagging, edge explored tracking
- Verify no `malloc` / `free` calls exist (`grep -rn "malloc\|free" src/*.c`)
- Verify `waitTick40ms()` is the only timing primitive in the main loop
- Verify `readAnalogSensors()` precedes `readLineSensors(0)` in every path through the loop
- Verify all state instances have their `onTick` handler defined

### 7.2 Simulation / Desktop Testing
- Compile and review serial output for correct state sequence
- Validate graph node/edge creation sequence against expected tree topology
- Check that `rotation_done()` terminates (error < 0.02 rad)
- Validate Dijkstra path output against known graph topologies

### 7.3 Hardware-In-The-Loop Testing

| Test Case | Procedure | Expected Result |
|-----------|-----------|-----------------|
| **TC-001: Straight line follow** | Place robot on straight black line | Robot drives straight, centered on line |
| **TC-002: Left turn at intersection** | Approach T-junction | Robot drives forward 0.15 m, creates node, links edge, turns 90° left, resumes follow |
| **TC-003: Corner detection** | Approach left or right corner | Robot detects corner after 3 ticks, creates corner node, links edge, turns appropriately |
| **TC-004: Target vs. intersection** | Run on map with known target | Target is detected; normal intersections are not misclassified |
| **TC-005: Graph building** | Run full exploration | Graph contains correct number of nodes and edges; all edges marked explored |
| **TC-006: Dijkstra return** | After full exploration | Robot returns to target then origin via shortest path, stopping within 5 cm |
| **TC-007: Emergency stop** | Press stop button during any phase | Motors halt within 40 ms; robot enters `STATE_IDLE` |
| **TC-008: Restart** | Press start button after DONE/IDLE | Mission resets and begins from origin |
| **TC-009: Lost line recovery** | Guide robot off line | Robot spins search; if line not found in 25 ticks (1 s), stops |
| **TC-010: Node merging** | Revisit an intersection | Existing node is reused; no duplicate nodes within 0.10 m |

### 7.4 Demo Acceptance (Assignment Deliverable)
- 3 consecutive successful runs on the physical track
- All phases complete without manual intervention
- Robot stops at start position

---

## 8. Risks & Constraints

### 8.1 Risks

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Odometry drift during return | Robot misses target node or start position | Medium | Use small `ORIGIN_TOLERANCE` (0.05 m); Dijkstra path provides waypoint sequence |
| False target detection | Mission continues but target is misrecorded | Low | Distance-threshold verification (`TARGET_DIST_THRESHOLD = 0.06 m`) |
| Graph full on complex trees | Mission aborts | Low | `MAX_NODES = 32` and `MAX_EDGES = 64` exceed expected tree complexity; safe abort to `STATE_DONE` |
| Wheel slip during turns | Heading error accumulates | Medium | PI rotation with integral term; `normalizeAngle()` keeps error bounded |
| Sensor noise causing lost line | Unnecessary recovery spins | Medium | `LOST_TICKS = 25` (1 s) timeout before giving up |
| Stop button not polled in time | Safety hazard | Low | `stopButton()` is first check after `waitTick40ms()` every tick |
| Corner misclassification | Wrong node type in graph | Low | `CORNER_TICKS = 3` requires persistent detection; node type is only a hint for direction selection |
| Dijkstra path not found | Return aborts | Low | Graph is built from physical exploration; path always exists if target was reached |

### 8.2 Constraints

| Constraint | Description |
|------------|-------------|
| **Platform** | PIC32MX microcontroller, MIPS M4K core, no hardware FPU |
| **Compiler** | PIC32 GCC 3.4.4 (legacy, limited C99 support) |
| **Language** | C only; no C++ |
| **Memory** | Static allocation only; RAM limited to PIC32 onboard memory |
| **Timing** | 40 ms hard real-time loop; no blocking operations |
| **Environment** | Acyclic tree graph (guaranteed by assignment); no cycles to handle |
| **Calibration** | Per-robot servo constants selected by `ROBOT` macro at compile time |

### 8.3 Assumptions

- The black-line graph is a tree (acyclic) as specified in the assignment
- The target area is measurably wider than normal path lines and intersections
- Wheel encoders and ground sensors are functional and calibrated
- The floor contrast (black line vs. white floor) is sufficient for the 5-bit digital sensor
- The robot starts at the origin facing a known direction (heading = 0 rad)

---

## 9. Appendices

### Appendix A: Glossary

| Term | Definition |
|------|------------|
| **Bang-bang control** | On-off control strategy where motor speeds switch between discrete levels based on sensor state |
| **Dead reckoning** | Estimating position by integrating wheel encoder data over time |
| **Dijkstra's algorithm** | Graph search algorithm that finds the shortest path between nodes in a graph with non-negative edge weights |
| **Graph node merging** | Reusing an existing graph node when a newly detected feature is within a spatial tolerance of a previously recorded node |
| **Left-hand rule** | Exploration strategy that prioritizes turning left, then straight, then right, then back at each intersection |
| **Popcount** | Number of set bits in a binary value |
| **Tick** | One 40 ms control loop iteration |

### Appendix B: Constants Reference

| Constant | Value | Unit | Description |
|----------|-------|------|-------------|
| `MAX_NODES` | 32 | entries | Maximum number of graph nodes |
| `MAX_EDGES` | 64 | entries | Maximum number of graph edges |
| `MAX_PATH_LENGTH` | 32 | entries | Maximum Dijkstra path length |
| `NODE_MERGE_TOLERANCE` | 0.10 | m | Distance threshold for reusing an existing graph node |
| `CORNER_TICKS` | 3 | ticks | Consecutive ticks required to confirm a corner detection |
| `TARGET_DIST_THRESHOLD` | 0.06 | m | Minimum distance traveled to confirm target |
| `BASE_SPEED` | 40 | — | Base wheel velocity setpoint |
| `TURN_SPEED` | 20 | — | Reduced wheel velocity for corrections and rotation saturation |
| `RECOVERY_SPEED` | 40 | — | Spin-search wheel velocity |
| `LOST_TICKS` | 25 | ticks | Timeout before giving up on lost line (1 s) |
| `ORIGIN_TOLERANCE` | 0.05 | m | Position tolerance for "reached node"判定 |
| `INTERSECTION_CENTER_DIST` | 0.15 | m | Distance to drive into intersection before graph operations |
| `KP_ROT` | 40.0 | — | Proportional gain for rotation PI |
| `KI_ROT` | 5.0 | — | Integral gain for rotation PI |

### Appendix C: Build Commands

```bash
# Enter development shell
nix develop --impure

# Standard build (default ROBOT=1)
make

# Build with specific robot calibration
make ROBOT=5

# Build and flash to robot
make flash

# Open serial terminal
make term

# Build + flash
make deploy

# Build + flash + terminal
make run

# Remove build artifacts
make clean

# Run host-runnable unit tests
cd tests && make
```

Build artifacts are placed in `build/`:
- `build/robot-agent.hex` — Flashable binary
- `build/robot-agent.elf` — Linked ELF
- `build/*.o` — Object files
- `build/robot-agent.map` — Linker map

### Appendix D: Related Documents

- `assignment.md` — Course assignment description
- `AGENTS.md` — Project structure, build instructions, and coding guidelines
- `docs/references.md` — External resources on PID, odometry, and PIC32 programming
- `src/rm-mr32.h` — Hardware API header
- `src/rm-mr32.c` — Hardware library implementation (READ-ONLY)
- `src/config.h` — Compile-time constants and shared type definitions
- `src/state_machine.h` — FSM state declarations and transition API
- `src/nav_graph.h` — Navigation graph API (nodes, edges, Dijkstra)
- `src/line_follower.h` — Line follower API
- `src/rotation.h` — Rotation controller API
- `src/logging.h` — Logging subsystem API
- `src/leds.h` — LED indicator API
- `tests/test_nav_graph.c` — Host-runnable unit tests
- `tests/rm-mr32.h` — Mock hardware header for host compilation
- `tests/Makefile` — Test build runner
