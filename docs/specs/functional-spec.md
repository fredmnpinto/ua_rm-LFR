# Functional Specification: TP2 Path Finder Robot

> **Version**: 1.1.0 | **Date**: 2026-06-21 | **Author**: Documenter Agent | **Status**: Implemented

## Change Log

| Version | Date | Author | Changes |
|---------|------|--------|---------|
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
- Left-turn priority at intersections (acyclic tree exploration)
- Target area detection (wide black line) vs. normal intersection discrimination
- Shortest-path return to start using parent-pointer DFS stack
- Emergency stop via physical stop button
- Unconditional logging subsystem for mission traceability

**Out of Scope (Non-Goals)**:
- Obstacle avoidance (obstacle sensors are not used in this assignment)
- Cycle detection or graph cycle handling (the environment is guaranteed acyclic)
- Multi-robot coordination
- Wireless telemetry or remote control
- Path planning algorithms beyond tree parent-pointer traversal (e.g., Dijkstra, A*)
- Dynamic memory allocation strategies
- Floating-point computation inside interrupt service routines

### 1.3 Audience

- **Developers**: Implementing or modifying `robot-agent.c`
- **Testers / Lab Operators**: Debugging robot behaviour during demo runs
- **Report Authors**: Extracting design decisions and algorithm descriptions for the Springer LNCS report
- **Course Evaluators**: Verifying compliance with assignment requirements

### 1.4 References

- `assignment.md` — Official assignment description (Robotica Movel - TP2)
- `AGENTS.md` — Project structure, build instructions, and coding guidelines
- `src/rm-mr32.h` / `src/rm-mr32.c` — DETI hardware library (READ-ONLY)
- `docs/references.md` — External resources on PID, odometry, and PIC32 programming
- `src/robot-agent.c` — Implementation file (this spec describes the system as implemented therein)

---

## 2. System Description

### 2.1 Current State (As-Is)

The MR32 robot platform provides:
- Differential drive with wheel encoders and closed-loop PID velocity control
- 5-bit ground IR sensor array (`readLineSensors(0)`)
- Dead-reckoning odometry (`getRobotPos` / `setRobotPos`)
- 40 ms periodic timer tick (`waitTick40ms()`)
- Physical start/stop buttons and 4 status LEDs

The agent (`robot-agent.c`) implements a complete 3-phase mission with a 9-state table-driven finite-state machine, non-blocking PID rotation, and a static pose stack for parent-pointer return navigation.

### 2.2 Target State (To-Be)

The robot shall, upon pressing the start button:
1. **Explore** the entire acyclic black-line tree by following lines and always turning left at intersections
2. **Detect** the target area (wide black line) and distinguish it from normal intersections
3. **Return** to the start position via the unique shortest path (parent-pointer DFS)
4. **Stop** exactly at the origin `(0, 0)` within a defined tolerance

### 2.3 Project Goals

| Goal | Metric |
|------|--------|
| Complete exploration | All reachable edges visited exactly once (guaranteed by left-turn priority on acyclic tree) |
| Target detection | Zero false positives (intersections misclassified as target) and zero false negatives |
| Return path optimality | Shortest path (unique simple path in a tree) |
| Start position accuracy | Stop within `ORIGIN_TOLERANCE` (0.05 m) of `(0, 0)` |
| Real-time safety | 40 ms control loop deadline never missed; emergency stop latency < 40 ms |

---

## 3. Functional Requirements

### Summary Table

| ID | Title | Priority | Status |
|----|-------|----------|--------|
| FR-001 | Hardware Initialization | Must | Implemented |
| FR-002 | 40 ms Tick Discipline | Must | Implemented |
| FR-003 | Sensor Read Order | Must | Implemented |
| FR-004 | Line Following | Must | Implemented |
| FR-005 | Intersection Detection & Left Turn | Must | Implemented |
| FR-006 | Target Detection | Must | Implemented |
| FR-007 | Shortest-Path Return | Must | Implemented |
| FR-008 | Stop at Start Position | Must | Implemented |
| FR-009 | Static Allocation Only | Must | Implemented |
| FR-010 | Emergency Stop | Must | Implemented |
| FR-011 | Logging Subsystem | Could | Implemented |

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

#### FR-005: Intersection Detection & Left Turn
**Description**: The system shall detect intersections (`ground == 0b11111`), push the current pose onto the navigation stack, drive forward into the intersection, execute a non-blocking 90° left turn, and resume line following. If a path to the left is detected (`(ground & SENSOR_LEFT) == SENSOR_LEFT`), the system shall skip target verification and proceed directly to turn preparation.
**Priority**: Must
**Source**: Assignment § "Objective"; `AGENTS.md` § "Algorithm Guidance / Phase 1"
**Dependencies**: FR-004
**Acceptance Criteria**:
- [ ] Intersection detected when `ground == SENSOR_ALL` (`0b11111`)
- [ ] `pathToTheLeft` detected when `(ground & SENSOR_LEFT) == SENSOR_LEFT` triggers direct transition to `STATE_PREPARE_TURN`
- [ ] On entering `STATE_PREPARE_TURN`, current pose `(x, y, h)` is pushed onto the static stack (max depth 32)
- [ ] Stack overflow is handled safely (transition to `STATE_DONE` with error)
- [ ] Robot drives forward `INTERSECTION_CENTER_DIST` (150 units) into the intersection before turning
- [ ] Left turn is exactly +90° (`+PI/2.0` rad) relative to current heading
- [ ] Turn uses non-blocking PID rotation (`rotationTick()` / `rotationDone()`)
- [ ] After turn completion, state transitions to `STATE_FOLLOW_LINE`
**Status**: Implemented

---

#### FR-006: Target Detection
**Description**: The system shall distinguish the target area (wide black line) from a normal intersection by requiring the `0b11111` pattern to persist while the robot travels at least `TARGET_DIST_THRESHOLD` (6 units).
**Priority**: Must
**Source**: Assignment § "Objective"; `AGENTS.md` § "Algorithm Guidance / Phase 2"
**Dependencies**: FR-004, FR-005
**Acceptance Criteria**:
- [ ] Target verification starts when `ground == 0b11111` is first seen
- [ ] Verification state (`STATE_VERIFY_TARGET`) monitors `ground == 0b11111`
- [ ] Distance traveled during verification is computed via `hypot(x - verifyStartX, y - verifyStartY)`
- [ ] Target confirmed only if `distance >= TARGET_DIST_THRESHOLD` (6)
- [ ] If pattern breaks before threshold, state transitions to `STATE_PREPARE_TURN` (normal intersection)
- [ ] On confirmation, robot stops and records target pose `(targetX, targetY, targetH)`
**Status**: Implemented

---

#### FR-007: Shortest-Path Return
**Description**: The system shall return from the target to the start position via the shortest path using a parent-pointer DFS stack recorded during Phase 1 exploration. During return, the robot follows the line using `centerRobotOnLine()` while checking distance to the parent node.
**Priority**: Must
**Source**: Assignment § "Objective"; `AGENTS.md` § "Algorithm Guidance / Phase 3"
**Dependencies**: FR-005, FR-006
**Acceptance Criteria**:
- [ ] Return begins by popping the top pose from the stack
- [ ] Robot turns toward the popped parent pose using `atan2(dy, dx)` and non-blocking PID rotation
- [ ] Robot follows the line toward the parent pose using `centerRobotOnLine()` until within `ORIGIN_TOLERANCE` (0.05 m)
- [ ] Upon reaching a parent node, if the stack is empty, the robot is at the start
- [ ] If the stack is not empty, the next parent is popped and the process repeats
- [ ] The path followed is the unique simple path (shortest path in a tree)
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
- [ ] If target is at start (stack empty immediately at `STATE_AT_TARGET`), transition directly to `STATE_DONE`
**Status**: Implemented

---

#### FR-009: Static Allocation Only
**Description**: The system shall use only static memory allocation. No `malloc` / `free` shall be used.
**Priority**: Must
**Source**: `AGENTS.md` § "Code Style & Constraints"
**Dependencies**: None
**Acceptance Criteria**:
- [ ] All data structures (stack, state variables, pose records) are declared as `static` global variables or local static arrays
- [ ] No calls to `malloc`, `calloc`, `realloc`, or `free` anywhere in `robot-agent.c`
- [ ] Stack maximum depth is a compile-time constant (`MAX_STACK_DEPTH = 32`)
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
- [ ] Logged events include: state transitions, stack push/pop, target detection, turn start/done, lost timeout, mission done, ground detection
- [ ] Logging functions: `logTransition()`, `logTarget()`, `logPush()`, `logPop()`, `logTurnStart()`, `logTurnDone()`, `logLostTimeout()`, `logGroundDetection()`, `logMissionDone()`
- [ ] No `printf` is called inside motor control or sensor read paths (only in state-transition logic)
- [ ] Standard compile command produces a binary with logging enabled: `pcompile robot-agent.c rm-mr32.c`
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
| NFR-005 | Portability | Logging | Compile-time | Clean compile with standard `pcompile` command |

---

#### NFR-001: Language Constraint
**Description**: The implementation shall be written in C only. C++ features (classes, templates, STL) are prohibited.
**Priority**: Must
**Acceptance Criteria**:
- [ ] Source file compiles with `pcompile` (PIC32 GCC 3.4.4) without `-x c++` or similar
- [ ] No `class`, `template`, `namespace`, or C++ standard library headers
**Status**: Implemented

---

#### NFR-002: No Floating-Point in ISRs
**Description**: All floating-point arithmetic shall be confined to the main loop. No `float` or `double` operations shall occur inside interrupt service routines.
**Priority**: Must
**Acceptance Criteria**:
- [ ] `rm-mr32.c` ISRs do not perform floating-point math (verified by library design)
- [ ] `robot-agent.c` does not define or trigger ISRs that use floating-point
- [ ] All trigonometric (`atan2`, `hypot`, `normalizeAngle`) and PID calculations occur in the main loop
**Status**: Implemented

---

#### NFR-003: No Dynamic Allocation
**Description**: The system shall not use dynamic memory allocation at runtime.
**Priority**: Must
**Acceptance Criteria**:
- [ ] No heap allocation functions linked or called
- [ ] Stack size is bounded by `MAX_STACK_DEPTH * sizeof(StackEntry)` (32 × 24 bytes = 768 bytes)
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
**Description**: The codebase shall compile cleanly with the standard `pcompile` command.
**Priority**: Should
**Acceptance Criteria**:
- [ ] `pcompile robot-agent.c rm-mr32.c` succeeds
- [ ] No linker errors or undefined references
- [ ] Code size is acceptable for PIC32 flash memory
**Status**: Implemented

---

## 5. Architecture Overview

### 5.1 Components

| Component | Purpose | Responsibilities |
|-----------|---------|-----------------|
| **Hardware Abstraction Layer** | `rm-mr32.c` / `rm-mr32.h` | Timer ISRs, ADC, encoder reading, motor PWM, odometry integration |
| **Main Control Loop** | `main()` in `robot-agent.c` | 40 ms tick synchronization, sensor polling, emergency stop, dispatches to current state's `onTick` |
| **Line Follower** | `centerRobotOnLine()` | Bang-bang control based on 5-bit ground sensor pattern; returns `pLost` flag |
| **Rotation Controller** | `startRotation()`, `rotationTick()`, `rotationDone()` | Non-blocking PID heading control using odometry feedback |
| **Navigation Stack** | `pushPose()`, `popPose()`, `isStackEmpty()` | Static LIFO stack of poses for parent-pointer DFS return path |
| **State Machine** | Table-driven `State` struct with `onEnter`/`onTick`/`onExit` callbacks | 9-state finite-state machine governing mission phases |
| **LED Indicator** | `updateLEDs()` | Visual feedback of current mission phase; includes blink pattern for error states (`g_doneReason != NULL`) |
| **Logging Subsystem** | `logTransition()`, `logTarget()`, etc. | Unconditional serial trace of mission events |

### 5.2 State Machine

The robot behaviour is governed by a 9-state table-driven finite-state machine. Each state is a global `State` instance containing function pointers for `onEnter`, `onTick`, and `onExit`, plus a human-readable name.

```
STATE_IDLE
    │ startButton() pressed
    ▼
STATE_FOLLOW_LINE ◄────────────────────────────────────┐
    │ intersection detected                              │
    ▼                                                    │
STATE_VERIFY_TARGET ──(pattern breaks)──► STATE_PREPARE_TURN
    │ (target confirmed)          │ push pose, drive forward 150
    ▼                             ▼
STATE_AT_TARGET ◄─────────── STATE_TURN_LEFT
    │ (turn done)                    │
    │ startReturnToParent()          │
    ▼                                │
STATE_RETURN_TURN ◄─────────────────┘
    │ rotationDone()
    ▼
STATE_RETURN_FOLLOW ──(reached parent & stack empty)──► STATE_DONE
    │ (reached parent & stack not empty)
    └────────────────► STATE_RETURN_TURN (next parent)
```

**State Descriptions**:

| State | Phase | Description |
|-------|-------|-------------|
| `STATE_IDLE` | — | Motors off, waiting for `startButton()` |
| `STATE_FOLLOW_LINE` | 1 | Normal bang-bang line following; detects intersections and left-path shortcuts |
| `STATE_VERIFY_TARGET` | 2 | Validates whether wide pattern is target (persistent + distance) |
| `STATE_PREPARE_TURN` | 1 | Push pose onto stack, drive forward 150 units into intersection, then turn left |
| `STATE_TURN_LEFT` | 1 | Non-blocking 90° left turn (+π/2 rad) |
| `STATE_AT_TARGET` | 2 | Target confirmed; record pose; initiate return |
| `STATE_RETURN_TURN` | 3 | Turn toward parent node on the stack |
| `STATE_RETURN_FOLLOW` | 3 | Follow line to parent pose until within tolerance |
| `STATE_DONE` | — | Mission complete; motors off; wait for restart |

**State Transitions**:

Transitions are performed by `changeState(State *newState, const char *detail)`, which:
1. Logs the transition via `logTransition()`
2. Calls `onExit()` of the old state (if defined)
3. Updates `g_currentState` to the new state
4. Calls `onEnter()` of the new state (if defined)

### 5.3 Data Flow

```
+---------------+     +-------------------+     +------------------+
|  Hardware     |     |  Main Control Loop |     |  State Machine   |
|  (rm-mr32)    |────►|  (40 ms tick)      |────►|  (g_currentState)|
+---------------+     +-------------------+     +------------------+
       │                                              │
       │ 1. Timer ISR sets tick40ms                   │ 2. Dispatch to state's onTick
       │ 2. ADC / encoder values updated              │ 3. Call centerRobotOnLine / rotationTick
       │ 3. Odometry integrated                       │ 4. Update LEDs
       │                                              │ 5. Log events
       ▼                                              ▼
+---------------+     +-------------------+     +------------------+
|  Sensors      |◄────|  Sensor Read Order |◄────|  Actuators       |
|  (ground,     |     |  analog → line     |     |  (setVel2, leds) |
|   pose)       |     |                    |     |                  |
+---------------+     +-------------------+     +------------------+
```

### 5.4 Data Structures

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

#### StackEntry
```c
typedef struct {
    double x;   // X coordinate [m]
    double y;   // Y coordinate [m]
    double h;   // Heading [rad]
} StackEntry;
```

**Usage**: Each intersection pose is pushed onto the static array `g_stack[MAX_STACK_DEPTH]` (32 entries). During return, poses are popped LIFO to follow parent pointers back to the start.

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

**Usage**: Set to a string when entering `STATE_DONE` due to an error (e.g., `"LOST"`, `"STACK_OVERFLOW"`, `"LOST_RETURN"`). `NULL` indicates successful completion. Used by `updateLEDs()` to select the LED blink pattern.

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
| `startButton` | Macro: `(!PORTBbits.RB3)` | Read start button | `main()` (idle, done), `statePrepareTurn_onEnter()` |
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
- `pathToTheLeft` shortcut: if `(ground & SENSOR_LEFT) == SENSOR_LEFT`, the system skips target verification and transitions directly to `STATE_PREPARE_TURN`
- Target vs. intersection is distinguished via persistence and distance threshold (FR-006)

### 6.3 Motor Commands

**Velocity Constants**:

| Constant | Value | Usage |
|----------|-------|-------|
| `BASE_SPEED` | 40 | Straight-line driving |
| `TURN_SPEED` | 20 (`BASE_SPEED / 2`) | Reduced wheel speed during bang-bang correction and rotation saturation |
| `RECOVERY_SPEED` | 40 (`BASE_SPEED`) | Spin-search speed when line is lost |

**Rotation PID Gains**:

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
| LED 0 | `STATE_IDLE` through `STATE_VERIFY_TARGET` | Phase 1: Exploring |
| LED 1 | `STATE_AT_TARGET` | Phase 2: At target |
| LED 2 | `STATE_RETURN_TURN`, `STATE_RETURN_FOLLOW` | Phase 3: Returning |
| LED 3 | `STATE_DONE` + `g_doneReason == NULL` | Mission complete (success — solid) |
| LED 3 | `STATE_DONE` + `g_doneReason != NULL` | Mission complete (error — fast blink) |

---

## 7. Testing Strategy

### 7.1 Unit-Level Validation (Code Review / Static Analysis)
- Verify no `malloc` / `free` calls exist (`grep -n "malloc\|free" src/robot-agent.c`)
- Verify `waitTick40ms()` is the only timing primitive in the main loop
- Verify `readAnalogSensors()` precedes `readLineSensors(0)` in every path through the loop
- Verify all state instances have their `onTick` handler defined

### 7.2 Simulation / Desktop Testing
- Compile and review serial output for correct state sequence
- Validate stack push/pop sequence against expected tree topology
- Check that `rotationDone()` terminates (error < 0.02 rad)

### 7.3 Hardware-In-The-Loop Testing

| Test Case | Procedure | Expected Result |
|-----------|-----------|-----------------|
| **TC-001: Straight line follow** | Place robot on straight black line | Robot drives straight, centered on line |
| **TC-002: Left turn at intersection** | Approach T-junction | Robot pushes pose, drives forward 150 units, turns 90° left, resumes follow |
| **TC-003: Target vs. intersection** | Run on map with known target | Target is detected; normal intersections are not misclassified |
| **TC-004: Return path** | After target detection | Robot returns to start via shortest path, stopping within 5 cm |
| **TC-005: Emergency stop** | Press stop button during any phase | Motors halt within 40 ms; robot enters `STATE_IDLE` |
| **TC-006: Restart** | Press start button after DONE/IDLE | Mission resets and begins from origin |
| **TC-007: Lost line recovery** | Guide robot off line | Robot spins search; if line not found in 25 ticks (1 s), stops |

### 7.4 Demo Acceptance (Assignment Deliverable)
- 3 consecutive successful runs on the physical track
- All phases complete without manual intervention
- Robot stops at start position

---

## 8. Risks & Constraints

### 8.1 Risks

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Odometry drift during return | Robot misses parent node or start position | Medium | Use small `ORIGIN_TOLERANCE` (0.05 m); parent-pointer strategy minimizes cumulative drift vs. pure odometry |
| False target detection | Mission ends prematurely | Low | Distance-threshold verification (`TARGET_DIST_THRESHOLD = 6`) |
| Stack overflow on deep trees | Mission aborts | Low | `MAX_STACK_DEPTH = 32` exceeds expected tree depth; safe abort to `STATE_DONE` |
| Wheel slip during turns | Heading error accumulates | Medium | PID rotation with integral term; `normalizeAngle()` keeps error bounded |
| Sensor noise causing lost line | Unnecessary recovery spins | Medium | `LOST_TICKS = 25` (1 s) timeout before giving up |
| Stop button not polled in time | Safety hazard | Low | `stopButton()` is first check after `waitTick40ms()` every tick |

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
| **Parent-pointer DFS** | Depth-first search where each node stores a pointer to its parent, enabling backtracking without revisiting |
| **Popcount** | Number of set bits in a binary value |
| **Tick** | One 40 ms control loop iteration |

### Appendix B: Constants Reference

| Constant | Value | Unit | Description |
|----------|-------|------|-------------|
| `MAX_STACK_DEPTH` | 32 | entries | Maximum navigation stack depth |
| `TARGET_DIST_THRESHOLD` | 6 | uncertain (possibly mm) | Minimum distance traveled to confirm target |
| `BASE_SPEED` | 40 | — | Base wheel velocity setpoint |
| `TURN_SPEED` | 20 | — | Reduced wheel velocity for corrections and rotation saturation |
| `RECOVERY_SPEED` | 40 | — | Spin-search wheel velocity |
| `LOST_TICKS` | 25 | ticks | Timeout before giving up on lost line (1 s) |
| `ORIGIN_TOLERANCE` | 0.05 | m | Position tolerance for "reached node"判定 |
| `INTERSECTION_CENTER_DIST` | 150 | units | Distance to drive into intersection before turning |
| `KP_ROT` | 40.0 | — | Proportional gain for rotation PID |
| `KI_ROT` | 5.0 | — | Integral gain for rotation PID |

### Appendix C: Build Commands

```bash
# Enter development shell
nix develop --impure

cd src

# Standard build
pcompile robot-agent.c rm-mr32.c

# Build with specific robot calibration
pcompile -DROBOT=5 robot-agent.c rm-mr32.c

# Flash to robot
ldpic32 -w robot-agent.hex

# Open serial terminal
pterm
```

### Appendix D: Related Documents

- `assignment.md` — Course assignment description
- `AGENTS.md` — Project structure, build instructions, and coding guidelines
- `docs/references.md` — External resources on PID, odometry, and PIC32 programming
- `src/rm-mr32.h` — Hardware API header
- `src/rm-mr32.c` — Hardware library implementation (READ-ONLY)
