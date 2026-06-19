/**
 * \file robot-agent.c
 * \brief TP2 Path Finder Robot — Left-turn exploration with parent-pointer
 * return
 *
 * Phase 1: Follow black lines, turn left at intersections (explores acyclic
 * tree) Phase 2: Detect wide black line target area, stop, record pose Phase 3:
 * Return to start via shortest path (parent-pointer DFS stack)
 *
 * Compile with logging: pcompile -DENABLE_LOGGING robot-agent.c rm-mr32.c
 */

#include "rm-mr32.h"
#include <math.h>

/* ========================================================================
 *   CONSTANTS
 * ======================================================================== */

#define MAX_STACK_DEPTH 32
#define TARGET_TICK_THRESHOLD 20
#define TARGET_DIST_THRESHOLD 6
#define BASE_SPEED 40
#define TURN_SPEED 30
#define RECOVERY_SPEED 40
#define LOST_TICKS 25
#define ORIGIN_TOLERANCE 0.05

#define KP_ROT 40.0
#define KI_ROT 5.0

/* Sensor bit patterns (bit4=left, bit0=right) */
#define SENSOR_NONE 0b00000 // 0b00000
#define SENSOR_CENTER 0b00100
#define SENSOR_LEFT 0b11000
#define SENSOR_RIGHT 0b00011
#define SENSOR_ALL 0b11111

/* ========================================================================
 *   STATE MACHINE
 * ======================================================================== */

typedef enum {
  STATE_IDLE,
  STATE_FOLLOW_LINE,
  STATE_DETECT_INTERSECTION,
  STATE_TURN_LEFT,
  STATE_VERIFY_TARGET,
  STATE_AT_TARGET,
  STATE_RETURN_TURN,
  STATE_RETURN_FOLLOW,
  STATE_DONE
} RobotState;

typedef struct State {
  void (*onEnter)(void);
  void (*onTick)(void);
  void (*onExit)(void);
  const char *name;
} State;

/* ========================================================================
 *   DATA STRUCTURES
 * ======================================================================== */

typedef struct {
  double x;
  double y;
  double h;
} StackEntry;

/* ========================================================================
 *   FORWARD DECLARATIONS OF STATE INSTANCES
 * ======================================================================== */

extern State g_stateIdle;
extern State g_stateFollowLine;
extern State g_stateDetectIntersection;
extern State g_stateTurnLeft;
extern State g_stateVerifyTarget;
extern State g_stateAtTarget;
extern State g_stateReturnTurn;
extern State g_stateReturnFollow;
extern State g_stateDone;

/* ========================================================================
 *   GLOBAL VARIABLES (all statically allocated)
 * ======================================================================== */

static StackEntry g_stack[MAX_STACK_DEPTH];
static int g_stack_top = -1;

static State *g_currentState = &g_stateIdle;

static unsigned int g_ground = 0;
static char g_logDetail[32];

/* Target pose */
static double g_targetX = 0.0;
static double g_targetY = 0.0;
static double g_targetH = 0.0;

/* Tick counters and flags */
static int g_lostTickCount = 0;

/* Target verification start pose */
static double g_verifyStartX = 0.0;
static double g_verifyStartY = 0.0;

/* Non-blocking rotation state */
static double g_rotTargetAngle = 0.0;
static double g_rotIntegral = 0.0;
static bool g_rotActive = false;

/* Return navigation target */
static double g_returnTargetX = 0.0;
static double g_returnTargetY = 0.0;

/* Mission tick counter */
static unsigned int g_missionTick = 0;
static bool g_logThisTick = false;

/* ========================================================================
 *   FORWARD DECLARATIONS OF HELPER FUNCTIONS
 * ======================================================================== */

static void restartMission(void);
static void startReturnToParent(void);

/* ========================================================================
 *   LOGGING SUBSYSTEM
 * ======================================================================== */

#define LOG_TICK()                                                             \
  do {                                                                         \
    g_missionTick++;                                                           \
    g_logThisTick = false;                                                     \
  } while (0)

static void formatGroundHex(unsigned int ground, char *out) {
  const char *hex = "0123456789ABCDEF";
  out[0] = 'g';
  out[1] = 'r';
  out[2] = 'o';
  out[3] = 'u';
  out[4] = 'n';
  out[5] = 'd';
  out[6] = '=';
  out[7] = '0';
  out[8] = 'x';
  out[9] = hex[(ground >> 4) & 0xF];
  out[10] = hex[ground & 0xF];
  out[11] = '\0';
}

static void logTransition(State *to, const char *detail) {
  if (g_logThisTick)
    return;
  const char *fromName = g_currentState ? g_currentState->name : "NULL";
  const char *toName = to ? to->name : "NULL";
  printf("[T %u] %s -> %s%s%s\n", g_missionTick, fromName, toName,
         detail ? " " : "", detail ? detail : "");
  g_logThisTick = true;
}

static void logTarget(double x, double y, double h) {
  if (g_logThisTick)
    return;
  printf("[T %u] TARGET x=%.3f y=%.3f h=%.2f\n", g_missionTick, x, y, h);
  g_logThisTick = true;
}

static void logPush(int depth, double x, double y, double h) {
  if (g_logThisTick)
    return;
  printf("[T %u] PUSH depth=%d x=%.3f y=%.3f h=%.2f\n", g_missionTick, depth, x,
         y, h);
  g_logThisTick = true;
}

static void logPop(int depth) {
  if (g_logThisTick)
    return;
  printf("[T %u] POP depth=%d\n", g_missionTick, depth);
  g_logThisTick = true;
}

static void logTurnStart(double targetAngle) {
  if (g_logThisTick)
    return;
  printf("[T %u] TURN_LEFT target=%.2f\n", g_missionTick, targetAngle);
  g_logThisTick = true;
}

static void logTurnDone(void) {
  if (g_logThisTick)
    return;
  printf("[T %u] TURN_DONE\n", g_missionTick);
  g_logThisTick = true;
}

static void logLostTimeout(void) {
  if (g_logThisTick)
    return;
  printf("[T %u] LOST_TIMEOUT\n", g_missionTick);
  g_logThisTick = true;
}

static void logMissionDone(double tx, double ty, double th, int depth,
                           int ticks) {
  if (g_logThisTick)
    return;
  printf("[T %u] DONE target=(%.2f,%.2f,%.2f) depth=%d ticks=%d\n",
         g_missionTick, tx, ty, th, depth, ticks);
  g_logThisTick = true;
}

/* ========================================================================
 *   STACK OPERATIONS
 * ======================================================================== */

static bool pushPose(double x, double y, double h) {
  if (g_stack_top >= MAX_STACK_DEPTH - 1) {
    return false; /* Stack overflow */
  }
  g_stack_top++;
  g_stack[g_stack_top].x = x;
  g_stack[g_stack_top].y = y;
  g_stack[g_stack_top].h = h;
  logPush(g_stack_top + 1, x, y, h);
  return true;
}

static bool popPose(double *x, double *y, double *h) {
  if (g_stack_top < 0) {
    return false; /* Stack underflow */
  }
  *x = g_stack[g_stack_top].x;
  *y = g_stack[g_stack_top].y;
  *h = g_stack[g_stack_top].h;
  g_stack_top--;
  return true;
}

static bool isStackEmpty(void) { return (g_stack_top < 0); }

/* ========================================================================
 *   BANG-BANG LINE FOLLOWER
 * ======================================================================== */

/**
 * \brief Execute one tick of bang-bang line following.
 * \param ground 5-bit sensor pattern from readLineSensors(0)
 * \param pIntersection output: set to true if intersection/target detected
 * \param pLost output: set to true if line is lost
 */
static void lineFollowTick(unsigned int ground, bool *pIntersection,
                           bool *pLost) {
  *pIntersection = false;
  *pLost = false;

  if (ground == SENSOR_NONE) {
    /* Lost line — signal to caller, do not set motors here */
    *pLost = true;
    return;
  }

  /* Count bits set (popcount) for wide-pattern detection */
  int popcount = 0;
  unsigned int tmp = ground;
  while (tmp) {
    popcount++;
    tmp &= tmp - 1;
  }

  if (ground == SENSOR_ALL) {
    /* Intersection or target area */
    *pIntersection = true;
    /* Keep moving slowly while deciding */
    setVel2(BASE_SPEED, BASE_SPEED);
    return;
  }

  /* Normal line following patterns */
  if (ground == SENSOR_CENTER) {
    /* Centered on line */
    setVel2(BASE_SPEED, BASE_SPEED);
  } else if ((ground & SENSOR_LEFT) != 0) {
    /* Line on left → turn left to recenter */
    setVel2(TURN_SPEED, BASE_SPEED);
  } else if ((ground & SENSOR_RIGHT) != 0) {
    /* Line on right → turn right to recenter */
    setVel2(BASE_SPEED, TURN_SPEED);
  } else {
    /* Fallback: any other pattern, drive straight slowly */
    setVel2(BASE_SPEED, BASE_SPEED);
  }
}

/* ========================================================================
 *   NON-BLOCKING PID ROTATION
 * ======================================================================== */

static void startRotation(double deltaAngle) {
  double x, y, h;
  getRobotPos(&x, &y, &h);
  g_rotTargetAngle = normalizeAngle(h + deltaAngle);
  g_rotIntegral = 0.0;
  g_rotActive = true;
  logTurnStart(g_rotTargetAngle);
}

static void rotationTick(void) {
  double x, y, h;
  double error;
  int cmdVel;

  if (!g_rotActive) {
    return;
  }

  getRobotPos(&x, &y, &h);
  error = normalizeAngle(g_rotTargetAngle - h);

  g_rotIntegral += error;
  if (g_rotIntegral > PI / 2.0) {
    g_rotIntegral = PI / 2.0;
  } else if (g_rotIntegral < -PI / 2.0) {
    g_rotIntegral = -PI / 2.0;
  }

  cmdVel = (int)((KP_ROT * error) + (KI_ROT * g_rotIntegral));

  if (cmdVel > TURN_SPEED) {
    cmdVel = TURN_SPEED;
  } else if (cmdVel < -TURN_SPEED) {
    cmdVel = -TURN_SPEED;
  }

  setVel2(-cmdVel, cmdVel);
}

static bool rotationDone(void) {
  double x, y, h;
  double error;

  if (!g_rotActive) {
    return true;
  }

  getRobotPos(&x, &y, &h);
  error = normalizeAngle(g_rotTargetAngle - h);

  if (fabs(error) < 0.02) {
    g_rotActive = false;
    setVel2(0, 0);
    logTurnDone();
    return true;
  }
  return false;
}

/* ========================================================================
 *   changeState() HELPER
 * ======================================================================== */

static void changeState(State *newState, const char *detail) {
  logTransition(newState, detail);
  if (g_currentState && g_currentState->onExit) {
    g_currentState->onExit();
  }
  g_currentState = newState;
  if (g_currentState && g_currentState->onEnter) {
    g_currentState->onEnter();
  }
}

/* ========================================================================
 *   STATE HANDLER FUNCTIONS
 * ======================================================================== */

/* -------------------------------------------------- */
/* STATE_IDLE                                         */
/* -------------------------------------------------- */
static void stateIdle_onTick(void) {
  setVel2(0, 0);
  if (startButton()) {
    restartMission();
    pushPose(0.0, 0.0, 0.0); /* Cannot overflow — stack just reset */
  }
}

/* -------------------------------------------------- */
/* STATE_FOLLOW_LINE                                  */
/* -------------------------------------------------- */
static void stateFollowLine_onTick(void) {
  bool intersection;
  bool lost;
  lineFollowTick(g_ground, &intersection, &lost);

  if (lost) {
    g_lostTickCount++;
    if (g_lostTickCount < LOST_TICKS) {
      /* Spin search: rotate in place */
      setVel2(-RECOVERY_SPEED, RECOVERY_SPEED);
    } else {
      /* Timeout — give up */
      setVel2(0, 0);
      logLostTimeout();
      changeState(&g_stateDone, "LOST");
    }
  } else {
    /* Line reacquired */
    g_lostTickCount = 0;

    if (intersection) {
      /* Could be intersection or target — verify */
      formatGroundHex(g_ground, g_logDetail);
      changeState(&g_stateVerifyTarget, g_logDetail);
    }
  }
}

/* -------------------------------------------------- */
/* STATE_VERIFY_TARGET                                */
/* -------------------------------------------------- */
static void stateVerifyTarget_onEnter(void) {
  double h;
  getRobotPos(&g_verifyStartX, &g_verifyStartY, &h);
}

static void stateVerifyTarget_onTick(void) {
  double x, y, h;
  double dist;

  if (g_ground == SENSOR_ALL) {
    getRobotPos(&x, &y, &h);
    dist = hypot(x - g_verifyStartX, y - g_verifyStartY);

    if (dist >= TARGET_DIST_THRESHOLD) {
      /* Confirmed target */
      changeState(&g_stateAtTarget, NULL);
    } else {
      /* Keep driving straight over the wide area */
      setVel2(BASE_SPEED, BASE_SPEED);
    }
  } else {

    /* Pattern broke — this was just an intersection */
    // printf("DEBUG --- Target pattern broke\n");

    changeState(&g_stateDetectIntersection, NULL);
  }
}

/* -------------------------------------------------- */
/* STATE_DETECT_INTERSECTION                          */
/* -------------------------------------------------- */
static void stateDetectIntersection_onTick(void) {
  double x, y, h;
  double distIntoIntersection;

  setVel2(0, 0);
  getRobotPos(&x, &y, &h);

  distIntoIntersection = hypot(x - g_verifyStartX, y - g_verifyStartY);
  // printf("DEBUG --- x, y = (%d, %d)\n", x, y);
  // printf("DEBUG --- g_verifyStart = (%d, %d)\n", g_verifyStartX,
  //        g_verifyStartY);
  //
  // printf("DEBUG --- distIntoIntersection = %f\n", distIntoIntersection);

  if (!pushPose(x, y, h)) {
    changeState(&g_stateDone, "STACK_OVERFLOW");
  } else if (distIntoIntersection <= 10) {
    setVel2(BASE_SPEED, BASE_SPEED);
  } else {
    changeState(&g_stateTurnLeft, NULL);
  }
}

/* -------------------------------------------------- */
/* STATE_TURN_LEFT                                    */
/* -------------------------------------------------- */
static void stateTurnLeft_onEnter(void) {
  startRotation(PI / 2.0); /* 90 deg left */
}

static void stateTurnLeft_onTick(void) {
  rotationTick();
  if (rotationDone()) {
    changeState(&g_stateFollowLine, NULL);
  }
}

/* -------------------------------------------------- */
/* STATE_AT_TARGET                                    */
/* -------------------------------------------------- */
static void stateAtTarget_onTick(void) {
  setVel2(0, 0);
  getRobotPos(&g_targetX, &g_targetY, &g_targetH);
  logTarget(g_targetX, g_targetY, g_targetH);

  if (isStackEmpty()) {
    /* Target is at start — nothing to return */
    changeState(&g_stateDone, "TARGET_AT_START");
  } else {
    /* Pop first parent and turn toward it */
    startReturnToParent();
  }
}

/* -------------------------------------------------- */
/* STATE_RETURN_TURN                                  */
/* -------------------------------------------------- */
static void stateReturnTurn_onEnter(void) {
  double x, y, h;
  getRobotPos(&x, &y, &h);
  double deltaAngle =
      normalizeAngle(atan2(g_returnTargetY - y, g_returnTargetX - x) - h);
  startRotation(deltaAngle);
}

static void stateReturnTurn_onTick(void) {
  rotationTick();
  if (rotationDone()) {
    changeState(&g_stateReturnFollow, NULL);
  }
}

/* -------------------------------------------------- */
/* STATE_RETURN_FOLLOW                                */
/* -------------------------------------------------- */
static void stateReturnFollow_onTick(void) {
  double x, y, h;
  double dist;

  getRobotPos(&x, &y, &h);
  dist = hypot(g_returnTargetX - x, g_returnTargetY - y);

  if (dist < ORIGIN_TOLERANCE) {
    /* Reached this node */
    setVel2(0, 0);
    if (isStackEmpty()) {
      /* Back at start */
      logMissionDone(g_targetX, g_targetY, g_targetH, g_stack_top + 1,
                     g_missionTick);
      changeState(&g_stateDone, NULL);
    } else {
      /* Next parent */
      startReturnToParent();
    }
  } else {
    /* Drive straight toward target */
    setVel2(BASE_SPEED, BASE_SPEED);
  }
}

/* -------------------------------------------------- */
/* STATE_DONE                                         */
/* -------------------------------------------------- */
static void stateDone_onTick(void) {
  setVel2(0, 0);
  if (startButton()) {
    restartMission();
    pushPose(0.0, 0.0, 0.0); /* Cannot overflow — stack just reset */
  }
}

/* ========================================================================
 *   STATE INSTANCE DEFINITIONS
 * ======================================================================== */

State g_stateIdle = {NULL, stateIdle_onTick, NULL, "IDLE"};
State g_stateFollowLine = {NULL, stateFollowLine_onTick, NULL, "FOLLOW_LINE"};
State g_stateDetectIntersection = {NULL, stateDetectIntersection_onTick, NULL,
                                   "DETECT_INTERSECTION"};
State g_stateTurnLeft = {stateTurnLeft_onEnter, stateTurnLeft_onTick, NULL,
                         "TURN_LEFT"};
State g_stateVerifyTarget = {stateVerifyTarget_onEnter,
                             stateVerifyTarget_onTick, NULL, "VERIFY_TARGET"};
State g_stateAtTarget = {NULL, stateAtTarget_onTick, NULL, "AT_TARGET"};
State g_stateReturnTurn = {stateReturnTurn_onEnter, stateReturnTurn_onTick,
                           NULL, "RETURN_TURN"};
State g_stateReturnFollow = {NULL, stateReturnFollow_onTick, NULL,
                             "RETURN_FOLLOW"};
State g_stateDone = {NULL, stateDone_onTick, NULL, "DONE"};

/* ========================================================================
 *   LED INDICATORS
 * ======================================================================== */

static void updateLEDs(void) {
  /* Clear all LEDs first */
  leds(0);

  if (g_currentState == &g_stateIdle || g_currentState == &g_stateFollowLine ||
      g_currentState == &g_stateDetectIntersection ||
      g_currentState == &g_stateTurnLeft ||
      g_currentState == &g_stateVerifyTarget) {
    /* Phase 1: exploring */
    led(0, 1);
  } else if (g_currentState == &g_stateAtTarget) {
    /* At target */
    led(1, 1);
  } else if (g_currentState == &g_stateReturnTurn ||
             g_currentState == &g_stateReturnFollow) {
    /* Phase 3: returning */
    led(2, 1);
  } else if (g_currentState == &g_stateDone) {
    /* Finished */
    led(3, 1);
  }
}

/* ========================================================================
 *   HELPER FUNCTIONS
 * ======================================================================== */

static void restartMission(void) {
  setRobotPos(0.0, 0.0, 0.0);
  g_stack_top = -1;
  g_lostTickCount = 0;
  g_rotActive = false;
  g_missionTick = 0;
  changeState(&g_stateFollowLine, NULL);
}

static void startReturnToParent(void) {
  double h;
  popPose(&g_returnTargetX, &g_returnTargetY, &h);
  logPop(g_stack_top + 1);
  changeState(&g_stateReturnTurn, NULL);
}

/* ========================================================================
 *   MAIN CONTROL LOOP
 * ======================================================================== */

int main(void) {
  initPIC32();
  closedLoopControl(true);
  setVel2(0, 0);
  setRobotPos(0.0, 0.0, 0.0);

  /* Wait for start button */
  while (!startButton()) {
    /* Blink LED 0 to show we are alive */
    led(0, 1);
    delay(50); /* 5 ms — outside control loop, OK for idle */
    led(0, 0);
    delay(50);
  }

  /* Reset state machine and push start pose onto stack */
  restartMission();
  pushPose(0.0, 0.0, 0.0); /* Cannot overflow — stack just reset */

  while (1) {
    /* ---- Timing: exactly one wait per iteration ---- */
    waitTick40ms();
    LOG_TICK();

    /* ---- Emergency stop ---- */
    if (stopButton()) {
      setVel2(0, 0);
      changeState(&g_stateIdle, "STOP_BUTTON");
      continue;
    }

    /* ---- Sensor read order: analog first, then line ---- */
    readAnalogSensors();
    g_ground = readLineSensors(0);

    /* ---- Update LEDs for visual debugging ---- */
    updateLEDs();

    /* ---- State machine dispatch ---- */
    if (g_currentState && g_currentState->onTick) {
      g_currentState->onTick();
    }
  }

  return 0;
}
