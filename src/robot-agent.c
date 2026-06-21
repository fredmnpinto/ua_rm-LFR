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
#define TARGET_DIST_THRESHOLD 6
#define BASE_SPEED 40
#define TURN_SPEED BASE_SPEED / 2
#define RECOVERY_SPEED BASE_SPEED
#define LOST_TICKS 25
#define ORIGIN_TOLERANCE 0.05
#define INTERSECTION_CENTER_DIST                                               \
  150 /* Distance to drive into intersection before turning */

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

/* Forward declarations of state handler functions */
static void stateIdle_onTick(void);
static void stateFollowLine_onTick(void);
static void statePrepareTurn_onEnter(void);
static void statePrepareTurn_onTick(void);
static void stateTurnLeft_onEnter(void);
static void stateTurnLeft_onTick(void);
static void stateTurnLeft_onExit(void);
static void stateVerifyTarget_onEnter(void);
static void stateVerifyTarget_onTick(void);
static void stateAtTarget_onTick(void);
static void stateReturnTurn_onEnter(void);
static void stateReturnTurn_onTick(void);
static void stateReturnTurn_onExit(void);
static void stateReturnFollow_onTick(void);
static void stateDone_onTick(void);

/* ========================================================================
 *   STATE INSTANCE DEFINITIONS
 * ======================================================================== */

State g_stateIdle = {NULL, stateIdle_onTick, NULL, "IDLE"};
State g_stateFollowLine = {NULL, stateFollowLine_onTick, NULL, "FOLLOW_LINE"};
State g_statePrepareTurn = {statePrepareTurn_onEnter, statePrepareTurn_onTick,
                            NULL, "PREPARE_TURN"};
State g_stateTurnLeft = {stateTurnLeft_onEnter, stateTurnLeft_onTick,
                         stateTurnLeft_onExit, "TURN_LEFT"};
State g_stateVerifyTarget = {stateVerifyTarget_onEnter,
                             stateVerifyTarget_onTick, NULL, "VERIFY_TARGET"};
State g_stateAtTarget = {NULL, stateAtTarget_onTick, NULL, "AT_TARGET"};
State g_stateReturnTurn = {stateReturnTurn_onEnter, stateReturnTurn_onTick,
                           stateReturnTurn_onExit, "RETURN_TURN"};
State g_stateReturnFollow = {NULL, stateReturnFollow_onTick, NULL,
                             "RETURN_FOLLOW"};
State g_stateDone = {NULL, stateDone_onTick, NULL, "DONE"};

/* ========================================================================
 *   GLOBAL VARIABLES (all statically allocated)
 * ======================================================================== */

static StackEntry g_stack[MAX_STACK_DEPTH];
static int g_stack_top = -1;

static State *g_currentState = &g_stateIdle;

static unsigned int g_ground = 0;

/* Target pose */
static double g_targetX = 0.0;
static double g_targetY = 0.0;
static double g_targetH = 0.0;

/* Tick counters and flags */
static int g_lostTickCount = 0;

/* Target verification start pose */
static double g_verifyStartX = 0.0;
static double g_verifyStartY = 0.0;

static double g_startJointX = 0.0;
static double g_startJointY = 0.0;
static double g_startJointH = 0.0;

/* Non-blocking rotation state */
static double g_rotTargetAngle = 0.0;
static double g_rotIntegral = 0.0;
static bool g_rotActive = false;

/* Return navigation target */
static double g_returnTargetX = 0.0;
static double g_returnTargetY = 0.0;

/* Mission tick counter */
static unsigned int g_missionTick = 0;

/* Cached robot pose - updated once per tick */
static double g_robotX = 0.0;
static double g_robotY = 0.0;
static double g_robotH = 0.0;

/* Done state reason for LED indication */
static const char *g_doneReason = NULL;

/* ========================================================================
 *   FORWARD DECLARATIONS OF HELPER FUNCTIONS
 * ======================================================================== */

static void restartMission(void);
static void startReturnToParent(void);

/* ========================================================================
 *   LOGGING SUBSYSTEM
 * ======================================================================== */

static void logTransition(State *to, const char *detail) {
  const char *fromName = g_currentState ? g_currentState->name : "NULL";
  const char *toName = to ? to->name : "NULL";
  printf("[T %u] %s -> %s%s%s\n", g_missionTick, fromName, toName,
         detail ? " " : "", detail ? detail : "");
}

static void logTarget(double x, double y, double h) {
  printf("[T %u] TARGET x=%.3f y=%.3f h=%.2f\n", g_missionTick, x, y, h);
}

static void logPush(int depth, double x, double y, double h) {
  printf("[T %u] PUSH depth=%d x=%.3f y=%.3f h=%.2f\n", g_missionTick, depth, x,
         y, h);
}

static void logPop(int depth) {
  printf("[T %u] POP depth=%d\n", g_missionTick, depth);
}

static void logTurnStart(double targetAngle) {
  printf("[T %u] TURN_LEFT target=%.2f\n", g_missionTick, targetAngle);
}

static void logTurnDone(void) { printf("[T %u] TURN_DONE\n", g_missionTick); }

static void logLostTimeout(void) {
  printf("[T %u] LOST_TIMEOUT\n", g_missionTick);
}

static void logGroundDetection(const char *type, unsigned int ground) {
  printf("[T %u] Detected %s, ground=", g_missionTick, type);
  printInt(ground, 2 | 5 << 16);
  printf("\n");
}

static void logMissionDone(double tx, double ty, double th, int depth,
                           int ticks) {
  printf("[T %u] DONE target=(%.2f,%.2f,%.2f) depth=%d ticks=%d\n",
         g_missionTick, tx, ty, th, depth, ticks);
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
 * \param pLost output: set to true if line is lost
 */
static void centerRobotOnLine(unsigned int ground, bool *pLost) {
  *pLost = false;

  if (ground == SENSOR_NONE) {
    /* Lost line — signal to caller, do not set motors here */
    *pLost = true;
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
  g_rotTargetAngle = normalizeAngle(g_robotH + deltaAngle);
  g_rotIntegral = 0.0;
  g_rotActive = true;
  logTurnStart(g_rotTargetAngle);
}

static void rotationTick(void) {
  double error;
  int cmdVel;

  if (!g_rotActive) {
    return;
  }

  error = normalizeAngle(g_rotTargetAngle - g_robotH);

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
  double error;

  if (!g_rotActive) {
    return true;
  }

  error = normalizeAngle(g_rotTargetAngle - g_robotH);

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
  bool lost;
  centerRobotOnLine(g_ground, &lost);

  bool intersection = g_ground == SENSOR_ALL;
  bool pathToTheLeft = (g_ground & SENSOR_LEFT) == SENSOR_LEFT;

  if (lost) {
    g_lostTickCount++;
    if (g_lostTickCount < LOST_TICKS) {
      /* Spin search: rotate in place */
      setVel2(-RECOVERY_SPEED, RECOVERY_SPEED);
    } else {
      /* Timeout — give up */
      setVel2(0, 0);
      g_doneReason = "LOST";
      logLostTimeout();
      changeState(&g_stateDone, "LOST");
    }
  } else {
    /* Line reacquired */
    g_lostTickCount = 0;

    if (intersection) {
      /* Could be intersection or target — verify */
      logGroundDetection("intersection", g_ground);
      changeState(&g_stateVerifyTarget, NULL);
    } else if (pathToTheLeft) {
      /* Path to the left — skip target verification, go directly to turn */
      logGroundDetection("left turn", g_ground);
      double h;
      getRobotPos(&g_verifyStartX, &g_verifyStartY, &h);
      changeState(&g_statePrepareTurn, NULL);
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

    changeState(&g_statePrepareTurn, NULL);
  }
}

/* -------------------------------------------------- */
/* STATE_PREPARE_TURN */
/* -------------------------------------------------- */
static void statePrepareTurn_onEnter(void) {
  double x, y, h;
  getRobotPos(&g_startJointX, &g_startJointX, &g_startJointH);

  if (!pushPose(g_startJointX, g_startJointY, g_startJointH)) {
    g_doneReason = "STACK_OVERFLOW";
    changeState(&g_stateDone, "STACK_OVERFLOW");
  }

  setVel2(0.0, 0.0);
  while (!startButton())
    ;
}

static void statePrepareTurn_onTick(void) {
  double x, y, h;
  double distIntoIntersection;

  getRobotPos(&x, &y, &h);

  distIntoIntersection =
      sqrt(pow(x - g_startJointX, 2) + pow(y - g_startJointY, 2));

  if (distIntoIntersection <= INTERSECTION_CENTER_DIST) {
    setVel2(BASE_SPEED, BASE_SPEED);
  } else {
    printf("DEBUG --- Finished prepare turn after dist = %f (mm)\n",
           distIntoIntersection);
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

static void stateTurnLeft_onExit(void) {
  g_rotActive = false;
  setVel2(0, 0);
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
    g_doneReason = "TARGET_AT_START";
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

static void stateReturnTurn_onExit(void) {
  g_rotActive = false;
  setVel2(0, 0);
}

/* -------------------------------------------------- */
/* STATE_RETURN_FOLLOW                                */
/* -------------------------------------------------- */
static void stateReturnFollow_onTick(void) {
  double x, y, h;
  double dist;
  bool lost;

  /* Follow the line like normal, but check if we've reached the parent node */
  centerRobotOnLine(g_ground, &lost);

  if (lost) {
    /* Lost line during return — try to recover */
    g_lostTickCount++;
    if (g_lostTickCount < LOST_TICKS) {
      setVel2(-RECOVERY_SPEED, RECOVERY_SPEED);
    } else {
      setVel2(0, 0);
      g_doneReason = "LOST_RETURN";
      logLostTimeout();
      changeState(&g_stateDone, "LOST_RETURN");
    }
    return;
  }

  g_lostTickCount = 0;

  /* Check distance to parent node */
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
 *   LED INDICATORS
 * ======================================================================== */

static void updateLEDs(void) {
  static unsigned int blinkCounter = 0;
  blinkCounter++;

  /* Clear all LEDs first */
  leds(0);

  if (g_currentState == &g_stateIdle || g_currentState == &g_stateFollowLine ||
      g_currentState == &g_statePrepareTurn ||
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
    /* Finished - blink pattern for errors */
    if (g_doneReason != NULL) {
      /* Error: fast blink LED 3 */
      if ((blinkCounter % 4) < 2) {
        led(3, 1);
      }
    } else {
      /* Success: solid LED 3 */
      led(3, 1);
    }
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
  g_rotTargetAngle = 0.0;
  g_rotIntegral = 0.0;
  g_targetX = 0.0;
  g_targetY = 0.0;
  g_targetH = 0.0;
  g_verifyStartX = 0.0;
  g_verifyStartY = 0.0;
  g_returnTargetX = 0.0;
  g_returnTargetY = 0.0;
  g_missionTick = 0;
  g_doneReason = NULL;
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
    g_missionTick++;

    /* ---- Emergency stop ---- */
    if (stopButton()) {
      setVel2(0, 0);
      changeState(&g_stateIdle, "STOP_BUTTON");
      continue;
    }

    /* ---- Sensor read order: analog first, then line ---- */
    readAnalogSensors();
    g_ground = readLineSensors(0);

    // logGroundDetection("-- DEBUG --", g_ground);

    /* ---- Update LEDs for visual debugging ---- */
    updateLEDs();

    /* ---- Cache robot pose for this tick ---- */
    getRobotPos(&g_robotX, &g_robotY, &g_robotH);

    /* ---- State machine dispatch ---- */
    if (g_currentState && g_currentState->onTick) {
      g_currentState->onTick();
    }
  }

  return 0;
}
