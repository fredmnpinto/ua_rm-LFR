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

/* ========================================================================
 *   DATA STRUCTURES
 * ======================================================================== */

typedef struct {
  double x;
  double y;
  double h;
} StackEntry;

/* ========================================================================
 *   GLOBAL VARIABLES (all statically allocated)
 * ======================================================================== */

static StackEntry g_stack[MAX_STACK_DEPTH];
static int g_stack_top = -1;

static RobotState g_state = STATE_IDLE;

/* Target pose */
static double g_targetX = 0.0;
static double g_targetY = 0.0;
static double g_targetH = 0.0;

/* Tick counters and flags */
static int g_lostTickCount = 0;
static int g_targetTickCount = 0;

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

/* ========================================================================
 *   LOGGING SUBSYSTEM
 * ======================================================================== */

static unsigned int g_missionTick = 0;
static bool g_logThisTick = false;

#define LOG_TICK()                                                             \
  do {                                                                         \
    g_missionTick++;                                                           \
    g_logThisTick = false;                                                     \
  } while (0)

static const char *stateName(RobotState s) {
  switch (s) {
  case STATE_IDLE:
    return "IDLE";
  case STATE_FOLLOW_LINE:
    return "FOLLOW_LINE";
  case STATE_DETECT_INTERSECTION:
    return "DETECT_INTERSECTION";
  case STATE_TURN_LEFT:
    return "TURN_LEFT";
  case STATE_VERIFY_TARGET:
    return "VERIFY_TARGET";
  case STATE_AT_TARGET:
    return "AT_TARGET";
  case STATE_RETURN_TURN:
    return "RETURN_TURN";
  case STATE_RETURN_FOLLOW:
    return "RETURN_FOLLOW";
  case STATE_DONE:
    return "DONE";
  default:
    return "UNKNOWN";
  }
}

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

static void logTransition(RobotState to, const char *detail) {
  if (g_logThisTick)
    return;
  printf("[T %u] %s -> %s%s%s\n", g_missionTick, stateName(g_state),
         stateName(to), detail ? " " : "", detail ? detail : "");
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

  // printf("DEBUG LINE SENSOR ---- ");
  // printInt(ground, 2 | 5 << 16); // System call
  // printf("\n");

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
    /* Line detected on left side → drift right */
    setVel2(TURN_SPEED, BASE_SPEED);
  } else if ((ground & SENSOR_RIGHT) != 0) {
    /* Line detected on right side → drift left */
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
 *   LED INDICATORS
 * ======================================================================== */

static void updateLEDs(void) {
  /* Clear all LEDs first */
  leds(0);

  switch (g_state) {
  case STATE_IDLE:
  case STATE_FOLLOW_LINE:
  case STATE_DETECT_INTERSECTION:
  case STATE_TURN_LEFT:
  case STATE_VERIFY_TARGET:
    /* Phase 1: exploring */
    led(0, 1);
    break;

  case STATE_AT_TARGET:
    /* At target */
    led(1, 1);
    break;

  case STATE_RETURN_TURN:
  case STATE_RETURN_FOLLOW:
    /* Phase 3: returning */
    led(2, 1);
    break;

  case STATE_DONE:
    /* Finished */
    led(3, 1);
    break;
  }
}

/* ========================================================================
 *   HELPER FUNCTIONS
 * ======================================================================== */

static void restartMission(void) {
  setRobotPos(0.0, 0.0, 0.0);
  g_stack_top = -1;
  g_lostTickCount = 0;
  g_targetTickCount = 0;
  g_rotActive = false;
  g_missionTick = 0;
  logTransition(STATE_FOLLOW_LINE, NULL);
  g_state = STATE_FOLLOW_LINE;
}

static void startReturnToParent(void) {
  double x, y, h;
  popPose(&g_returnTargetX, &g_returnTargetY, &h);
  logPop(g_stack_top + 1);
  getRobotPos(&x, &y, &h);
  double deltaAngle =
      normalizeAngle(atan2(g_returnTargetY - y, g_returnTargetX - x) - h);
  startRotation(deltaAngle);
  logTransition(STATE_RETURN_TURN, NULL);
  g_state = STATE_RETURN_TURN;
}

/* ========================================================================
 *   MAIN CONTROL LOOP
 * ======================================================================== */

int main(void) {
  unsigned int ground;
  bool intersection;
  bool lost;
  double x, y, h;
  double dist;
  char logDetail[32];

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
  if (!pushPose(0.0, 0.0, 0.0)) {
    /* Stack overflow on very first push — should never happen */
    setVel2(0, 0);
    logTransition(STATE_DONE, "STACK_OVERFLOW");
    g_state = STATE_DONE;
  } else {
    logPush(g_stack_top + 1, 0.0, 0.0, 0.0);
  }

  while (1) {
    /* ---- Timing: exactly one wait per iteration ---- */
    waitTick40ms();
    LOG_TICK();

    /* ---- Emergency stop ---- */
    if (stopButton()) {
      setVel2(0, 0);
      logTransition(STATE_IDLE, "STOP_BUTTON");
      g_state = STATE_IDLE;
      continue;
    }

    /* ---- Sensor read order: analog first, then line ---- */
    readAnalogSensors();
    ground = readLineSensors(0);

    /* ---- Update LEDs for visual debugging ---- */
    updateLEDs();

    /* ---- State machine dispatch ---- */
    switch (g_state) {

    /* -------------------------------------------------- */
    /* STATE_IDLE — wait for restart                        */
    /* -------------------------------------------------- */
    case STATE_IDLE:
      setVel2(0, 0);
      if (startButton()) {
        restartMission();
        if (!pushPose(0.0, 0.0, 0.0)) {
          setVel2(0, 0);
          logTransition(STATE_DONE, "STACK_OVERFLOW");
          g_state = STATE_DONE;
        } else {
          logPush(g_stack_top + 1, 0.0, 0.0, 0.0);
        }
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_FOLLOW_LINE — normal line following            */
    /* -------------------------------------------------- */
    case STATE_FOLLOW_LINE:
      lineFollowTick(ground, &intersection, &lost);

      if (lost) {
        g_lostTickCount++;
        if (g_lostTickCount < LOST_TICKS) {
          /* Spin search: rotate in place */
          setVel2(-RECOVERY_SPEED, RECOVERY_SPEED);
        } else {
          /* Timeout — give up */
          setVel2(0, 0);
          logLostTimeout();
          logTransition(STATE_DONE, "LOST");
          g_state = STATE_DONE;
        }
      } else {
        /* Line reacquired */
        g_lostTickCount = 0;

        if (intersection) {
          /* Could be intersection or target — verify */
          getRobotPos(&g_verifyStartX, &g_verifyStartY, &h);
          g_targetTickCount = 0;
          formatGroundHex(ground, logDetail);
          logTransition(STATE_VERIFY_TARGET, logDetail);
          g_state = STATE_VERIFY_TARGET;
        }
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_VERIFY_TARGET — distinguish target vs crossing */
    /* -------------------------------------------------- */
    case STATE_VERIFY_TARGET:

      if (ground == SENSOR_ALL) {
        g_targetTickCount++;
        getRobotPos(&x, &y, &h);
        dist = sqrt((x - g_verifyStartX) * (y - g_verifyStartY));

        // printf("DEBUG --- targetTickCount = %d - dist = %f\n",
        //        g_targetTickCount, dist);

        if (dist >= TARGET_DIST_THRESHOLD) {
          /* Confirmed target */
          logTransition(STATE_AT_TARGET, NULL);
          g_state = STATE_AT_TARGET;
        } else {
          /* Keep driving straight over the wide area */
          setVel2(BASE_SPEED, BASE_SPEED);
        }
      } else {

        /* Pattern broke — this was just an intersection */
        // printf("DEBUG --- Target pattern broke\n");

        g_targetTickCount = 0;
        logTransition(STATE_DETECT_INTERSECTION, NULL);
        g_state = STATE_DETECT_INTERSECTION;
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_DETECT_INTERSECTION — halt, push pose, turn    */
    /* -------------------------------------------------- */
    case STATE_DETECT_INTERSECTION:
      setVel2(0, 0);
      getRobotPos(&x, &y, &h);

      double distIntoIntersection =
          sqrt((x - g_verifyStartX) * (y - g_verifyStartY));
      // printf("DEBUG --- x, y = (%d, %d)\n", x, y);
      // printf("DEBUG --- g_verifyStart = (%d, %d)\n", g_verifyStartX,
      //        g_verifyStartY);
      //
      // printf("DEBUG --- distIntoIntersection = %f\n", distIntoIntersection);

      if (!pushPose(x, y, h)) {
        /* Stack full — abort safely */
        setVel2(0, 0);
        logTransition(STATE_DONE, "STACK_OVERFLOW");
        g_state = STATE_DONE;
      } else if (distIntoIntersection <= 10) {
        setVel2(BASE_SPEED, BASE_SPEED);
      } else {
        logPush(g_stack_top + 1, x, y, h);
        startRotation(PI / 2.0); /* 90 deg left */
        logTransition(STATE_TURN_LEFT, NULL);
        g_state = STATE_TURN_LEFT;
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_TURN_LEFT — non-blocking 90° left turn         */
    /* -------------------------------------------------- */
    case STATE_TURN_LEFT:
      rotationTick();
      if (rotationDone()) {
        logTransition(STATE_FOLLOW_LINE, NULL);
        g_state = STATE_FOLLOW_LINE;
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_AT_TARGET — record pose, start return          */
    /* -------------------------------------------------- */
    case STATE_AT_TARGET:
      setVel2(0, 0);
      getRobotPos(&g_targetX, &g_targetY, &g_targetH);
      logTarget(g_targetX, g_targetY, g_targetH);

      if (isStackEmpty()) {
        /* Target is at start — nothing to return */
        logTransition(STATE_DONE, "TARGET_AT_START");
        g_state = STATE_DONE;
      } else {
        /* Pop first parent and turn toward it */
        startReturnToParent();
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_RETURN_TURN — turn toward parent node          */
    /* -------------------------------------------------- */
    case STATE_RETURN_TURN:
      rotationTick();
      if (rotationDone()) {
        logTransition(STATE_RETURN_FOLLOW, NULL);
        g_state = STATE_RETURN_FOLLOW;
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_RETURN_FOLLOW — drive straight to parent       */
    /* -------------------------------------------------- */
    case STATE_RETURN_FOLLOW:
      getRobotPos(&x, &y, &h);
      dist = hypot(g_returnTargetX - x, g_returnTargetY - y);

      if (dist < ORIGIN_TOLERANCE) {
        /* Reached this node */
        setVel2(0, 0);
        if (isStackEmpty()) {
          /* Back at start */
          logMissionDone(g_targetX, g_targetY, g_targetH, g_stack_top + 1,
                         g_missionTick);
          logTransition(STATE_DONE, NULL);
          g_state = STATE_DONE;
        } else {
          /* Next parent */
          startReturnToParent();
        }
      } else {
        /* Drive straight toward target */
        setVel2(BASE_SPEED, BASE_SPEED);
      }
      break;

    /* -------------------------------------------------- */
    /* STATE_DONE — halt, wait for restart                  */
    /* -------------------------------------------------- */
    case STATE_DONE:
      setVel2(0, 0);
      if (startButton()) {
        restartMission();
        if (!pushPose(0.0, 0.0, 0.0)) {
          setVel2(0, 0);
          logTransition(STATE_DONE, "STACK_OVERFLOW");
          g_state = STATE_DONE;
        } else {
          logPush(g_stack_top + 1, 0.0, 0.0, 0.0);
        }
      }
      break;
    }
  }

  return 0;
}
