#include "state_machine.h"
#include "nav_stack.h"
#include "line_follower.h"
#include "rotation.h"
#include "logging.h"
#include <math.h>

/* ========================================================================
 *   STATE STRUCT
 * ======================================================================== */

typedef struct State {
    void (*onEnter)(void);
    void (*onTick)(void);
    void (*onExit)(void);
    const char *name;
} State;

/* ========================================================================
 *   FORWARD DECLARATIONS OF STATE HANDLER FUNCTIONS
 * ======================================================================== */

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

static State s_stateIdle = {NULL, stateIdle_onTick, NULL, "IDLE"};
static State s_stateFollowLine = {NULL, stateFollowLine_onTick, NULL, "FOLLOW_LINE"};
static State s_statePrepareTurn = {statePrepareTurn_onEnter, statePrepareTurn_onTick,
                                   NULL, "PREPARE_TURN"};
static State s_stateTurnLeft = {stateTurnLeft_onEnter, stateTurnLeft_onTick,
                                stateTurnLeft_onExit, "TURN_LEFT"};
static State s_stateVerifyTarget = {stateVerifyTarget_onEnter,
                                    stateVerifyTarget_onTick, NULL, "VERIFY_TARGET"};
static State s_stateAtTarget = {NULL, stateAtTarget_onTick, NULL, "AT_TARGET"};
static State s_stateReturnTurn = {stateReturnTurn_onEnter, stateReturnTurn_onTick,
                                  stateReturnTurn_onExit, "RETURN_TURN"};
static State s_stateReturnFollow = {NULL, stateReturnFollow_onTick, NULL,
                                    "RETURN_FOLLOW"};
static State s_stateDone = {NULL, stateDone_onTick, NULL, "DONE"};

/* ========================================================================
 *   MODULE-PRIVATE DATA
 * ======================================================================== */

static State *s_currentState = &s_stateIdle;

/* Target pose */
static double s_targetX = 0.0;
static double s_targetY = 0.0;
static double s_targetH = 0.0;

/* Tick counters and flags */
static int s_lostTickCount = 0;

/* Target verification start pose */
static double s_verifyStartX = 0.0;
static double s_verifyStartY = 0.0;

static double s_startJointX = 0.0;
static double s_startJointY = 0.0;
static double s_startJointH = 0.0;

/* Return navigation target */
static double s_returnTargetX = 0.0;
static double s_returnTargetY = 0.0;

/* Done state reason for LED indication */
static const char *s_doneReason = NULL;

/* Cached robot pose and tick - updated once per tick */
static unsigned int s_tick = 0;
static unsigned int s_ground = 0;
static double s_robotX = 0.0;
static double s_robotY = 0.0;
static double s_robotH = 0.0;

/* Current phase for LED mapping */
static Phase s_phase = PHASE_EXPLORING;

/* ========================================================================
 *   FORWARD DECLARATIONS OF HELPER FUNCTIONS
 * ======================================================================== */

static void changeState(State *newState, const char *detail);
static void restartMission(void);
static void startReturnToParent(void);

/* ========================================================================
 *   changeState() HELPER
 * ======================================================================== */

static void changeState(State *newState, const char *detail) {
    log_transition(s_tick, s_currentState ? s_currentState->name : "NULL",
                   newState ? newState->name : "NULL", detail);

    if (s_currentState && s_currentState->onExit) {
        s_currentState->onExit();
    }
    s_currentState = newState;

    /* Update phase for LED mapping */
    if (s_currentState == &s_stateIdle ||
        s_currentState == &s_stateFollowLine ||
        s_currentState == &s_statePrepareTurn ||
        s_currentState == &s_stateTurnLeft ||
        s_currentState == &s_stateVerifyTarget) {
        s_phase = PHASE_EXPLORING;
    } else if (s_currentState == &s_stateAtTarget) {
        s_phase = PHASE_AT_TARGET;
    } else if (s_currentState == &s_stateReturnTurn ||
               s_currentState == &s_stateReturnFollow) {
        s_phase = PHASE_RETURNING;
    } else if (s_currentState == &s_stateDone) {
        s_phase = (s_doneReason != NULL) ? PHASE_DONE_ERROR : PHASE_DONE_SUCCESS;
    }

    if (s_currentState && s_currentState->onEnter) {
        s_currentState->onEnter();
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
        Pose startPose = {0.0, 0.0, 0.0};
        nav_push(&startPose);
        log_push(s_tick, nav_getDepth(), 0.0, 0.0, 0.0);
    }
}

/* -------------------------------------------------- */
/* STATE_FOLLOW_LINE                                  */
/* -------------------------------------------------- */
static void stateFollowLine_onTick(void) {
    bool lost;
    lineFollower_centerOnLine(s_ground, &lost);

    bool intersection = (s_ground == SENSOR_ALL);
    bool pathToTheLeft = ((s_ground & SENSOR_LEFT) == SENSOR_LEFT);

    if (lost) {
        s_lostTickCount++;
        if (s_lostTickCount < LOST_TICKS) {
            /* Spin search: rotate in place */
            setVel2(-RECOVERY_SPEED, RECOVERY_SPEED);
        } else {
            /* Timeout — give up */
            setVel2(0, 0);
            s_doneReason = "LOST";
            log_lostTimeout(s_tick);
            changeState(&s_stateDone, "LOST");
        }
    } else {
        /* Line reacquired */
        s_lostTickCount = 0;

        if (intersection) {
            /* Could be intersection or target — verify */
            log_groundDetection(s_tick, "intersection", s_ground);
            changeState(&s_stateVerifyTarget, NULL);
        } else if (pathToTheLeft) {
            /* Path to the left — skip target verification, go directly to turn */
            log_groundDetection(s_tick, "left turn", s_ground);
            double h;
            getRobotPos(&s_verifyStartX, &s_verifyStartY, &h);
            changeState(&s_statePrepareTurn, NULL);
        }
    }
}

/* -------------------------------------------------- */
/* STATE_VERIFY_TARGET                                */
/* -------------------------------------------------- */
static void stateVerifyTarget_onEnter(void) {
    double h;
    getRobotPos(&s_verifyStartX, &s_verifyStartY, &h);
}

static void stateVerifyTarget_onTick(void) {
    double x, y, h;
    double dist;

    if (s_ground == SENSOR_ALL) {
        getRobotPos(&x, &y, &h);
        dist = hypot(x - s_verifyStartX, y - s_verifyStartY);

        if (dist >= TARGET_DIST_THRESHOLD) {
            /* Confirmed target */
            changeState(&s_stateAtTarget, NULL);
        } else {
            /* Keep driving straight over the wide area */
            setVel2(BASE_SPEED, BASE_SPEED);
        }
    } else {
        /* Pattern broke — this was just an intersection */
        changeState(&s_statePrepareTurn, NULL);
    }
}

/* -------------------------------------------------- */
/* STATE_PREPARE_TURN                                 */
/* -------------------------------------------------- */
static void statePrepareTurn_onEnter(void) {
    /* BUG FIX 1: use &s_startJointY (not &s_startJointX twice) */
    getRobotPos(&s_startJointX, &s_startJointY, &s_startJointH);

    Pose pose = {s_startJointX, s_startJointY, s_startJointH};
    if (!nav_push(&pose)) {
        s_doneReason = "STACK_OVERFLOW";
        changeState(&s_stateDone, "STACK_OVERFLOW");
        return;
    }
    log_push(s_tick, nav_getDepth(), s_startJointX, s_startJointY, s_startJointH);

    setVel2(0, 0);
    /* BUG FIX 2: removed while(!startButton()) busy-wait */
}

static void statePrepareTurn_onTick(void) {
    double x, y, h;
    double distIntoIntersection;

    getRobotPos(&x, &y, &h);

    distIntoIntersection =
        sqrt(pow(x - s_startJointX, 2) + pow(y - s_startJointY, 2));

    if (distIntoIntersection <= INTERSECTION_CENTER_DIST) {
        setVel2(BASE_SPEED, BASE_SPEED);
    } else {
        /* BUG FIX 3: removed printf("DEBUG --- ...") */
        changeState(&s_stateTurnLeft, NULL);
    }
}

/* -------------------------------------------------- */
/* STATE_TURN_LEFT                                    */
/* -------------------------------------------------- */
static void stateTurnLeft_onEnter(void) {
    rotation_start(s_robotH, PI / 2.0); /* 90 deg left */
    log_turnStart(s_tick, normalizeAngle(s_robotH + PI / 2.0));
}

static void stateTurnLeft_onTick(void) {
    rotation_tick(s_robotH);
    if (rotation_done(s_robotH)) {
        log_turnDone(s_tick);
        changeState(&s_stateFollowLine, NULL);
    }
}

static void stateTurnLeft_onExit(void) {
    rotation_stop();
}

/* -------------------------------------------------- */
/* STATE_AT_TARGET                                    */
/* -------------------------------------------------- */
static void stateAtTarget_onTick(void) {
    setVel2(0, 0);
    getRobotPos(&s_targetX, &s_targetY, &s_targetH);
    log_target(s_tick, s_targetX, s_targetY, s_targetH);

    if (nav_isEmpty()) {
        /* Target is at start — nothing to return */
        s_doneReason = "TARGET_AT_START";
        changeState(&s_stateDone, "TARGET_AT_START");
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
        normalizeAngle(atan2(s_returnTargetY - y, s_returnTargetX - x) - h);
    rotation_start(s_robotH, deltaAngle);
    log_turnStart(s_tick, normalizeAngle(s_robotH + deltaAngle));
}

static void stateReturnTurn_onTick(void) {
    rotation_tick(s_robotH);
    if (rotation_done(s_robotH)) {
        log_turnDone(s_tick);
        changeState(&s_stateReturnFollow, NULL);
    }
}

static void stateReturnTurn_onExit(void) {
    rotation_stop();
}

/* -------------------------------------------------- */
/* STATE_RETURN_FOLLOW                                */
/* -------------------------------------------------- */
static void stateReturnFollow_onTick(void) {
    double x, y, h;
    double dist;
    bool lost;

    /* Follow the line like normal, but check if we've reached the parent node */
    lineFollower_centerOnLine(s_ground, &lost);

    if (lost) {
        /* Lost line during return — try to recover */
        s_lostTickCount++;
        if (s_lostTickCount < LOST_TICKS) {
            setVel2(-RECOVERY_SPEED, RECOVERY_SPEED);
        } else {
            setVel2(0, 0);
            s_doneReason = "LOST_RETURN";
            log_lostTimeout(s_tick);
            changeState(&s_stateDone, "LOST_RETURN");
        }
        return;
    }

    s_lostTickCount = 0;

    /* Check distance to parent node */
    getRobotPos(&x, &y, &h);
    dist = hypot(s_returnTargetX - x, s_returnTargetY - y);

    if (dist < ORIGIN_TOLERANCE) {
        /* Reached this node */
        setVel2(0, 0);
        if (nav_isEmpty()) {
            /* Back at start */
            log_missionDone(s_tick, s_targetX, s_targetY, s_targetH,
                            nav_getDepth(), s_tick);
            changeState(&s_stateDone, NULL);
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
        Pose startPose = {0.0, 0.0, 0.0};
        nav_push(&startPose);
        log_push(s_tick, nav_getDepth(), 0.0, 0.0, 0.0);
    }
}

/* ========================================================================
 *   HELPER FUNCTIONS
 * ======================================================================== */

static void restartMission(void) {
    setRobotPos(0.0, 0.0, 0.0);
    nav_reset();
    s_lostTickCount = 0;
    rotation_stop();
    s_targetX = 0.0;
    s_targetY = 0.0;
    s_targetH = 0.0;
    s_verifyStartX = 0.0;
    s_verifyStartY = 0.0;
    s_returnTargetX = 0.0;
    s_returnTargetY = 0.0;
    s_doneReason = NULL;
    changeState(&s_stateFollowLine, NULL);
}

static void startReturnToParent(void) {
    Pose pose;
    nav_pop(&pose);
    s_returnTargetX = pose.x;
    s_returnTargetY = pose.y;
    log_pop(s_tick, nav_getDepth());
    changeState(&s_stateReturnTurn, NULL);
}

/* ========================================================================
 *   PUBLIC API
 * ======================================================================== */

void stateMachine_init(void) {
    s_currentState = &s_stateIdle;
    s_phase = PHASE_EXPLORING;
    s_doneReason = NULL;
}

void stateMachine_start(void) {
    restartMission();
    Pose startPose = {0.0, 0.0, 0.0};
    nav_push(&startPose);
    log_push(s_tick, nav_getDepth(), 0.0, 0.0, 0.0);
}

void stateMachine_tick(unsigned int tick, unsigned int ground, double x, double y, double h) {
    s_tick = tick;
    s_ground = ground;
    s_robotX = x;
    s_robotY = y;
    s_robotH = h;

    if (s_currentState && s_currentState->onTick) {
        s_currentState->onTick();
    }
}

void stateMachine_stop(unsigned int tick) {
    s_tick = tick;
    setVel2(0, 0);
    changeState(&s_stateIdle, "STOP_BUTTON");
}

Phase stateMachine_getPhase(void) {
    return s_phase;
}

const char *stateMachine_getDoneReason(void) {
    return s_doneReason;
}
