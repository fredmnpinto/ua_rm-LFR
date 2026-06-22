#include "state_machine.h"
#include "line_follower.h"
#include "logging.h"
#include "nav_graph.h"
#include "rotation.h"
#include <math.h>

/* =======================================================================
 *   STATE STRUCT
 * ======================================================================== */

typedef struct State {
  void (*onEnter)(void);
  void (*onTick)(void);
  void (*onExit)(void);
  const char *name;
} State;

/* =======================================================================
 *   FORWARD DECLARATIONS OF STATE HANDLER FUNCTIONS
 * ======================================================================== */

static void stateIdle_onTick(void);
static void stateFollowLine_onTick(void);
static void stateVerifyTarget_onEnter(void);
static void stateVerifyTarget_onTick(void);
static void stateAtTarget_onTick(void);
static void stateApproachCenter_onEnter(void);
static void stateApproachCenter_onTick(void);
static void stateChooseNext_onTick(void);
static void stateTurnToEdge_onEnter(void);
static void stateTurnToEdge_onTick(void);
static void stateTurnToEdge_onExit(void);
static void stateReturnTurn_onEnter(void);
static void stateReturnTurn_onTick(void);
static void stateReturnTurn_onExit(void);
static void stateReturnFollow_onTick(void);
static void stateReturnAtNode_onTick(void);
static void stateDone_onTick(void);

/* =======================================================================
 *   STATE INSTANCE DEFINITIONS
 * ======================================================================== */

static State s_stateIdle = {NULL, stateIdle_onTick, NULL, "IDLE"};
static State s_stateFollowLine = {NULL, stateFollowLine_onTick, NULL,
                                  "FOLLOW_LINE"};
static State s_stateVerifyTarget = {
    stateVerifyTarget_onEnter, stateVerifyTarget_onTick, NULL, "VERIFY_TARGET"};
static State s_stateAtTarget = {NULL, stateAtTarget_onTick, NULL, "AT_TARGET"};
static State s_stateApproachCenter = {stateApproachCenter_onEnter,
                                      stateApproachCenter_onTick, NULL,
                                      "APPROACH_CENTER"};
static State s_stateChooseNext = {NULL, stateChooseNext_onTick, NULL,
                                  "CHOOSE_NEXT"};
static State s_stateTurnToEdge = {stateTurnToEdge_onEnter,
                                  stateTurnToEdge_onTick,
                                  stateTurnToEdge_onExit, "TURN_TO_EDGE"};
static State s_stateReturnTurn = {stateReturnTurn_onEnter,
                                  stateReturnTurn_onTick,
                                  stateReturnTurn_onExit, "RETURN_TURN"};
static State s_stateReturnFollow = {NULL, stateReturnFollow_onTick, NULL,
                                    "RETURN_FOLLOW"};
static State s_stateReturnAtNode = {NULL, stateReturnAtNode_onTick, NULL,
                                    "RETURN_AT_NODE"};
static State s_stateDone = {NULL, stateDone_onTick, NULL, "DONE"};

/* =======================================================================
 *   MODULE-PRIVATE DATA
 * ======================================================================== */

static State *s_currentState = &s_stateIdle;

/* Target pose */
static double s_targetX = 0.0;
static double s_targetY = 0.0;
static double s_targetH = 0.0;

/* Tick counters and flags */
static int s_lostTickCount = 0;
static int s_leftCornerTicks = 0;
static int s_rightCornerTicks = 0;

/* Target verification start pose */
static double s_verifyStartX = 0.0;
static double s_verifyStartY = 0.0;

/* Approach center start pose */
static double s_approachStartX = 0.0;
static double s_approachStartY = 0.0;
static double s_approachStartH = 0.0;

/* Navigation graph state */
static unsigned char s_prevNodeId = 0;
static NodeType s_detectedNodeType = NODE_TYPE_INTERSECTION;
static double s_chosenDeltaAngle = 0.0;

/* Return navigation */
static unsigned char s_returnPath[MAX_PATH_LENGTH];
static unsigned char s_returnPathLen = 0;
static unsigned char s_returnPathIndex = 0;
static bool s_returnToTarget = true; /* true = leg 1 (origin->target) */

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

/* =======================================================================
 *   FORWARD DECLARATIONS OF HELPER FUNCTIONS
 * ======================================================================== */

static void changeState(State *newState, const char *detail);
static void restartMission(void);
static void startReturnNavigation(void);
static const char *nodeTypeToString(NodeType type);

/* =======================================================================
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
  if (s_currentState == &s_stateIdle || s_currentState == &s_stateFollowLine ||
      s_currentState == &s_stateVerifyTarget ||
      s_currentState == &s_stateApproachCenter ||
      s_currentState == &s_stateChooseNext ||
      s_currentState == &s_stateTurnToEdge) {
    s_phase = PHASE_EXPLORING;
  } else if (s_currentState == &s_stateAtTarget) {
    s_phase = PHASE_AT_TARGET;
  } else if (s_currentState == &s_stateReturnTurn ||
             s_currentState == &s_stateReturnFollow ||
             s_currentState == &s_stateReturnAtNode) {
    if (s_returnToTarget) {
      s_phase = PHASE_RETURNING_TO_TARGET;
    } else {
      s_phase = PHASE_RETURNING;
    }
  } else if (s_currentState == &s_stateDone) {
    s_phase = (s_doneReason != NULL) ? PHASE_DONE_ERROR : PHASE_DONE_SUCCESS;
  }

  if (s_currentState && s_currentState->onEnter) {
    s_currentState->onEnter();
  }
}

static const char *nodeTypeToString(NodeType type) {
  switch (type) {
  case NODE_TYPE_INTERSECTION:
    return "INTERSECTION";
  case NODE_TYPE_LEFT_CORNER:
    return "LEFT_CORNER";
  case NODE_TYPE_RIGHT_CORNER:
    return "RIGHT_CORNER";
  case NODE_TYPE_DEADEND:
    return "DEADEND";
  default:
    return "UNKNOWN";
  }
}

/* =======================================================================
 *   STATE HANDLER FUNCTIONS
 * ======================================================================== */

/* -------------------------------------------------- */
/* STATE_IDLE                                         */
/* -------------------------------------------------- */
static void stateIdle_onTick(void) {
  setVel2(0, 0);
  if (startButton()) {
    restartMission();
  }
}

/* -------------------------------------------------- */
/* STATE_FOLLOW_LINE                                  */
/* -------------------------------------------------- */
static void stateFollowLine_onTick(void) {
  bool lost;
  lineFollower_centerOnLine(s_ground, &lost);

  bool intersection = (s_ground == SENSOR_ALL);
  bool leftPath =
      ((s_ground & SENSOR_LEFT) == SENSOR_LEFT) && (s_ground != SENSOR_ALL);
  bool rightPath = ((s_ground & SENSOR_RIGHT) == SENSOR_RIGHT);

  if (lost) {
    s_lostTickCount++;
    s_leftCornerTicks = 0;
    s_rightCornerTicks = 0;
    if (s_lostTickCount < LOST_TICKS) {
      /* Spin search: rotate in place */
      setVel2(-RECOVERY_SPEED, RECOVERY_SPEED);
    } else {
      /* Timeout — deadend */
      setVel2(0, 0);
      s_detectedNodeType = NODE_TYPE_DEADEND;
      log_cornerDetect(s_tick, "deadend", s_ground);
      changeState(&s_stateApproachCenter, "DEADEND");
    }
    return;
  }

  /* Line reacquired */
  s_lostTickCount = 0;

  if (leftPath) {
    s_leftCornerTicks++;
    if (s_leftCornerTicks >= CORNER_TICKS) {
      s_rightCornerTicks = 0;
      s_detectedNodeType = NODE_TYPE_LEFT_CORNER;
      log_cornerDetect(s_tick, "left_corner", s_ground);
      changeState(&s_stateApproachCenter, "LEFT_CORNER");
      return;
    }
  } else {
    s_leftCornerTicks = 0;
    if (intersection) {
      /* Could be intersection or target — verify */
      s_rightCornerTicks = 0;
      log_groundDetection(s_tick, "intersection", s_ground);
      changeState(&s_stateVerifyTarget, NULL);
      return;
    }
  }

  if (rightPath) {
    s_rightCornerTicks++;
    if (s_rightCornerTicks >= CORNER_TICKS) {
      s_leftCornerTicks = 0;
      s_detectedNodeType = NODE_TYPE_RIGHT_CORNER;
      log_cornerDetect(s_tick, "right_corner", s_ground);
      changeState(&s_stateApproachCenter, "RIGHT_CORNER");
      return;
    }
  } else {
    s_rightCornerTicks = 0;
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
    dist = sqrt(pow(x - s_verifyStartX, 2) + pow(y - s_verifyStartY, 2));

    printf("Verify Target = IN PROGRESS, dist = %f\n", dist);
    if (dist >= TARGET_DIST_THRESHOLD) {
      /* Confirmed target */
      changeState(&s_stateAtTarget, NULL);
    } else {
      /* Keep driving straight over the wide area */
      setVel2(BASE_SPEED, BASE_SPEED);
    }
  } else {
    printf("Verify Target = NOT TARGET, dist = %f\n", dist);
    /* Pattern broke — this was just an intersection */
    s_detectedNodeType = NODE_TYPE_INTERSECTION;
    changeState(&s_stateApproachCenter, "INTERSECTION");
  }
}

/* -------------------------------------------------- */
/* STATE_AT_TARGET                                    */
/* -------------------------------------------------- */
static void stateAtTarget_onTick(void) {
  setVel2(0, 0);
  getRobotPos(&s_targetX, &s_targetY, &s_targetH);
  log_target(s_tick, s_targetX, s_targetY, s_targetH);

  /* Record target in graph */
  unsigned char currentNode = navGraph_getCurrentNode();
  navGraph_setTargetNode(currentNode);
  navGraph_setNodeTarget(currentNode);
  changeState(&s_stateChooseNext, "CONTINUE_EXPLORE");
}

/* -------------------------------------------------- */
/* STATE_APPROACH_CENTER                              */
/* -------------------------------------------------- */
static void stateApproachCenter_onEnter(void) {
  double h;
  getRobotPos(&s_approachStartX, &s_approachStartY, &h);
  s_approachStartH = h;
}

static void stateApproachCenter_onTick(void) {
  double x, y, h;
  double dist;

  getRobotPos(&x, &y, &h);
  dist = sqrt(pow(x - s_approachStartX, 2) + pow(y - s_approachStartY, 2));

  if (dist < INTERSECTION_CENTER_DIST) {
    setVel2(BASE_SPEED, BASE_SPEED);
  } else {
    printf("Reached center. Dist = %f\n", dist);

    /* Reached center — create/merge node and link edge */
    setVel2(0, 0);

    Pose pose = {x, y, h};
    char existingId = navGraph_findNodeAt(&pose, NODE_MERGE_TOLERANCE);
    unsigned char currentNode;
    bool isNewNode = false;

    if (existingId >= 0) {
      currentNode = (unsigned char)existingId;
      navGraph_setCurrentNode(currentNode);
    } else {
      if (!navGraph_addNode(&pose, s_detectedNodeType, &currentNode)) {
        s_doneReason = "GRAPH_FULL";
        changeState(&s_stateDone, "GRAPH_FULL");
        return;
      }
      navGraph_setCurrentNode(currentNode);
      isNewNode = true;
      log_nodeAdd(s_tick, currentNode, x, y,
                  nodeTypeToString(s_detectedNodeType));
    }

    /* Link edge from previous node to current node */
    if (s_prevNodeId != currentNode) {
      const GraphNode *prevNode = navGraph_getNode(s_prevNodeId);
      if (prevNode) {
        double edgeDist = hypot(x - prevNode->pose.x, y - prevNode->pose.y);
        unsigned char edgeId = 0xFF;
        bool edgeCreated = navGraph_addEdge(s_prevNodeId, currentNode,
                                            (float)edgeDist, &edgeId);
        if (edgeCreated) {
          log_edgeAdd(s_tick, edgeId, s_prevNodeId, currentNode,
                      (float)edgeDist);
          navGraph_setEdgeExplored(edgeId);
          log_edgeExplored(s_tick, edgeId);
        }
      }
    }

    s_prevNodeId = currentNode;
    changeState(&s_stateChooseNext, NULL);
  }
}

/* -------------------------------------------------- */
/* STATE_CHOOSE_NEXT                                  */
/* -------------------------------------------------- */
static void stateChooseNext_onTick(void) {
  unsigned char currentNode = navGraph_getCurrentNode();

  /* Check if exploration is complete */
  if (currentNode == 0 && navGraph_allEdgesExplored() &&
      navGraph_getTargetNode() != 0xFF) {
    log_explorationDone(s_tick, navGraph_getNodeCount(),
                        navGraph_getEdgeCount());
    startReturnNavigation();
    return;
  }

  const GraphNode *node = navGraph_getNode(currentNode);
  if (!node) {
    s_doneReason = "INVALID_NODE";
    changeState(&s_stateDone, "INVALID_NODE");
    return;
  }

  /* Determine available directions using node type as a hint */
  bool hasLeft = false, hasStraight = false, hasRight = false;
  switch (node->type) {
  case NODE_TYPE_INTERSECTION:
    hasLeft = true;
    hasStraight = true;
    hasRight = true;
    break;
  case NODE_TYPE_LEFT_CORNER:
    hasLeft = true;
    hasStraight = true;
    break;
  case NODE_TYPE_RIGHT_CORNER:
    hasRight = true;
    hasStraight = true;
    break;
  case NODE_TYPE_DEADEND:
    break;
  }

  /* Check ALL edges of current node for unexplored status regardless of node
     type. Node type is only used as a hint for which new directions are
     available. */
  double currentHeading = s_robotH;
  double tolerance = 0.6; /* ~35 degrees */
  char edgeId;

  /* Check left */
  edgeId = navGraph_getEdgeInDirection(currentNode, currentHeading, PI / 2.0,
                                       tolerance);
  if (edgeId < 0) {
    if (hasLeft) {
      s_chosenDeltaAngle = PI / 2.0;
      changeState(&s_stateTurnToEdge, "LEFT");
      return;
    }
  } else if (!navGraph_isEdgeExplored((unsigned char)edgeId)) {
    s_chosenDeltaAngle = PI / 2.0;
    changeState(&s_stateTurnToEdge, "LEFT");
    return;
  }

  /* Check straight */
  edgeId =
      navGraph_getEdgeInDirection(currentNode, currentHeading, 0.0, tolerance);
  if (edgeId < 0) {
    if (hasStraight) {
      s_chosenDeltaAngle = 0.0;
      changeState(&s_stateTurnToEdge, "STRAIGHT");
      return;
    }
  } else if (!navGraph_isEdgeExplored((unsigned char)edgeId)) {
    s_chosenDeltaAngle = 0.0;
    changeState(&s_stateTurnToEdge, "STRAIGHT");
    return;
  }

  /* Check right */
  edgeId = navGraph_getEdgeInDirection(currentNode, currentHeading, -PI / 2.0,
                                       tolerance);
  if (edgeId < 0) {
    if (hasRight) {
      s_chosenDeltaAngle = -PI / 2.0;
      changeState(&s_stateTurnToEdge, "RIGHT");
      return;
    }
  } else if (!navGraph_isEdgeExplored((unsigned char)edgeId)) {
    s_chosenDeltaAngle = -PI / 2.0;
    changeState(&s_stateTurnToEdge, "RIGHT");
    return;
  }

  /* Go back */
  s_chosenDeltaAngle = PI;
  changeState(&s_stateTurnToEdge, "BACK");
}

/* -------------------------------------------------- */
/* STATE_TURN_TO_EDGE                                 */
/* -------------------------------------------------- */
static void stateTurnToEdge_onEnter(void) {
  rotation_start(s_robotH, s_chosenDeltaAngle);
  log_turnStart(s_tick, normalizeAngle(s_robotH + s_chosenDeltaAngle));
}

static void stateTurnToEdge_onTick(void) {
  rotation_tick(s_robotH);
  if (rotation_done(s_robotH)) {
    log_turnDone(s_tick);
    changeState(&s_stateFollowLine, NULL);
  }
}

static void stateTurnToEdge_onExit(void) { rotation_stop(); }

/* -------------------------------------------------- */
/* STATE_RETURN_TURN                                  */
/* -------------------------------------------------- */
static void stateReturnTurn_onEnter(void) {
  if (s_returnPathIndex + 1 >= s_returnPathLen) {
    /* No more nodes to turn toward */
    changeState(&s_stateReturnAtNode, "PATH_END");
    return;
  }

  unsigned char currentNode = s_returnPath[s_returnPathIndex];
  unsigned char nextNode = s_returnPath[s_returnPathIndex + 1];
  const GraphNode *cn = navGraph_getNode(currentNode);
  const GraphNode *nn = navGraph_getNode(nextNode);

  if (!cn || !nn) {
    s_doneReason = "RETURN_INVALID_NODE";
    changeState(&s_stateDone, "RETURN_INVALID_NODE");
    return;
  }

  double deltaAngle = normalizeAngle(
      atan2(nn->pose.y - cn->pose.y, nn->pose.x - cn->pose.x) - s_robotH);
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

static void stateReturnTurn_onExit(void) { rotation_stop(); }

/* -------------------------------------------------- */
/* STATE_RETURN_FOLLOW                                */
/* -------------------------------------------------- */
static void stateReturnFollow_onTick(void) {
  double x, y, h;
  double dist;
  bool lost;

  if (s_returnPathIndex + 1 >= s_returnPathLen) {
    changeState(&s_stateReturnAtNode, "PATH_END");
    return;
  }

  unsigned char nextNode = s_returnPath[s_returnPathIndex + 1];
  const GraphNode *nn = navGraph_getNode(nextNode);
  if (!nn) {
    s_doneReason = "RETURN_INVALID_NODE";
    changeState(&s_stateDone, "RETURN_INVALID_NODE");
    return;
  }

  lineFollower_centerOnLine(s_ground, &lost);

  if (lost) {
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

  /* Check distance to next node */
  getRobotPos(&x, &y, &h);
  dist = hypot(nn->pose.x - x, nn->pose.y - y);

  if (dist < ORIGIN_TOLERANCE) {
    setVel2(0, 0);
    changeState(&s_stateReturnAtNode, NULL);
  }
}

/* -------------------------------------------------- */
/* STATE_RETURN_AT_NODE                               */
/* -------------------------------------------------- */
static void stateReturnAtNode_onTick(void) {
  s_returnPathIndex++;

  if (s_returnPathIndex >= s_returnPathLen - 1) {
    /* Reached end of current leg */
    if (s_returnToTarget) {
      /* Finished origin -> target, now run target -> origin */
      unsigned char targetNode = navGraph_getTargetNode();
      if (navGraph_dijkstra(targetNode, 0, s_returnPath, &s_returnPathLen)) {
        s_returnPathIndex = 0;
        s_returnToTarget = false;
        log_dijkstraPath(s_tick, s_returnPath, s_returnPathLen);
        changeState(&s_stateReturnTurn, "LEG2_START");
      } else {
        s_doneReason = "NO_PATH_HOME";
        changeState(&s_stateDone, "NO_PATH_HOME");
      }
    } else {
      /* Finished target -> origin, mission complete */
      log_missionDone(s_tick, s_targetX, s_targetY, s_targetH,
                      navGraph_getNodeCount(), s_tick);
      changeState(&s_stateDone, NULL);
    }
    return;
  }

  /* More nodes to visit on this leg */
  changeState(&s_stateReturnTurn, NULL);
}

/* -------------------------------------------------- */
/* STATE_DONE                                         */
/* -------------------------------------------------- */
static void stateDone_onTick(void) {
  setVel2(0, 0);
  if (startButton()) {
    restartMission();
  }
}

/* =======================================================================
 *   HELPER FUNCTIONS
 * ======================================================================== */

static void restartMission(void) {
  setRobotPos(0.0, 0.0, 0.0);
  navGraph_reset();
  s_lostTickCount = 0;
  s_leftCornerTicks = 0;
  s_rightCornerTicks = 0;
  rotation_stop();
  s_targetX = 0.0;
  s_targetY = 0.0;
  s_targetH = 0.0;
  s_verifyStartX = 0.0;
  s_verifyStartY = 0.0;
  s_prevNodeId = 0;
  s_returnPathLen = 0;
  s_returnPathIndex = 0;
  s_returnToTarget = true;
  s_doneReason = NULL;

  /* Create origin node */
  Pose origin = {0.0, 0.0, 0.0};
  unsigned char originId;
  navGraph_addNode(&origin, NODE_TYPE_INTERSECTION, &originId);
  navGraph_setCurrentNode(originId);
  s_prevNodeId = originId;
  log_nodeAdd(s_tick, originId, 0.0, 0.0, "ORIGIN");

  changeState(&s_stateFollowLine, NULL);
}

static void startReturnNavigation(void) {
  unsigned char targetNode = navGraph_getTargetNode();
  if (targetNode == 0xFF ||
      !navGraph_dijkstra(0, targetNode, s_returnPath, &s_returnPathLen)) {
    s_doneReason = "NO_PATH_TO_TARGET";
    changeState(&s_stateDone, "NO_PATH_TO_TARGET");
    return;
  }
  s_returnPathIndex = 0;
  s_returnToTarget = true;
  log_dijkstraPath(s_tick, s_returnPath, s_returnPathLen);
  changeState(&s_stateReturnTurn, "RETURN_START");
}

/* =======================================================================
 *   PUBLIC API
 * ======================================================================== */

void stateMachine_init(void) {
  s_currentState = &s_stateIdle;
  s_phase = PHASE_EXPLORING;
  s_doneReason = NULL;
}

void stateMachine_start(void) { restartMission(); }

void stateMachine_tick(unsigned int tick, unsigned int ground, double x,
                       double y, double h) {
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

Phase stateMachine_getPhase(void) { return s_phase; }

const char *stateMachine_getDoneReason(void) { return s_doneReason; }
