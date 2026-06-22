#ifndef LOGGING_H
#define LOGGING_H

#include "config.h"
#include "rm-mr32.h"

/**
 * Log a state machine transition.
 * @param tick   Current tick counter.
 * @param from   Name of the previous state.
 * @param to     Name of the new state.
 * @param detail Optional detail string (may be NULL).
 */
void log_transition(unsigned int tick, const char *from, const char *to, const char *detail);

/**
 * Log detection of the mission target.
 * @param tick Current tick counter.
 * @param x    Target X position (metres).
 * @param y    Target Y position (metres).
 * @param h    Target heading (radians).
 */
void log_target(unsigned int tick, double x, double y, double h);

/**
 * Log a graph push event (entering a new node during exploration).
 * @param tick  Current tick counter.
 * @param depth Current recursion/stack depth.
 * @param x     Node X position (metres).
 * @param y     Node Y position (metres).
 * @param h     Node heading (radians).
 */
void log_push(unsigned int tick, int depth, double x, double y, double h);

/**
 * Log a graph pop event (backtracking during exploration).
 * @param tick  Current tick counter.
 * @param depth Current recursion/stack depth.
 */
void log_pop(unsigned int tick, int depth);

/**
 * Log the start of an in-place turn.
 * @param tick        Current tick counter.
 * @param targetAngle Desired heading after the turn (radians).
 */
void log_turnStart(unsigned int tick, double targetAngle);

/**
 * Log completion of an in-place turn.
 * @param tick Current tick counter.
 */
void log_turnDone(unsigned int tick);

/**
 * Log a line-lost timeout event.
 * @param tick Current tick counter.
 */
void log_lostTimeout(unsigned int tick);

/**
 * Log ground sensor detection of a feature.
 * @param tick   Current tick counter.
 * @param type   Feature type string (e.g. "intersection").
 * @param ground 5-bit ground sensor pattern.
 */
void log_groundDetection(unsigned int tick, const char *type, unsigned int ground);

/**
 * Log mission completion.
 * @param tick  Current tick counter.
 * @param tx    Target X position (metres).
 * @param ty    Target Y position (metres).
 * @param th    Target heading (radians).
 * @param depth Final graph depth.
 * @param ticks Total elapsed ticks.
 */
void log_missionDone(unsigned int tick, double tx, double ty, double th, int depth, unsigned int ticks);

/* Graph event logging */

/**
 * Log addition of a new graph node.
 * @param tick   Current tick counter.
 * @param nodeId ID assigned to the new node.
 * @param x      Node X position (metres).
 * @param y      Node Y position (metres).
 * @param type   Node type string.
 */
void log_nodeAdd(unsigned int tick, unsigned char nodeId, double x, double y, const char *type);

/**
 * Log addition of a new graph edge.
 * @param tick     Current tick counter.
 * @param edgeId   ID assigned to the new edge.
 * @param nodeA    First node ID.
 * @param nodeB    Second node ID.
 * @param distance Edge length (metres).
 */
void log_edgeAdd(unsigned int tick, unsigned char edgeId, unsigned char nodeA, unsigned char nodeB, float distance);

/**
 * Log that an edge has been explored.
 * @param tick   Current tick counter.
 * @param edgeId ID of the explored edge.
 */
void log_edgeExplored(unsigned int tick, unsigned char edgeId);

/**
 * Log detection of a corner feature.
 * @param tick   Current tick counter.
 * @param type   Corner type string (e.g. "left_corner").
 * @param ground 5-bit ground sensor pattern.
 */
void log_cornerDetect(unsigned int tick, const char *type, unsigned int ground);

/**
 * Log a Dijkstra-computed return path.
 * @param tick     Current tick counter.
 * @param path     Array of node IDs forming the path.
 * @param pathLen  Length of the path array.
 */
void log_dijkstraPath(unsigned int tick, const unsigned char *path, unsigned char pathLen);

/**
 * Log completion of the exploration phase.
 * @param tick      Current tick counter.
 * @param nodeCount Total number of nodes in the graph.
 * @param edgeCount Total number of edges in the graph.
 */
void log_explorationDone(unsigned int tick, unsigned char nodeCount, unsigned char edgeCount);

#endif /* LOGGING_H */
