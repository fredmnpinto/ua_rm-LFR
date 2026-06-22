#ifndef NAV_GRAPH_H
#define NAV_GRAPH_H

#include "config.h"
#include "rm-mr32.h"

/* ========================================================================
 *   NODE TYPE
 * ======================================================================== */

typedef enum {
  NODE_TYPE_INTERSECTION,
  NODE_TYPE_LEFT_CORNER,
  NODE_TYPE_RIGHT_CORNER,
  NODE_TYPE_DEADEND
} NodeType;

/* ========================================================================
 *   FLAGS
 * ======================================================================== */

#define NODE_FLAG_IS_TARGET 0x01
#define EDGE_FLAG_EXPLORED 0x01

/* ========================================================================
 *   DATA STRUCTURES
 * ======================================================================== */

typedef struct {
  Pose pose;
  unsigned char edges[4];
  unsigned char edgeCount;
  unsigned char flags;
  NodeType type;
} GraphNode;

typedef struct {
  float distance;
  unsigned char nodeA;
  unsigned char nodeB;
  unsigned char flags;
} Edge;

/* ========================================================================
 *   PUBLIC API
 * ======================================================================== */

/**
 * Initialise the navigation graph (calls navGraph_reset).
 */
void navGraph_init(void);

/**
 * Reset the navigation graph, clearing all nodes and edges.
 */
void navGraph_reset(void);

/**
 * Add a new node to the graph.
 * @param pose      Pose (x, y, heading) of the node.
 * @param type      Type of node (intersection, corner, deadend).
 * @param outNodeId Output: ID assigned to the new node.
 * @return true if the node was added, false if the graph is full.
 */
bool navGraph_addNode(const Pose *pose, NodeType type,
                      unsigned char *outNodeId);

/**
 * Find an existing node near the given pose.
 * @param pose      Pose to search near.
 * @param tolerance Distance threshold for matching (metres).
 * @return Node ID if found, -1 otherwise.
 */
char navGraph_findNodeAt(const Pose *pose, float tolerance);

/**
 * Add an undirected edge between two nodes.
 * @param nodeA      First node ID.
 * @param nodeB      Second node ID.
 * @param distance   Edge length in metres.
 * @param outEdgeId  Output: ID assigned to the new edge.
 * @return true if the edge was added or already exists, false on error.
 */
bool navGraph_addEdge(unsigned char nodeA, unsigned char nodeB, float distance,
                      unsigned char *outEdgeId);

/**
 * Find an existing edge between two nodes.
 * @param nodeA First node ID.
 * @param nodeB Second node ID.
 * @return Edge ID if found, -1 otherwise.
 */
char navGraph_findEdge(unsigned char nodeA, unsigned char nodeB);

/**
 * Mark an edge as explored.
 * @param edgeId ID of the edge to mark.
 */
void navGraph_setEdgeExplored(unsigned char edgeId);

/**
 * Check whether an edge has been explored.
 * @param edgeId ID of the edge to check.
 * @return true if explored, false otherwise.
 */
bool navGraph_isEdgeExplored(unsigned char edgeId);

/**
 * Get the ID of the current node (robot position).
 * @return Current node ID.
 */
unsigned char navGraph_getCurrentNode(void);

/**
 * Set the current node ID.
 * @param nodeId ID of the node to set as current.
 */
void navGraph_setCurrentNode(unsigned char nodeId);

/**
 * Get the total number of nodes in the graph.
 * @return Node count.
 */
unsigned char navGraph_getNodeCount(void);

/**
 * Get the total number of edges in the graph.
 * @return Edge count.
 */
unsigned char navGraph_getEdgeCount(void);

/**
 * Check whether all edges in the graph have been explored.
 * @return true if all edges are explored, false otherwise.
 */
bool navGraph_allEdgesExplored(void);

/**
 * Set the target node for return navigation.
 * @param nodeId ID of the target node.
 */
void navGraph_setTargetNode(unsigned char nodeId);

/**
 * Get the target node ID.
 * @return Target node ID, or 0xFF if not set.
 */
unsigned char navGraph_getTargetNode(void);

/**
 * Mark a node as the mission target.
 * @param nodeId ID of the node to mark.
 */
void navGraph_setNodeTarget(unsigned char nodeId);

/**
 * Compute the shortest path between two nodes using Dijkstra's algorithm.
 * @param fromNode    Starting node ID.
 * @param toNode      Destination node ID.
 * @param outPath     Output array to store the path (node IDs).
 * @param outPathLen  Output: length of the path.
 * @return true if a path was found, false otherwise.
 */
bool navGraph_dijkstra(unsigned char fromNode, unsigned char toNode,
                       unsigned char *outPath, unsigned char *outPathLen);

/**
 * Get a pointer to a node by ID.
 * @param nodeId ID of the node.
 * @return Pointer to the GraphNode, or NULL if invalid ID.
 */
const GraphNode *navGraph_getNode(unsigned char nodeId);

/**
 * Find an edge leaving a node in a specific direction.
 * @param nodeId        ID of the node to search from.
 * @param currentHeading Current robot heading (radians).
 * @param desiredDelta   Desired heading change (radians).
 * @param tolerance      Angular tolerance (radians).
 * @return Edge ID if found, -1 otherwise.
 */
char navGraph_getEdgeInDirection(unsigned char nodeId, double currentHeading,
                                 double desiredDelta, double tolerance);

#endif /* NAV_GRAPH_H */
