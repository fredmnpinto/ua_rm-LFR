#include "nav_graph.h"
#include "rm-mr32.h"
#include <math.h>

/* =======================================================================
 *   STATIC STORAGE (no dynamic allocation)
 * ======================================================================== */

static GraphNode s_nodes[MAX_NODES];
static Edge s_edges[MAX_EDGES];
static unsigned char s_nodeCount = 0;
static unsigned char s_edgeCount = 0;
static unsigned char s_currentNode = 0;
static unsigned char s_targetNode = 0xFF;

/* =======================================================================
 *   INITIALISATION
 * ======================================================================== */

void navGraph_init(void) { navGraph_reset(); }

void navGraph_reset(void) {
  s_nodeCount = 0;
  s_edgeCount = 0;
  s_currentNode = 0;
  s_targetNode = 0xFF;
}

/* =======================================================================
 *   NODE OPERATIONS
 * ======================================================================== */

bool navGraph_addNode(const Pose *pose, NodeType type,
                      unsigned char *outNodeId) {
  unsigned char id;
  if (s_nodeCount >= MAX_NODES) {
    return false;
  }
  id = s_nodeCount;
  s_nodes[id].pose = *pose;
  s_nodes[id].edgeCount = 0;
  s_nodes[id].flags = 0;
  s_nodes[id].type = type;
  s_nodeCount++;
  if (outNodeId) {
    *outNodeId = id;
  }
  return true;
}

char navGraph_findNodeAt(const Pose *pose, float tolerance) {
  unsigned char i;
  for (i = 0; i < s_nodeCount; i++) {
    double dx = s_nodes[i].pose.x - pose->x;
    double dy = s_nodes[i].pose.y - pose->y;
    double dist = sqrt(pow(dx, 2) + pow(dy, 2));
    if (dist <= tolerance) {
      printf("Found existing node at distance of {%f}\n", dist);
      return (char)i;
    }
  }
  return -1;
}

const GraphNode *navGraph_getNode(unsigned char nodeId) {
  if (nodeId < s_nodeCount) {
    return &s_nodes[nodeId];
  }
  return 0;
}

unsigned char navGraph_getNodeCount(void) { return s_nodeCount; }

unsigned char navGraph_getEdgeCount(void) { return s_edgeCount; }

/* =======================================================================
 *   EDGE OPERATIONS
 * ======================================================================== */

bool navGraph_addEdge(unsigned char nodeA, unsigned char nodeB, float distance,
                      unsigned char *outEdgeId) {
  char existing;
  unsigned char id;
  if (s_edgeCount >= MAX_EDGES || nodeA >= s_nodeCount ||
      nodeB >= s_nodeCount) {
    return false;
  }
  if (nodeA == nodeB) {
    return false;
  }
  /* Check if edge already exists */
  existing = navGraph_findEdge(nodeA, nodeB);
  if (existing >= 0) {
    if (outEdgeId) {
      *outEdgeId = (unsigned char)existing;
    }
    return true;
  }
  id = s_edgeCount;
  s_edges[id].nodeA = nodeA;
  s_edges[id].nodeB = nodeB;
  s_edges[id].distance = distance;
  s_edges[id].flags = 0;
  s_edgeCount++;

  /* Link edge to both nodes */
  if (s_nodes[nodeA].edgeCount < 4) {
    s_nodes[nodeA].edges[s_nodes[nodeA].edgeCount++] = id;
  }
  if (s_nodes[nodeB].edgeCount < 4) {
    s_nodes[nodeB].edges[s_nodes[nodeB].edgeCount++] = id;
  }

  if (outEdgeId) {
    *outEdgeId = id;
  }
  return true;
}

char navGraph_findEdge(unsigned char nodeA, unsigned char nodeB) {
  unsigned char i;
  for (i = 0; i < s_edgeCount; i++) {
    if ((s_edges[i].nodeA == nodeA && s_edges[i].nodeB == nodeB) ||
        (s_edges[i].nodeA == nodeB && s_edges[i].nodeB == nodeA)) {
      return (char)i;
    }
  }
  return -1;
}

void navGraph_setEdgeExplored(unsigned char edgeId) {
  if (edgeId < s_edgeCount) {
    s_edges[edgeId].flags |= EDGE_FLAG_EXPLORED;
  }
}

bool navGraph_isEdgeExplored(unsigned char edgeId) {
  if (edgeId < s_edgeCount) {
    return (s_edges[edgeId].flags & EDGE_FLAG_EXPLORED) != 0;
  }
  return false;
}

bool navGraph_allEdgesExplored(void) {
  unsigned char i;
  for (i = 0; i < s_edgeCount; i++) {
    if (!navGraph_isEdgeExplored(i)) {
      return false;
    }
  }
  return true;
}

/* =======================================================================
 *   CURRENT / TARGET NODE
 * ======================================================================== */

unsigned char navGraph_getCurrentNode(void) { return s_currentNode; }

void navGraph_setCurrentNode(unsigned char nodeId) {
  if (nodeId < s_nodeCount) {
    s_currentNode = nodeId;
  }
}

void navGraph_setTargetNode(unsigned char nodeId) {
  if (nodeId < s_nodeCount) {
    s_targetNode = nodeId;
  }
}

unsigned char navGraph_getTargetNode(void) { return s_targetNode; }

void navGraph_setNodeTarget(unsigned char nodeId) {
  if (nodeId < s_nodeCount) {
    s_nodes[nodeId].flags |= NODE_FLAG_IS_TARGET;
    s_nodes[nodeId].type = NODE_TYPE_DEADEND;
  }
}

/* =======================================================================
 *   DIRECTION HELPER
 * ======================================================================== */

char navGraph_getEdgeInDirection(unsigned char nodeId, double currentHeading,
                                 double desiredDelta, double tolerance) {
  const GraphNode *node;
  unsigned char i;
  if (nodeId >= s_nodeCount) {
    return -1;
  }
  node = &s_nodes[nodeId];
  for (i = 0; i < node->edgeCount; i++) {
    unsigned char edgeId = node->edges[i];
    const Edge *edge = &s_edges[edgeId];
    unsigned char otherNodeId =
        (edge->nodeA == nodeId) ? edge->nodeB : edge->nodeA;
    double dx = s_nodes[otherNodeId].pose.x - node->pose.x;
    double dy = s_nodes[otherNodeId].pose.y - node->pose.y;
    double edgeHeading = atan2(dy, dx);
    double delta = normalizeAngle(edgeHeading - currentHeading);
    double err = normalizeAngle(delta - desiredDelta);
    if (fabs(err) < tolerance) {
      return (char)edgeId;
    }
  }
  return -1;
}

/* =======================================================================
 *   DIJKSTRA (O(V^2) linear scan, V <= 32)
 * ======================================================================== */

bool navGraph_dijkstra(unsigned char fromNode, unsigned char toNode,
                       unsigned char *outPath, unsigned char *outPathLen) {
  static float dist[MAX_NODES];
  static char parent[MAX_NODES];
  static bool visited[MAX_NODES];
  static unsigned char revPath[MAX_PATH_LENGTH];
  unsigned char i;
  unsigned char count;
  unsigned char u;
  float minDist;
  unsigned char revLen;
  unsigned char cur;

  if (fromNode >= s_nodeCount || toNode >= s_nodeCount || outPath == 0 ||
      outPathLen == 0) {
    return false;
  }
  if (fromNode == toNode) {
    outPath[0] = fromNode;
    *outPathLen = 1;
    return true;
  }

  for (i = 0; i < s_nodeCount; i++) {
    dist[i] = 1e9f;
    parent[i] = -1;
    visited[i] = false;
  }
  dist[fromNode] = 0.0f;

  for (count = 0; count < s_nodeCount; count++) {
    /* Find unvisited node with minimum distance */
    u = 0xFF;
    minDist = 1e9f;
    for (i = 0; i < s_nodeCount; i++) {
      if (!visited[i] && dist[i] < minDist) {
        minDist = dist[i];
        u = i;
      }
    }
    if (u == 0xFF) {
      break; /* No reachable nodes remain */
    }
    visited[u] = true;

    if (u == toNode) {
      break; /* Reached destination */
    }

    /* Relax edges of u */
    {
      const GraphNode *node = &s_nodes[u];
      for (i = 0; i < node->edgeCount; i++) {
        unsigned char edgeId = node->edges[i];
        const Edge *edge = &s_edges[edgeId];
        unsigned char v = (edge->nodeA == u) ? edge->nodeB : edge->nodeA;
        float w = edge->distance;
        if (visited[v]) {
          continue;
        }
        if (dist[u] + w < dist[v]) {
          dist[v] = dist[u] + w;
          parent[v] = (char)u;
        }
      }
    }
  }

  if (parent[toNode] < 0) {
    return false; /* No path found */
  }

  /* Reconstruct path into temporary array (reverse order) */
  revLen = 0;
  cur = toNode;
  while (cur != fromNode && revLen < MAX_PATH_LENGTH) {
    revPath[revLen++] = cur;
    cur = (unsigned char)parent[cur];
  }
  if (revLen >= MAX_PATH_LENGTH) {
    return false;
  }
  revPath[revLen++] = fromNode;

  /* Reverse into output */
  for (i = 0; i < revLen; i++) {
    outPath[i] = revPath[revLen - 1 - i];
  }
  *outPathLen = revLen;
  return true;
}
