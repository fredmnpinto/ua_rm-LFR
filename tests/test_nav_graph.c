#include <stdio.h>
#include <math.h>
#include <string.h>
#include "rm-mr32.h"   /* Mock rm-mr32.h must be included before nav_graph.h */
#include "nav_graph.h"

#define TEST_ASSERT(cond) do { \
    if (!(cond)) { \
        printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        return 1; \
    } \
} while(0)

#define TEST_ASSERT_EQ(expected, actual) do { \
    if ((expected) != (actual)) { \
        printf("FAIL: %s:%d: expected %d, got %d\n", __FILE__, __LINE__, (int)(expected), (int)(actual)); \
        return 1; \
    } \
} while(0)

#define TEST_ASSERT_FLOAT_EQ(expected, actual, tol) do { \
    if (fabs((expected) - (actual)) > (tol)) { \
        printf("FAIL: %s:%d: expected %f, got %f\n", __FILE__, __LINE__, (double)(expected), (double)(actual)); \
        return 1; \
    } \
} while(0)

static int test_node_creation(void) {
    navGraph_reset();

    Pose p1 = {0.0, 0.0, 0.0};
    unsigned char id1;
    TEST_ASSERT(navGraph_addNode(&p1, NODE_TYPE_INTERSECTION, &id1));
    TEST_ASSERT_EQ(0, id1);
    TEST_ASSERT_EQ(1, navGraph_getNodeCount());

    const GraphNode *n1 = navGraph_getNode(id1);
    TEST_ASSERT(n1 != NULL);
    TEST_ASSERT_FLOAT_EQ(0.0, n1->pose.x, 0.001);
    TEST_ASSERT_EQ(NODE_TYPE_INTERSECTION, n1->type);

    return 0;
}

static int test_node_merge(void) {
    navGraph_reset();

    Pose p1 = {0.0, 0.0, 0.0};
    unsigned char id1;
    navGraph_addNode(&p1, NODE_TYPE_INTERSECTION, &id1);

    Pose p2 = {0.05, 0.0, 0.0}; /* 5cm away, within NODE_MERGE_TOLERANCE (0.10m) */
    char found = navGraph_findNodeAt(&p2, NODE_MERGE_TOLERANCE);
    TEST_ASSERT_EQ(0, found);

    Pose p3 = {0.15, 0.0, 0.0}; /* 15cm away, outside tolerance */
    found = navGraph_findNodeAt(&p3, NODE_MERGE_TOLERANCE);
    TEST_ASSERT_EQ(-1, found);

    return 0;
}

static int test_edge_linking(void) {
    navGraph_reset();

    Pose p1 = {0.0, 0.0, 0.0};
    Pose p2 = {1.0, 0.0, 0.0};
    unsigned char id1, id2;

    navGraph_addNode(&p1, NODE_TYPE_INTERSECTION, &id1);
    navGraph_addNode(&p2, NODE_TYPE_INTERSECTION, &id2);

    unsigned char edgeId;
    TEST_ASSERT(navGraph_addEdge(id1, id2, 1.0f, &edgeId));
    TEST_ASSERT_EQ(0, edgeId);
    TEST_ASSERT_EQ(1, navGraph_getEdgeCount());

    /* Duplicate edge should return existing */
    unsigned char edgeId2;
    TEST_ASSERT(navGraph_addEdge(id1, id2, 1.0f, &edgeId2));
    TEST_ASSERT_EQ(0, edgeId2); /* same edge */
    TEST_ASSERT_EQ(1, navGraph_getEdgeCount()); /* no new edge */

    /* Self-loop should fail */
    TEST_ASSERT(!navGraph_addEdge(id1, id1, 0.0f, &edgeId2));

    /* Check edge is linked to both nodes */
    const GraphNode *n1 = navGraph_getNode(id1);
    TEST_ASSERT_EQ(1, n1->edgeCount);
    TEST_ASSERT_EQ(0, n1->edges[0]);

    return 0;
}

static int test_dijkstra_simple(void) {
    navGraph_reset();

    /* Create a simple line: 0 -- 1 -- 2 */
    Pose p0 = {0.0, 0.0, 0.0};
    Pose p1 = {1.0, 0.0, 0.0};
    Pose p2 = {2.0, 0.0, 0.0};
    unsigned char id0, id1, id2;

    navGraph_addNode(&p0, NODE_TYPE_INTERSECTION, &id0);
    navGraph_addNode(&p1, NODE_TYPE_INTERSECTION, &id1);
    navGraph_addNode(&p2, NODE_TYPE_INTERSECTION, &id2);

    navGraph_addEdge(id0, id1, 1.0f, NULL);
    navGraph_addEdge(id1, id2, 1.0f, NULL);

    unsigned char path[32];
    unsigned char pathLen;

    /* 0 -> 2 */
    TEST_ASSERT(navGraph_dijkstra(id0, id2, path, &pathLen));
    TEST_ASSERT_EQ(3, pathLen);
    TEST_ASSERT_EQ(0, path[0]);
    TEST_ASSERT_EQ(1, path[1]);
    TEST_ASSERT_EQ(2, path[2]);

    /* 0 -> 0 (same node) */
    TEST_ASSERT(navGraph_dijkstra(id0, id0, path, &pathLen));
    TEST_ASSERT_EQ(1, pathLen);
    TEST_ASSERT_EQ(0, path[0]);

    return 0;
}

static int test_dijkstra_branches(void) {
    navGraph_reset();

    /* Create a T-junction: 0 -- 1 -- 2
     *                          |
     *                          3
     */
    Pose p0 = {0.0, 0.0, 0.0};
    Pose p1 = {1.0, 0.0, 0.0};
    Pose p2 = {2.0, 0.0, 0.0};
    Pose p3 = {1.0, 1.0, 0.0};
    unsigned char id0, id1, id2, id3;

    navGraph_addNode(&p0, NODE_TYPE_INTERSECTION, &id0);
    navGraph_addNode(&p1, NODE_TYPE_INTERSECTION, &id1);
    navGraph_addNode(&p2, NODE_TYPE_INTERSECTION, &id2);
    navGraph_addNode(&p3, NODE_TYPE_INTERSECTION, &id3);

    navGraph_addEdge(id0, id1, 1.0f, NULL);
    navGraph_addEdge(id1, id2, 1.0f, NULL);
    navGraph_addEdge(id1, id3, 2.0f, NULL);

    unsigned char path[32];
    unsigned char pathLen;

    /* 0 -> 3 should go 0-1-3 */
    TEST_ASSERT(navGraph_dijkstra(id0, id3, path, &pathLen));
    TEST_ASSERT_EQ(3, pathLen);
    TEST_ASSERT_EQ(0, path[0]);
    TEST_ASSERT_EQ(1, path[1]);
    TEST_ASSERT_EQ(3, path[2]);

    return 0;
}

static int test_target_node(void) {
    navGraph_reset();

    Pose p1 = {0.0, 0.0, 0.0};
    unsigned char id1;
    navGraph_addNode(&p1, NODE_TYPE_INTERSECTION, &id1);

    TEST_ASSERT_EQ(0xFF, navGraph_getTargetNode());
    navGraph_setTargetNode(id1);
    TEST_ASSERT_EQ(id1, navGraph_getTargetNode());

    navGraph_setNodeTarget(id1);
    const GraphNode *n = navGraph_getNode(id1);
    TEST_ASSERT((n->flags & NODE_FLAG_IS_TARGET) != 0);

    return 0;
}

static int test_edge_explored(void) {
    navGraph_reset();

    Pose p1 = {0.0, 0.0, 0.0};
    Pose p2 = {1.0, 0.0, 0.0};
    unsigned char id1, id2, eid;

    navGraph_addNode(&p1, NODE_TYPE_INTERSECTION, &id1);
    navGraph_addNode(&p2, NODE_TYPE_INTERSECTION, &id2);
    navGraph_addEdge(id1, id2, 1.0f, &eid);

    TEST_ASSERT(!navGraph_isEdgeExplored(eid));
    navGraph_setEdgeExplored(eid);
    TEST_ASSERT(navGraph_isEdgeExplored(eid));
    TEST_ASSERT(navGraph_allEdgesExplored());

    return 0;
}

static int test_dijkstra_max_length(void) {
    navGraph_reset();

    /* Create a path of exactly MAX_PATH_LENGTH nodes to test off-by-one fix */
    unsigned char ids[MAX_PATH_LENGTH];
    int i;
    for (i = 0; i < MAX_PATH_LENGTH; i++) {
        Pose p = {(double)i, 0.0, 0.0};
        navGraph_addNode(&p, NODE_TYPE_INTERSECTION, &ids[i]);
        if (i > 0) {
            navGraph_addEdge(ids[i-1], ids[i], 1.0f, NULL);
        }
    }

    unsigned char path[MAX_PATH_LENGTH];
    unsigned char pathLen;

    /* Path from first to last should have exactly MAX_PATH_LENGTH nodes */
    TEST_ASSERT(navGraph_dijkstra(ids[0], ids[MAX_PATH_LENGTH - 1], path, &pathLen));
    TEST_ASSERT_EQ(MAX_PATH_LENGTH, pathLen);

    return 0;
}

int main(void) {
    int failures = 0;

    printf("Running nav_graph tests...\n");

    failures += test_node_creation();
    failures += test_node_merge();
    failures += test_edge_linking();
    failures += test_dijkstra_simple();
    failures += test_dijkstra_branches();
    failures += test_target_node();
    failures += test_edge_explored();
    failures += test_dijkstra_max_length();

    if (failures == 0) {
        printf("All tests passed!\n");
    } else {
        printf("%d test(s) failed.\n", failures);
    }

    return failures;
}
