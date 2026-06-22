#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================
 *   COMPILE-TIME CONSTANTS
 * ======================================================================== */

#define MAX_NODES 32
#define MAX_EDGES 64
#define NODE_MERGE_TOLERANCE 0.10f
#define CORNER_TICKS 3
#define MAX_PATH_LENGTH 32
#define TARGET_DIST_THRESHOLD 0.06f
#define BASE_SPEED 40
#define TURN_SPEED 20
#define RECOVERY_SPEED 40
#define LOST_TICKS 25
#define ORIGIN_TOLERANCE 0.05f
#define INTERSECTION_CENTER_DIST 0.15f

#define KP_ROT 40.0
#define KI_ROT 5.0

/* Sensor bit patterns (bit4=left, bit0=right) */
#define SENSOR_NONE   0b00000
#define SENSOR_CENTER 0b00100
#define SENSOR_LEFT   0b11000
#define SENSOR_RIGHT  0b00011
#define SENSOR_ALL    0b11111

/* ========================================================================
 *   DATA TYPES
 * ======================================================================== */

typedef struct {
    double x;
    double y;
    double h;
} Pose;

#endif /* CONFIG_H */
