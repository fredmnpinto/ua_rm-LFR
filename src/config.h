#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================
 *   COMPILE-TIME CONSTANTS
 * ======================================================================== */

#define MAX_NODES 32
#define MAX_EDGES 64
#define NODE_MERGE_TOLERANCE 20.0f
#define CORNER_TICKS 3
#define MAX_PATH_LENGTH 32
#define TARGET_DIST_THRESHOLD 10.0f
#define BASE_SPEED 20
#define TURN_SPEED BASE_SPEED * 3 / 4
#define RECOVERY_SPEED BASE_SPEED * 2
#define LOST_TICKS 5
#define ORIGIN_TOLERANCE NODE_MERGE_TOLERANCE / 2
#define INTERSECTION_CENTER_DIST 70.0f

#define KP_ROT 40.0
#define KI_ROT 5.0

/* Sensor bit patterns (bit4=left, bit0=right) */
#define SENSOR_NONE 0b00000
#define SENSOR_CENTER 0b00100
#define SENSOR_LEFT 0b11000
#define SENSOR_RIGHT 0b00011
#define SENSOR_ALL 0b11111

/* ========================================================================
 *   DATA TYPES
 * ======================================================================== */

typedef struct {
  double x;
  double y;
  double h;
} Pose;

#endif /* CONFIG_H */
