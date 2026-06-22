#ifndef __MR32_H
#define __MR32_H

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#ifndef ROBOT
#define ROBOT 1
#endif

#define PI	3.141592654

/* Mock normalizeAngle for host testing */
static inline double normalizeAngle(double angle) {
    while (angle <= -PI) angle += 2.0 * PI;
    while (angle > PI) angle -= 2.0 * PI;
    return angle;
}

/* Mock printInt for logging compatibility */
static inline void printInt(int value, int format) {
    (void)format;
    printf("%d", value);
}

#endif
