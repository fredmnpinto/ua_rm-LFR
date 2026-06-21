#ifndef ROTATION_H
#define ROTATION_H

#include "config.h"
#include "rm-mr32.h"
#include <math.h>

void rotation_start(double currentHeading, double deltaAngle);
void rotation_tick(double currentHeading);
bool rotation_done(double currentHeading);
void rotation_stop(void);

#endif /* ROTATION_H */
