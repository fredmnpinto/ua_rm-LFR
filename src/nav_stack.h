#ifndef NAV_STACK_H
#define NAV_STACK_H

#include "config.h"
#include "rm-mr32.h"

bool nav_push(const Pose *pose);
bool nav_pop(Pose *pose);
bool nav_isEmpty(void);
int nav_getDepth(void);
void nav_reset(void);

#endif /* NAV_STACK_H */
