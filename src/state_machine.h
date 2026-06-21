#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "config.h"
#include "rm-mr32.h"
#include "leds.h"

void stateMachine_init(void);
void stateMachine_start(void);
void stateMachine_tick(unsigned int tick, unsigned int ground, double x, double y, double h);
void stateMachine_stop(unsigned int tick);
Phase stateMachine_getPhase(void);
const char *stateMachine_getDoneReason(void);

#endif /* STATE_MACHINE_H */
