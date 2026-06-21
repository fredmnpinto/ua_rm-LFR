#ifndef LOGGING_H
#define LOGGING_H

#include "config.h"
#include "rm-mr32.h"

void log_transition(unsigned int tick, const char *from, const char *to, const char *detail);
void log_target(unsigned int tick, double x, double y, double h);
void log_push(unsigned int tick, int depth, double x, double y, double h);
void log_pop(unsigned int tick, int depth);
void log_turnStart(unsigned int tick, double targetAngle);
void log_turnDone(unsigned int tick);
void log_lostTimeout(unsigned int tick);
void log_groundDetection(unsigned int tick, const char *type, unsigned int ground);
void log_missionDone(unsigned int tick, double tx, double ty, double th, int depth, unsigned int ticks);

#endif /* LOGGING_H */
