#ifndef LEDS_H
#define LEDS_H

#include "rm-mr32.h"

typedef enum {
    PHASE_EXPLORING,
    PHASE_AT_TARGET,
    PHASE_RETURNING_TO_TARGET,
    PHASE_RETURNING,
    PHASE_DONE_SUCCESS,
    PHASE_DONE_ERROR
} Phase;

void leds_update(Phase phase, unsigned int tick);

#endif /* LEDS_H */
