#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "config.h"
#include "rm-mr32.h"
#include "leds.h"

/**
 * Initialise the state machine. Sets the initial state to IDLE.
 */
void stateMachine_init(void);

/**
 * Start the mission. Resets odometry, creates the origin node,
 * and transitions to the FOLLOW_LINE state.
 */
void stateMachine_start(void);

/**
 * Execute one tick of the state machine.
 * @param tick   Current tick counter.
 * @param ground 5-bit ground sensor pattern.
 * @param x      Robot X position (metres).
 * @param y      Robot Y position (metres).
 * @param h      Robot heading (radians).
 */
void stateMachine_tick(unsigned int tick, unsigned int ground, double x, double y, double h);

/**
 * Stop the mission and return to the IDLE state.
 * @param tick Current tick counter.
 */
void stateMachine_stop(unsigned int tick);

/**
 * Get the current mission phase for LED indication.
 * @return Current phase enum value.
 */
Phase stateMachine_getPhase(void);

/**
 * Get the reason the mission ended (for error diagnosis).
 * @return Null-terminated reason string, or NULL if not done / success.
 */
const char *stateMachine_getDoneReason(void);

#endif /* STATE_MACHINE_H */
