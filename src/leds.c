#include "leds.h"

void leds_update(Phase phase, unsigned int tick) {
    /* Clear all LEDs first */
    leds(0);

    switch (phase) {
        case PHASE_EXPLORING:
            /* Phase 1: exploring */
            led(0, 1);
            break;

        case PHASE_AT_TARGET:
            /* At target */
            led(1, 1);
            break;

        case PHASE_RETURNING_TO_TARGET:
            /* Phase 2: returning to target */
            led(0, 1);
            led(1, 1);
            break;

        case PHASE_RETURNING:
            /* Phase 3: returning to origin */
            led(2, 1);
            break;

        case PHASE_DONE_SUCCESS:
            /* Success: solid LED 3 */
            led(3, 1);
            break;

        case PHASE_DONE_ERROR:
            /* Error: fast blink LED 3 */
            if ((tick % 4) < 2) {
                led(3, 1);
            }
            break;
    }
}
