#include "line_follower.h"

/**
 * \brief Execute one tick of bang-bang line following.
 * \param ground 5-bit sensor pattern from readLineSensors(0)
 * \param pLost output: set to true if line is lost
 */
void lineFollower_centerOnLine(unsigned int ground, bool *pLost) {
    *pLost = false;

    if (ground == SENSOR_NONE) {
        /* Lost line — signal to caller, do not set motors here */
        *pLost = true;
        return;
    }

    /* Normal line following patterns */
    if (ground == SENSOR_CENTER) {
        /* Centered on line */
        setVel2(BASE_SPEED, BASE_SPEED);
    } else if ((ground & SENSOR_LEFT) != 0) {
        /* Line on left → turn left to recenter */
        setVel2(TURN_SPEED, BASE_SPEED);
    } else if ((ground & SENSOR_RIGHT) != 0) {
        /* Line on right → turn right to recenter */
        setVel2(BASE_SPEED, TURN_SPEED);
    } else {
        /* Fallback: any other pattern, drive straight slowly */
        setVel2(BASE_SPEED, BASE_SPEED);
    }
}
