#include "rotation.h"

static double s_targetAngle;
static double s_integral;
static bool s_active;

void rotation_start(double currentHeading, double deltaAngle) {
    s_targetAngle = normalizeAngle(currentHeading + deltaAngle);
    s_integral = 0.0;
    s_active = true;
}

void rotation_tick(double currentHeading) {
    double error;
    int cmdVel;

    if (!s_active) {
        return;
    }

    error = normalizeAngle(s_targetAngle - currentHeading);

    s_integral += error;
    if (s_integral > PI / 2.0) {
        s_integral = PI / 2.0;
    } else if (s_integral < -PI / 2.0) {
        s_integral = -PI / 2.0;
    }

    cmdVel = (int)((KP_ROT * error) + (KI_ROT * s_integral));

    if (cmdVel > TURN_SPEED) {
        cmdVel = TURN_SPEED;
    } else if (cmdVel < -TURN_SPEED) {
        cmdVel = -TURN_SPEED;
    }

    setVel2(-cmdVel, cmdVel);
}

bool rotation_done(double currentHeading) {
    double error;

    if (!s_active) {
        return true;
    }

    error = normalizeAngle(s_targetAngle - currentHeading);

    if (fabs(error) < 0.02) {
        s_active = false;
        setVel2(0, 0);
        return true;
    }
    return false;
}

void rotation_stop(void) {
    s_active = false;
    setVel2(0, 0);
}
