#include "rm-mr32.h"
#include "state_machine.h"
#include "leds.h"
#include "nav_graph.h"

int main(void) {
    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);
    setRobotPos(0.0, 0.0, 0.0);

    navGraph_init();
    stateMachine_init();

    while (!startButton()) {
        led(0, 1);
        delay(50);
        led(0, 0);
        delay(50);
    }

    stateMachine_start();

    unsigned int tick = 0;
    while (1) {
        waitTick40ms();
        tick++;

        if (stopButton()) {
            stateMachine_stop(tick);
            continue;
        }

        readAnalogSensors();
        unsigned int ground = readLineSensors(0);

        double x, y, h;
        getRobotPos(&x, &y, &h);

        stateMachine_tick(tick, ground, x, y, h);
        leds_update(stateMachine_getPhase(), tick);
    }

    return 0;
}
