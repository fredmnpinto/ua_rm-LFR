#include "logging.h"

void log_transition(unsigned int tick, const char *from, const char *to, const char *detail) {
    printf("[T %u] %s -> %s%s%s\n", tick, from, to,
           detail ? " " : "", detail ? detail : "");
}

void log_target(unsigned int tick, double x, double y, double h) {
    printf("[T %u] TARGET x=%.3f y=%.3f h=%.2f\n", tick, x, y, h);
}

void log_push(unsigned int tick, int depth, double x, double y, double h) {
    printf("[T %u] PUSH depth=%d x=%.3f y=%.3f h=%.2f\n", tick, depth, x, y, h);
}

void log_pop(unsigned int tick, int depth) {
    printf("[T %u] POP depth=%d\n", tick, depth);
}

void log_turnStart(unsigned int tick, double targetAngle) {
    printf("[T %u] TURN_LEFT target=%.2f\n", tick, targetAngle);
}

void log_turnDone(unsigned int tick) {
    printf("[T %u] TURN_DONE\n", tick);
}

void log_lostTimeout(unsigned int tick) {
    printf("[T %u] LOST_TIMEOUT\n", tick);
}

void log_groundDetection(unsigned int tick, const char *type, unsigned int ground) {
    printf("[T %u] Detected %s, ground=", tick, type);
    printInt(ground, 2 | 5 << 16);
    printf("\n");
}

void log_missionDone(unsigned int tick, double tx, double ty, double th, int depth, unsigned int ticks) {
    printf("[T %u] DONE target=(%.2f,%.2f,%.2f) depth=%d ticks=%d\n",
           tick, tx, ty, th, depth, ticks);
}
