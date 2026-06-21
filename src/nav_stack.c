#include "nav_stack.h"

static Pose s_stack[MAX_STACK_DEPTH];
static int s_top = -1;

bool nav_push(const Pose *pose) {
  if (s_top >= MAX_STACK_DEPTH - 1) {
    return false; /* Stack overflow */
  }
  s_top++;
  s_stack[s_top] = *pose;
  return true;
}

bool nav_pop(Pose *pose) {
  if (s_top < 0) {
    return false; /* Stack underflow */
  }
  *pose = s_stack[s_top];
  s_top--;
  return true;
}

bool nav_isEmpty(void) { return (s_top < 0); }

int nav_getDepth(void) { return s_top + 1; }

void nav_reset(void) { s_top = -1; }
