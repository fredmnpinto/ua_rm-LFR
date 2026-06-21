# PIC32 Robot Agent Makefile
# Usage: make          (build with ROBOT=1)
#        make ROBOT=5  (build with ROBOT=5)
#        make flash    (build then program robot)
#        make clean    (remove artifacts)

ROBOT ?= 1
SRCS = robot-agent.c nav_stack.c line_follower.c rotation.c logging.c leds.c state_machine.c rm-mr32.c
TARGET = robot-agent

all:
	pcompile -DROBOT=$(ROBOT) $(SRCS)

clean:
	rm -f *.o *.hex *.elf *.map $(TARGET)

flash: all
	ldpic32 -w $(TARGET).hex

.PHONY: all clean flash
