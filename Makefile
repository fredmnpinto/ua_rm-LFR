# TP2 Robot Project Makefile
# PIC32 differential-drive robot agent
#
# Usage:
#   make              Build with default ROBOT=1
#   make ROBOT=5      Build with robot 5 calibration
#   make flash        Build and flash to robot
#   make term         Open serial terminal
#   make deploy       Build + flash
#   make run          Build + flash + terminal
#   make clean        Remove build artifacts

# Directories
SRCDIR   = src
BUILDDIR = build

# Toolchain
PCOMPILE = $(SRCDIR)/pcompile
FLASH    = ldpic32
TERM     = pterm

# Robot calibration (default: 1)
ROBOT    ?= 1

# Target and source files
TARGET   = robot-agent
HEX      = $(BUILDDIR)/$(TARGET).hex
SRCS     = $(SRCDIR)/robot-agent.c $(SRCDIR)/nav_stack.c $(SRCDIR)/line_follower.c $(SRCDIR)/rotation.c $(SRCDIR)/logging.c $(SRCDIR)/leds.c $(SRCDIR)/state_machine.c $(SRCDIR)/rm-mr32.c
HDRS     = $(SRCDIR)/config.h $(SRCDIR)/nav_stack.h $(SRCDIR)/line_follower.h $(SRCDIR)/rotation.h $(SRCDIR)/logging.h $(SRCDIR)/leds.h $(SRCDIR)/state_machine.h $(SRCDIR)/rm-mr32.h

# Phony targets
.PHONY: all build flash term deploy clean run

# Default target
.DEFAULT_GOAL := build
all: build

# Build: compile source files to produce robot-agent.hex
build: $(HEX)

$(HEX): $(SRCS) $(HDRS)
	@mkdir -p $(BUILDDIR)
	@echo "  CC      $(SRCS) -> $(HEX)"
	cd $(BUILDDIR) && ../$(PCOMPILE) -DROBOT=$(ROBOT) $(addprefix ../,$(SRCS))
	@echo "  MOVE    build artifacts to $(BUILDDIR)/"
	@mv $(SRCDIR)/*.o $(SRCDIR)/*.elf $(SRCDIR)/*.hex $(SRCDIR)/*.map $(BUILDDIR)/ 2>/dev/null || true
	@echo "  BUILD   $(HEX) ready"

# Flash: build (if needed) then program the robot
flash: build
	@if [ ! -f $(HEX) ]; then \
		echo "Error: $(HEX) not found. Build may have failed."; \
		exit 1; \
	fi
	@echo "  FLASH   $(HEX)"
	$(FLASH) -w $(HEX)
	@echo "  FLASH   complete"

# Terminal: start serial monitor
term:
	@echo "  TERM    Starting pterm (Ctrl+C to exit)..."
	$(TERM)

# Deploy: build + flash
deploy: flash
	@echo "  DEPLOY  complete"

# Clean: remove all build artifacts from both src/ and build/
clean:
	@echo "  CLEAN   removing build artifacts"
	rm -rf $(BUILDDIR)
	rm -f $(SRCDIR)/*.o $(SRCDIR)/*.elf $(SRCDIR)/*.hex $(SRCDIR)/*.map
	@echo "  CLEAN   done"

# Run: full workflow — build, flash, then open terminal
run: flash
	@echo "  RUN     starting terminal..."
	$(TERM)
