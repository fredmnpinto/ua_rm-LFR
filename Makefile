# TP2 Robot Project Makefile
# PIC32 differential-drive robot agent

# Directories
SRCDIR   = src
BUILDDIR = build

# Toolchain
CC       = pcompile
CFLAGS   =
FLASH    = ldpic32
TERM     = pterm

# Target and source files
TARGET   = robot-agent
HEX      = $(BUILDDIR)/$(TARGET).hex
SRCS     = $(SRCDIR)/robot-agent.c $(SRCDIR)/nav_stack.c $(SRCDIR)/line_follower.c $(SRCDIR)/rotation.c $(SRCDIR)/logging.c $(SRCDIR)/leds.c $(SRCDIR)/state_machine.c $(SRCDIR)/rm-mr32.c
HDRS     = $(SRCDIR)/rm-mr32.h

# Phony targets (do not represent files)
.PHONY: all build flash term deploy clean run

# Default target when running `make` with no arguments
.DEFAULT_GOAL := build
all: build

# ---------------------------------------------------------------------------
# Build: compile source files to produce robot-agent.hex
# ---------------------------------------------------------------------------
build: $(HEX)

$(HEX): $(SRCS) $(HDRS)
	@mkdir -p $(BUILDDIR)
	@echo "  CC      $(SRCS) -> $(HEX)"
	cd $(BUILDDIR) && $(CC) $(CFLAGS) $(addprefix ../,$(SRCS))
	cp $(SRCDIR)/*.hex $(BUILDDIR)
	@echo "  BUILD   $(HEX) ready"

# ---------------------------------------------------------------------------
# Flash: build (if needed) then program the robot
# ---------------------------------------------------------------------------
flash: build
	@if [ ! -f $(HEX) ]; then \
		echo "Error: $(HEX) not found. Build may have failed."; \
		exit 1; \
	fi
	@echo "  FLASH   $(HEX)"
	$(FLASH) -w $(HEX)
	@echo "  FLASH   complete"

# ---------------------------------------------------------------------------
# Terminal: start serial monitor (blocks until Ctrl+C)
# ---------------------------------------------------------------------------
term:
	@echo "  TERM    Starting pterm (Ctrl+C to exit)..."
	$(TERM)

# ---------------------------------------------------------------------------
# Deploy: convenient one-command build + flash
# ---------------------------------------------------------------------------
deploy: flash
	@echo "  DEPLOY  complete"

# ---------------------------------------------------------------------------
# Clean: remove all build artifacts
# ---------------------------------------------------------------------------
clean:
	@echo "  CLEAN   removing build artifacts"
	rm -rf $(BUILDDIR)
	@echo "  CLEAN   done"

# ---------------------------------------------------------------------------
# Run: full workflow — build, flash, then open terminal
# ---------------------------------------------------------------------------
run: flash
	@echo "  RUN     starting terminal..."
	$(TERM)
