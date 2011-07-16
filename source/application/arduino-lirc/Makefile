#-----------------------------------------------------------------------------
# Purpose: Main files for the adruino lirc project
ifeq ($(INC_PART),)
$(if $(MAKECMDGOALS),$(MAKECMDGOALS),all):
	@make -C .. $@
endif

CSRC :=
CSRC += missing.c

CXXSRC :=
CXXSRC += main.cpp

# Add to include path
#ADD_TO_INC_PATH := y