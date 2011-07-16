#-----------------------------------------------------------------------------
# Purpose: Main files for the adruino lirc project
ifeq ($(INC_PART),)
# DELETE ME from top level makefile Leave me in every dir under top level and 
# I will allow you to build from any point in the build system
$(if $(MAKECMDGOALS),$(MAKECMDGOALS),all):
	@make -C .. $@
endif

CSRC :=
CSRC += missing.c

CXXSRC :=
CXXSRC += main.cpp

# Add to include path
#ADD_TO_INC_PATH := y