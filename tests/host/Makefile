# Host unit tests — run the firmware's testable logic off-target (no
# arm-none-eabi, no HAL, no scheduler).
#
#   make            # build + run ALL suites
#   make clean
#
# Suites:
#   build_integrity      — static build-config checks (Makefile<->CMake source
#                          parity, orphan/missing sources, extern "C" task
#                          entry points). Catches link-breaking build drift the
#                          logic suites are blind to (check_build_integrity.sh).
#   test_safety_eval     — pure IWDG-monitor stall-detection (safety_eval.c)
#   test_state_machine    — AS state machine + EBS init sequence
#                           (state_manager.cpp / ebs_manager.cpp, linked for
#                           real against a controllable hardware_io stub)

CORE_INC := ../../Core/Inc
CORE_SRC := ../../Core/Src
STUBS    := stubs

CXX      ?= c++
CC       ?= cc

# Strict flags for our own dependency-free unit (safety_eval).
CFLAGS   := -std=c11   -I$(CORE_INC) -Wall -Wextra -Werror -g -O0
CXXFLAGS := -std=c++17 -I$(CORE_INC) -Wall -Wextra -Werror -g -O0

# State-machine suite links real firmware sources (state_manager, ebs_manager,
# can_globals, ros_globals). Stubs/ shadows the RTOS headers those pull in;
# -Wall (not -Werror) so pre-existing style warnings in the brought-over
# sources don't block the behavioural tests.
SM_CXXFLAGS := -std=c++17 -I$(STUBS) -I$(CORE_INC) -Wall -Wextra -g -O0
SM_SRCS     := test_state_machine.cpp \
               $(CORE_SRC)/state_manager.cpp \
               $(CORE_SRC)/ebs_manager.cpp \
               $(CORE_SRC)/can_globals.cpp \
               $(CORE_SRC)/ros_globals.cpp

BINS := test_safety_eval test_state_machine test_rules_compliance test_as_transition

.PHONY: all test clean
all: test

# ---- suite 1: safety_eval ------------------------------------------------
test_safety_eval: test_safety_eval.cpp safety_eval.o
	$(CXX) $(CXXFLAGS) -o $@ test_safety_eval.cpp safety_eval.o

safety_eval.o: $(CORE_SRC)/safety_eval.c $(CORE_INC)/safety_eval.h
	$(CC) $(CFLAGS) -c -o $@ $(CORE_SRC)/safety_eval.c

# ---- suite 2: state machine + EBS ---------------------------------------
test_state_machine: $(SM_SRCS)
	$(CXX) $(SM_CXXFLAGS) -o $@ $(SM_SRCS)

# ---- suite 3: rules-compliance timing budget (mostly static_assert) -----
test_rules_compliance: test_rules_compliance.cpp safety_eval.o
	$(CXX) $(CXXFLAGS) -o $@ test_rules_compliance.cpp safety_eval.o

# ---- suite 4: AS transition decision (pipeline handshake + DV watchdog) --
# Header-only pure logic (as_transition.hpp); needs the RTOS-type stubs only
# because state_manager.hpp transitively includes them. No firmware .cpp linked.
test_as_transition: test_as_transition.cpp
	$(CXX) $(SM_CXXFLAGS) -o $@ test_as_transition.cpp

test: $(BINS)
	@echo "=== build_integrity ===";       sh ./check_build_integrity.sh
	@echo "=== test_safety_eval ===";      ./test_safety_eval
	@echo "=== test_state_machine ===";    ./test_state_machine
	@echo "=== test_rules_compliance ==="; ./test_rules_compliance
	@echo "=== test_as_transition ===";    ./test_as_transition

clean:
	rm -rf $(BINS) *.o *.dSYM
