# Host unit tests for the safety-monitor stall-detection logic.
#
# Builds the dependency-free Core/Src/safety_eval.c against the host
# toolchain (no arm-none-eabi, no HAL, no FreeRTOS) and runs the test.
#
#   make            # build + run
#   make clean
#
# The point of keeping safety_eval.c HAL-free is exactly this: the
# safety-critical stall decision is testable on a laptop.

CORE_INC := ../../Core/Inc
CORE_SRC := ../../Core/Src

CXX      ?= c++
CC       ?= cc
CFLAGS   := -std=c11   -I$(CORE_INC) -Wall -Wextra -Werror -g -O0
CXXFLAGS := -std=c++17 -I$(CORE_INC) -Wall -Wextra -Werror -g -O0

BIN := test_safety_eval

.PHONY: all test clean
all: test

$(BIN): test_safety_eval.cpp safety_eval.o
	$(CXX) $(CXXFLAGS) -o $@ test_safety_eval.cpp safety_eval.o

safety_eval.o: $(CORE_SRC)/safety_eval.c $(CORE_INC)/safety_eval.h
	$(CC) $(CFLAGS) -c -o $@ $(CORE_SRC)/safety_eval.c

test: $(BIN)
	./$(BIN)

clean:
	rm -rf $(BIN) *.o *.dSYM
