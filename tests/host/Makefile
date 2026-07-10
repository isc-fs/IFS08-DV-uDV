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
#   test_rules_compliance — FS-Rules timing-budget static_asserts
#   test_as_transition    — pure AS transition (pipeline handshake + DV watchdog)
#   test_missions         — mission registry + per-mission bodies (inspection
#                           sweep/cadence/self-finish, pipeline relay, ebs stub)
#   test_attitude         — complementary-filter roll/pitch (attitude.c) [ex-dev]
#   test_bmi088_convert   — BMI088 raw->scaled + bind validation (bmi088.c) [ex-dev]
#   test_imu_align        — IMU->car yaw mount rotation (imu_align.c)

CORE_INC := ../../Core/Inc
CORE_SRC := ../../Core/Src
STUBS    := stubs

CXX      ?= c++
CC       ?= cc

# Strict flags for our own dependency-free unit (safety_eval).
CFLAGS   := -std=c11   -I$(CORE_INC) -Wall -Wextra -Werror -g -O0
CXXFLAGS := -std=c++17 -I$(CORE_INC) -Wall -Wextra -Werror -g -O0

# State-machine suite links real firmware sources (state_manager, ebs_manager,
# can_globals, ros_globals). Stubs/ shadows the RTOS headers those pull in, and
# mocks/ shadows stm32h7xx_hal.h — can_globals.h now includes bmi088.h (for
# can_c_send_imu), which pulls in the HAL; the suite never calls into it.
# -Wall (not -Werror) so pre-existing style warnings in the brought-over
# sources don't block the behavioural tests.
SM_CXXFLAGS := -std=c++17 -I$(STUBS) -Imocks -I$(CORE_INC) -Wall -Wextra -g -O0
SM_SRCS     := test_state_machine.cpp \
               $(CORE_SRC)/state_manager.cpp \
               $(CORE_SRC)/ebs_manager.cpp \
               $(CORE_SRC)/can_globals.cpp \
               $(CORE_SRC)/ros_globals.cpp

# Sensor-math suites (recovered from dev). The C code-under-test (attitude.c,
# bmi088.c) is byte-identical to dev, so its off-target math stays covered.
# mocks/ shadows stm32h7xx_hal.h / main.h / i2c_utils so the HAL-touching
# sources link on the host. Plain -Wall (brought-over sources), not -Werror.
MOCKS       := mocks
SENS_CFLAGS := -std=c11 -I$(MOCKS) -I$(CORE_INC) -Wall -Wextra -g -O0

BINS := test_safety_eval test_state_machine test_rules_compliance \
        test_as_transition test_as_actuation test_missions test_attitude \
        test_bmi088_convert test_imu_align

# Mission suite — pure mission bodies + registry, no HAL/RTOS/CAN. Strict flags
# (-Werror): new dependency-free code, held to the same bar as safety_eval.
MISSION_SRCS := test_missions.cpp \
                $(CORE_SRC)/mission_registry.cpp \
                $(CORE_SRC)/mission_inspection.cpp \
                $(CORE_SRC)/mission_pipeline.cpp \
                $(CORE_SRC)/mission_ebstest.cpp

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

# ---- suite: pure AS safe-actuation (EBS release / SDC open per state) ----
# Header-only as_actuation.hpp; same include story as test_as_transition.
test_as_actuation: test_as_actuation.cpp
	$(CXX) $(SM_CXXFLAGS) -o $@ test_as_actuation.cpp

# ---- suite 4b: mission registry + per-mission bodies --------------------
# Pure modules (mission.h only): link the real mission sources directly, no
# stubs. Strict CXXFLAGS with -Werror.
test_missions: $(MISSION_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $(MISSION_SRCS)

# ---- suite 5: attitude math (complementary filter, roll/pitch) ----------
# Links the real Core/Src/attitude.c against the HAL mock; needs libm.
test_attitude: test_attitude.c $(CORE_SRC)/attitude.c mocks/hal_stubs.c mocks/i2c_utils_stubs.c
	$(CC) $(SENS_CFLAGS) -o $@ test_attitude.c $(CORE_SRC)/attitude.c \
	      mocks/hal_stubs.c mocks/i2c_utils_stubs.c -lm

# ---- suite 6: BMI088 raw->scaled conversion + bind validation -----------
test_bmi088_convert: test_bmi088_convert.c $(CORE_SRC)/bmi088.c mocks/hal_stubs.c mocks/i2c_utils_stubs.c
	$(CC) $(SENS_CFLAGS) -o $@ test_bmi088_convert.c $(CORE_SRC)/bmi088.c \
	      mocks/hal_stubs.c mocks/i2c_utils_stubs.c -lm

# ---- suite 7: IMU->car yaw mount alignment (pure rotation math) ----------
# Links the real Core/Src/imu_align.c; bmi088.h pulls the HAL mock, so use the
# same mock include path + stubs as the other sensor suites. Needs libm (trig).
test_imu_align: test_imu_align.c $(CORE_SRC)/imu_align.c mocks/hal_stubs.c mocks/i2c_utils_stubs.c
	$(CC) $(SENS_CFLAGS) -o $@ test_imu_align.c $(CORE_SRC)/imu_align.c \
	      mocks/hal_stubs.c mocks/i2c_utils_stubs.c -lm

test: $(BINS)
	@echo "=== build_integrity ===";       sh ./check_build_integrity.sh
	@echo "=== test_safety_eval ===";      ./test_safety_eval
	@echo "=== test_state_machine ===";    ./test_state_machine
	@echo "=== test_rules_compliance ==="; ./test_rules_compliance
	@echo "=== test_as_transition ===";    ./test_as_transition
	@echo "=== test_as_actuation ===";     ./test_as_actuation
	@echo "=== test_missions ===";         ./test_missions
	@echo "=== test_attitude ===";         ./test_attitude
	@echo "=== test_bmi088_convert ===";   ./test_bmi088_convert
	@echo "=== test_imu_align ===";        ./test_imu_align

clean:
	rm -rf $(BINS) *.o *.dSYM
