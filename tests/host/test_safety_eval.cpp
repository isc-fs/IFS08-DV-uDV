/**
 * @file test_safety_eval.cpp
 * @brief Host unit tests for the pure safety-monitor stall detection.
 *
 * Builds and runs on the host (no HAL / FreeRTOS) — it links only the
 * dependency-free Core/Src/safety_eval.c. Pins the safety-critical
 * decision the IWDG watchdog supervisor makes: when is a task healthy,
 * when is it stalled, and that boot-time blocking (unarmed tasks) never
 * false-trips.
 *
 *   make            # build + run
 */
#include <cstdio>
#include <cstring>

extern "C" {
#include "safety_eval.h"
}

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        ++g_checks;                                                       \
        if (!(cond)) {                                                    \
            ++g_failures;                                                 \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                 \
    } while (0)

/* ---- helpers -------------------------------------------------------- */

static task_watch_t make_watch(uint32_t deadline_ms)
{
    task_watch_t w;
    std::memset(&w, 0, sizeof(w));
    w.deadline_ms = deadline_ms;
    return w;
}

/* ---- tests ---------------------------------------------------------- */

/* A task that keeps advancing its heartbeat is always healthy. */
static void test_advancing_task_is_healthy(void)
{
    task_watch_t w[1] = { make_watch(100) };
    bool armed[1] = { true };
    uint32_t counts[1] = { 0 };

    for (uint32_t t = 0; t <= 1000; t += 10) {
        counts[0]++;                       /* advances every cycle */
        CHECK(safety_eval(w, counts, armed, 1, t) == -1);
    }
}

/* A task whose heartbeat freezes is flagged once the deadline passes —
 * and NOT before (boundary is strict ">"). */
static void test_frozen_task_stalls_after_deadline(void)
{
    task_watch_t w[1] = { make_watch(100) };
    bool armed[1] = { true };
    uint32_t counts[1] = { 5 };

    /* Baseline at t=0. */
    CHECK(safety_eval(w, counts, armed, 1, 0) == -1);
    /* Frozen counter: still healthy up to and including the deadline. */
    CHECK(safety_eval(w, counts, armed, 1, 50)  == -1);
    CHECK(safety_eval(w, counts, armed, 1, 100) == -1);  /* == deadline: alive */
    CHECK(safety_eval(w, counts, armed, 1, 101) ==  0);  /* > deadline: stalled */
}

/* A single late heartbeat resets the deadline timer (recovery). */
static void test_late_heartbeat_resets_timer(void)
{
    task_watch_t w[1] = { make_watch(100) };
    bool armed[1] = { true };
    uint32_t counts[1] = { 0 };

    CHECK(safety_eval(w, counts, armed, 1, 0) == -1);
    CHECK(safety_eval(w, counts, armed, 1, 90) == -1);   /* nearly stalled */
    counts[0]++;                                          /* heartbeat! */
    CHECK(safety_eval(w, counts, armed, 1, 95) == -1);   /* timer reset */
    CHECK(safety_eval(w, counts, armed, 1, 190) == -1);  /* 95 since reset */
    CHECK(safety_eval(w, counts, armed, 1, 196) == 0);   /* now > 100 */
}

/* Unarmed tasks are ignored entirely — a task that never arms (still
 * calibrating / blocked in init) must never trip the watchdog. */
static void test_unarmed_task_never_stalls(void)
{
    task_watch_t w[1] = { make_watch(50) };
    bool armed[1] = { false };
    uint32_t counts[1] = { 0 };

    for (uint32_t t = 0; t <= 100000; t += 100) {
        CHECK(safety_eval(w, counts, armed, 1, t) == -1);
    }
}

/* A task that arms late baselines cleanly: the long pre-arm period does
 * not count against its deadline. */
static void test_late_arm_baselines_clean(void)
{
    task_watch_t w[1] = { make_watch(100) };
    bool armed[1] = { false };
    uint32_t counts[1] = { 0 };

    /* 6 s of "calibration" — unarmed, ignored. */
    CHECK(safety_eval(w, counts, armed, 1, 6000) == -1);
    /* Arms at t=6000 with a frozen counter; baseline starts now. */
    armed[0] = true;
    CHECK(safety_eval(w, counts, armed, 1, 6000) == -1);
    CHECK(safety_eval(w, counts, armed, 1, 6100) == -1);  /* == deadline */
    CHECK(safety_eval(w, counts, armed, 1, 6101) == 0);   /* > deadline */
}

/* Multiple tasks: the first stalled index is returned; healthy tasks
 * alongside a stalled one don't mask it. */
static void test_multiple_tasks_report_first_stalled(void)
{
    task_watch_t w[2] = { make_watch(100), make_watch(100) };
    bool armed[2] = { true, true };
    uint32_t counts[2] = { 0, 0 };

    CHECK(safety_eval(w, counts, armed, 2, 0) == -1);
    /* Task 0 keeps ticking, task 1 freezes. */
    counts[0] = 1;
    CHECK(safety_eval(w, counts, armed, 2, 100) == -1);
    counts[0] = 2;
    CHECK(safety_eval(w, counts, armed, 2, 201) == 1);   /* task 1 stalled */
}

/* Heartbeat counter wrap (0xFFFFFFFF -> 0) is a valid advance. */
static void test_counter_wrap_is_an_advance(void)
{
    task_watch_t w[1] = { make_watch(100) };
    bool armed[1] = { true };
    uint32_t counts[1] = { 0xFFFFFFFFu };

    CHECK(safety_eval(w, counts, armed, 1, 0) == -1);     /* baseline */
    counts[0] = 0u;                                       /* wrapped */
    CHECK(safety_eval(w, counts, armed, 1, 90) == -1);    /* advance seen */
    CHECK(safety_eval(w, counts, armed, 1, 200) == 0);    /* then freezes */
}

/* Time wrap near uint32 max is handled by modular subtraction. */
static void test_time_wrap_is_handled(void)
{
    task_watch_t w[1] = { make_watch(100) };
    bool armed[1] = { true };
    uint32_t counts[1] = { 7 };

    uint32_t base = 0xFFFFFFF0u;                          /* 16 ms before wrap */
    CHECK(safety_eval(w, counts, armed, 1, base) == -1);  /* baseline */
    /* 50 ms later, having wrapped past 0: elapsed must read 50, healthy. */
    CHECK(safety_eval(w, counts, armed, 1, base + 50u) == -1);
    /* 101 ms after baseline (also wrapped): stalled. */
    CHECK(safety_eval(w, counts, armed, 1, base + 101u) == 0);
}

/* Disarm-then-rearm re-baselines (no stale stall carried across). */
static void test_disarm_rearm_rebaselines(void)
{
    task_watch_t w[1] = { make_watch(100) };
    bool armed[1] = { true };
    uint32_t counts[1] = { 0 };

    CHECK(safety_eval(w, counts, armed, 1, 0) == -1);
    /* Freeze long enough that it WOULD be stalled... */
    CHECK(safety_eval(w, counts, armed, 1, 200) == 0);
    /* ...but a disarm clears the watch... */
    armed[0] = false;
    CHECK(safety_eval(w, counts, armed, 1, 250) == -1);
    /* ...and re-arm baselines fresh, so it's healthy again. */
    armed[0] = true;
    CHECK(safety_eval(w, counts, armed, 1, 260) == -1);
    CHECK(safety_eval(w, counts, armed, 1, 360) == -1);  /* == deadline */
    CHECK(safety_eval(w, counts, armed, 1, 361) == 0);
}

int main(void)
{
    test_advancing_task_is_healthy();
    test_frozen_task_stalls_after_deadline();
    test_late_heartbeat_resets_timer();
    test_unarmed_task_never_stalls();
    test_late_arm_baselines_clean();
    test_multiple_tasks_report_first_stalled();
    test_counter_wrap_is_an_advance();
    test_time_wrap_is_handled();
    test_disarm_rearm_rebaselines();

    std::printf("\nsafety_eval: %d checks, %d failures\n",
                g_checks, g_failures);
    if (g_failures == 0) {
        std::printf("All tests green.\n");
        return 0;
    }
    return 1;
}
