#include <atomic>

extern "C" {
#include "watchdog_monitor.h"
}

static std::atomic<bool> g_watchdog_triggered{false};

extern "C" void watchdog_set_triggered(void)
{
    g_watchdog_triggered.store(true, std::memory_order_release);
}

extern "C" bool watchdog_is_triggered(void)
{
    return g_watchdog_triggered.load(std::memory_order_acquire);
}

extern "C" void watchdog_clear_triggered(void)
{
    g_watchdog_triggered.store(false, std::memory_order_release);
}

extern "C" bool watchdog_consume_triggered(void)
{
    // Atomic read-and-clear; on Cortex-M this is lock-free and non-blocking
    return g_watchdog_triggered.exchange(false, std::memory_order_acq_rel);
}
