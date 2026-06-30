/**
 * @file iwdg.h
 * @brief STM32H7 Independent Watchdog (IWDG1) — minimal register driver.
 *
 * The IWDG is the hardware backstop of the two-tier safety watchdog
 * (see safety_monitor.c). It runs off the always-on LSI (~32 kHz),
 * independent of the main clock tree, so it resets the MCU even if the
 * CPU is wedged with interrupts disabled — the one failure the software
 * monitor cannot catch. On reset the EBS fires via the hardware
 * fail-safe (confirmed: loss of the MCU brings the car to a safe stop).
 *
 * Implemented with direct register writes rather than the HAL IWDG
 * module on purpose: this repo's fix/15 base has no CubeMX IWDG in its
 * .ioc and HAL_IWDG_MODULE_ENABLED is off. Driving the registers keeps
 * the watchdog self-contained and survives a CubeMX regeneration
 * (CubeMX never touches this file).
 *
 * IWDG cannot run a timeout callback — it only resets. The graceful
 * emergency actions (EBS assert, ASSI-emergency CAN) are owned by the
 * software monitor; this peripheral is purely the last-resort reset.
 */
#ifndef IWDG_H
#define IWDG_H

/* ---- Timing budget (single source of truth; HAL-free so it can be
 *      asserted against the FS-Rules limits in the host test suite). ----
 *
 *   timeout_ms = (RLR + 1) * prescaler_div * 1000 / f_LSI
 *
 * PR register encoding: 0=/4 1=/8 2=/16 3=/32 4=/64 5=/128 6=/256.
 * LSI is nominal here; its real tolerance is wide, which is why the
 * timeout is many times the monitor refresh period.
 */
#define IWDG_LSI_HZ              32000u
#define IWDG_PRESCALER_PR       2u      /* register value → /16            */
#define IWDG_PRESCALER_DIV      16u     /* must match IWDG_PRESCALER_PR    */
#define IWDG_RELOAD_VALUE       200u    /* RLR                             */
#define IWDG_TIMEOUT_MS_NOMINAL \
    (((IWDG_RELOAD_VALUE + 1u) * IWDG_PRESCALER_DIV * 1000u) / IWDG_LSI_HZ)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure and start IWDG1 (~100 ms timeout on the LSI).
 *
 * Once started the IWDG cannot be stopped (by hardware design); it must
 * be refreshed via iwdg_refresh() at least every timeout period or the
 * MCU resets. Call once, from the safety task, after the scheduler is
 * running. Also freezes the IWDG while the core is halted under a
 * debugger so single-stepping doesn't spuriously reset.
 */
void iwdg_init(void);

/**
 * @brief Refresh (kick) the IWDG counter. Call well within the timeout.
 *
 * Cheap single register write; safe from task or ISR context.
 */
void iwdg_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* IWDG_H */
