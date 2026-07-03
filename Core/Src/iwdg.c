/**
 * @file iwdg.c
 * @brief STM32H7 IWDG1 minimal register driver. See iwdg.h.
 */
#include "iwdg.h"
#include "main.h"   /* pulls CMSIS device header → IWDG1, DBGMCU, etc. */

/* IWDG key register magic values (RM0468 §47.4.1). */
#define IWDG_KEY_UNLOCK   0x00005555u  /* enable PR/RLR/WINR write access  */
#define IWDG_KEY_RELOAD   0x0000AAAAu  /* refresh the counter (kick)       */
#define IWDG_KEY_START    0x0000CCCCu  /* start the watchdog               */

/* Timeout budget: IWDG_PRESCALER_PR / IWDG_RELOAD_VALUE /
 * IWDG_TIMEOUT_MS_NOMINAL (~100 ms) live in iwdg.h as the single source
 * of truth, so the host test suite can assert them against the FS-Rules
 * limits. ~100 ms is ~10× the safety-monitor refresh period — generous
 * margin against scheduling jitter and the LSI's wide tolerance, while
 * still resetting a wedged CPU before the car travels far. */

void iwdg_init(void)
{
    /* Note on debugging: unlike the TIM-based software watchdog, the H7
     * IWDG does NOT freeze when the core is halted via a simple DBGMCU
     * register bit — debug/stop/standby freeze is a FLASH option-byte
     * setting (FLASH_OPTSR_FZ_IWDG_STOP / _SDBY). It is intentionally
     * not changed here (option bytes are persistent device config, not
     * firmware state). While single-stepping with the IWDG armed, keep
     * it refreshed or set the option byte once via the programmer. */

    IWDG1->KR  = IWDG_KEY_UNLOCK;       /* unlock PR/RLR for writing      */
    IWDG1->PR  = IWDG_PRESCALER_PR;     /* prescaler /16                  */
    IWDG1->RLR = IWDG_RELOAD_VALUE;     /* reload → timeout               */

    /* Wait until PR and RLR have been copied to the watchdog clock
     * domain (PVU/RVU clear). Bounded spin — these clear within a few
     * LSI cycles; the guard count prevents a hang if LSI is dead. */
    for (uint32_t guard = 0u;
         (IWDG1->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u && guard < 100000u;
         ++guard) {
        /* spin */
    }

    IWDG1->KR = IWDG_KEY_RELOAD;        /* load the counter               */
    IWDG1->KR = IWDG_KEY_START;         /* start — cannot be stopped now  */
}

void iwdg_refresh(void)
{
    IWDG1->KR = IWDG_KEY_RELOAD;
}
