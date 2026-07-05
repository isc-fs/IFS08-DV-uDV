/**
 * @file    mission_registry.cpp
 * @brief   AMI-code -> Mission lookup table (the single dispatch point).
 *
 * The one place that maps the 0-based AMI mission code (CAN 0x503 byte[0]) to a
 * Mission. app_task calls mission_for_code() once per tick; adding or repointing
 * a mission is a one-line edit here. Each real mission lives in its own TU and
 * is referenced by extern; the trivial no-op MANUAL is defined inline.
 */
#include "mission.h"

/* Mission singletons defined in their own translation units. */
extern const Mission mission_inspection;   /* mission_inspection.cpp — code 6 */
extern const Mission mission_pipeline;     /* mission_pipeline.cpp   — codes 1-4 */
extern const Mission mission_ebstest;      /* mission_ebstest.cpp    — code 5 */

namespace {

/* MANUAL (code 0): autonomous system selected but the car is driven manually —
 * no actuation, never self-finishes. A genuine no-op needs no own file. */
const Mission mission_manual = {
    "manual",   /* name           */
    false,      /* needs_pipeline */
    nullptr,    /* on_enter       */
    nullptr,    /* on_tick        */
    nullptr,    /* is_complete    */
    nullptr,    /* on_exit        */
};

/* Indexed by AmiMission code. nullptr = no uDV mission for that code; app_task
 * refuses the GO (READY->DRIVING) edge for a nullptr mission, so SHUTDOWN and
 * the undefined aux codes can never release the brakes. Order MUST match the
 * AmiMission enum in mission.h. */
const Mission* const k_by_code[] = {
    &mission_manual,      /* 0  MISSION_MANUAL                       */
    &mission_pipeline,    /* 1  MISSION_ACCEL      (pipeline)         */
    &mission_pipeline,    /* 2  MISSION_SKIDPAD    (pipeline)         */
    &mission_pipeline,    /* 3  MISSION_AUTOCROSS  (pipeline)         */
    &mission_pipeline,    /* 4  MISSION_TRACKDRIVE (pipeline)         */
    &mission_ebstest,     /* 5  MISSION_EBS_TEST                      */
    &mission_inspection,  /* 6  MISSION_INSPECTION                    */
    nullptr,              /* 7  MISSION_SHUTDOWN — not a drive mission */
    nullptr,              /* 8  aux1 (AMI menu only, no uDV mission)   */
    nullptr,              /* 9  aux2 (AMI menu only, no uDV mission)   */
};

constexpr int k_num_codes = (int)(sizeof(k_by_code) / sizeof(k_by_code[0]));

} // namespace

const Mission* mission_for_code(int ami_code)
{
    if (ami_code < 0 || ami_code >= k_num_codes)
        return nullptr;
    return k_by_code[ami_code];
}
