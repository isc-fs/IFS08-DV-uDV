#include "can_interface.hpp"
#include "as_state.h"
#include "bench_stubs.h"   /* BENCH_STUB_RES (all 0 on dev) */
#include "pit_diag.h"      /* CAN_ID_PITDIAG_ARM + pit_diag_arm_from_can */
#include <cstring>
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "FreeRTOS.h"
#include "task.h"

/* CAN IDs — FDCAN3 (AMI + steering bus, local DV peripherals) */
static constexpr uint32_t CAN_ID_MISSION_SELECT    = 0x503u;
static constexpr uint32_t CAN_ID_MISSION_ACK       = 0x50Au; /* echo back to AMI */
/* ASSI status byte on the local FDCAN3 bus. The ECU->AMS VCU_heartbeat also
 * uses 0x100 but lives on the ACU bus (uDV FDCAN2) — different wire, no
 * clash; do NOT emit this id on FDCAN2. */
static constexpr uint32_t CAN_ID_ASSI              = 0x100u;

/* CAN IDs — FDCAN2 (ACU bus: ECU/VCU + AMS). The ECU<->uDV contract,
 * matching IFS08-CE-ECU Core/Inc/can/messages/*.def (ECU side is
 * HIL-validated, Block L). The ECU's own FDCAN2 is the same wire. */
static constexpr uint32_t CAN_ID_TS_ACTIVE         = 0x504u; /* ECU->uDV: byte0 bool, 100 ms */
static constexpr uint32_t CAN_ID_BRAKE_OVER_LIMIT  = 0x505u; /* ECU->uDV: byte0 bool verdict
                                                                (brake_raw > BrakeDvHardRaw), 100 ms */
static constexpr uint32_t CAN_ID_MOTOR_RPM         = 0x506u; /* ECU->uDV: int32 LE mechanical
                                                                shaft rpm (signed), 10 ms */
static constexpr uint32_t CAN_ID_TORQUE_CMD        = 0x507u; /* uDV->ECU: int32 LE percent
                                                                0..100, 20 ms; stale => 0 */
static constexpr uint32_t CAN_ID_R2D_REQUEST       = 0x510u; /* uDV->ECU: byte0 != 0 requests
                                                                DV ready-to-drive (after GO) */
static constexpr uint32_t CAN_ID_R2D_CONFIRM       = 0x511u; /* ECU->uDV: byte0 != 0 confirms
                                                                DV R2D latched (acyclic) */
static constexpr uint32_t CAN_ID_IMU               = 0x512u; /* uDV->ECU: ax/ay/az int16 mg +
                                                                gx int16 0.1 dps, 50 Hz.
                                                                0x5xx on purpose: telemetry
                                                                must never win arbitration
                                                                over the safety frames (the
                                                                dev-era 0x001 outranked
                                                                everything on the bus).
                                                                ECU-side .def still needed. */
/* |mechanical rpm| below this counts as vehicle standstill (0x506 decode). */
static constexpr int32_t  RPM_STANDSTILL           = 10;
/* Steering — LWS sensor → controller → uDV (RX), uDV → controller (TX) */
static constexpr uint32_t CAN_ID_STEERING          = 0x2B0u;
static constexpr uint32_t CAN_ID_STEER_MOTOR       = 0x010u;
static constexpr uint32_t CAN_ID_STEER_CMD         = 0x020u;
/* Steering controller → uDV feedback (RX, FDCAN3): DrivingDynamics1 @20 Hz.
 * Numerically 0x500 (same as the FDCAN1 data-logger DYN1 TX frame), but a
 * distinct frame on a different bus — keep its own name so the two never
 * read as the same message. See CAN_ID_DL_DYN1 for the TX counterpart. */
static constexpr uint32_t CAN_ID_STEER_FEEDBACK    = 0x500u;

/* CAN IDs — FDCAN1 (RES CANopen + DataLogger TX) */
static constexpr uint32_t RES_NODE_ID              = 0x11u;
static constexpr uint32_t CAN_ID_NMT               = 0x000u;
static constexpr uint32_t CAN_ID_RES_PDO_TX        = 0x180u + RES_NODE_ID; /* 0x191 */
static constexpr uint32_t CAN_ID_RES_BOOTUP        = 0x700u + RES_NODE_ID; /* 0x711 */
static constexpr uint32_t CAN_ID_DL_DYN1           = 0x500u; /* uDV → data logger TX (see CAN_ID_STEER_FEEDBACK for the 0x500 RX frame) */
static constexpr uint32_t CAN_ID_DL_DYN2           = 0x501u;
static constexpr uint32_t CAN_ID_DL_STATUS         = 0x502u;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/* Last ASSI status code emitted on CAN 0x100 (FS-Rules T14.9:
 * OFF=0x00 EMERGENCY=0x01 READY=0x02 DRIVING=0x03 FINISHED=0x04).
 * Cached on every sendAssiStatus() so the micro-ROS layer can mirror it
 * on /assi/state — the ROS topic then carries the identical byte as the
 * CAN frame, by construction (no second mapping to drift). */
static std::atomic<uint8_t> g_assi_status_code{0x00u};

namespace Can {

void init()
{
    // Configure FDCAN3 for both TX and RX
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,
        .FilterID2    = 0x000, // Accept all standard IDs
    };
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

void initRes()
{
    /* CubeMX only enables FDCAN1_IT1 — RX FIFO0 events are wired to IT0,
     * so it must be enabled explicitly here.  Priority 5 matches FDCAN3. */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    /* Dual filter: accept RES PDO (0x191) and RES boot-up (0x711). */
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_DUAL,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = CAN_ID_RES_PDO_TX,   /* 0x191 */
        .FilterID2    = CAN_ID_RES_BOOTUP,   /* 0x711 */
    };
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    sendNmtSetOperational();
}

void initEcu()
{
    /* FDCAN2 = the ACU bus (ECU/VCU + AMS). Pinned and CubeMX-initialised
     * since forever but never STARTED — which is why "0x504 was never
     * received" on the car: the ECU transmits on this wire, not on FDCAN3.
     *
     * Accept only the ECU->uDV contract range 0x504..0x511 (one RANGE
     * filter — StdFiltersNbr is 1). Everything else on the ACU bus (AMS
     * heartbeats etc.) is rejected so it can't flood the shared RX queue. */
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = CAN_ID_TS_ACTIVE,     /* 0x504 */
        .FilterID2    = CAN_ID_R2D_CONFIRM,   /* 0x511 */
    };
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    /* Second filter: the pit-diag arm frame (0x7DE) -> FIFO0. Outside the
     * 0x504..0x511 contract range, so it needs its own slot (StdFiltersNbr
     * is 2). Lets the pit tool enable the diag stream over CAN. */
    FDCAN_FilterTypeDef arm_filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 1,
        .FilterType   = FDCAN_FILTER_DUAL,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = CAN_ID_PITDIAG_ARM,   /* 0x7DE */
        .FilterID2    = CAN_ID_PITDIAG_ARM,
    };
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &arm_filter) != HAL_OK) {
        Error_Handler();
    }

    /* CubeMX only enables FDCAN2_IT1 (same class of gotcha as FDCAN1's
     * IT0 — see initRes). Instead of adding another IRQ handler, route the
     * RX-FIFO0 new-message interrupt onto the already-wired LINE1. */
    if (HAL_FDCAN_ConfigInterruptLines(&hfdcan2,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            FDCAN_INTERRUPT_LINE1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

void sendAccel(float accel)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];

    TxHeader.Identifier = CAN_ID_TORQUE_CMD;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    /* 0x507 UDV_torque_cmd is an INTEGER percent, int32 LE 0..100 (agreed
     * with the ECU 2026-07-03, superseding the original float32 idea — a
     * float bit-pattern decodes as a huge int32 and the ECU clamps it to
     * 100 %!). Input is the pipeline's normalised throttle [-1..1]; the
     * negative half (regen/brake) has no DV torque path, so it clamps to 0.
     * The ECU applies the same conditioning (neg -> 0, >100 -> 100). */
    float pct_f = accel * 100.0f;
    int32_t pct = (pct_f <= 0.0f)   ? 0
                : (pct_f >= 100.0f) ? 100
                : (int32_t)(pct_f + 0.5f);
    memcpy(&TxData[0], &pct, sizeof(pct));

    /* TX failure (queue full / bus passive / peripheral not started) is
     * recoverable — never call Error_Handler from here.  It used to hang
     * the MCU when this function ran from an ISR before FDCAN3 was
     * started; see the watchdog boot-hang fix. */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void sendR2dRequest(uint8_t request)
{
    /* 0x510 UDV_r2d_request: byte0 != 0 asks the ECU for the DV
     * ready-to-drive transition (only honoured while the ECU's own brake
     * sensor confirms hard braking — the 0x505 verdict). The ECU latches
     * the edge for the drive cycle and confirms on 0x511. */
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = CAN_ID_R2D_REQUEST,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_1,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };

    /* TX failure is recoverable — see sendAccel comment. */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, &request);
}

void sendIMU(const bmi088_scaled_t &imu)
{
    /* 0x512 uDV->ECU IMU broadcast on the ACU bus, 50 Hz (imu_task
     * downsamples the 400 Hz stream). Dev-era wire layout restored:
     *   [0..1] ax, [2..3] ay, [4..5] az   int16 LE, milli-g
     *   [6..7] gx                          int16 LE, 0.1 dps
     * Sent from imu_task (NOT ros_task) so the ECU keeps its IMU feed
     * with no DVPC/agent connected. Axis choice (gx) is the historical
     * contract — confirm against the board mounting at commissioning. */
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier = CAN_ID_IMU;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    const int16_t ax_scaled = (int16_t)(imu.ax_g * 1000.0f);
    const int16_t ay_scaled = (int16_t)(imu.ay_g * 1000.0f);
    const int16_t az_scaled = (int16_t)(imu.az_g * 1000.0f);
    const int16_t gx_scaled = (int16_t)(imu.gx_dps * 10.0f);

    memcpy(&TxData[0], &ax_scaled, sizeof(int16_t));
    memcpy(&TxData[2], &ay_scaled, sizeof(int16_t));
    memcpy(&TxData[4], &az_scaled, sizeof(int16_t));
    memcpy(&TxData[6], &gx_scaled, sizeof(int16_t));

    /* TX failure is recoverable — see sendAccel comment. */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void isr_push_rx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxdata) != HAL_OK)
        return;

    can_msg_t msg;
    msg.id  = rxh.Identifier;
    /* HAL_FDCAN_GetRxMessage returns DataLength as the DLC code (1..8 for the
     * classic frames this bus uses) — already the byte count, NOT a value that
     * needs the old >>16 unshift. The stale `>> 16` made msg.dlc read 0 on
     * every frame, so every `if (msg->dlc < N) break;` guard tripped: the AMI
     * mission (0x503) was dropped (no store, no 0x50A ACK), steering feedback
     * (0x500) and the e-stop decode never ran. */
    msg.dlc = (uint8_t)rxh.DataLength;
    memcpy(msg.data, rxdata, 8);
    /* Tag the message with its source FDCAN bus so downstream dispatch
     * can distinguish messages from multiple CAN peripherals. */
    if (hfdcan->Instance == FDCAN1) {
        msg.bus = 1;
    } else if (hfdcan->Instance == FDCAN2) {
        msg.bus = 2;
    } else if (hfdcan->Instance == FDCAN3) {
        msg.bus = 3;
    } else {
        msg.bus = 0;
    }

    /* Route to the per-bus queue:
     *   FDCAN1 = RES CANopen → resRxQueueHandle
     *   else   = AMI + steering bus → canRxQueueHandle */
    if (msg.bus == 1) {
        if (resRxQueueHandle != NULL) {
            osMessageQueuePut(resRxQueueHandle, &msg, 0, 0);
        }
    } else {
        osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
    }
}

void rx_dispatch(const can_msg_t *msg)    //CAN FDCAN3
{
    HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
    switch (msg->id)
    {
    case CAN_ID_STEER_FEEDBACK: /* 0x500 DV_DRIVING_DYNAMICS_1 — steering feedback,
                          * 20 Hz on the AMI+steering bus (steering's FDCAN1). */
        if (msg->dlc < 6U) break;   /* need bytes [2..5]; short frame = ignore */
        g_steer_angle_actual.store((int8_t)msg->data[2] * 0.5f);
        g_steer_angle_target.store((int8_t)msg->data[3] * 0.5f);
        g_steer_angle_motor.store((int8_t)msg->data[4] * 0.5f);
        /* Byte 5: stepper-driver state (ESTADO_MOTOR_*). EMERGENCIA (-1) is a
         * grave fault — app_task folds it into the AS emergency trigger. */
        g_steer_motor_state.store((int8_t)msg->data[5]);
        break;

    case CAN_ID_MISSION_SELECT: {
        if (msg->dlc < 1U) break;   /* malformed: no store, no ACK — AMI retries */
        g_can_mission_id.store((int)msg->data[0]);
        /* ACK the mission back to the AMI (0x50A, byte echoed verbatim).
         * The AMI retries 0x503 every 500 ms until it sees this echo, so
         * a lost frame in either direction just delays the handshake.
         * rx_dispatch runs in canTask context — safe to enqueue TX here. */
        FDCAN_TxHeaderTypeDef AckHeader = {
            .Identifier          = CAN_ID_MISSION_ACK,
            .IdType              = FDCAN_STANDARD_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = FDCAN_DLC_BYTES_1,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_OFF,
            .FDFormat            = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0,
        };
        uint8_t ack = msg->data[0];
        (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &AckHeader, &ack);
        break;
    }

    /* --- ECU (VCU) -> uDV contract frames ---
     * These populate the CAN atomics StateManager / EbsManager read. The
     * decode matches the ECU's .def files (see the id block above); the old
     * pre-#17 layouts (0x505 float32 bar, 0x506 SDC bool, 0x509 R2D) are
     * gone — the ECU never transmitted them. */
    case CAN_ID_TS_ACTIVE:               /* 0x504 — byte0 bool */
        if (msg->dlc >= 1U) {
            g_can_ts_active.store(msg->data[0] != 0U);
        }
        break;

    case CAN_ID_BRAKE_OVER_LIMIT:        /* 0x505 — byte0 bool verdict */
        if (msg->dlc >= 1U) {
            g_can_brake_over_limit.store(msg->data[0] != 0U);
        }
        break;

    case CAN_ID_MOTOR_RPM: {             /* 0x506 — int32 LE mechanical rpm */
        if (msg->dlc >= 4U) {
            int32_t rpm = 0;
            memcpy(&rpm, msg->data, sizeof(rpm));
            g_can_motor_rpm.store(rpm);
            /* Standstill for the FINISHED-vs-EMERGENCY decision: the shaft
             * not turning is the ground truth (previously never written). */
            g_imu_vehicle_standstill.store(rpm > -RPM_STANDSTILL &&
                                           rpm <  RPM_STANDSTILL);
        }
        break;
    }

    case CAN_ID_R2D_CONFIRM:             /* 0x511 — ECU latched DV R2D */
        if (msg->dlc >= 1U) {
            g_can_r2d.store(msg->data[0] != 0U);
        }
        break;

    case CAN_ID_PITDIAG_ARM:             /* 0x7DE — pit tool enables diag stream */
        pit_diag_arm_from_can(msg->data, msg->dlc);
        break;

    case CAN_ID_STEERING:
        //snprintf(srv_msg, sizeof(srv_msg), "id direcion detectado");
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
        /* LWS sensor packet on FDCAN3.  Byte layout (re-ported from v0.1):
         *   [0..1] angle_raw, int16 little-endian, 0.1 deg/bit
         *   [2]    speed_raw, uint8, 4 deg/s per bit
         *   [3]    status, bit0=OK / bit1=CAL / bit2=TRIM            */
        if (msg->dlc >= 4U) {
            // 1. Parseo y validación del Ángulo (Little Endian)
            uint16_t raw_u = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[0];
            int16_t raw_angle = (int16_t)raw_u;
            
            /* Store RAW wire units (0.1 deg/bit) — consumers scale.
             * Storing a scaled float truncated through the int16 atomic
             * and every reader re-scaled it (10x low). */
            if (raw_u != 0x7FFF) {
                g_steering_angle_raw.store(raw_angle);
            } else {
                g_steering_angle_raw.store(0);
            }

            // 2. Parseo y validación de la Velocidad (con signo int8_t)
            if (msg->data[2] != 0xFF) {
                /* Raw wire units too (4 deg/s per bit) — *4 overflowed
                 * the byte-wide atomic for |speed| > 63. */
                g_steering_speed_raw.store((int8_t)msg->data[2]);
            } else {
                g_steering_speed_raw.store(0);
            }

            // 3. Estado (Si quieres seguir guardando el byte crudo)
            g_steering_status.store(msg->data[3]);
            
            // 4. Timestamp
            g_steering_last_rx_tick.store(osKernelGetTickCount());
        }
        
        break;

    default:
        break;
    }
}

void resRxDispatch(const can_msg_t *msg)  //CAN FDCAN1
{
    /* Count every frame through the FDCAN1 filter (0x191 + 0x711): a nonzero
     * count means FDCAN1 RX is alive and the RES is on the bus, even before the
     * 0x191 PDO streams. Read on pit-diag 0x7A5 to split "dead bus" from
     * "RES silent". */
    g_res_rx_frame_count.fetch_add(1, std::memory_order_relaxed);

    switch (msg->id)
    {
    case CAN_ID_RES_PDO_TX: {
        if (msg->dlc < 1U) break;   /* dlc=0 is legal CAN — don't decode e-stop from garbage */
        g_res_raw0.store(msg->data[0]);   /* raw byte for pit-diag 0x7A1 */
        g_res_estop.store((msg->data[0] & 0x01U) == 0U);
        g_res_go_signal.store((msg->data[0] & 0x04U) != 0U ? 1U : 0U);
        if (msg->dlc >= 7U) g_res_radio_quality.store(msg->data[6]);
        if (msg->dlc >= 8U) g_res_pre_alarm.store(((msg->data[7] >> 6) & 0x01U) != 0U);
        g_res_last_rx_tick.store(osKernelGetTickCount());
        //Para hacer testing si no fucniona ros
        //HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin,g_res_go_signal.load() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    }

    case CAN_ID_RES_BOOTUP:
        /* The RES announces its boot-up once (0x711, data[0]==0x00) and we
         * answer with NMT set-operational so it starts streaming its 0x191
         * PDO (which carries GO / e-stop). This handles the RES-boots-while-
         * we-are-listening case; the power-up-order case (uDV reset AFTER the
         * RES, so this one-shot frame is missed) is covered by the periodic
         * re-arm in can_task — see StartCanTask. */
        if (msg->dlc >= 1U && msg->data[0] == 0x00U)
        {
            sendNmtSetOperational();
        }
        break;

    default:
        break;
    }
}

void sendNmtSetOperational()
{
    /* NMT "start remote node" broadcast (node-id 0 = ALL nodes). Starts the RES
     * into OPERATIONAL regardless of its configured node ID — robust for a
     * single-RES bus and removes the node-id assumption behind 0x191/0x711.
     * Was addressed to RES_NODE_ID (0x11); if the RES ID ever differed, the
     * targeted NMT was ACKed but ignored. */
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = CAN_ID_NMT,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_2,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    uint8_t nmt[2] = { 0x01u, 0x00u };   /* 0x00 = broadcast (all nodes) */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, nmt);
    g_nmt_sent_count.fetch_add(1, std::memory_order_relaxed);  /* pit-diag 0x7A5 */
}

/* 0x010 motor control, 4-byte payload: byte[0] is motor_start (rest 0).
 * The steering controller reads only data[0] (len>=1) and runs the motor
 * ONLY when data[0] == MOTOR_CMD_ON (1); any other value is a clean stop
 * (re-homes on the next 0x10=1). See IFS08-DV-STEERING@fix/2-can-timing-ram-
 * safety: comunicacion_direccion.c `motor_start = data[0]` + main.c
 * `if(motor_start != MOTOR_CMD_ON)`. So start=1, stop=0 (0 = the natural off,
 * matching DIR_INACTIVO). */
static void sendSteeringMotorCmd(uint8_t motor_start)
{
    uint8_t data[4] = { motor_start, 0u, 0u, 0u };
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = CAN_ID_STEER_MOTOR,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_4,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, data);
}

void sendSteeringStart() { sendSteeringMotorCmd(1u); }
void sendSteeringStop()  { sendSteeringMotorCmd(0u); }

void sendSteeringAngle(float angle_deg)
{
    /* Scale: int32 LE in 0.01-deg units (matches dev's steering_angle_cmd) */
    int32_t scaled = (int32_t)(angle_deg * 100.0f);
    uint8_t data[4];
    data[0] = (uint8_t)(scaled & 0xFFu);
    data[1] = (uint8_t)((scaled >> 8) & 0xFFu);
    data[2] = (uint8_t)((scaled >> 16) & 0xFFu);
    data[3] = (uint8_t)((scaled >> 24) & 0xFFu);

    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = CAN_ID_STEER_CMD,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_4,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, data);
}

void sendDataLogger()
{
    /* 0x500 — DV driving dynamics 1 (8 bytes) */
    uint8_t d500[8] = {0};
    /* Steering_angle_actual: byte 2, signed 0.5 deg/bit.
     * Convert from g_steering raw (0.1 deg/bit) → degrees → 0.5-deg ticks. */
    int8_t steer_scaled =
        (int8_t)((int32_t)(g_steering_angle_raw.load() * 0.1f / 0.5f));
    d500[2] = (uint8_t)steer_scaled;

    FDCAN_TxHeaderTypeDef hdr500 = {
        .Identifier          = CAN_ID_DL_DYN1,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr500, d500);

    /* 0x501 — DV driving dynamics 2 (6 bytes), populated when IMU
     * fields are mapped to the spec.  Placeholder zeros for now. */
    uint8_t d501[6] = {0};
    FDCAN_TxHeaderTypeDef hdr501 = hdr500;
    hdr501.Identifier = CAN_ID_DL_DYN2;
    hdr501.DataLength = FDCAN_DLC_BYTES_6;
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr501, d501);

    /* 0x502 — DV system status (5 bytes) */
    uint8_t d502[5] = {0};
    /* AS_status low nibble (FS DV-logger encoding): 1=OFF 2=READY
     * 3=DRIVING 4=EMERGENCY 5=FINISHED. Mapped from the live AS state
     * machine (ros_get_as_state(): ASState OFF=0 READY=1 DRIVING=2
     * EMERGENCY=3 FINISHED=4). CONFIRM against the team CAN DBC. */
    static const uint8_t k_as_to_dl_nibble[5] = {1u, 2u, 3u, 4u, 5u};
    uint8_t as = ros_get_as_state();
    d502[0] = (as < 5u) ? k_as_to_dl_nibble[as] : 1u;
    /* AMI_state bits 5-7: derived from mission id (1..7 maps to slots 1..7) */
    int mid = g_can_mission_id.load();
    if (mid >= 0 && mid < 7) {
        d502[0] |= (uint8_t)(((mid + 1) & 0x07u) << 5);
    }
    /* Steering_state bit (byte 1, bit 0): available if status OK */
    d502[1] = (uint8_t)(g_steering_status.load() & 0x01u);

    FDCAN_TxHeaderTypeDef hdr502 = hdr500;
    hdr502.Identifier = CAN_ID_DL_STATUS;
    hdr502.DataLength = FDCAN_DLC_BYTES_5;
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr502, d502);
}

void sendAssiStatus(uint8_t status)
{
    /* ASSI state on FDCAN3 ID 0x100, 1-byte payload (FS-Rules T14.9:
     * OFF=0x00 EMERGENCY=0x01 READY=0x02 DRIVING=0x03 FINISHED=0x04).
     * The ASSI peripheral does the flashing/buzzer; the uDV only emits
     * the state code. Non-blocking enqueue, safe from any context. */
    /* Mirror the same byte to the micro-ROS /assi/state publisher. */
    g_assi_status_code.store(status);
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = CAN_ID_ASSI,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_1,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    uint8_t payload = status;
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, &payload);
}

} // namespace Can

// C-compatible wrapper for ISR — routes both FDCAN1 (RES bus) and
// FDCAN3 (AMI + steering bus) into the dispatcher.  isr_push_rx
// itself decides which queue the frame ends up in based on hfdcan.
/* HAL weak-function overrides — called by HAL_FDCAN_IRQHandler from stm32h7xx_it.c */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        Can::isr_push_rx(hfdcan);
}

extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    /* Re-arm RX notifications silenced by HAL on overrun */
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

extern "C" void can_interface_rx_isr_callback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1 || hfdcan->Instance == FDCAN3) {
        Can::isr_push_rx(hfdcan);
    }
}

/* C-callable ASSI EMERGENCY emitter for the safety monitor (and any ISR
 * path). Sends the T14.9 EMERGENCY code (0x01) on 0x100 — non-blocking
 * FIFO enqueue, so it is safe from the supervisor task or an ISR. */
extern "C" void can_interface_send_assi_emergency_from_isr(void)
{
    Can::sendAssiStatus(0x01u);
}

/* C-callable accessors so freertos.c (C file) can read atomic CAN state */
extern "C" float can_c_get_steer_angle_actual(void)
{
    return g_steer_angle_actual.load();
}

extern "C" float can_c_get_steer_angle_target(void)
{
    return g_steer_angle_target.load();
}

extern "C" float can_c_get_steer_angle_motor(void)
{
    return g_steer_angle_motor.load();
}

extern "C" int8_t can_c_get_steer_motor_state(void)
{
    return g_steer_motor_state.load();
}

extern "C" float can_c_get_steering_angle_deg(void)
{
    return (float)g_steering_angle_raw.load() * 0.1f;
}

extern "C" int32_t can_c_get_res_status(uint32_t now_tick, uint32_t timeout_ms)
{
    uint32_t last = g_res_last_rx_tick.load();
    /* Bench stub (bench_stubs.h, 0 on dev — folds away): with no RES box on
     * the bench, no 0x191 ever arrives, so `last` stays 0 and the real logic
     * below returns -2 (never received) -> res_ok false -> READY unreachable.
     * Report a healthy OK link instead so the bench can arm to READY. Only
     * while NOTHING real has been seen: once a real 0x191 lands, `last` != 0
     * and the genuine e-stop/GO/timeout logic takes over (a CAN tool sending a
     * real GO still crosses the GO edge). */
    if (BENCH_STUB_RES && last == 0U)            return  0; /* stub: healthy OK */
    if (last == 0U)                              return -2; /* nunca recibido */
    if ((now_tick - last) > timeout_ms)          return -1; /* timeout */
    if (g_res_estop.load())                      return  1; /* E-Stop activo */
    if (g_res_go_signal.load())                      return  2; /* Go signal activo */
    return 0;
}

extern "C" int32_t can_c_get_mission_index(void)
{
    return (int32_t)g_can_mission_id.load();
}

extern "C" uint8_t can_c_get_go_signal(void)
{
    return g_res_go_signal.load();
}

extern "C" int32_t can_c_get_motor_rpm(void)
{
    /* Mechanical shaft rpm from the ECU (0x506, 10 ms). The ECU already
     * divides the inverter's electrical rpm by the pole-pair count. */
    return g_can_motor_rpm.load();
}

extern "C" void can_c_send_imu(const bmi088_scaled_t *imu)
{
    /* C shim for imu_task (C file) — see Can::sendIMU. */
    Can::sendIMU(*imu);
}

extern "C" uint8_t can_c_get_assi_status_code(void)
{
    /* Last AS state byte emitted on CAN 0x100 (FS-Rules T14.9). The
     * micro-ROS /assi/state publisher mirrors this so the DV pipeline
     * reads the identical code the ASSI peripheral does. */
    return g_assi_status_code.load();
}

