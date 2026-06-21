#include "can_interface.hpp"
#include <cstring>
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "FreeRTOS.h"
#include "task.h"

/* CAN IDs — FDCAN3 (AMI + steering bus + autonomy control) */
static constexpr uint32_t CAN_ID_MISSION_SELECT    = 0x503u;
static constexpr uint32_t CAN_ID_TS_ACTIVE         = 0x504u;
static constexpr uint32_t CAN_ID_BRAKE_PRESSURE    = 0x505u;
static constexpr uint32_t CAN_ID_SDC_RES_OPEN      = 0x506u;
static constexpr uint32_t CAN_ID_CONTROL_ACCEL     = 0x507u;
static constexpr uint32_t CAN_ID_CONTROL_STEER     = 0x508u;
static constexpr uint32_t CAN_ID_R2D               = 0x509u;
static constexpr uint32_t CAN_ID_ASSI              = 0x100u;
static constexpr uint32_t CAN_ID_IMU               = 0x001u;
/* Steering — LWS sensor → controller → uDV (RX), uDV → controller (TX) */
static constexpr uint32_t CAN_ID_STEERING          = 0x2B0u;
static constexpr uint32_t CAN_ID_STEER_MOTOR       = 0x010u;
static constexpr uint32_t CAN_ID_STEER_CMD         = 0x020u;

/* CAN IDs — FDCAN1 (RES CANopen + DataLogger TX) */
static constexpr uint32_t RES_NODE_ID              = 0x11u;
static constexpr uint32_t CAN_ID_NMT               = 0x000u;
static constexpr uint32_t CAN_ID_RES_PDO_TX        = 0x180u + RES_NODE_ID; /* 0x191 */
static constexpr uint32_t CAN_ID_RES_BOOTUP        = 0x700u + RES_NODE_ID; /* 0x711 */
static constexpr uint32_t CAN_ID_DL_DYN1           = 0x500u;
static constexpr uint32_t CAN_ID_DL_DYN2           = 0x501u;
static constexpr uint32_t CAN_ID_DL_STATUS         = 0x502u;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;

/* amiTask handle — defined in freertos.c */
extern osThreadId_t amiTaskHandle;

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

void sendControl(float accel, float steer)
{
    // Backwards-compatible helper: send two separate frames for accel and steer
    sendAccel(accel);
    sendSteer(steer);
}

void sendAccel(float accel)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];

    TxHeader.Identifier = CAN_ID_CONTROL_ACCEL;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    memcpy(&TxData[0], &accel, sizeof(float));

    /* TX failure (queue full / bus passive / peripheral not started) is
     * recoverable — never call Error_Handler from here.  It used to hang
     * the MCU when this function ran from an ISR before FDCAN3 was
     * started; see the watchdog boot-hang fix. */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}

void sendSteer(float steer)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];

    TxHeader.Identifier = CAN_ID_CONTROL_STEER;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    memcpy(&TxData[0], &steer, sizeof(float));

    /* TX failure is recoverable — see sendAccel comment. */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}

void isr_push_rx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxdata) != HAL_OK)
        return;

    can_msg_t msg;
    msg.id  = rxh.Identifier;
    msg.dlc = (uint8_t)(rxh.DataLength >> 16);
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
    switch (msg->id)
    {
    case CAN_ID_MISSION_SELECT:
        g_can_mission_id.store((int)msg->data[0]);
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
        if (amiTaskHandle != NULL) {
            xTaskNotifyGive((TaskHandle_t)amiTaskHandle);
        }
        break;
    case CAN_ID_STEERING:
        /* LWS sensor packet on FDCAN3.  Byte layout (re-ported from v0.1):
         *   [0..1] angle_raw, int16 little-endian, 0.1 deg/bit
         *   [2]    speed_raw, uint8, 4 deg/s per bit
         *   [3]    status, bit0=OK / bit1=CAL / bit2=TRIM            */
        if (msg->dlc >= 4U) {
            // 1. Parseo y validación del Ángulo (Little Endian)
            uint16_t raw_u = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[0];
            int16_t raw_angle = (int16_t)raw_u;
            
            if (raw_u != 0x7FFF) {
                g_steering_angle_raw.store(raw_angle * 0.1f); // Requiere que g_steering_angle_raw sea float
            } else {
                g_steering_angle_raw.store(0.0f); 
            }

            // 2. Parseo y validación de la Velocidad (con signo int8_t)
            if (msg->data[2] != 0xFF) {
                int8_t raw_speed = (int8_t)msg->data[2];
                g_steering_speed_raw.store(raw_speed * 4.0f); // Requiere que g_steering_speed_raw sea float
            } else {
                g_steering_speed_raw.store(0.0f);
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
    switch (msg->id)
    {
    case CAN_ID_RES_PDO_TX: {
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
        sendNmtSetOperational();
        break;

    default:
        break;
    }
}

void sendNmtSetOperational()
{
    /* NMT "set operational" command for node 0x11. */
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
    uint8_t nmt[2] = { 0x01u, (uint8_t)RES_NODE_ID };
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, nmt);
}

void sendSteeringMotor(uint8_t start)
{
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = CAN_ID_STEER_MOTOR,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_1,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    uint8_t payload = start;
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, &payload);
}

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
    /* AS_status low nibble: 1 = OFF (placeholder until state_manager wires) */
    d502[0] = 1u;
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

/* C-callable accessors so freertos.c (C file) can read atomic CAN state */
extern "C" float can_c_get_steering_angle_deg(void)
{
    return (float)g_steering_angle_raw.load() * 0.1f;
}

extern "C" int32_t can_c_get_res_status(uint32_t now_tick, uint32_t timeout_ms)
{
    uint32_t last = g_res_last_rx_tick.load();
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

