#include "can_service.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* Map FDCAN DataLength code to actual byte count (matches HAL driver DLCtoBytes) */
static inline uint8_t dlc_code_to_bytes(uint32_t code)
{
    static const uint8_t map[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    if (code < 16) return map[code];
    return 0;
}

/* ---------------------------------------------------------------------------
 * CAN IDs
 * --------------------------------------------------------------------------- */
/* FDCAN3 — AMI + Steering bus */
#define CAN_ID_MISSION_SELECT  0x503u
#define CAN_ID_STEERING        0x2B0u  /* LWS sensor → steering controller → uDV */
#define CAN_ID_STEER_MOTOR     0x010u  /* uDV → steering: motor start/stop       */
#define CAN_ID_STEER_CMD       0x020u  /* uDV → steering: desired angle           */

/* FDCAN1 — RES CANopen (Node-ID 0x03) */
#define RES_NODE_ID            0x11u   /* Note: Change to 0x11 for FSG event target */
#define CAN_ID_NMT             0x000u
#define CAN_ID_RES_PDO_TX      (0x180u + RES_NODE_ID)  /* 0x183 */
#define CAN_ID_RES_BOOTUP      (0x700u + RES_NODE_ID)  /* 0x703 */

/* FDCAN1 — Data Logger (DS 2.2, 100 ms cycle) */
#define CAN_ID_DL_DYN1         0x500u
#define CAN_ID_DL_DYN2         0x501u
#define CAN_ID_DL_STATUS       0x502u

/* ---------------------------------------------------------------------------
 * Shared globals
 * --------------------------------------------------------------------------- */
volatile steering_data_t g_steering = {0};
volatile res_status_t    g_res      = {0};

/* ---------------------------------------------------------------------------
 * FDCAN3 runtime bring-up (AMI + steering bus)
 * --------------------------------------------------------------------------- */
void can_service_init(void)
{
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,
        .FilterID2    = 0x000,          /* pass-all */
    };
    HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ---------------------------------------------------------------------------
 * FDCAN1 runtime bring-up (RES CANopen bus)
 * --------------------------------------------------------------------------- */
void res_service_init(void)
{
    /* Enable FDCAN1 IT0 for RX FIFO0 (CubeMX only enables IT1) */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    /* Filter 0: accept RES PDO TX (0x183) */
    FDCAN_FilterTypeDef filter0 = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,   /* pass-all mask for testing and diagnostic */
        .FilterID2    = 0x000,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter0);

    /* Accept non-matching standard IDs (diagnostic mode). */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ---------------------------------------------------------------------------
 * Generic CAN TX
 * --------------------------------------------------------------------------- */
void can_tx_send(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                 const uint8_t *data, uint8_t dlc)
{
    FDCAN_TxHeaderTypeDef txh = {
        .Identifier          = id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = ((uint32_t)dlc << 16),  /* FDCAN DLC encoding */
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, (uint8_t *)data);
}

/* ---------------------------------------------------------------------------
 * RES CANopen NMT master
 * --------------------------------------------------------------------------- */
void res_nmt_set_operational(void)
{
    /* NMT command: set addressed node(s) to operational.
     * Changed target node from 0x00 (Broadcast) to RES_NODE_ID (0x03) 
     * to eliminate edge-case ignores inside the GrossFunk hardware. */
    uint8_t nmt[2] = { 0x01, RES_NODE_ID };
    can_tx_send(&hfdcan1, CAN_ID_NMT, nmt, 2);
    {
        char dbg[64];
        snprintf(dbg, sizeof(dbg), "#sym:res_nmt_set_operational cmd=0x%02X node=0x%02X", nmt[0], nmt[1]);
        (void)osMessageQueuePut(debugQueueHandle, dbg, 0, 0);
    }
}

/* ---------------------------------------------------------------------------
 * RES message dispatcher
 * --------------------------------------------------------------------------- */
void res_rx_dispatch(const can_msg_t *msg)
{
    /* Publish generic debug for every RES message */
    {
        char dbg[128];
        int n = snprintf(dbg, sizeof(dbg), "CAN%u id=0x%03X dlc=%u data=", (unsigned)msg->bus, (unsigned)msg->id, (unsigned)msg->dlc);
        int off = (n > 0 && n < (int)sizeof(dbg)) ? n : 0;
        for (int i = 0; i < msg->dlc && i < 4 && off < (int)sizeof(dbg) - 3; ++i) {
            off += snprintf(&dbg[off], sizeof(dbg) - off, "%02X ", msg->data[i]);
        }
        (void)osMessageQueuePut(debugQueueHandle, dbg, 0, 0);
    }
    switch (msg->id)
    {
    case CAN_ID_RES_PDO_TX:   /* 0x183 — 8-byte cyclic PDO, every 30 ms */
        g_res.e_stop        = (msg->data[0] & 0x01);       /* PDO 2000 bit 0 */
        g_res.go_signal     = (msg->data[0] >> 1) & 0x03;  /* K2 bits 1-2    */
        g_res.radio_quality = msg->data[6];                 /* PDO 2006       */
        g_res.pre_alarm     = (msg->data[7] >> 6) & 0x01;  /* PDO 2007 bit 6 */
        g_res.last_rx_tick  = osKernelGetTickCount();
        {
            char ev[64];
            int n = snprintf(ev, sizeof(ev), "#sym:CAN_ID_RES_PDO_TX dlc=%u ", (unsigned)msg->dlc);
            int off = (n > 0 && n < (int)sizeof(ev)) ? n : 0;
            for (int i = 0; i < msg->dlc && i < 4 && off < (int)sizeof(ev) - 3; ++i) {
                off += snprintf(&ev[off], sizeof(ev) - off, "%02X ", msg->data[i]);
            }
            (void)osMessageQueuePut(debugQueueHandle, ev, 0, 0);
        }
        break;
        
    case CAN_ID_RES_BOOTUP:   /* 0x703 — boot-up message */
        /* RES just booted, set it to operational */
        res_nmt_set_operational();
        {
            char ev[64];
            snprintf(ev, sizeof(ev), "#sym:CAN_ID_RES_BOOTUP id=0x%03X", (unsigned)msg->id);
            (void)osMessageQueuePut(debugQueueHandle, ev, 0, 0);
        }
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * ISR bridge: read frames from FIFO0 in a loop and push to the right queue
 * --------------------------------------------------------------------------- */
extern osThreadId_t amiTaskHandle;
extern volatile uint8_t g_mission_index;

void can_isr_push_rx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    /* WHILE loop extracts all hardware buffered back-to-back frames 
     * preventing downstream task schedules from missing immediate bursts */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxdata) != HAL_OK)
            continue;

        can_msg_t msg;
        msg.id  = rxh.Identifier;
        uint8_t bytes = dlc_code_to_bytes(rxh.DataLength);
        msg.dlc = bytes;
        
        memset(msg.data, 0, sizeof(msg.data));
        if (bytes > 0) memcpy(msg.data, rxdata, (bytes <= 8) ? bytes : 8);

        if (hfdcan == &hfdcan1) msg.bus = 1;
        else if (hfdcan == &hfdcan2) msg.bus = 2;
        else if (hfdcan == &hfdcan3) msg.bus = 3;
        else msg.bus = 0;

        if (hfdcan == &hfdcan1)
            osMessageQueuePut(resRxQueueHandle, &msg, 0, 0);
        else
            osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
    }
}

/* ---------------------------------------------------------------------------
 * FDCAN3 message dispatcher (AMI + steering)
 * --------------------------------------------------------------------------- */
void can_rx_dispatch(const can_msg_t *msg)
{
    /* Publish generic debug for every non-RES CAN message */
    {
        char dbg[128];
        int n = snprintf(dbg, sizeof(dbg), "CAN%u id=0x%03X dlc=%u data=", (unsigned)msg->bus, (unsigned)msg->id, (unsigned)msg->dlc);
        int off = (n > 0 && n < (int)sizeof(dbg)) ? n : 0;
        for (int i = 0; i < msg->dlc && i < 4 && off < (int)sizeof(dbg) - 3; ++i) {
            off += snprintf(&dbg[off], sizeof(dbg) - off, "%02X ", msg->data[i]);
        }
        (void)osMessageQueuePut(debugQueueHandle, dbg, 0, 0);
    }
    switch (msg->id)
    {
    case CAN_ID_MISSION_SELECT:
        g_mission_index = msg->data[0];
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
        if (amiTaskHandle != NULL)
            xTaskNotifyGive((TaskHandle_t)amiTaskHandle);
        break;

    case CAN_ID_STEERING:
        g_steering.angle_raw = (int16_t)(msg->data[0] | ((uint16_t)msg->data[1] << 8));
        g_steering.speed_raw = msg->data[2];
        g_steering.status    = msg->data[3];
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Steering TX commands (FDCAN3)
 * --------------------------------------------------------------------------- */
void steering_motor_cmd(uint8_t start)
{
    can_tx_send(&hfdcan3, CAN_ID_STEER_MOTOR, &start, 1);
}

void steering_angle_cmd(float angle_deg)
{
    int32_t scaled = (int32_t)(angle_deg * 100.0f);
    uint8_t data[4];
    data[0] = (uint8_t)(scaled & 0xFF);
    data[1] = (uint8_t)((scaled >> 8) & 0xFF);
    data[2] = (uint8_t)((scaled >> 16) & 0xFF);
    data[3] = (uint8_t)((scaled >> 24) & 0xFF);
    can_tx_send(&hfdcan3, CAN_ID_STEER_CMD, data, 4);
}

/* ---------------------------------------------------------------------------
 * Data Logger TX (0x500, 0x501, 0x502) — DS 2.2, 100 ms cycle
 * --------------------------------------------------------------------------- */
void datalogger_tx(void)
{
    /* 0x500 — DV driving dynamics 1 (8 B) */
    uint8_t d500[8] = {0};
    int8_t steer_scaled = (int8_t)((g_steering.angle_raw * 0.1f) / 0.5f);
    d500[2] = (uint8_t)steer_scaled;
    can_tx_send(&hfdcan1, CAN_ID_DL_DYN1, d500, 8);

    /* 0x501 — DV driving dynamics 2 (6 B) */
    uint8_t d501[6] = {0};
    can_tx_send(&hfdcan1, CAN_ID_DL_DYN2, d501, 6);

    /* 0x502 — DV system status (5 B) */
    uint8_t d502[5] = {0};
    d502[0] = 1;
    if (g_mission_index != 0xFF && g_mission_index < 7)
        d502[0] |= ((g_mission_index + 1) & 0x07) << 5;
    d502[1] = (g_steering.status & 0x01);
    can_tx_send(&hfdcan1, CAN_ID_DL_STATUS, d502, 5);
}

/* ---------------------------------------------------------------------------
 * HAL weak callback overrides — routes and secures FDCAN hardware interrupts
 * --------------------------------------------------------------------------- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        can_isr_push_rx(hfdcan);
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    /* Critical Recovery Guard: If the HAL engine flags an Overrun error, 
     * it silently shuts down notifications. We override and force them back alive. */
    if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
    else if (hfdcan->Instance == hfdcan3.Instance)
    {
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}