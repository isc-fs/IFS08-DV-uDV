#include "can_service.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * CAN IDs
 * --------------------------------------------------------------------------- */
/* FDCAN3 — AMI + Steering bus */
#define CAN_ID_MISSION_SELECT  0x503u
#define CAN_ID_STEERING        0x2B0u  /* LWS sensor → steering controller → uDV */
#define CAN_ID_STEER_MOTOR     0x010u  /* uDV → steering: motor start/stop       */
#define CAN_ID_STEER_CMD       0x020u  /* uDV → steering: desired angle           */

/* FDCAN1 — RES CANopen (Node-ID 0x11) */
#define RES_NODE_ID            0x11u
#define CAN_ID_NMT             0x000u
#define CAN_ID_RES_PDO_TX      (0x180u + RES_NODE_ID)  /* 0x191 */
#define CAN_ID_RES_BOOTUP      (0x700u + RES_NODE_ID)  /* 0x711 */

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

    /* Filter 0: accept RES PDO TX (0x191) */
    FDCAN_FilterTypeDef filter0 = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_DUAL,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = CAN_ID_RES_PDO_TX,   /* 0x191 */
        .FilterID2    = CAN_ID_RES_BOOTUP,    /* 0x711 */
    };
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter0);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_REJECT,           /* reject non-matching std */
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
    /* NMT command: set node 0x11 to operational */
    uint8_t nmt[2] = { 0x01, RES_NODE_ID };
    can_tx_send(&hfdcan1, CAN_ID_NMT, nmt, 2);
}

/* ---------------------------------------------------------------------------
 * RES message dispatcher
 * --------------------------------------------------------------------------- */
void res_rx_dispatch(const can_msg_t *msg)
{
    switch (msg->id)
    {
    case CAN_ID_RES_PDO_TX:   /* 0x191 — 8-byte cyclic PDO, every 30 ms */
        g_res.e_stop        = (msg->data[0] & 0x01);       /* PDO 2000 bit 0 */
        g_res.go_signal     = (msg->data[0] >> 1) & 0x03;  /* K2 bits 1-2    */
        g_res.radio_quality = msg->data[6];                 /* PDO 2006       */
        g_res.pre_alarm     = (msg->data[7] >> 6) & 0x01;  /* PDO 2007 bit 6 */
        g_res.last_rx_tick  = osKernelGetTickCount();
        break;

    case CAN_ID_RES_BOOTUP:   /* 0x711 — boot-up message */
        /* RES just booted, set it to operational */
        res_nmt_set_operational();
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * ISR bridge: read frame from FIFO0 and push to the right queue
 * --------------------------------------------------------------------------- */
extern osThreadId_t amiTaskHandle;
extern volatile uint8_t g_mission_index;

void can_isr_push_rx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxdata) != HAL_OK)
        return;

    can_msg_t msg;
    msg.id  = rxh.Identifier;
    msg.dlc = (uint8_t)(rxh.DataLength >> 16);
    memcpy(msg.data, rxdata, 8);

    if (hfdcan == &hfdcan1)
        osMessageQueuePut(resRxQueueHandle, &msg, 0, 0);
    else
        osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
}

/* ---------------------------------------------------------------------------
 * FDCAN3 message dispatcher (AMI + steering)
 * --------------------------------------------------------------------------- */
void can_rx_dispatch(const can_msg_t *msg)
{
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
 * For this release, most fields are zero/placeholder.
 * Steering angle and IMU data are filled from available globals.
 * --------------------------------------------------------------------------- */
void datalogger_tx(void)
{
    /* 0x500 — DV driving dynamics 1 (8 B) */
    uint8_t d500[8] = {0};
    /* Steering_angle_actual: bits 16-23, signed, scale 0.5 deg */
    int8_t steer_scaled = (int8_t)((g_steering.angle_raw * 0.1f) / 0.5f);
    d500[2] = (uint8_t)steer_scaled;
    can_tx_send(&hfdcan1, CAN_ID_DL_DYN1, d500, 8);

    /* 0x501 — DV driving dynamics 2 (6 B) — populated from IMU later */
    uint8_t d501[6] = {0};
    can_tx_send(&hfdcan1, CAN_ID_DL_DYN2, d501, 6);

    /* 0x502 — DV system status (5 B) */
    uint8_t d502[5] = {0};
    /* AS_status bits 0-2: 1=off for now */
    d502[0] = 1;
    /* AMI_state bits 5-7: map mission index */
    if (g_mission_index != 0xFF && g_mission_index < 7)
        d502[0] |= ((g_mission_index + 1) & 0x07) << 5;
    /* Steering_state bit 8: available if status OK */
    d502[1] = (g_steering.status & 0x01);
    can_tx_send(&hfdcan1, CAN_ID_DL_STATUS, d502, 5);
}

/* ---------------------------------------------------------------------------
 * HAL weak callback override — routes all FDCAN RX FIFO0 interrupts
 * --------------------------------------------------------------------------- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        can_isr_push_rx(hfdcan);
    }
}
