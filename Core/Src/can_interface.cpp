#include "can_interface.hpp"
#include <cstring>
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "FreeRTOS.h"
#include "task.h"

/* CAN IDs */
#define CAN_ID_MISSION_SELECT  0x503u
#define CAN_ID_TS_ACTIVE       0x504u
#define CAN_ID_BRAKE_PRESSURE  0x505u
#define CAN_ID_SDC_RES_OPEN    0x506u

extern FDCAN_HandleTypeDef hfdcan3;

/* amiTask handle and shared mission index — defined in freertos.c */
extern osThreadId_t amiTaskHandle;
extern volatile uint8_t g_mission_index;

void CanInterface::init()
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

void CanInterface::sendControl(float accel, float steer)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier = 0x000; // TODO: Define proper CAN ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    memcpy(&TxData[0], &accel, sizeof(float));
    memcpy(&TxData[4], &steer, sizeof(float));

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData) != HAL_OK) {
        Error_Handler();
    }
}

void CanInterface::sendR2D(bool r2d)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[1];

    TxHeader.Identifier = 0x000; // TODO: Define proper CAN ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxData[0] = r2d;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData) != HAL_OK) {
        Error_Handler();
    } else {
        g_can_r2d.store(true);
    }
}

void CanInterface::sendRawCANFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (data == nullptr || dlc == 0 || dlc > 8) return;

    FDCAN_TxHeaderTypeDef TxHeader;
    
    TxHeader.Identifier = can_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = (uint32_t)dlc << 16; // FDCAN DLC encoding
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, (uint8_t*)data) != HAL_OK) {
        // Optional: Error handling
    }
}

void CanInterface::sendAssiStatus(uint8_t status)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[1];

    TxHeader.Identifier = 0x100; // ASSI Status CAN ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxData[0] = status;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData) != HAL_OK) {
        // Optional: Error handling
    }
}

void CanInterface::isr_push_rx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxdata) != HAL_OK)
        return;

    can_msg_t msg;
    msg.id  = rxh.Identifier;
    msg.dlc = (uint8_t)(rxh.DataLength >> 16);
    memcpy(msg.data, rxdata, 8);

    osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
}

void CanInterface::rx_dispatch(const can_msg_t *msg)
{
    switch (msg->id)
    {
    case CAN_ID_MISSION_SELECT:
        g_mission_index = msg->data[0];
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
        if (amiTaskHandle != NULL) {
            xTaskNotifyGive((TaskHandle_t)amiTaskHandle);
        }
        break;

    case CAN_ID_TS_ACTIVE:
        g_can_ts_active.store(msg->data[0] != 0U);
        break;

    case CAN_ID_BRAKE_PRESSURE:
        if (msg->dlc >= 4U) {
            float brake_pressure = 0.0f;
            memcpy(&brake_pressure, msg->data, sizeof(brake_pressure));
            g_can_brake_pressure.store(brake_pressure);
        }
        break;

    case CAN_ID_SDC_RES_OPEN:
        if (msg->dlc >= 1U) {
            bool sdc_open = (msg->data[0] != 0U);
            // Update CAN global so StateManager reads it
            g_can_sdc_res_open.store(sdc_open);
        }
        break;

    default:
        break;
    }
}

// C-compatible wrapper for ISR
extern "C" void can_interface_rx_isr_callback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN3) {
        CanInterface::getInstance().isr_push_rx(hfdcan);
    }
}

extern "C" void can_interface_send_assi_emergency_from_isr(void)
{
    CanInterface::getInstance().sendAssiEmergency();
}

