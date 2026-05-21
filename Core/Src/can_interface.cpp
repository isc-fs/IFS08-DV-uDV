#include "can_interface.hpp"
#include "state_manager.hpp"
#include <cstring>
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "FreeRTOS.h"
#include "task.h"

/* CAN IDs */
static constexpr uint32_t CAN_ID_MISSION_SELECT    = 0x503u;
static constexpr uint32_t CAN_ID_TS_ACTIVE         = 0x504u;
static constexpr uint32_t CAN_ID_BRAKE_PRESSURE    = 0x505u;
static constexpr uint32_t CAN_ID_SDC_RES_OPEN      = 0x506u;
static constexpr uint32_t CAN_ID_CONTROL_ACCEL     = 0x507u;
static constexpr uint32_t CAN_ID_CONTROL_STEER     = 0x508u;
static constexpr uint32_t CAN_ID_R2D               = 0x509u;
static constexpr uint32_t CAN_ID_ASSI              = 0x100u;
static constexpr uint32_t CAN_ID_IMU               = 0x001u;

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

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData) != HAL_OK) {
        Error_Handler();
    }
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

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData) != HAL_OK) {
        Error_Handler();
    }
}
void sendIMU(const bmi088_scaled_t &imu)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];

    const int16_t ax_scaled = (int16_t)(imu.ax_g * 1000.0f);
    const int16_t ay_scaled = (int16_t)(imu.ay_g * 1000.0f);
    const int16_t az_scaled = (int16_t)(imu.az_g * 1000.0f);
    const int16_t gx_scaled = (int16_t)(imu.gx_dps * 10.0f);

    memcpy(&TxData[0], &ax_scaled, sizeof(int16_t));
    memcpy(&TxData[2], &ay_scaled, sizeof(int16_t));
    memcpy(&TxData[4], &az_scaled, sizeof(int16_t));
    memcpy(&TxData[6], &gx_scaled, sizeof(int16_t));

    TxHeader.Identifier = CAN_ID_IMU;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData) != HAL_OK) {
        Error_Handler();
    }
}

void sendAssiStatus(uint8_t status)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[1];

    TxHeader.Identifier = CAN_ID_ASSI;
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
        Error_Handler();
    }
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

    osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
}

void rx_dispatch(const can_msg_t *msg)
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

    case CAN_ID_R2D:
        if (msg->dlc >= 1U) {
            bool r2d_received = (msg->data[0] != 0U);
            // Only accept external R2D when app_task has enabled listening
            if (r2d_received && g_can_listen_go.load()) {
                g_can_r2d.store(true);
            }
        }
        break;

    default:
        break;
    }
}

} // namespace Can

// C-compatible wrapper for ISR
extern "C" void can_interface_rx_isr_callback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN3) {
        Can::isr_push_rx(hfdcan);
    }
}

extern "C" void can_interface_send_assi_emergency_from_isr(void)
{
    Can::sendAssiStatus(StateManager::getAssiStatusCode(ASState::EMERGENCY));
}

