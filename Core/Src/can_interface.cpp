#include "can_interface.hpp"
#include <cstring>

extern "C" {
    #include "main.h"
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_hal_fdcan.h"
    #include "can_service.h"
}

extern "C" FDCAN_HandleTypeDef hfdcan1;

void CanInterface::init()
{
    // Initialize CAN service (filters, RX notification)
    can_service_init();

    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

void CanInterface::sendControl(float accel, float steer)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier = 0x000;
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

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
    {
        Error_Handler();
    }
}

void CanInterface::sendR2D(bool r2d)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[1];

    TxHeader.Identifier = 0x000;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxData[0] = r2d;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        g_can_r2d.store(true);
    }
}

void CanInterface::sendRawCANFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (data == nullptr || dlc == 0 || dlc > 8) return;

    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier = can_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = dlc;  // FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, ...
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    memcpy(TxData, data, dlc);

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void CanInterface::sendAssiStatus(uint8_t status)
{
    // Send generic ASSI status message
    // Codes: 0x00=OFF, 0x02=READY, 0x03=DRIVING, 0x01=EMERGENCY, 0x04=FINISHED
    
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[1];

    TxHeader.Identifier = 0x100;  // Adjust to match your CAN protocol
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxData[0] = status;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

// C-compatible wrapper so C code (ISR) can request an ASSI emergency transmit
extern "C" void can_interface_send_assi_emergency_from_isr(void)
{
    CanInterface::sendAssiEmergency();
}
