#include "comunicacion_direccion.h"

CAN_Sistema_t G_SistemaCAN = {0, 0.0f};

static void Prepare_TxHeader(FDCAN_TxHeaderTypeDef *pHeader, uint32_t id, uint32_t dlc) {
    pHeader->Identifier = id;
    pHeader->IdType = FDCAN_STANDARD_ID;
    pHeader->TxFrameType = FDCAN_DATA_FRAME;
    pHeader->DataLength = dlc;
    pHeader->ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    pHeader->BitRateSwitch = FDCAN_BRS_OFF;
    pHeader->FDFormat = FDCAN_CLASSIC_CAN;
    pHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pHeader->MessageMarker = 0;
}

HAL_StatusTypeDef CAN_Send_MotorStart(FDCAN_HandleTypeDef *hfdcan, uint8_t state) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t data[1] = {state};
    Prepare_TxHeader(&TxHeader, CAN_ID_MOTOR_CTRL, FDCAN_DLC_BYTES_1);
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}

HAL_StatusTypeDef CAN_Send_DesiredAngle(FDCAN_HandleTypeDef *hfdcan, float angle) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t data[4];
    
    // Escalamos: -720.55 -> -72055
    int32_t scaled_angle = (int32_t)(angle * 100.0f);
    
    Prepare_TxHeader(&TxHeader, CAN_ID_ANGULO_CMD, FDCAN_DLC_BYTES_4);
    
    // Descomponemos el int32 en 4 bytes (Little Endian)
    data[0] = (uint8_t)(scaled_angle & 0xFF);
    data[1] = (uint8_t)((scaled_angle >> 8) & 0xFF);
    data[2] = (uint8_t)((scaled_angle >> 16) & 0xFF);
    data[3] = (uint8_t)((scaled_angle >> 24) & 0xFF);

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}

void CAN_Process_IncomingFrame(uint32_t identifier, uint8_t *data) {
    switch (identifier) {
        case CAN_ID_MOTOR_CTRL:
            G_SistemaCAN.motor_start = data[0];
            break;

        case CAN_ID_ANGULO_CMD: {
            // Recomponemos el int32_t
            int32_t raw = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
            // Devolvemos los decimales
            G_SistemaCAN.angulo_deseado = (float)raw / 100.0f;
            break;
        }
    }
}