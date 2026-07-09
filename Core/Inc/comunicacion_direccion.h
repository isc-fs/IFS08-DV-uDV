#ifndef COMUNICACION_DIRECCION_H
#define COMUNICACION_DIRECCION_H

#include "main.h"

#define CAN_ID_MOTOR_CTRL   0x520
#define CAN_ID_ANGULO_CMD   0x521

typedef struct {
    uint8_t  motor_start;
    float    angulo_deseado;   // Lo guardamos como float para usarlo fácil en el código
} CAN_Sistema_t;

extern CAN_Sistema_t G_SistemaCAN;

/* --- Funciones para el EMISOR --- */
HAL_StatusTypeDef CAN_Send_MotorStart(FDCAN_HandleTypeDef *hfdcan, uint8_t state);
HAL_StatusTypeDef CAN_Send_DesiredAngle(FDCAN_HandleTypeDef *hfdcan, float angle);

/* --- Funciones para el RECEPTOR --- */
void CAN_Process_IncomingFrame(uint32_t identifier, uint8_t *data);

#endif


/*

#include "comunicacion_direccion.h"

// Ejemplo de uso:
CAN_Send_MotorStart(&hfdcan1, 1);       // Arrancar
CAN_Send_DesiredAngle(&hfdcan1, 180);   // Ir a 180 grados


*/