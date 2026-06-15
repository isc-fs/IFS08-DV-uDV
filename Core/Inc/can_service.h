#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdint.h>

/* CAN message struct for queue transport (16 bytes, ISR-safe) */
typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
    uint8_t  bus;     /* 1,2,3 = FDCAN instance number */
    uint8_t  _pad[3];
} can_msg_t;

/* Steering sensor data (CAN ID 0x2B0, FDCAN3) */
typedef struct {
    int16_t  angle_raw;    /* 0.1 deg/bit, little-endian */
    uint8_t  speed_raw;    /* 4 deg/s per bit            */
    uint8_t  status;       /* bit0=OK, bit1=CAL, bit2=TRIM */
} steering_data_t;

extern volatile steering_data_t g_steering;

/* RES status (CANopen PDO from FDCAN1, CAN-ID 0x191) */
typedef struct {
    uint8_t  e_stop;          /* 1 = E-Stop active           */
    uint8_t  go_signal;       /* K2/K3 switch state          */
    uint8_t  radio_quality;   /* 0-100%                      */
    uint8_t  pre_alarm;       /* 1 = radio loss imminent     */
    uint32_t last_rx_tick;    /* osKernelGetTickCount at last PDO */
} res_status_t;

extern volatile res_status_t g_res;

/* Shared mission index indicator */
extern volatile uint8_t g_mission_index;

/* Queue handles — created in freertos.c, used by ISR and canTask */
extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t resRxQueueHandle;
extern osMessageQueueId_t debugQueueHandle;

/* Initialize FDCAN3 runtime: filter, start, enable RX notification */
void can_service_init(void);

/* Initialize FDCAN1 for RES CANopen: filter, start, enable RX */
void res_service_init(void);

/* Send NMT "set operational" to RES (Node-ID 0x11) via FDCAN1 */
void res_nmt_set_operational(void);

/* Dispatch RES messages (0x191 PDO, 0x711 boot-up) */
void res_rx_dispatch(const can_msg_t *msg);

/* Generic CAN TX: queue a frame on any FDCAN peripheral */
/* Cambia el 'void' original por 'HAL_StatusTypeDef' en la línea 62 */
HAL_StatusTypeDef can_tx_send(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                             const uint8_t *data, uint8_t dlc);

/* Steering motor start/stop (0x10): 0=stop, 1=start — FDCAN3 */
void steering_motor_cmd(uint8_t start);

/* Steering desired angle (0x20): int32 LE, scale 1/100 deg — FDCAN3 */
void steering_angle_cmd(float angle_deg);

/* Transmit Data Logger messages (0x500/0x501/0x502) on FDCAN1 */
void datalogger_tx(void);

/* Called from HAL_FDCAN_RxFifo0Callback (ISR context) */
void can_isr_push_rx(FDCAN_HandleTypeDef *hfdcan);

/* Dispatch a received message by CAN ID — called from canTask */
void can_rx_dispatch(const can_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* CAN_SERVICE_H */