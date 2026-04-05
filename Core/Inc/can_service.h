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
    uint8_t  _pad[3];
} can_msg_t;

/* Queue handle — created in freertos.c, used by ISR and canTask */
extern osMessageQueueId_t canRxQueueHandle;

/* Initialize FDCAN3 runtime: filter, start, enable RX notification */
void can_service_init(void);

/* Called from HAL_FDCAN_RxFifo0Callback (ISR context) */
void can_isr_push_rx(FDCAN_HandleTypeDef *hfdcan);

/* Dispatch a received message by CAN ID — called from canTask */
void can_rx_dispatch(const can_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* CAN_SERVICE_H */
