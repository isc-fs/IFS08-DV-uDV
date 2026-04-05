#include "can_service.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * CAN IDs — add new IDs here as the bus grows
 * --------------------------------------------------------------------------- */
#define CAN_ID_MISSION_SELECT  0x503u

/* ---------------------------------------------------------------------------
 * FDCAN3 runtime bring-up
 * --------------------------------------------------------------------------- */
void can_service_init(void)
{
    /* Pass-all standard ID filter (mask = 0x000 → accept everything) */
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,
        .FilterID2    = 0x000,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);

    /* Reject non-matching extended and remote frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
        FDCAN_ACCEPT_IN_RX_FIFO0,  /* non-matching std → FIFO0 */
        FDCAN_REJECT,               /* non-matching ext → reject */
        FDCAN_REJECT_REMOTE,        /* remote std → reject */
        FDCAN_REJECT_REMOTE);       /* remote ext → reject */

    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ---------------------------------------------------------------------------
 * ISR bridge: read one frame from FIFO0 and push to canRxQueue
 * Called from HAL_FDCAN_RxFifo0Callback — runs in interrupt context.
 * --------------------------------------------------------------------------- */
void can_isr_push_rx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxdata) != HAL_OK)
        return;

    can_msg_t msg;
    msg.id  = rxh.Identifier;
    msg.dlc = (uint8_t)(rxh.DataLength >> 16);  /* FDCAN DLC encoding */
    memcpy(msg.data, rxdata, 8);

    osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);  /* non-blocking */
}

/* ---------------------------------------------------------------------------
 * Message dispatcher — switch on CAN ID, add new cases as needed
 * --------------------------------------------------------------------------- */
/* amiTask handle and shared mission index — defined in freertos.c */
extern osThreadId_t amiTaskHandle;
extern volatile uint8_t g_mission_index;

void can_rx_dispatch(const can_msg_t *msg)
{
    switch (msg->id)
    {
    case CAN_ID_MISSION_SELECT:
        g_mission_index = msg->data[0];
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
        /* Wake amiTask to update LEDs */
        if (amiTaskHandle != NULL)
        {
            xTaskNotifyGive((TaskHandle_t)amiTaskHandle);
        }
        break;

    default:
        break;
    }
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
