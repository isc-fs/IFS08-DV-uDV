/* =============================================================================
 * can_service.c
 * CAN bus service layer — FDCAN1 (RES CANopen) + FDCAN3 (AMI + Steering)
 * Fixes vs previous version:
 *   - DLC RX: rxh.DataLength >> 16 antes de indexar la tabla
 *   - DLC TX: tabla inversa bytes→HAL macro en lugar de shift manual
 *   - RES_NODE_ID unificado a 0x11 (consistente con main.c y TxData[1]=0x11)
 *   - CAN_ID_RES_BOOTUP corregido a 0x700 + 0x11 = 0x711
 * ============================================================================= */

#include "can_service.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * DLC helpers
 * --------------------------------------------------------------------------- */

/**
 * @brief Convierte el índice DLC (0-15) al número real de bytes.
 *        Llamar SIEMPRE con (rxh.DataLength >> 16) — el HAL devuelve
 *        el código en bits [19:16], no como índice directo.
 */
static inline uint8_t dlc_index_to_bytes(uint32_t index)
{
    static const uint8_t map[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return (index < 16u) ? map[index] : 0u;
}

/**
 * @brief Convierte número de bytes (0-8, CAN clásico) a la macro HAL de DataLength.
 *        Necesario para rellenar FDCAN_TxHeaderTypeDef.DataLength correctamente.
 */
static inline uint32_t bytes_to_dlc_hal(uint8_t bytes)
{
    /* Macros HAL: FDCAN_DLC_BYTES_N = N << 16  para N in [0..8]
     * Para CAN clásico solo usamos 0-8 bytes; FD queda fuera de scope. */
    static const uint32_t tbl[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    return (bytes <= 8u) ? tbl[bytes] : FDCAN_DLC_BYTES_8;
}

/* ---------------------------------------------------------------------------
 * CAN IDs
 * --------------------------------------------------------------------------- */

/* FDCAN3 — AMI + Steering bus */
#define CAN_ID_MISSION_SELECT   0x503u
#define CAN_ID_STEERING         0x2B0u  /* LWS sensor → steering controller → uDV */
#define CAN_ID_STEER_MOTOR      0x010u  /* uDV → steering: motor start/stop        */
#define CAN_ID_STEER_CMD        0x020u  /* uDV → steering: desired angle            */

/* FDCAN1 — RES CANopen
 * Node-ID = 0x11 (DIP switches 1 y 5 en ON), igual que en main.c (TxData[1]=0x11) */
#define RES_NODE_ID             0x11u
#define CAN_ID_NMT              0x000u
#define CAN_ID_RES_PDO_TX       (0x180u + RES_NODE_ID)   /* 0x191 */
#define CAN_ID_RES_BOOTUP       (0x700u + RES_NODE_ID)   /* 0x711 */

/* FDCAN1 — Data Logger (DS 2.2, ciclo 100 ms) */
#define CAN_ID_DL_DYN1          0x500u
#define CAN_ID_DL_DYN2          0x501u
#define CAN_ID_DL_STATUS        0x502u

/* ---------------------------------------------------------------------------
 * Shared globals
 * --------------------------------------------------------------------------- */
volatile steering_data_t g_steering    = {0};
volatile res_status_t    g_res         = {0};
volatile uint8_t         g_mission_index = 0xFF;

/* ---------------------------------------------------------------------------
 * Helpers internos
 * --------------------------------------------------------------------------- */

/** Publica cadena de debug en la cola sin bloquear. */
static inline void dbg_put(const char *s)
{
    (void)osMessageQueuePut(debugQueueHandle, s, 0, 0);
}

/* ---------------------------------------------------------------------------
 * FDCAN3 runtime bring-up (AMI + Steering bus)
 * --------------------------------------------------------------------------- */
void can_service_init(void)
{
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000u,
        .FilterID2    = 0x000u,   /* máscara 0 → acepta todos los IDs */
    };
    HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ---------------------------------------------------------------------------
 * FDCAN1 runtime bring-up (RES CANopen bus)
 * --------------------------------------------------------------------------- */
void res_service_init(void)
{
    /* CubeMX habilita IT1 por defecto; necesitamos IT0 para FIFO0. */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    /* Filtro pass-all (modo diagnóstico / bring-up) */
    FDCAN_FilterTypeDef filter0 = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000u,
        .FilterID2    = 0x000u,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter0);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ---------------------------------------------------------------------------
 * Generic CAN TX
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef can_tx_send(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                             const uint8_t *data, uint8_t dlc)
{
    FDCAN_TxHeaderTypeDef txh = {
        .Identifier          = id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = bytes_to_dlc_hal(dlc),
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    
    // Retornamos el estado para no silenciar errores de bus lleno o periférico apagado
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, (uint8_t *)data);
}

/* ---------------------------------------------------------------------------
 * RES CANopen NMT master
 * --------------------------------------------------------------------------- */
void res_nmt_set_operational(void)
{
    uint8_t nmt[2] = { 0x01u, 0x11u }; // Hardcodea el 0x11u aquí un segundo para descartar fallos del .h
    
    if (can_tx_send(&hfdcan1, 0x000U, nmt, 2) != HAL_OK)
    {
        // Pon un breakpoint aquí. Si entra, tu FDCAN1 no está listo para transmitir.
        Error_Handler(); 
    }
    dbg_put("hola\r\n");
}

/* ---------------------------------------------------------------------------
 * ISR bridge: extrae todos los frames del FIFO0 y los encola
 * --------------------------------------------------------------------------- */
extern osThreadId_t amiTaskHandle;

void can_isr_push_rx(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    /* Bucle: vaciamos todos los frames acumulados en el FIFO hardware
     * para no perder bursts consecutivos entre planificaciones de tareas. */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxdata) != HAL_OK)
            continue;

        can_msg_t msg;
        msg.id = rxh.Identifier;

        uint8_t bytes = dlc_index_to_bytes(rxh.DataLength >> 16u);
        msg.dlc = bytes;

        memset(msg.data, 0, sizeof(msg.data));
        if (bytes > 0u)
            memcpy(msg.data, rxdata, (bytes <= 8u) ? bytes : 8u);

        /* Identificar bus origen */
        if      (hfdcan->Instance == hfdcan1.Instance) msg.bus = 1u;
        else if (hfdcan->Instance == hfdcan2.Instance) msg.bus = 2u;
        else if (hfdcan->Instance == hfdcan3.Instance) msg.bus = 3u;
        else                                           msg.bus = 0u;

        /* Enrutamiento a la cola correcta según bus */
        if (hfdcan->Instance == hfdcan1.Instance)
            osMessageQueuePut(resRxQueueHandle, &msg, 0, 0);
        else
            osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
    }
}

/* ---------------------------------------------------------------------------
 * FDCAN3 message dispatcher (AMI + Steering)
 * --------------------------------------------------------------------------- */
void can_rx_dispatch(const can_msg_t *msg)
{
    /* Debug genérico de cada frame recibido */
    {
        char dbg[80];
        int n   = snprintf(dbg, sizeof(dbg),
                           "CAN%u id=0x%03X dlc=%u data=",
                           (unsigned)msg->bus,
                           (unsigned)msg->id,
                           (unsigned)msg->dlc);
        int off = (n > 0 && n < (int)sizeof(dbg)) ? n : 0;
        for (int i = 0; i < (int)msg->dlc && i < 4 && off < (int)sizeof(dbg) - 3; i++)
            off += snprintf(&dbg[off], sizeof(dbg) - off, "%02X ", msg->data[i]);
        dbg_put(dbg);
    }

    switch (msg->id)
    {
    case CAN_ID_MISSION_SELECT:   /* 0x503 */
        g_mission_index = msg->data[0];
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
        if (amiTaskHandle != NULL)
            xTaskNotifyGive((TaskHandle_t)amiTaskHandle);
        break;

    case CAN_ID_STEERING:         /* 0x2B0 — LWS sensor */
        /* Ángulo: bytes 0-1 Little Endian, resolución 0.1°/bit */
        g_steering.angle_raw = (int16_t)((uint16_t)msg->data[0]
                                        | ((uint16_t)msg->data[1] << 8u));
        /* Velocidad angular: byte 2, resolución 4°/s por bit */
        g_steering.speed_raw = msg->data[2];
        /* Byte de estado: OK=bit0, CAL=bit1, TRIM=bit2 */
        g_steering.status    = msg->data[3];
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * FDCAN1 RES message dispatcher (CANopen)
 * --------------------------------------------------------------------------- */
void res_rx_dispatch(const can_msg_t *msg)
{
    /* Debug genérico */
    {
        char dbg[80];
        int n   = snprintf(dbg, sizeof(dbg),
                           "RES CAN%u id=0x%03X dlc=%u data=",
                           (unsigned)msg->bus,
                           (unsigned)msg->id,
                           (unsigned)msg->dlc);
        int off = (n > 0 && n < (int)sizeof(dbg)) ? n : 0;
        for (int i = 0; i < (int)msg->dlc && i < 4 && off < (int)sizeof(dbg) - 3; i++)
            off += snprintf(&dbg[off], sizeof(dbg) - off, "%02X ", msg->data[i]);
        dbg_put(dbg);
    }

    switch (msg->id)
    {
    case CAN_ID_RES_PDO_TX:   /* 0x191 — PDO cíclico cada ~30 ms */
        /* Parsing adaptado a especificación FSG 2026 (Gross-Funk RES):
         *   data[0] bit 0    → E-Stop (PDO 2000)
         *   data[0] bit 1-2  → Señal GO: K2 (bit 1) y K3 (bit 2)
         *   data[3] bit 7    → E-Stop Redundante (PDO 2003)
         *   data[6]          → Calidad de radio (0-100) (PDO 2006)
         *   data[7] bit 6    → Pre-alarma de radio (PDO 2007) */

        // 1. GESTIÓN DEL E-STOP (Elige la opción A o B según tu máquina de estados):
        // Opción A (Lógica directa del bus: 1 = OK / Libre, 0 = Seta pulsada)
        g_res.e_stop        = (msg->data[0] & 0x01U); 
        
        // Opción B (Lógica invertida: 1 = Seta pulsada, 0 = OK / Libre) -> Descomenta si usas esta:
        // g_res.e_stop     = ((msg->data[0] & 0x01U) == 0U);

        // 2. GESTIÓN DEL GO (Extrae bits 1 y 2 completos para soportar tanto K2 como K3)
        g_res.go_signal     = (msg->data[0] >> 1U) & 0x03U;

        // 3. E-STOP REDUNDANTE (Opcional por si tu estructura tiene un campo secundario)
        // if (msg->dlc >= 4U) { g_res.e_stop_redundant = (msg->data[3] >> 7U) & 0x01U; }

        // Calidad de radio (requiere al menos 7 bytes)
        if (msg->dlc >= 7U)
        {
            g_res.radio_quality = msg->data[6];
        }

        // Pre-alarma de radio (requiere los 8 bytes completos)
        if (msg->dlc >= 8U)
        {
            g_res.pre_alarm     = ((msg->data[7] >> 6U) & 0x01U) != 0U;
        }

        // Actualización del timestamp del RTOS
        g_res.last_rx_tick  = osKernelGetTickCount();
        break;

    case CAN_ID_RES_BOOTUP:   /* 0x711 — boot-up del nodo RES */
        {
            char ev[48];
            snprintf(ev, sizeof(ev), "RES_BOOTUP id=0x%03X", (unsigned)msg->id);
            dbg_put(ev);
        }
        /* El nodo acaba de arrancar: mandamos NMT Start Operational */
        res_nmt_set_operational();
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
    can_tx_send(&hfdcan3, CAN_ID_STEER_MOTOR, &start, 1u);
}

void steering_angle_cmd(float angle_deg)
{
    /* Escala: 0.01°/bit en int32 Little Endian */
    int32_t scaled = (int32_t)(angle_deg * 100.0f);
    uint8_t data[4];
    data[0] = (uint8_t)( scaled        & 0xFFu);
    data[1] = (uint8_t)((scaled >>  8u) & 0xFFu);
    data[2] = (uint8_t)((scaled >> 16u) & 0xFFu);
    data[3] = (uint8_t)((scaled >> 24u) & 0xFFu);
    can_tx_send(&hfdcan3, CAN_ID_STEER_CMD, data, 4u);
}

/* ---------------------------------------------------------------------------
 * Data Logger TX (0x500, 0x501, 0x502) — DS 2.2, ciclo 100 ms
 * --------------------------------------------------------------------------- */
void datalogger_tx(void)
{
    /* 0x500 — DV driving dynamics 1 (8 B) */
    uint8_t d500[8] = {0};
    /* Ángulo de dirección: escala 0.5°/bit → dividir entre 0.5
     * angle_raw * 0.1°/bit / 0.5°/bit = angle_raw * 0.2 */
    int8_t steer_scaled = (int8_t)((float)g_steering.angle_raw * 0.2f);
    d500[2] = (uint8_t)steer_scaled;
    can_tx_send(&hfdcan1, CAN_ID_DL_DYN1, d500, 8u);

    /* 0x501 — DV driving dynamics 2 (6 B) */
    uint8_t d501[6] = {0};
    can_tx_send(&hfdcan1, CAN_ID_DL_DYN2, d501, 6u);

    /* 0x502 — DV system status (5 B)
     * Byte 0 bits[0]:   AS State = 1 (ready)
     * Byte 0 bits[7:5]: Mission selected + 1                         */
    uint8_t d502[5] = {0};
    d502[0] = 0x01u;
    if (g_mission_index != 0xFFu && g_mission_index < 7u)
        d502[0] |= (uint8_t)(((g_mission_index + 1u) & 0x07u) << 5u);
    /* Byte 1: steering sensor OK flag (bit 0 del status LWS) */
    d502[1] = (g_steering.status & 0x01u);
    can_tx_send(&hfdcan1, CAN_ID_DL_STATUS, d502, 5u);
}

/* ---------------------------------------------------------------------------
 * HAL FDCAN interrupt callbacks
 * --------------------------------------------------------------------------- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        can_isr_push_rx(hfdcan);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    /* Si el HAL entra en error de overrun, silencia las notificaciones.
     * Las forzamos de vuelta para no perder mensajes futuros. */
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}