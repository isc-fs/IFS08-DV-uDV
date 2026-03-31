#include "usb_cdc_transport.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"

volatile uint8_t g_transport_open_called = 0;

#define USB_CDC_RX_BUFFER_SIZE 2048U

static uint8_t rx_buffer[USB_CDC_RX_BUFFER_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

static uint32_t ring_next(uint32_t index)
{
    return (index + 1U) % USB_CDC_RX_BUFFER_SIZE;
}

static void transport_delay_ms(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

static void transport_set_err(uint8_t *err, uint8_t value)
{
    if (err != NULL) {
        *err = value;
    }
}

bool cubemx_transport_open(uxrCustomTransport * transport)
{
    (void)transport;
    rx_head = 0;
    rx_tail = 0;
    g_transport_open_called = 1;
    transport_delay_ms(100);
    return true;
}

bool cubemx_transport_close(uxrCustomTransport * transport)
{
    (void)transport;
    return true;
}

size_t cubemx_transport_write(uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err)
{
    (void)transport;

    if (buf == NULL || len == 0 || len > 512U) {
        transport_set_err(err, 1);
        return 0;
    }

    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < 1000U)
    {
        if (CDC_Transmit_HS((uint8_t *)buf, (uint16_t)len) == USBD_OK)
        {
            transport_set_err(err, 0);
            return len;
        }

        transport_delay_ms(1);
    }

    transport_set_err(err, 1);
    return 0;
}

size_t cubemx_transport_read(uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err)
{
    (void)transport;

    if (buf == NULL || len == 0) {
        transport_set_err(err, 1);
        return 0;
    }

    size_t count = 0;
    uint32_t start = HAL_GetTick();
    uint32_t wait_ms = (timeout > 0) ? (uint32_t)timeout : 0U;

    while (count < len)
    {
        while ((rx_tail != rx_head) && (count < len))
        {
            buf[count++] = rx_buffer[rx_tail];
            rx_tail = ring_next(rx_tail);
        }

        if (count > 0)
        {
            transport_set_err(err, 0);
            return count;
        }

        if ((HAL_GetTick() - start) >= wait_ms)
        {
            break;
        }

        transport_delay_ms(1);
    }

    transport_set_err(err, 0);
    return count;
}

void usb_cdc_transport_receive(uint8_t * buf, uint32_t len)
{
    if (buf == NULL || len == 0) {
        return;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t next = ring_next(rx_head);
        if (next != rx_tail)
        {
            rx_buffer[rx_head] = buf[i];
            rx_head = next;
        }
    }
}