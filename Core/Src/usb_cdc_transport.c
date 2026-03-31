#include "usb_cdc_transport.h"
#include "usbd_cdc_if.h"

#define USB_CDC_RX_BUFFER_SIZE 2048U

static uint8_t rx_buffer[USB_CDC_RX_BUFFER_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

static uint32_t ring_next(uint32_t index)
{
    return (index + 1U) % USB_CDC_RX_BUFFER_SIZE;
}

bool cubemx_transport_open(uxrCustomTransport * transport)
{
    (void)transport;
    rx_head = 0;
    rx_tail = 0;
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

    if (CDC_Transmit_HS((uint8_t *)buf, (uint16_t)len) == USBD_OK) {
        *err = 0;
        return len;
    }

    *err = 1;
    return 0;
}

size_t cubemx_transport_read(uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err)
{
    (void)transport;
    (void)timeout;

    size_t count = 0;

    while ((count < len) && (rx_tail != rx_head)) {
        buf[count] = rx_buffer[rx_tail];
        rx_tail = ring_next(rx_tail);
        count++;
    }

    *err = 0;
    return count;
}

void usb_cdc_transport_receive(uint8_t * buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint32_t next = ring_next(rx_head);
        if (next != rx_tail) {
            rx_buffer[rx_head] = buf[i];
            rx_head = next;
        }
    }
}