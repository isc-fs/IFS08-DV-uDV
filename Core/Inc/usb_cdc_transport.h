#ifndef USB_CDC_TRANSPORT_H
#define USB_CDC_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool cubemx_transport_open(uxrCustomTransport * transport);
bool cubemx_transport_close(uxrCustomTransport * transport);
size_t cubemx_transport_write(uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

void usb_cdc_transport_receive(uint8_t * buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif