#pragma once

#include <stdbool.h>

#include "core/project.h"

typedef enum {
  USB_MAGIC_REBOOT = 'R',
  USB_MAGIC_SOFT_REBOOT = 'S',
  USB_MAGIC_MSP = '$',
  USB_MAGIC_QUIC = '#',
} usb_magics;


typedef int (*configurator_recv_fn_t)(uint8_t *data, uint32_t len);
typedef void (*configurator_send_fn_t)(uint8_t *data, uint32_t len);

typedef struct {
  configurator_recv_fn_t recv;
  configurator_send_fn_t send;
} configurator_port;

void usb_serial_passthrough(configurator_port * cport, serial_ports_t port, uint32_t baudrate, uint8_t stop_bits, bool half_duplex);
void usb_process_msp();
void usb_process_quic();
void usb_quic_logf(const char *fmt, ...);
void usb_configurator();
void serial_configurator();

