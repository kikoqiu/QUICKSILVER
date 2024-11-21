#include "io/usb_configurator.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "core/debug.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/reset.h"
#include "driver/serial.h"
#include "driver/usb.h"
#include "flight/control.h"
#include "io/msp.h"
#include "io/quic.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#define BUFFER_SIZE (4 * 1024)

static void msp_send(configurator_port *cport, msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {

  if (magic == MSP2_MAGIC) {
    const uint8_t size = len + MSP2_HEADER_LEN + 1;

    uint8_t frame[size];
    frame[0] = '$';
    frame[1] = MSP2_MAGIC;
    frame[2] = '>';
    frame[3] = 0; // flag
    frame[4] = (cmd >> 0) & 0xFF;
    frame[5] = (cmd >> 8) & 0xFF;
    frame[6] = (len >> 0) & 0xFF;
    frame[7] = (len >> 8) & 0xFF;

    memcpy(frame + MSP2_HEADER_LEN, data, len);
    frame[len + MSP2_HEADER_LEN] = crc8_dvb_s2_data(0, frame + 3, len + 5);

    cport->send(frame, size);
  } else {
    const uint8_t size = len + MSP_HEADER_LEN + 1;

    uint8_t frame[size];
    frame[0] = '$';
    frame[1] = MSP1_MAGIC;
    frame[2] = '>';
    frame[3] = len;
    frame[4] = cmd;

    memcpy(frame + MSP_HEADER_LEN, data, len);

    uint8_t chksum = len;
    for (uint8_t i = 4; i < (size - 1); i++) {
      chksum ^= frame[i];
    }
    frame[len + MSP_HEADER_LEN] = chksum;

    cport->send(frame, size);
  }
}

void usb_quic_send(uint8_t *data, uint32_t len, void *priv) {
  usb_serial_write(data, len);
}

static quic_t quic = {
    .send = usb_quic_send,
};

void usb_quic_logf(const char *fmt, ...) {
  const uint32_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);

  quic_send_str(&quic, QUIC_CMD_LOG, QUIC_FLAG_NONE, str);
}

void usb_serial_passthrough(configurator_port *cport, serial_ports_t port, uint32_t baudrate, uint8_t stop_bits, bool half_duplex) {
  uint8_t tx_data[512];
  ring_buffer_t tx_buffer = {
      .buffer = tx_data,
      .head = 0,
      .tail = 0,
      .size = 512,
  };

  uint8_t rx_data[512];
  ring_buffer_t rx_buffer = {
      .buffer = rx_data,
      .head = 0,
      .tail = 0,
      .size = 512,
  };

  serial_port_t serial = {
      .rx_buffer = &rx_buffer,
      .tx_buffer = &tx_buffer,

      .tx_done = true,
  };

  serial_port_config_t config;
  config.port = port;
  config.baudrate = baudrate;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = stop_bits == 2 ? SERIAL_STOP_BITS_2 : SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = half_duplex;
  config.half_duplex_pp = false;

  serial_init(&serial, config);

  uint8_t data[512];
  while (1) {
    while (1) {
      const uint32_t size = cport->recv(data, 512);
      if (size == 0) {
        break;
      }
      serial_write_bytes(&serial, data, size);
    }
    while (1) {
      const uint32_t size = serial_read_bytes(&serial, data, 512);
      if (size == 0) {
        break;
      }
      cport->send(data, size);
    }
  }
}
int usb_configurator_recv(uint8_t *data, uint32_t len) {
  return usb_serial_read(data, len);
}
void usb_configurator_send(uint8_t *data, uint32_t len) {
  usb_serial_write(data, len);
}
static configurator_port usb_configurator_port = {
    .recv = usb_configurator_recv,
    .send = usb_configurator_send,
};


static void usb_msp_send( msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {
  msp_send(&usb_configurator_port, magic, direction,cmd,data,len);
}


// double promition in the following is intended
#pragma GCC diagnostic ignored "-Wdouble-promotion"
// This function will be where all usb send/receive coms live
void usb_configurator() {
  uint32_t buffer_size = 1;
  static uint8_t buffer[BUFFER_SIZE];

  if (usb_serial_read(buffer, 1) != 1) {
    return;
  }

  switch (buffer[0]) {
  case USB_MAGIC_REBOOT:
    //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
    system_reset_to_bootloader();
    break;

  case USB_MAGIC_SOFT_REBOOT:
    system_reset();
    break;

  case USB_MAGIC_MSP: {
    msp_t msp = {
        .buffer = buffer,
        .buffer_size = BUFFER_SIZE,
        .buffer_offset = 1,
        .send = usb_msp_send,
        .device = MSP_DEVICE_FC,
    };


    uint8_t data = 0;
    while (true) {
      if (usb_serial_read(&data, 1) != 1) {
        continue;
      }

      msp_status_t status = msp_process_serial(&msp, &usb_configurator_port, data);
      if (status != MSP_EOF) {
        break;
      }
    }
    break;
  }
  case USB_MAGIC_QUIC: {

    uint8_t data = 0;
    while (true) {
      if (buffer_size == BUFFER_SIZE) {
        quic_send_str(&quic, QUIC_CMD_INVALID, QUIC_FLAG_ERROR, "EOF");
        break;
      }
      if (quic_process(&usb_configurator_port, &quic, buffer, buffer_size)) {
        break;
      }
      if (usb_serial_read(&data, 1) == 1) {
        buffer[buffer_size++] = data;
      }
    }
    break;
  }
  }

  // this will block and handle all usb traffic while active
  looptime_reset();
}
#pragma GCC diagnostic pop

static uint8_t tx_data[2048];
static ring_buffer_t tx_buffer = {
    .buffer = tx_data,
    .head = 0,
    .tail = 0,
    .size = 2048,
};

static uint8_t rx_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

serial_port_t serial_quic_port = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,

    .tx_done = true,
};

static bool serial_quic_enabled = false;
void serial_quic_init() {
  serial_quic_enabled = false;
  if (serial_is_soft(profile.serial.quic)) {
    return;
  }

  const target_serial_port_t *dev = &target.serial_ports[profile.serial.quic];
  if (!target_serial_port_valid(dev)) {
    return;
  }

  serial_port_config_t config;
  config.port = profile.serial.quic;
  config.baudrate = 115200;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.direction = SERIAL_DIR_TX_RX;

#if defined(INVERT_UART)
  // inversion is hard defined, always invert
  config.invert = true;
#endif

  config.half_duplex = false;
  config.half_duplex_pp = false;

  serial_init(&serial_quic_port, config);
  serial_quic_enabled = true;
}

static void serial_quic_send(uint8_t *data, uint32_t len, void *priv) {
  serial_write_bytes(&serial_quic_port, data, len);
}

static quic_t serial_quic = {
    .send = serial_quic_send,
};

int serial_configurator_recv(uint8_t *data, uint32_t len) {
  return serial_read_bytes(&serial_quic_port, data, len);
}
void serial_configurator_send(uint8_t *data, uint32_t len) {
  serial_write_bytes(&serial_quic_port, data, len);
}
static  configurator_port serial_configurator_port = {
    .recv = serial_configurator_recv,
    .send = serial_configurator_send,
};


static void serial_msp_send(msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {
  msp_send(&serial_configurator_port, magic, direction,cmd,data,len);
}

inline static void looptime_auto_reset(){
  if (!flags.arm_state){
    looptime_reset();
  }
}

static bool serial_quic_inited = false;
// double promition in the following is intended
#pragma GCC diagnostic ignored "-Wdouble-promotion"
// This function will be where all usb send/receive coms live
void serial_configurator() {
  if (!serial_quic_inited) {
    serial_quic_inited = true;
    serial_quic_init();
  }
  if (!serial_quic_enabled)
    return;


  static uint32_t buffer_size;
  static uint8_t buffer[BUFFER_SIZE];
  static msp_t msp = {
    .buffer = buffer,
    .buffer_size = BUFFER_SIZE,
    .buffer_offset = 0,
    .send = serial_msp_send,
    .device = MSP_DEVICE_FC,
  };
  static uint8_t current_mode=0;


  //int start_time = time_micros();
  for(int i = 0 ;i < 100;++i){//max process 100 bytes
    //int diff = time_micros() - start_time;
    uint8_t data_byte;
    if (serial_read_bytes(&serial_quic_port, &data_byte, 1) != 1) {
      looptime_auto_reset();
      return;
    }

    switch (current_mode) {
    case 0:
      {
          switch (data_byte) {
          case USB_MAGIC_REBOOT:
            //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
            system_reset_to_bootloader();
            looptime_auto_reset();
            return;//only process one command
            break;
          case USB_MAGIC_SOFT_REBOOT:
            system_reset();
            looptime_auto_reset();
            return;//only process one command
            break;
          case USB_MAGIC_MSP:
            buffer[0] = data_byte;
            current_mode = data_byte;
            msp.buffer_offset = 1;
            break;
          case USB_MAGIC_QUIC:
            buffer[0] = data_byte;
            current_mode = data_byte;
            buffer_size = 1;
            break;
          default:
            //do nothing
            break;
          }
      }
      break;
    case USB_MAGIC_MSP: {
      msp_status_t status = msp_process_serial(&msp, &serial_configurator_port, data_byte);
      if(status != MSP_EOF){
        current_mode = 0;
        msp.buffer_offset = 0;
        looptime_auto_reset();
        return;//only process one command
      }
      break;
    }
    case USB_MAGIC_QUIC: {      
      buffer[buffer_size++] = data_byte;
      if (quic_process(&serial_configurator_port, &serial_quic, buffer, buffer_size)) {
        current_mode = 0;
        buffer_size = 0;
        looptime_auto_reset();
        return;//only process one command
        break;
      }
      if (buffer_size == BUFFER_SIZE) {
        quic_send_str(&serial_quic, QUIC_CMD_INVALID, QUIC_FLAG_ERROR, "EOF");
        current_mode = 0;
        buffer_size = 0;
        looptime_auto_reset();
        return;//only process one command
        break;
      }
      break;
    }
    }
  }

  // reset looptime for serial passthrough when not armed
  looptime_auto_reset();
}
#pragma GCC diagnostic pop
