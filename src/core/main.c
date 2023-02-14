#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "core/debug.h"
#include "core/failloop.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/reset.h"
#include "driver/adc.h"
#include "driver/fmc.h"
#include "driver/gpio.h"
#include "driver/motor.h"
#include "driver/rgb_led.h"
#include "driver/serial.h"
#include "driver/spi.h"
#include "driver/spi_soft.h"
#include "driver/time.h"
#include "driver/usb.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/gestures.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/led.h"
#include "io/rgb_led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd/render.h"
#include "project.h"
#include "rx/rx.h"
#include "util/util.h"

uint32_t lastlooptime;
uint8_t looptime_warning;
uint8_t blown_loop_counter;

__attribute__((__used__)) void memory_section_init() {
#ifdef USE_FAST_RAM
  extern uint8_t _fast_ram_start;
  extern uint8_t _fast_ram_end;
  extern uint8_t _fast_ram_data;
  memcpy(&_fast_ram_start, &_fast_ram_data, (size_t)(&_fast_ram_end - &_fast_ram_start));
#endif
#ifdef USE_DMA_RAM
  extern uint8_t _dma_ram_start;
  extern uint8_t _dma_ram_end;
  extern uint8_t _dma_ram_data;

#ifdef STM32H7
  HAL_MPU_Disable();

  MPU_Region_InitTypeDef mpu_init;
  mpu_init.Enable = MPU_REGION_ENABLE;
  mpu_init.BaseAddress = (uint32_t)&_dma_ram_start;
  mpu_init.Size = MPU_REGION_SIZE_256KB;
  mpu_init.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu_init.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  mpu_init.IsCacheable = MPU_ACCESS_CACHEABLE;
  mpu_init.IsShareable = MPU_ACCESS_SHAREABLE;
  mpu_init.Number = MPU_REGION_NUMBER0;
  mpu_init.TypeExtField = MPU_TEX_LEVEL1;
  mpu_init.SubRegionDisable = 0x00;
  mpu_init.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&mpu_init);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif

  memcpy(&_dma_ram_start, &_dma_ram_data, (size_t)(&_dma_ram_end - &_dma_ram_start));
#endif
}

void looptime_update() {
  // looptime_autodetect sequence
  static uint8_t loop_delay = 0;
  if (loop_delay < 200) {
    loop_delay++;
  }

  static float loop_avg = 0;
  static uint8_t loop_counter = 0;

  if (loop_delay >= 200 && loop_counter < 200) {
    loop_avg += state.looptime_us;
    loop_counter++;
  }

  if (loop_counter == 200) {
    loop_avg /= 200;

    if (loop_avg < 130.f) {
      state.looptime_autodetect = LOOPTIME_8K;
    } else if (loop_avg < 255.f) {
      state.looptime_autodetect = LOOPTIME_4K;
    } else {
      state.looptime_autodetect = LOOPTIME_2K;
    }

    loop_counter++;
  }

  if (loop_counter == 201) {
    if (state.cpu_load > state.looptime_autodetect + 5) {
      blown_loop_counter++;
    }

    if (blown_loop_counter > 100) {
      blown_loop_counter = 0;
      loop_counter = 0;
      loop_avg = 0;
      looptime_warning++;
    }
  }
}

__attribute__((__used__)) int main() {
  // init some initial values
  // attempt 8k looptime for f405 or 4k looptime for f411
  state.looptime = LOOPTIME * 1e-6;
  state.looptime_autodetect = LOOPTIME;

  // init timer so we can use delays etc
  time_init();

  // load default profile
  profile_set_defaults();

  // setup filters early
  filter_global_init();
  pid_init();

  // load flash saved variables
  flash_load();

  // wait for flash to stabilze
  time_delay_us(100);

  // init some hardware things
  gpio_init();

  // Turn on LED during boot so that if a delay is used as part of using programming pins for other functions,
  // the FC does not appear inactive while programming times out
  led_init();
  led_on(LEDALL);

  debug_pin_init();
  buzzer_init();

  usb_init();
  spi_init();

  rx_spektrum_bind();

  // init motors and send values so escs init
  motor_init();
  motor_set_all(MOTOR_OFF);
  motor_update();

  // wait for devices to wake up
  time_delay_ms(300);

  if (!sixaxis_init()) {
    // gyro not found
    failloop(FAILLOOP_GYRO);
  }

  // give adc time to settle
  time_delay_ms(100);

  adc_init();
  vbat_init();

  rx_init();
  vtx_init();
  rgb_init();

  osd_init();

  // give the gyro some time to settle
  time_delay_ms(100);

  sixaxis_gyro_cal();

#ifdef ENABLE_BLACKBOX
  blackbox_init();
#endif

  imu_init();

  osd_clear();
  perf_counter_init();

  lastlooptime = time_micros();

  while (1) {
    perf_counter_start(PERF_COUNTER_TOTAL);

    uint32_t time = time_micros();
    state.looptime_us = ((uint32_t)(time - lastlooptime));
    lastlooptime = time;

    if (state.looptime_us <= 0) {
      state.looptime_us = 1;
    }

    // max loop 20ms
    if (state.looptime_us > 20000) {
      failloop(FAILLOOP_LOOPTIME);
    }

    looptime_update();

    state.looptime = state.looptime_us * 1e-6f;

    state.uptime += state.looptime;
    if (flags.arm_state) {
      state.armtime += state.looptime;
    }

    // read gyro and accelerometer data
    perf_counter_start(PERF_COUNTER_GYRO);
    sixaxis_read();
    perf_counter_end(PERF_COUNTER_GYRO);

    // all flight calculations and motors
    perf_counter_start(PERF_COUNTER_CONTROL);
    control();
    perf_counter_end(PERF_COUNTER_CONTROL);

    perf_counter_start(PERF_COUNTER_MISC);

    // attitude calculations for level mode
    imu_calc();

    // battery low logic
    vbat_calc();

    // check gestures
    if (flags.on_ground && !flags.gestures_disabled) {
      gestures();
    }

    // handle led commands
    led_update();

#if (RGB_LED_NUMBER > 0)
    // RGB led control
    rgb_led_lvc();
#ifdef RGB_LED_DMA
    rgb_dma_start();
#endif
#endif

    buzzer_update();
    vtx_update();

    perf_counter_end(PERF_COUNTER_MISC);

    // receiver function
    perf_counter_start(PERF_COUNTER_RX);
    rx_update();
    perf_counter_end(PERF_COUNTER_RX);

    uint8_t blackbox_active = 0;

#ifdef ENABLE_BLACKBOX
    perf_counter_start(PERF_COUNTER_BLACKBOX);
    blackbox_active = blackbox_update();
    perf_counter_end(PERF_COUNTER_BLACKBOX);
#endif

    if (!blackbox_active) {
      perf_counter_start(PERF_COUNTER_OSD);
      osd_display();
      perf_counter_end(PERF_COUNTER_OSD);
    }

    state.cpu_load = (time_micros() - lastlooptime);

    perf_counter_end(PERF_COUNTER_TOTAL);
    perf_counter_update();

    if (usb_detect()) {
      flags.usb_active = 1;
#ifndef ALLOW_USB_ARMING
      if (flags.arm_switch)
        flags.arm_safety = 1; // final safety check to disallow arming during USB operation
#endif
      usb_configurator();
    } else {
      flags.usb_active = 0;
      motor_test.active = 0;
    }

    while ((time_micros() - time) < state.looptime_autodetect)
      __NOP();

  } // end loop
}
