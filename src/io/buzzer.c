
#include "io/buzzer.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"

static void buzzer_on() {
  if (target.buzzer.pin == PIN_NONE) {
    return;
  }

  if (target.buzzer.invert) {
    gpio_pin_reset(target.buzzer.pin);
  } else {
    gpio_pin_set(target.buzzer.pin);
  }
}

static void buzzer_off() {
  if (target.buzzer.pin == PIN_NONE) {
    return;
  }

  if (target.buzzer.invert) {
    gpio_pin_set(target.buzzer.pin);
  } else {
    gpio_pin_reset(target.buzzer.pin);
  }
}

void buzzer_init() {
  if (target.buzzer.pin == PIN_NONE) {
    return;
  }

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init( target.buzzer.pin, gpio_init);

  buzzer_off();
}

void buzzer_update() {
  // waits 5 seconds
  // before configuring the gpio buzzer pin to ensure
  // there is time to program the chip (if using SWDAT or SWCLK)

  static bool once_armed=false;
  if(flags.arm_state){
    once_armed=true;
  }
  if (( (once_armed && flags.failsafe) || rx_aux_on(AUX_BUZZER_ENABLE)) && !flags.usb_active) {
    buzzer_on();
  } else {
    buzzer_off();
  }
}
