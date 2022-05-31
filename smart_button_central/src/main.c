#include <drivers/gpio.h>
#include <zephyr.h>

#include "ble.h"

#define BUTTON_TIMEOUT_MS 250

static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);

static struct gpio_callback button_cb_data;
static uint64_t last_press_time = 0;

static void button_callback(const struct device *dev, struct gpio_callback *cb,
                            uint32_t pins) {
  uint64_t now = k_uptime_get();
  if ((now - last_press_time) > BUTTON_TIMEOUT_MS) {
    printk("Button pressed\n");
    ble_toogle_led();
  }
  last_press_time = now;
}

void main(void) {
  // make sure the GPIO device is ready
  if (!device_is_ready(button.port)) {
    return;
  };

  // configure the button pin
  gpio_pin_configure_dt(&button, GPIO_INPUT);
  gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);

  // setup the button press callback
  gpio_init_callback(&button_cb_data, button_callback, BIT(button.pin));
  gpio_add_callback(button.port, &button_cb_data);

  ble_setup();
}
