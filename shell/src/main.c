#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

enum LED_STATE { LED_OFF, LED_ON, LED_BLINKING };

struct led {
  const struct gpio_dt_spec device;
  enum LED_STATE target_state;
};

static struct led led_green = {
    .device = GPIO_DT_SPEC_GET(DT_NODELABEL(ledgreen), gpios),
    .target_state = LED_OFF,
};

static struct led led_yellow = {
    .device = GPIO_DT_SPEC_GET(DT_NODELABEL(ledyellow), gpios),
    .target_state = LED_OFF,
};

static struct led led_red = {
    .device = GPIO_DT_SPEC_GET(DT_NODELABEL(ledred), gpios),
    .target_state = LED_OFF,
};

static void set_led_state(struct led* l) {
  switch (l->target_state) {
    case LED_OFF:
      gpio_pin_set_dt(&l->device, 0);
      break;
    case LED_ON:
      gpio_pin_set_dt(&l->device, 1);
      break;
    case LED_BLINKING:
      gpio_pin_toggle_dt(&l->device);
      break;
  }
}

void main(void) {
  // make sure the GPIO device is ready
  if (!device_is_ready(led_green.device.port)) {
    return;
  }

  gpio_pin_configure_dt(&led_green.device, GPIO_OUTPUT);
  gpio_pin_configure_dt(&led_yellow.device, GPIO_OUTPUT);
  gpio_pin_configure_dt(&led_red.device, GPIO_OUTPUT);

  while (1) {
    set_led_state(&led_green);
    set_led_state(&led_yellow);
    set_led_state(&led_red);
    k_sleep(K_MSEC(1000));
  }
}

static void set_led_state_by_color(enum LED_STATE state, char* led_color) {
  if (strcmp(led_color, "red") == 0) {
    led_red.target_state = state;
  } else if (strcmp(led_color, "yellow") == 0) {
    led_yellow.target_state = state;
  } else if (strcmp(led_color, "green") == 0) {
    led_green.target_state = state;
  }
}

// callback for the turn on subcommand
static int handle_turn_on(const struct shell* shell, size_t argc, char** argv) {
  for (int i = 1; i < argc; i++) {
    shell_print(shell, "Turning on %s LED.", argv[i]);
    set_led_state_by_color(LED_ON, argv[i]);
  }
  return 0;
}

// callback for the turn off subcommand
static int handle_turn_off(const struct shell* shell, size_t argc,
                           char** argv) {
  for (int i = 1; i < argc; i++) {
    shell_print(shell, "Turning off %s LED.", argv[i]);
    set_led_state_by_color(LED_OFF, argv[i]);
  }
  return 0;
}

// callback for the blink subcommand
static int handle_blink(const struct shell* shell, size_t argc, char** argv) {
  for (int i = 1; i < argc; i++) {
    shell_print(shell, "Blinking %s LED.", argv[i]);
    set_led_state_by_color(LED_BLINKING, argv[i]);
  }
  return 0;
}

// level 2 commands
SHELL_STATIC_SUBCMD_SET_CREATE(led_names,
                               SHELL_CMD(red, NULL, "Red LED.", NULL),
                               SHELL_CMD(yellow, NULL, "Yellow LED.", NULL),
                               SHELL_CMD(green, NULL, "Green LED.", NULL),
                               SHELL_SUBCMD_SET_END);

// level 1 commands
SHELL_STATIC_SUBCMD_SET_CREATE(
    led_actions,
    SHELL_CMD(turn_on, &led_names, "Turn the LED on.", handle_turn_on),
    SHELL_CMD(turn_off, &led_names, "Turn the LED off.", handle_turn_off),
    SHELL_CMD(blink, &led_names, "Make the LED blink.", handle_blink),
    SHELL_SUBCMD_SET_END);

// Root level 0 command
SHELL_CMD_REGISTER(led_ctrl, &led_actions, "Command to change LED states.",
                   NULL);
