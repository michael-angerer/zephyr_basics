#include "ble.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>

// Worker
K_SEM_DEFINE(sem, 1, 1);
static struct k_work work;

// Events
K_EVENT_DEFINE(event);
enum {
  EV_LED_FOUND = BIT(1),
  EV_CONNECTED = BIT(2),
  EV_HANDLE_FOUND = BIT(3),
  EV_STATE_READ = BIT(4),
  EV_STATE_WRITTEN = BIT(5),
};
#define EVENT_TIMEOUT_MS 3000

// Service and Characteristics UUIDs
static struct bt_uuid_128 led_state_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x9c85a726, 0xb7f1, 0x11ec, 0xb909, 0x0242ac120002));

#define LED_SERVICE_UUID_VAL \
  BT_UUID_128_ENCODE(0xf7547938, 0x68ba, 0x11ec, 0x90d6, 0x0242ac120003)

static struct bt_uuid_128 led_svc_uuid = BT_UUID_INIT_128(LED_SERVICE_UUID_VAL);

// BLE Data

static uint8_t led_state;
static uint16_t led_state_handle;
static bt_addr_le_t led_address;
static struct bt_conn *led_conn;
static struct bt_gatt_discover_params discover_params;

// Scaning and andvertisement parsing

static void stop_scan() {
  int err = bt_le_scan_stop();
  if (err) {
    printk("Stop LE scan failed (err %d)\n", err);
  } else {
    printk("Scanning successfully stopped.\n");
  }
}

static bool led_found_cb(struct bt_data *data, void *user_data) {
  struct bt_uuid_128 found_uuid;
  int err;

  // we are only interested in ads with a single 128-bit UUID
  if ((data->type != BT_DATA_UUID128_ALL) ||
      (data->data_len != BT_UUID_SIZE_128)) {
    return true;
  }

  // check if the found UUID matches
  bt_uuid_create(&found_uuid.uuid, data->data, BT_UUID_SIZE_128);
  if (bt_uuid_cmp(&found_uuid.uuid, &led_svc_uuid.uuid) != 0) {
    return true;
  } else {
    printk("LED service found.\n");
    memcpy(&led_address, user_data, BT_ADDR_LE_SIZE);
    stop_scan();
    k_event_set(&event, EV_LED_FOUND);
    return false;
  }
}

static void device_found_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                            struct net_buf_simple *ad) {
  bt_data_parse(ad, led_found_cb, (void *)addr);
}

void start_scan() {
  int err;

  struct bt_le_scan_param scan_param = {
      .type = BT_LE_SCAN_TYPE_PASSIVE,
      .options = BT_LE_SCAN_OPT_NONE,
      .interval = BT_GAP_SCAN_FAST_INTERVAL,
      .window = BT_GAP_SCAN_FAST_WINDOW,
  };

  err = bt_le_scan_start(&scan_param, device_found_cb);
  if (err) {
    printk("Scanning failed to start (err %d)\n", err);
    return;
  }

  printk("Scanning successfully started\n");
}

// Connection and disconnection

static void connect() {
  struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;
  int err;

  err = bt_conn_le_create(&led_address, BT_CONN_LE_CREATE_CONN,
                          BT_LE_CONN_PARAM_DEFAULT, &led_conn);
  if (err) {
    printk("Create conn failed (err %d)\n", err);
  }
}

/*
static void disconnect() {
  int err;

  err = bt_conn_disconnect(led_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
  if (err) {
    printk("Disconnect failed with error: %d\n", err);
  }
}
*/

static void connected(struct bt_conn *conn, uint8_t conn_err) {
  if (!conn_err) {
    printk("Connected.\n");
    k_event_set(&event, EV_CONNECTED);
  } else {
    printk("Failed to connect.\n");
    bt_conn_unref(led_conn);
    led_conn = NULL;
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  printk("Disconnected.\n");
  bt_conn_unref(led_conn);
  led_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

// Service Discovery

static uint8_t led_service_discover_cb(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       struct bt_gatt_discover_params *params) {
  led_state_handle = bt_gatt_attr_value_handle(attr);
  printk("LED Control value handle %u\n", led_state_handle);
  k_event_set(&event, EV_HANDLE_FOUND);
  return BT_GATT_ITER_STOP;
}

static void discover_led_service() {
  int err;

  discover_params.uuid = &led_state_char_uuid.uuid;
  discover_params.func = led_service_discover_cb;
  discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
  discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
  discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

  err = bt_gatt_discover(led_conn, &discover_params);
  if (err) {
    printk("Discover failed(err %d)\n", err);
    return;
  }
}

// Read LED State

static uint8_t read_func(struct bt_conn *conn, uint8_t err,
                         struct bt_gatt_read_params *params, const void *data,
                         uint16_t length) {
  led_state = *((uint8_t *)data);
  printk("LED Control value: %u\n", led_state);
  k_event_set(&event, EV_STATE_READ);
  return BT_GATT_ITER_STOP;
}

static void read_led_state() {
  printk("Reading current LED state\n");
  static struct bt_gatt_read_params read_params;
  read_params.handle_count = 1;
  read_params.single.handle = led_state_handle;
  read_params.single.offset = 0;
  read_params.func = read_func;
  bt_gatt_read(led_conn, &read_params);
}

// Write LED State

static void write_func(struct bt_conn *conn, uint8_t err,
                       struct bt_gatt_write_params *params) {
  if (err) {
    printk("Write did not work: err %u\n", err);
  } else {
    printk("Turned LED %s\n", led_state ? "on" : "off");
  }
  k_event_set(&event, EV_STATE_WRITTEN);
}

static void write_led_state() {
  static struct bt_gatt_write_params write_params;

  printk("Current LED state %s - turning LED %s\n", led_state ? "on" : "off",
         led_state ? "off" : "on");
  led_state = !led_state;

  write_params.handle = led_state_handle;
  write_params.offset = 0;
  write_params.data = &led_state;
  write_params.length = 1;
  write_params.func = write_func;
  bt_gatt_write(led_conn, &write_params);
}

static void work_handler(struct k_work *work) {
  int ret;

  if (led_conn == NULL) {
    start_scan();
    ret = k_event_wait(&event, EV_LED_FOUND, false, K_MSEC(EVENT_TIMEOUT_MS));
    if (!ret) {
      printk("LED not found\n");
      stop_scan();
      k_sem_give(&sem);
      return;
    }

    connect();
    k_event_wait(&event, EV_CONNECTED, false, K_MSEC(EVENT_TIMEOUT_MS));
  }

  if (!led_state_handle) {
    discover_led_service();
    k_event_wait(&event, EV_HANDLE_FOUND, false, K_MSEC(EVENT_TIMEOUT_MS));
  }

  read_led_state();
  k_event_wait(&event, EV_STATE_READ, false, K_MSEC(EVENT_TIMEOUT_MS));

  write_led_state();
  k_event_wait(&event, EV_STATE_WRITTEN, false, K_MSEC(EVENT_TIMEOUT_MS));

  k_sem_give(&sem);
}

void ble_setup() {
  int err;

  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }
  printk("Bluetooth initialized\n");

  k_work_init(&work, work_handler);
}

void ble_toogle_led() {
  if (k_sem_take(&sem, K_NO_WAIT) != 0) {
    return;
  };

  k_work_submit(&work);
}
