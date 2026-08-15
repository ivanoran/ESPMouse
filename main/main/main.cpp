/*
 * BLE HID Mouse for ESP32-C3 with PAW3395 sensor
 * 
 * This firmware implements:
 * - BLE HID (NimBLE) connection
 * - PAW3395 sensor motion tracking (16-bit delta_x/delta_y)
 * - 5 buttons support (left, right, middle, wheel up, wheel down)
 * - Configurable settings via PC application
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "hid/host/hid_host.h"
#include "hid/device/hid_device.h"

// Include existing classes
#include "paw3395.h"
#include "buttons.h"
#include "settings.h"

static const char *TAG = "mouse";

// Global instances
PAW3395 sensor;
Buttons buttons;
Settings settings;

// Button states
static uint8_t button_state = 0;
#define BUTTON_LEFT     (1 << 0)
#define BUTTON_RIGHT    (1 << 1)
#define BUTTON_MIDDLE   (1 << 2)
#define BUTTON_WHEEL_UP (1 << 3)
#define BUTTON_WHEEL_DOWN (1 << 4)

// Motion accumulator
static int16_t acc_delta_x = 0;
static int16_t acc_delta_y = 0;

// HID Report Descriptor for mouse with 16-bit movement
// Supports: X/Y (16-bit each), Wheel (8-bit signed), Buttons (5 bits)
static const uint8_t hid_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x05,        //   Usage Maximum (0x05) - 5 buttons
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x05,        //   Report Count (5)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null position)
    0x75, 0x03,        //   Report Size (3)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null position) - Padding
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //   Usage (X)
    0x16, 0x00, 0x80,  //   Logical Minimum (-32768) - for 16-bit signed
    0x26, 0xFF, 0x7F,  //   Logical Maximum (32767)
    0x36, 0x00, 0x80,  //   Physical Minimum (-32768)
    0x46, 0xFF, 0x7F,  //   Physical Maximum (32767)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x06,        //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null position)
    0x09, 0x31,        //   Usage (Y)
    0x16, 0x00, 0x80,  //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (32767)
    0x36, 0x00, 0x80,  //   Physical Minimum (-32768)
    0x46, 0xFF, 0x7F,  //   Physical Maximum (32767)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x06,        //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null position)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x38,        //   Usage (Wheel)
    0x15, 0x81,        //   Logical Minimum (-127)
    0x25, 0x7F,        //   Logical Maximum (127)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x06,        //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null position)
    0xC0,              // End Collection
    
    // Feature report for settings
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,        // Usage (0x01)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x09, 0x02,        //   Usage (0x02) - Command/Data
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x40,        //   Report Count (64)
    0x82, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x09, 0x02,        //   Usage (0x02) - Command/Data
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x40,        //   Report Count (64)
    0x92, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,Non-volatile,Null State)
    0xC0               // End Collection
};

static struct ble_hs_cfg ble_hs_config = {
    .hs_gatts_register_cb = NULL,
    .hs_gatts_register_arg = NULL,
};

// Custom GATT service UUID for settings
static const ble_uuid128_t settings_service_uuid = 
    BLE_UUID128_INIT(0x2d, 0x71, 0x00, 0x00, 0xb5, 0xa6, 0x4c, 0x9d, 0x9e, 0x3f, 0x8a, 0x1b, 0x2c, 0x3d, 0x4e, 0x5f);
static const ble_uuid128_t settings_char_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0x00, 0x01, 0xb5, 0xa6, 0x4c, 0x9d, 0x9e, 0x3f, 0x8a, 0x1b, 0x2c, 0x3d, 0x4e, 0x5f);

static uint16_t settings_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t settings_val_handle;

// Protocol commands
#define CMD_GET_SETTINGS_LIST   0x01
#define CMD_GET_SETTING         0x02
#define CMD_SET_SETTING         0x03
#define CMD_APPLY_SETTINGS      0x04

// Buffer for settings communication
static uint8_t settings_buffer[64];

static void send_settings_response(uint8_t cmd, const uint8_t* data, uint8_t len)
{
    if (settings_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    
    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(&cmd, 1);
    if (len > 0 && data != NULL) {
        ble_hs_mbuf_append(om, data, len);
    }
    
    ble_gattc_notify_custom(settings_conn_handle, settings_val_handle, om);
}

static void handle_get_settings_list(void)
{
    // Send list of all settings: name, value, min, max for each
    uint8_t response[64];
    uint8_t idx = 0;
    
    // First byte is count
    response[idx++] = Settings::data_count;
    
    for (uint8_t i = 0; i < Settings::data_count && idx < 60; i++) {
        // Each setting: name length (1 byte), name chars, value (4 bytes), min (4 bytes), max (4 bytes)
        size_t name_len = strlen(settings.data[i].name);
        if (idx + 1 + name_len + 12 > 64) break;
        
        response[idx++] = (uint8_t)name_len;
        memcpy(&response[idx], settings.data[i].name, name_len);
        idx += name_len;
        
        // Value (int32_t, little endian)
        int32_t val = settings.get(settings.data[i].name);
        response[idx++] = val & 0xFF;
        response[idx++] = (val >> 8) & 0xFF;
        response[idx++] = (val >> 16) & 0xFF;
        response[idx++] = (val >> 24) & 0xFF;
        
        // Min (int32_t, little endian)
        int32_t min_val = settings.data[i].min;
        response[idx++] = min_val & 0xFF;
        response[idx++] = (min_val >> 8) & 0xFF;
        response[idx++] = (min_val >> 16) & 0xFF;
        response[idx++] = (min_val >> 24) & 0xFF;
        
        // Max (int32_t, little endian)
        int32_t max_val = settings.data[i].max;
        response[idx++] = max_val & 0xFF;
        response[idx++] = (max_val >> 8) & 0xFF;
        response[idx++] = (max_val >> 16) & 0xFF;
        response[idx++] = (max_val >> 24) & 0xFF;
    }
    
    send_settings_response(CMD_GET_SETTINGS_LIST, response, idx);
}

static void handle_get_setting(const char* name)
{
    int32_t val = settings.get(name);
    uint8_t response[64];
    uint8_t idx = 0;
    
    size_t name_len = strlen(name);
    if (name_len > 60) name_len = 60;
    
    response[idx++] = (uint8_t)name_len;
    memcpy(&response[idx], name, name_len);
    idx += name_len;
    
    response[idx++] = val & 0xFF;
    response[idx++] = (val >> 8) & 0xFF;
    response[idx++] = (val >> 16) & 0xFF;
    response[idx++] = (val >> 24) & 0xFF;
    
    send_settings_response(CMD_GET_SETTING, response, idx);
}

static void handle_set_setting(const uint8_t* data, uint8_t len)
{
    if (len < 2) return;
    
    uint8_t name_len = data[0];
    if (name_len > 60 || len < 2 + name_len + 4) return;
    
    char name[61];
    memcpy(name, &data[1], name_len);
    name[name_len] = '\0';
    
    int32_t value = data[1 + name_len] | 
                   (data[2 + name_len] << 8) |
                   (data[3 + name_len] << 16) |
                   (data[4 + name_len] << 24);
    
    settings.set(name, value);
    
    ESP_LOGI(TAG, "Setting updated: %s = %ld", name, (long)value);
    
    // Apply setting to sensor
    sensor.set(name, value);
    
    // Send confirmation
    uint8_t response[64];
    response[0] = name_len;
    memcpy(&response[1], name, name_len);
    response[1 + name_len] = value & 0xFF;
    response[2 + name_len] = (value >> 8) & 0xFF;
    response[3 + name_len] = (value >> 16) & 0xFF;
    response[4 + name_len] = (value >> 24) & 0xFF;
    
    send_settings_response(CMD_SET_SETTING, response, 5 + name_len);
}

static int settings_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        settings_conn_handle = conn_handle;
        
        // Read the feature report data
        uint8_t data[64];
        int read_len = ctxt->om->om_len;
        if (read_len > 64) read_len = 64;
        
        os_mbuf_copydata(ctxt->om, 0, read_len, data);
        
        if (read_len >= 1) {
            uint8_t cmd = data[0];
            
            switch (cmd) {
                case CMD_GET_SETTINGS_LIST:
                    handle_get_settings_list();
                    break;
                    
                case CMD_GET_SETTING:
                    if (read_len >= 2) {
                        char name[61];
                        uint8_t name_len = data[1];
                        if (name_len <= 60 && read_len >= 2 + name_len) {
                            memcpy(name, &data[2], name_len);
                            name[name_len] = '\0';
                            handle_get_setting(name);
                        }
                    }
                    break;
                    
                case CMD_SET_SETTING:
                    if (read_len >= 2) {
                        handle_set_setting(&data[1], read_len - 1);
                    }
                    break;
                    
                case CMD_APPLY_SETTINGS:
                    // All settings are applied immediately when set
                    send_settings_response(CMD_APPLY_SETTINGS, NULL, 0);
                    break;
            }
        }
        
        return 0;
    }
    
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def settings_svc_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &settings_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &settings_char_uuid.u,
                .access_cb = settings_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &settings_val_handle,
            },
            {
                0, /* No more characteristics in this service. */
            },
        },
    },
    {
        0, /* No more services. */
    },
};

static int bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Connection established");
            } else {
                ESP_LOGI(TAG, "Connection failed, code=%d", event->connect.status);
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected");
            settings_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            break;
            
        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event, op=%d", event->subscribe.cur_op);
            break;
    }
    
    return 0;
}

static void bleprph_on_sync(void)
{
    ble_hs_id_infer_auto(0, NULL);
    
    ble_gap_adv_set_fields(
        &(const ble_adv_fields_t){
            .flags = BLE_HS_ADV_F_DISC_GEN,
            .tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
            .appearance = BLE_HS_ADV_APPEARANCE_HID_MOUSE,
            .appearance_is_present = 1,
            .complete_svcs128_is_present = 1,
            .complete_svcs128 = &settings_service_uuid.u,
            .num_complete_svcs128 = 1,
        }
    );
    
    ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, NULL, bleprph_gap_event, NULL);
    ESP_LOGI(TAG, "Advertising started");
}

static void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Send HID report with motion and buttons
static void send_hid_report(int16_t delta_x, int16_t delta_y, uint8_t buttons, int8_t wheel)
{
    uint8_t report[7];
    report[0] = 0x01;  // Report ID
    report[1] = buttons;
    report[2] = delta_x & 0xFF;
    report[3] = (delta_x >> 8) & 0xFF;
    report[4] = delta_y & 0xFF;
    report[5] = (delta_y >> 8) & 0xFF;
    report[6] = wheel;
    
    esp_ble_hid_dev_send_report(HID_DEV_REPORT_IN, 0x01, report, sizeof(report));
}

// Read buttons and update state
static uint8_t read_buttons(void)
{
    uint8_t state = 0;
    
    // Map buttons according to user requirements:
    // left, right, middle, wheel up, wheel down
    if (buttons.read(button_func::left)) {
        state |= BUTTON_LEFT;
    }
    if (buttons.read(button_func::right)) {
        state |= BUTTON_RIGHT;
    }
    if (buttons.read(button_func::middle)) {
        state |= BUTTON_MIDDLE;
    }
    if (buttons.read(button_func::pan)) {
        state |= BUTTON_WHEEL_UP;
    }
    if (buttons.read(button_func::macro_1)) {
        state |= BUTTON_WHEEL_DOWN;
    }
    
    return state;
}

// Main mouse task
static void mouse_task(void *pvParameters)
{
    // Wait for BLE to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Initialize sensor with settings
    sensor.init();
    buttons.init();
    
    // Apply initial settings
    for (uint8_t i = 0; i < Settings::data_count; i++) {
        int32_t val = settings.get(settings.data[i].name);
        sensor.set(settings.data[i].name, val);
    }
    
    uint8_t last_button_state = 0;
    int8_t wheel = 0;
    
    while (1) {
        // Read motion from sensor
        if (sensor.moving()) {
            motion_read_data mrd = sensor.read_motion();
            
            if (mrd.motion & 0x80) {  // Motion valid
                acc_delta_x += mrd.delta_x;
                acc_delta_y += mrd.delta_y;
            }
        }
        
        // Read buttons
        uint8_t current_buttons = read_buttons();
        
        // Handle wheel buttons (generate single scroll events)
        if ((current_buttons & BUTTON_WHEEL_UP) && !(last_button_state & BUTTON_WHEEL_UP)) {
            wheel = 1;
        } else if ((current_buttons & BUTTON_WHEEL_DOWN) && !(last_button_state & BUTTON_WHEEL_DOWN)) {
            wheel = -1;
        } else {
            wheel = 0;
        }
        
        // Send report if there's motion or button change
        if (acc_delta_x != 0 || acc_delta_y != 0 || current_buttons != last_button_state || wheel != 0) {
            send_hid_report(acc_delta_x, acc_delta_y, current_buttons, wheel);
            
            acc_delta_x = 0;
            acc_delta_y = 0;
            last_button_state = current_buttons;
        }
        
        vTaskDelay(pdMS_TO_TICKS(8));  // ~125 Hz polling
    }
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    
    ESP_ERROR_CHECK(esp_nimble_hci_init());
    nimble_port_init();
    
    // Register HID device
    esp_ble_hid_dev_init();
    
    // Register HID report descriptor
    esp_ble_hid_dev_set_report_descriptor(hid_report_desc, sizeof(hid_report_desc));
    
    // Register custom settings service
    ble_gatts_count_cfg(settings_svc_defs);
    ble_gatts_add_svcs(settings_svc_defs);
    
    // Start BLE host task
    nimble_port_freertos_init(bleprph_host_task);
    
    // Configure GAP
    ble_svc_gap_device_name_set("PAW3395_Mouse");
    
    // Start mouse task
    xTaskCreate(mouse_task, "mouse_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Application started");
}
