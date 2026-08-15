/*
 * BLE HID Mouse for ESP32-C3 with PAW3395 sensor
 * 
 * This firmware implements:
 * - BLE HID connection using esp_hid_gap
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
#include "esp_hid_gap.h"
#include "esp_ble_hid_dev.h"

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

// Protocol commands
#define CMD_GET_SETTINGS_LIST   0x01
#define CMD_GET_SETTING         0x02
#define CMD_SET_SETTING         0x03
#define CMD_APPLY_SETTINGS      0x04

// Buffer for settings communication
static uint8_t settings_buffer[64];
static bool settings_pending = false;

static void handle_get_settings_list(void)
{
    uint8_t response[64];
    uint8_t idx = 0;
    
    response[idx++] = Settings::data_count;
    
    for (uint8_t i = 0; i < Settings::data_count && idx < 60; i++) {
        size_t name_len = strlen(settings.data[i].name);
        if (idx + 1 + name_len + 12 > 64) break;
        
        response[idx++] = (uint8_t)name_len;
        memcpy(&response[idx], settings.data[i].name, name_len);
        idx += name_len;
        
        int32_t val = settings.get(settings.data[i].name);
        response[idx++] = val & 0xFF;
        response[idx++] = (val >> 8) & 0xFF;
        response[idx++] = (val >> 16) & 0xFF;
        response[idx++] = (val >> 24) & 0xFF;
        
        int32_t min_val = settings.data[i].min;
        response[idx++] = min_val & 0xFF;
        response[idx++] = (min_val >> 8) & 0xFF;
        response[idx++] = (min_val >> 16) & 0xFF;
        response[idx++] = (min_val >> 24) & 0xFF;
        
        int32_t max_val = settings.data[i].max;
        response[idx++] = max_val & 0xFF;
        response[idx++] = (max_val >> 8) & 0xFF;
        response[idx++] = (max_val >> 16) & 0xFF;
        response[idx++] = (max_val >> 24) & 0xFF;
    }
    
    memcpy(settings_buffer, response, idx);
    settings_pending = true;
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
    
    memcpy(settings_buffer, response, idx);
    settings_pending = true;
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
    
    sensor.set(name, value);
    
    uint8_t response[64];
    response[0] = name_len;
    memcpy(&response[1], name, name_len);
    response[1 + name_len] = value & 0xFF;
    response[2 + name_len] = (value >> 8) & 0xFF;
    response[3 + name_len] = (value >> 16) & 0xFF;
    response[4 + name_len] = (value >> 24) & 0xFF;
    
    memcpy(settings_buffer, response, 5 + name_len);
    settings_pending = true;
}

static void esp_hid_event_handler(void *handler_args, esp_event_base_t base, 
                                   int32_t event_id, void *event_data)
{
    esp_hid_gap_event_t *event = (esp_hid_gap_event_t *)event_data;
    
    switch (event_id) {
        case ESP_HID_GAP_EVENT_OPEN:
            ESP_LOGI(TAG, "HID Open");
            break;
            
        case ESP_HID_GAP_EVENT_CLOSE:
            ESP_LOGI(TAG, "HID Close");
            break;
            
        case ESP_HID_GAP_EVENT_WRITE:
            if (event->write.report_id == 0x02 && event->write.length > 0) {
                uint8_t cmd = event->write.data[0];
                
                switch (cmd) {
                    case CMD_GET_SETTINGS_LIST:
                        handle_get_settings_list();
                        break;
                        
                    case CMD_GET_SETTING:
                        if (event->write.length >= 2) {
                            char name[61];
                            uint8_t name_len = event->write.data[1];
                            if (name_len <= 60 && event->write.length >= 2 + name_len) {
                                memcpy(name, &event->write.data[2], name_len);
                                name[name_len] = '\0';
                                handle_get_setting(name);
                            }
                        }
                        break;
                        
                    case CMD_SET_SETTING:
                        if (event->write.length >= 2) {
                            handle_set_setting(&event->write.data[1], event->write.length - 1);
                        }
                        break;
                        
                    case CMD_APPLY_SETTINGS:
                        settings_pending = true;
                        settings_buffer[0] = CMD_APPLY_SETTINGS;
                        break;
                }
            }
            break;
            
        default:
            break;
    }
}

// Send HID report with motion and buttons
static void send_hid_report(int16_t delta_x, int16_t delta_y, uint8_t buttons, int8_t wheel)
{
    uint8_t report[7];
    report[0] = 0x01;
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
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    sensor.init();
    buttons.init();
    
    for (uint8_t i = 0; i < Settings::data_count; i++) {
        int32_t val = settings.get(settings.data[i].name);
        sensor.set(settings.data[i].name, val);
    }
    
    uint8_t last_button_state = 0;
    int8_t wheel = 0;
    
    while (1) {
        if (sensor.moving()) {
            motion_read_data mrd = sensor.read_motion();
            
            if (mrd.motion & 0x80) {
                acc_delta_x += mrd.delta_x;
                acc_delta_y += mrd.delta_y;
            }
        }
        
        uint8_t current_buttons = read_buttons();
        
        if ((current_buttons & BUTTON_WHEEL_UP) && !(last_button_state & BUTTON_WHEEL_UP)) {
            wheel = 1;
        } else if ((current_buttons & BUTTON_WHEEL_DOWN) && !(last_button_state & BUTTON_WHEEL_DOWN)) {
            wheel = -1;
        } else {
            wheel = 0;
        }
        
        if (acc_delta_x != 0 || acc_delta_y != 0 || current_buttons != last_button_state || wheel != 0) {
            send_hid_report(acc_delta_x, acc_delta_y, current_buttons, wheel);
            
            acc_delta_x = 0;
            acc_delta_y = 0;
            last_button_state = current_buttons;
        }
        
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

// Settings task - sends pending responses
static void settings_task(void *pvParameters)
{
    while (1) {
        if (settings_pending) {
            esp_ble_hid_dev_send_report(HID_DEV_REPORT_FEATURE, 0x02, settings_buffer, 64);
            settings_pending = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    ESP_ERROR_CHECK(esp_hid_gap_register_callback(esp_hid_event_handler, NULL));
    
    esp_hid_device_config_t device_config = {
        .device_name = (char*)"PAW3395 Mouse",
        .manufacturer_name = (char*)"Custom",
        .serial_number = (char*)"1234567890",
        .firmware_version = (char*)"1.0",
        .hardware_version = (char*)"1.0",
        .report_map = hid_report_desc,
        .report_map_len = sizeof(hid_report_desc),
    };
    
    ESP_ERROR_CHECK(esp_hid_gap_start_adv(&device_config));
    
    ESP_LOGI(TAG, "BLE HID Mouse started, advertising...");
    
    xTaskCreate(mouse_task, "mouse_task", 4096, NULL, 5, NULL);
    xTaskCreate(settings_task, "settings_task", 4096, NULL, 5, NULL);
}
