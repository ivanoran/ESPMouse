#pragma once

#include <string>
#include <cstring>
#include "nvs_flash.h"
#include "nvs.h"

struct setting_line {
    const char* name;       // текстовый ключ
    int32_t     value;      // текущее значение
    int32_t     min;        // минимально допустимое
    int32_t     max;        // максимально допустимое
};

class Settings
{
    public:
        static const uint8_t data_count = 11;
        
        const setting_line data[data_count] = 
        {
        //  {name               value   min     max}
            {"ripple_control",  0,      0,      1},
            {"angle_snap",      0,      0,      1},
            {"swap_xy",         0,      0,      1},
            {"invert_x",        0,      0,      1},
            {"invert_y",        0,      0,      1},
            {"lift_config",     0,      0,      1},
            {"angle_tune_ena",  0,      0,      1},
            {"angle_tune_val",  0,      -128,   127},
            {"read_burst_ena",  0,      0,      1},
            {"resolution",      0,      0,      300},
            {"spi_speed",       1,      1,      39},
        };

        Settings()
        {
            esp_err_t err = nvs_flash_init();
            if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                err = nvs_flash_init();
            }
            ESP_ERROR_CHECK(err);

            err = nvs_open("config", NVS_READWRITE, &_nvs_handle);
            ESP_ERROR_CHECK(err);

            // Инициализация значений по умолчанию, если их нет в NVS
            for (uint8_t i = 0; i < data_count; i++) {
                int32_t val;
                if (nvs_get_i32(_nvs_handle, data[i].name, &val) != ESP_OK) {
                    nvs_set_i32(_nvs_handle, data[i].name, data[i].value);
                }
            }
            nvs_commit(_nvs_handle);
        }

        ~Settings() {
            if (_nvs_handle) {
                nvs_close(_nvs_handle);
            }
        }

        int32_t get(const char* name)
        {
            int32_t value = 0;
            for (uint8_t i = 0; i < data_count; i++) {
                if (strcmp(data[i].name, name) == 0) {
                    esp_err_t err = nvs_get_i32(_nvs_handle, name, &value);
                    if (err != ESP_OK) {
                        value = data[i].value; // возвращаем дефолт при ошибке
                    }
                    break;
                }
            }
            return value;
        }

        void set(const char* name, int32_t value)
        {
            for (uint8_t i = 0; i < data_count; i++) {
                if (strcmp(data[i].name, name) == 0) {
                    if (value >= data[i].min && value <= data[i].max) {
                        esp_err_t err = nvs_set_i32(_nvs_handle, name, value);
                        if (err == ESP_OK) {
                            nvs_commit(_nvs_handle);
                        }
                    }
                    break;
                }
            }
        }

    private:
        nvs_handle_t _nvs_handle = 0;
};

