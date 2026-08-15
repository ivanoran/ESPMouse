idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_bt esp_ble_nimble nvs_flash esp_hid)
