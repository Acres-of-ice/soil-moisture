idf_component_register(SRCS "data.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_wifi nvs_flash sensor espnow_lib valve_control rtc fatfs vfs http_server lcd)