idf_component_register(SRCS "valve_control.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_wifi nvs_flash sensor espnow_lib soil_comm wifi_app rtc lcd gsm calibration)