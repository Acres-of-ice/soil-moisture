idf_component_register(SRCS "i2cdev.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_wifi nvs_flash sensor espnow_lib valve_control sensor soil_comm lcd)