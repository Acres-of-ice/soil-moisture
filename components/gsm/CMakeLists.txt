idf_component_register(SRCS "gsm.c" 
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_wifi nvs_flash sensor espnow_lib valve_control soil_comm)