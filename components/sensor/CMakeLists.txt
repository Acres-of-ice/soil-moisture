idf_component_register(SRCS "sensor.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_adc esp_wifi nvs_flash ra01s)
