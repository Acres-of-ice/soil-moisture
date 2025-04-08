idf_component_register(SRCS "sensor.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_wifi nvs_flash ra01s ssd1306 ds18b20 onewire spiffs)
