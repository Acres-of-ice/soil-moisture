idf_component_register(SRCS "wifi_app.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_timer esp_wifi nvs_flash espressif__mdns-v1.4.0 http_server lcd)