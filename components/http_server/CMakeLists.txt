idf_component_register(SRCS "http_server.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc esp_timer esp_wifi nvs_flash esp_http_server esp_https_server esp_https_ota app_update spiffs vfs sensor wifi_app data)