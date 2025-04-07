idf_component_register(SRCS "app_main.c" 
                    INCLUDE_DIRS "."
                    REQUIRES sensor esp_wifi nvs_flash ra01s soil_comm espnow_lib wifi_app http_server espressif__mdns-v1.4.0)
                    #set(EXTRA_COMPONENT_DIRS components/espressif__mdns-v1.4.0)


