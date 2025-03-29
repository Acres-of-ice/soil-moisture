idf_component_register(SRCS "app_main.c" 
                    INCLUDE_DIRS "."
                    REQUIRES sensor esp_wifi nvs_flash ra01s soil_comm)


