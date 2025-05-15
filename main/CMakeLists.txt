# First, create the list of source files
set(SRCS_LIST 
    "app_main.c"
    "valve_control.c"
    # "button_control.c"
    # "rtc_operations.c"
    "data.c"
    "lcd.c"
    # "gsm.c"
    "soil_comm.c"
)

if(CONFIG_CONDUCTOR)
    list(APPEND SRCS_LIST "sensor.c")
    list(APPEND SRCS_LIST "ads1x1x.c")
    list(APPEND SRCS_LIST "wifi_app.c")
    list(APPEND SRCS_LIST "http_server.c")
endif()

idf_component_register(SRCS "app_main.c" 
                    INCLUDE_DIRS "."
                    REQUIRES fatfs esp_wifi spiffs nvs_flash rtc espnow_lib espressif__mdns-v1.4.0)
