# First, create the list of source files
set(SRCS_LIST 
    "app_main.c"
    "valve_control.c"
    "button_control.c"
    "data.c"
    "lcd.c"
    # "gsm.c"
    "soil_comm.c"
    "soil_sensor.c"
    "sensor.c"
    "ads1x1x.c"
)

if(CONFIG_MASTER)
    list(APPEND SRCS_LIST "wifi_app.c")
    list(APPEND SRCS_LIST "http_server.c")
endif()

idf_component_register(SRCS ${SRCS_LIST}
                    INCLUDE_DIRS "."
                    REQUIRES esp32-button app_update esp_http_server esp_app_format fatfs esp_wifi spiffs nvs_flash rtc espnow_lib espressif__mdns-v1.4.0)
