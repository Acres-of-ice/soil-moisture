idf_component_register(SRCS "rtc.c" "i2cdev.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_adc)
