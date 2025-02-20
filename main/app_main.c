/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "sensor.h"


void app_main(void)
{

    /*lora initialisation*/
    lora_init();

#if CONFIG_SENDER
    /*nvs,wifi and sntp are required for fetching the real time
    when the transmitter is getting started the wifi has to be connectedd for a brief period
    for now the wifi ssid is "myssid" and password is "mypassword"*/
    nvs_flash_init(); 
    wifi_connect();  
    initialize_sntp(); 
    wait_for_sntp_sync();
    vTaskDelay(5);
    /*Task for reading sensor and battery values*/
    xTaskCreate(&sensor_task, "read", 1024*4, NULL, 3, NULL);
    /*Task for transmitting */
    xTaskCreate(&task_tx, "TX", 1024*4, NULL, 5, NULL);
#endif

#if CONFIG_RECEIVER
    /*Task for transmitting */
    xTaskCreate(&task_rx, "RX", 1024*4, NULL, 5, NULL);
#endif

}

