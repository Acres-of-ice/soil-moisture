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
#include "soil_comm.h"


void app_main(void)
{
    vext_on(); // ✅ Turn on OLED power

    vTaskDelay(pdMS_TO_TICKS(100));  // Short delay
    /*lora initialisation*/
    // wifi_init_sta();
    // vTaskDelay(5); 
    // initialize_sntp(); 
    // wait_for_sntp_sync();
    // wifi_stop();
     i2c_init();
    //lora_init();
    espnow_init();

    vTaskDelay(pdMS_TO_TICKS(2000));

#if CONFIG_SENDER
    xTaskCreate(&sensor_task, "read", 1024*4, NULL, 3, NULL);
    xTaskCreate(&vTaskESPNOW_TX, "transmit", 1024*4, NULL, 3, NULL);
#endif

#if CONFIG_RECEIVER
    xTaskCreate(vTaskESPNOW_RX, "receive", 1024*4, NULL, 3, NULL);
#endif
    

// #if CONFIG_SENDER
//     /*nvs,wifi and sntp are required for fetching the real time
//     when the transmitter is getting started the wifi has to be connectedd for a brief period
//     for now the wifi ssid is "myssid" and password is "mypassword"*/
//     //nvs_flash_init(); 
//     //wifi_connect(); 
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//       ESP_ERROR_CHECK(nvs_flash_erase());
//       ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     wifi_init_sta();
//     vTaskDelay(5); 
//     initialize_sntp(); 
//     wait_for_sntp_sync();
//     wifi_stop();
 
//     /*Task for reading sensor and battery values*/
//     xTaskCreate(&sensor_task, "read", 1024*4, NULL, 3, NULL);
//     /*Task for transmitting */
//     vTaskDelay(pdMS_TO_TICKS(3000));
//     xTaskCreate(&task_tx, "TX", 1024*4, NULL, 5, NULL);
// #endif

// #if CONFIG_RECEIVER
//     /*Task for transmitting */
//     xTaskCreate(&task_rx, "RX", 1024*4, NULL, 5, NULL);
// #endif

}

