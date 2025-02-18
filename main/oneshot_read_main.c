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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "ra01s.h"

const static char *TAG = "EXAMPLE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_4
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_5
#else
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_1
#define EXAMPLE_ADC1_CHAN2          ADC_CHANNEL_2
#define EXAMPLE_ADC1_CHAN3          ADC_CHANNEL_3
#endif


#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

static int adc_raw_1[2][10];
static int adc_raw_2[2][10];
static int adc_raw_3[2][10];
static int voltage_1[2][10];
static int voltage_2[2][10];
static int voltage_3[2][10];
int sensor_read[3];
uint8_t buf[256]; // Maximum Payload size of SX1261/62/68 is 255


//#if CONFIG_SENDER
void task_tx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    const TickType_t tenSecondDelay = pdMS_TO_TICKS(10000); // 10 seconds
    while(1) {
        TickType_t nowTick = xTaskGetTickCount();
         int txLen = snprintf((char *)(buf + 3), sizeof(buf) - 3,
                        "soil : %d\nbattery : %d\ntemperature : %d\n",buf[0], buf[1], buf[2]);
        ESP_LOGI(pcTaskGetName(NULL), "Transmitting:\n%s", buf + 3);
        // Wait for transmission to complete
        if (LoRaSend(buf, txLen, SX126x_TXMODE_SYNC) == false) {
            ESP_LOGE(pcTaskGetName(NULL),"LoRaSend fail");
        }

        int lost = GetPacketLost();
        if (lost != 0) {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    } // end while
}


#if CONFIG_RECEIVER
void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t buf[256]; // Maximum Payload size of SX1261/62/68 is 255
    while(1) {
        uint8_t rxLen = LoRaReceive(buf, sizeof(buf));
        if ( rxLen > 0 ) { 
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen, buf);

            int8_t rssi, snr;
            GetPacketStatus(&rssi, &snr);
            ESP_LOGI(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);
        }
        vTaskDelay(1); // Avoid WatchDog alerts
    } // end while
}
#endif

void sensor_task(void *pvParameters)
{
        
        adc_oneshot_unit_handle_t adc1_handle;
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
       
        adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = EXAMPLE_ADC_ATTEN,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN2, &config));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN3, &config));
    
        //-------------ADC1 Calibration Init---------------//
        adc_cali_handle_t adc1_cali_handle = NULL;


    while (1) 
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw_1[0][0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN2, &adc_raw_2[0][0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN3, &adc_raw_3[0][0]));

       buf[0] = (uint8_t)(adc_raw_1[0][0] & 0xFF);
       buf[1] = (uint8_t)(adc_raw_2[0][0] & 0xFF);
       buf[2] = (uint8_t)(adc_raw_3[0][0] & 0xFF);
       vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{

    LoRaInit();
    int8_t txPowerInDbm = 22;

    uint32_t frequencyInHz = 0;
    frequencyInHz = 915000000;

    #if CONFIG_USE_TCXO
    ESP_LOGW(TAG, "Enable TCXO");
    float tcxoVoltage = 3.3; // use TCXO
    bool useRegulatorLDO = true; // use DCDC + LDO
#else
    ESP_LOGW(TAG, "Disable TCXO");
    float tcxoVoltage = 0.0; // don't use TCXO
    bool useRegulatorLDO = false; // use only LDO in all modes
#endif

    //LoRaDebugPrint(true);
    if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
        ESP_LOGE(TAG, "Does not recognize the module");
        while(1) {
            vTaskDelay(1);
        }
    }
    ESP_LOGE(TAG,"inside main2");
    uint8_t spreadingFactor = 7;
    uint8_t bandwidth = 0;//4;
    uint8_t codingRate = 1;
    uint16_t preambleLength = 8;
    uint8_t payloadLen = 0;
    bool crcOn = true;
    bool invertIrq = false;
#if CONFIG_ADVANCED
    spreadingFactor = CONFIG_SF_RATE;
    bandwidth = CONFIG_BANDWIDTH;
    codingRate = CONFIG_CODING_RATE;
#endif
    LoRaConfig(spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn, invertIrq);

//#if CONFIG_SENDER
    xTaskCreate(&sensor_task, "read", 1024*4, NULL, 3, NULL);
    //xTaskCreatePinnedToCore(&task_tx,"TX",1024*4, NULL, 5, NULL,1);
    //vTaskDelay(5);
    xTaskCreate(&task_tx, "TX", 1024*4, NULL, 5, NULL);
    
//#endif
    //ESP_LOGE(TAG,"inside main5");
//#endif
 #if CONFIG_RECEIVER
    xTaskCreate(&task_rx, "RX", 1024*4, NULL, 5, NULL);
 #endif







    //Tear Down
//     ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
//     if (do_calibration1) {
//         example_adc_calibration_deinit(adc1_cali_handle);
//     }

// #if EXAMPLE_USE_ADC2
//     ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
//     if (do_calibration2) {
//         example_adc_calibration_deinit(adc2_cali_handle);
//     }
// #endif //#if EXAMPLE_USE_ADC2
}


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
// static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
// {
//     adc_cali_handle_t handle = NULL;
//     esp_err_t ret = ESP_FAIL;
//     bool calibrated = false;

// #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
//     if (!calibrated) {
//         ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
//         adc_cali_curve_fitting_config_t cali_config = {
//             .unit_id = unit,
//             .atten = atten,
//             .bitwidth = ADC_BITWIDTH_DEFAULT,
//         };
//         ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
//         if (ret == ESP_OK) {
//             calibrated = true;
//         }
//     }
// #endif

// #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
//     if (!calibrated) {
//         ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
//         adc_cali_line_fitting_config_t cali_config = {
//             .unit_id = unit,
//             .atten = atten,
//             .bitwidth = ADC_BITWIDTH_DEFAULT,
//         };
//         ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
//         if (ret == ESP_OK) {
//             calibrated = true;
//         }
//     }
// #endif

//     *out_handle = handle;
//     if (ret == ESP_OK) {
//         ESP_LOGI(TAG, "Calibration Success");
//     } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
//         ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
//     } else {
//         ESP_LOGE(TAG, "Invalid arg or no memory");
//     }

//     return calibrated;
// }

// static void example_adc_calibration_deinit(adc_cali_handle_t handle)
// {
// #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
//     ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
//     ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

// #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
//     ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
//     ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
// #endif
// }
