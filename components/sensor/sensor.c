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
#include "esp_system.h"
#include "esp_mac.h"
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "nvs_flash.h"


#include "ra01s.h"

const static char *TAG = "EXAMPLE";


#define WIFI_SSID "myssid"
#define WIFI_PASS "mypassword"



#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_1
#define EXAMPLE_ADC1_CHAN2          ADC_CHANNEL_2
#define EXAMPLE_ADC1_CHAN3          ADC_CHANNEL_3



#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

static int adc_raw_1[2][10];
static int adc_raw_2[2][10];
static int adc_raw_3[2][10];

uint8_t device_id[6];
uint8_t buf[256] = {0}; // Maximum Payload size of SX1261/62/68 is 255
uint8_t rx_buf[256] = {0}; // Maximum Payload size of SX1261/62/68 is 255
uint8_t received[256] = {0};
uint8_t response[256] = {0};


void wifi_connect() 
{
    ESP_LOGI("WiFi", "Connecting to %s...", WIFI_SSID);
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}


void time_sync_notification_cb(struct timeval *tv) 
{
    ESP_LOGI("SNTP", "Time synchronized!");
}

void initialize_sntp() 
{
    ESP_LOGI("SNTP", "Initializing SNTP");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    //  Set timezone to IST (UTC +5:30)
    setenv("TZ", "IST-5:30", 1);
    tzset();

    // Wait for time to sync
    vTaskDelay(pdMS_TO_TICKS(5000));

    //  Store time in RTC (ESP32 internal clock)
    time_t now;
    struct timeval tv;
    time(&now);
    tv.tv_sec = now;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
}

void wait_for_sntp_sync() 
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int max_retries = 15;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < max_retries) {
        ESP_LOGW(TAG, "Waiting for SNTP sync... (%d/%d)", retry, max_retries);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == max_retries) 
    {
        ESP_LOGE(TAG, "SNTP sync failed! Time may be incorrect.");
    } 
    else 
    {
        ESP_LOGI(TAG, "Time successfully synchronized!");
    }
}

char* get_current_time() 
{    
    static char time_str[50];  // Buffer to store the formatted time
    time_t now;
    struct tm timeinfo;

    time(&now);
   // now += 5 * 3600 + 30 * 60;  
    localtime_r(&now, &timeinfo);

    snprintf(time_str, sizeof(time_str), "%02d-%02d-%04d %02d:%02d:%02d", 
            timeinfo.tm_mday,timeinfo.tm_mon + 1, timeinfo.tm_year + 1900, 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    vTaskDelay(2);
    return time_str;
}



#if CONFIG_SENDER

void task_tx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    esp_efuse_mac_get_default(device_id);
    const TickType_t delay = pdMS_TO_TICKS(10000); // 10 seconds
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) 
    {
        char* timestamp = get_current_time();
        memset(buf, 0, sizeof(buf)); // Clear buffer before use
        memset(received, 0, sizeof(received)); // Clear received buffer


        /*Constructing the transmission message THE byte structure is 
          TRA[mac id]S[soil sensor]B[battery]T[temperature]D[date and time]END*/ 
        int txLen = snprintf((char *)buf, sizeof(buf),
            "TRAID[%02X:%02X:%02X:%02X:%02X:%02X]S[%d]B[%d]T[%d]D[%s]END",
            device_id[0], device_id[1], device_id[2],
            device_id[3], device_id[4], device_id[5],
            buf[0], buf[1], buf[2], 
            timestamp);

        if (txLen >= sizeof(buf)) 
        {
            ESP_LOGE(pcTaskGetName(NULL), "Buffer overflow, message too long!");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(pcTaskGetName(NULL), "Transmitting: %s", buf);

        //  Send Data
        if (!LoRaSend(buf, txLen, SX126x_TXMODE_SYNC)) 
        {
            ESP_LOGE(pcTaskGetName(NULL), "LoRaSend failed");
        }
        
        vTaskDelay(pdMS_TO_TICKS(3000));



        //  Receive Acknowledgment
        uint8_t ackLen = LoRaReceive(received, sizeof(received) - 1);
        received[ackLen] = '\0'; // Null-terminate to avoid garbage data

        if (ackLen > 0)
        {
           ESP_LOGI(pcTaskGetName(NULL), "RX: Received %d bytes: %s", ackLen, received);
           int8_t rssi, snr;
           GetPacketStatus(&rssi, &snr);
           ESP_LOGI(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);

           //  Check if it starts with "REC"
           if (memcmp(received, "REC", 3) == 0)
           { 

             char ack_mac[18] = {0}; //  Reset before use
             int extracted = sscanf((char *)received, "REC[%17[^]]]", ack_mac);
             ack_mac[17] = '\0';  //  Ensure null termination

             if (extracted == 1)
             { 
                 /*Convert `ack_mac` to bytes and compare*/  
                 uint8_t extracted_mac[6] = {0};
                 if (sscanf(ack_mac, "%hhX:%hhX:%hhX:%hhX:%hhX:%hhX", 
                     &extracted_mac[0], &extracted_mac[1], &extracted_mac[2], 
                     &extracted_mac[3], &extracted_mac[4], &extracted_mac[5]) == 6)
                  {
                     if (memcmp(extracted_mac, device_id, 6) == 0)
                     {
                         ESP_LOGI(pcTaskGetName(NULL), "✅ MAC ID Matched!");
                     }
                  }
              }
           }
        }
     vTaskDelay(pdMS_TO_TICKS(5500));
    }

}

#endif

#if CONFIG_RECEIVER

void task_rx(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    while (1) 
    {
        memset(rx_buf, 0, sizeof(rx_buf)); // Clear buffer before receiving
        uint8_t rxLen = LoRaReceive(rx_buf, sizeof(rx_buf) - 1);
        rx_buf[rxLen] = '\0'; // Ensure null termination

        if (rxLen > 0) 
        { 
            //ESP_LOGI(pcTaskGetName(NULL), "RX: Received %d bytes: %s", rxLen, rx_buf);

            int8_t rssi, snr;
            GetPacketStatus(&rssi, &snr);
            ESP_LOGI(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);

            // ✅ Check if message starts with "TRA"
            if (memcmp(rx_buf, "TRA", 3) == 0) 
            {
                uint8_t mac_id[20] = {0};
                int soil, battery, temp;
                char timestamp[20] = {0};

                // ✅ Extract values from received message
                int extracted = sscanf((char *)rx_buf, 
                    "TRAID[%19[^]]]S[%d]B[%d]T[%d]D[%49[^]]]",
                    mac_id, &soil, &battery, &temp, timestamp);

                if (extracted == 5) 
                {
                    ESP_LOGI(pcTaskGetName(NULL), "Extracted Data:");
                    ESP_LOGI(pcTaskGetName(NULL), "  MAC ID: %s", mac_id);
                    ESP_LOGI(pcTaskGetName(NULL), "  Soil Moisture: %d", soil);
                    ESP_LOGI(pcTaskGetName(NULL), "  Battery: %d", battery);
                    ESP_LOGI(pcTaskGetName(NULL), "  Temperature: %d", temp);
                    ESP_LOGI(pcTaskGetName(NULL), "  Time: %s", timestamp);

                    vTaskDelay(pdMS_TO_TICKS(1));

                    /*Prepare acknowledgment: the byte structure is REC[MAC_ID]END*/
                    memset(response, 0, sizeof(response));
                   int ack_len =  snprintf((char *)response, sizeof(response),
                        "REC[%s]END",mac_id);

                    //  Send acknowledgment back to the transmitter
                    LoRaSend(response, ack_len, SX126x_TXMODE_SYNC);
                    ESP_LOGI(pcTaskGetName(NULL), "Transmitting: %s", response);
;
                }
                else
                {
                    ESP_LOGW(pcTaskGetName(NULL), "Failed to extract data from received message!");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2)); // Avoid Watchdog alerts
    }
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

        buf[0] = (uint8_t)((adc_raw_1[0][0] * 100) / 4095);
        buf[2] = (uint8_t)((adc_raw_2[0][0] * 100) / 4095);
        buf[3] = (uint8_t)((adc_raw_3[0][0] * 100) / 4095);
       vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void lora_init()
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

    if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
        ESP_LOGE(TAG, "Does not recognize the module");
        while(1) {
            vTaskDelay(1);
        }
    }

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
}