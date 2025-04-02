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
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "ssd1306.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_sleep.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "sensor.h"



#include "ra01s.h"

const static char *TAG = "EXAMPLE";


#define WIFI_SSID "myssid"
#define WIFI_PASS "mypassword"



#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_1
#define EXAMPLE_ADC1_CHAN2          ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN3          ADC_CHANNEL_4



#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

#define EXAMPLE_ONEWIRE_BUS_GPIO    GPIO_NUM_3  // Set the GPIO for OneWire data line
#define EXAMPLE_ONEWIRE_MAX_DS18B20 2           // Maximum DS18B20 devices to detect

static int adc_raw_1[2][10];
static int adc_raw_2[2][10];
static int adc_raw_3[2][10];

extern uint8_t device_id[6];
uint8_t buf[256] = {0}; // Maximum Payload size of SX1261/62/68 is 255
uint8_t rx_buf[256] = {0}; // Maximum Payload size of SX1261/62/68 is 255
uint8_t received[256] = {0};
uint8_t response[256] = {0};

#define SCL_GPIO 18       // GPIO for SCL
#define SDA_GPIO 17       // GPIO for SDA
#define I2C_MASTER_FREQ_HZ 100000  // I2C clock frequency
#define I2C_MASTER_NUM I2C_NUM_0   // I2C port number
#define OLED_ADDR            0x3C  /*!< OLED Display I2C Address */
#define VEXT_GPIO       36 // Power control (from Arduino code)

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1



static int s_retry_num = 0;



QueueHandle_t espnow_queue = NULL;


void i2c_init()
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
}

void i2c_scan() {
    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (result == ESP_OK) 
        {
            ESP_LOGI(TAG, "Device found at address 0x%02X", addr);
        }
        else 
        {ESP_LOGI(TAG, "Device not found");}
    }

    ESP_LOGI(TAG, "I2C scan complete.");
}

void vext_on() {
    gpio_reset_pin(VEXT_GPIO);
    gpio_set_direction(VEXT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(VEXT_GPIO, 0);  // LOW to enable OLED power
}

void reset_i2c_bus() {
    gpio_reset_pin(SDA_GPIO);
    gpio_reset_pin(SCL_GPIO);
    gpio_set_direction(SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);

    // Generate 9 clock pulses to reset I2C devices
    for (int i = 0; i < 9; i++) {
        gpio_set_level(SCL_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(SCL_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "I2C Bus Reset Done.");
}

static void event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
esp_wifi_connect();
} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
esp_wifi_connect();
s_retry_num++;
ESP_LOGI(TAG, "retry to connect to the AP");
} else {
xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
}
ESP_LOGI(TAG,"connect to the AP fail");
} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
s_retry_num = 0;
xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
}
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}
void wifi_stop() 
{
 
    vTaskDelay(5);
    esp_wifi_stop();
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

void enter_light_sleep() 
{
    //ESP_LOGI(TAG, "Entering light sleep for 5 minutes...");
    
    
    esp_sleep_enable_timer_wakeup(300000000);   //(5 minutes = 300 seconds = 300,000,000 microseconds)


 
    esp_light_sleep_start();

    //ESP_LOGI(TAG, "Woke up from light sleep!");
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
        
        //memset(buf, 0, sizeof(buf)); // Clear buffer before use
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

        //enter_light_sleep();

        //  Receive Acknowledgment
        uint8_t ackLen = LoRaReceive(received, sizeof(received) - 1);
        received[ackLen] = '\0'; // Null-terminate to avoid garbage data

        if (ackLen > 0)
        {
          // ESP_LOGI(pcTaskGetName(NULL), "RX: Received %d bytes: %s", ackLen, received);
           int8_t rssi, snr;
           GetPacketStatus(&rssi, &snr);
           ESP_LOGI(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);

           //  Check if it starts with "REC"
           if (memcmp(received, "REC", 3) == 0)
           { 

             char ack_mac[18] = {0}; //  Reset before use
             //int extracted = sscanf((char *)received, "REC[%17[^]]]", ack_mac);
             ack_mac[17] = '\0';  //  Ensure null termination

             ESP_LOGI(pcTaskGetName(NULL), "✅ MAC ID Matched!");

            //  if (extracted == 1)
            //  { 
            //      /*Convert `ack_mac` to bytes and compare*/  
            //      uint8_t extracted_mac[6] = {0};
            //      if (sscanf(ack_mac, "%hhX:%hhX:%hhX:%hhX:%hhX:%hhX", 
            //          &extracted_mac[0], &extracted_mac[1], &extracted_mac[2], 
            //          &extracted_mac[3], &extracted_mac[4], &extracted_mac[5]) == 6)
            //       {
            //          if (memcmp(extracted_mac, device_id, 6) == 0)
            //          {
            //              ESP_LOGI(pcTaskGetName(NULL), "✅ MAC ID Matched!");
            //              // ✅ Show Received MAC ID on OLED
            //          }
            //       }
            //   }
            enter_light_sleep();
           }
        }
     memset(buf, 0, sizeof(buf)); // Clear buffer before use
     memset(received, 0, sizeof(received));
     vTaskDelay(pdMS_TO_TICKS(5500));
     //enter_light_sleep();
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

                //  Extract values from received message
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
                 /*INITIALISING ADC FOR MOISTURE SENSOR*/
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

        adc_oneshot_unit_handle_t adc2_handle;
        adc_oneshot_unit_init_cfg_t init_config2 = {
            .unit_id = ADC_UNIT_2,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));
        adc_cali_handle_t adc2_cali_handle = NULL;
        
    
         /*INITIALISING ONE WIRE AND SEARCHING FOR DS18B20 FOR TEMPERATURE SENSOR*/
     onewire_bus_handle_t bus = NULL;
     onewire_bus_config_t bus_config = {
         .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
     };
     onewire_bus_rmt_config_t rmt_config = {
         .max_rx_bytes = 10,  
     };
     ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
 
     // Search for DS18B20 devices
     int ds18b20_device_num = 0;
     ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
     onewire_device_iter_handle_t iter = NULL;
     onewire_device_t next_onewire_device;
     esp_err_t search_result = ESP_OK;
 
     // Create 1-Wire device iterator
    //  ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    //  ESP_LOGI(TAG, "Device iterator created, start searching...");
 
    //  do {
    //      search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
    //      if (search_result == ESP_OK) {
    //          ds18b20_config_t ds_cfg = {};
    //          // Check if the found device is a DS18B20
    //          if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
    //              //ESP_LOGI(TAG, "Found DS18B20[%d], Address: %016llX", ds18b20_device_num, next_onewire_device.address);
    //              ds18b20_device_num++;
    //          } else {
    //              ESP_LOGW(TAG, "Unknown device found at Address: %016llX", next_onewire_device.address);
    //          }
    //      }
    //  } while (search_result != ESP_ERR_NOT_FOUND);
 
    //  ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    //  //ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);
 
    //  // Check if any DS18B20 devices were found
    //  if (ds18b20_device_num == 0) {
    //      ESP_LOGE(TAG, "No DS18B20 devices found! Exiting task...");
    //      vTaskDelete(NULL);
    //  }
     espnow_queue = xQueueCreate(20, sizeof(espnow_message_t));
     if (espnow_queue == NULL) {
     ESP_LOGE("Queue", "Failed to create queue!");
     }

    while (1) 
    {
        //for (int i = 0; i < ds18b20_device_num; i++) {
        float temperature = 0.0;


        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw_1[0][0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN3, &adc_raw_3[0][0]));

        espnow_message_t message;

        message.soil_moisture = (uint8_t)(100 - ((adc_raw_1[0][0] * 100) / 3300));
        message.temperature = (uint8_t)temperature;
        message.battery_level = (uint8_t)((adc_raw_3[0][0] * 100) / 4095);
        
        UBaseType_t available = uxQueueSpacesAvailable(espnow_queue);
        ESP_LOGI("SensorTask", "Queue space available: %d", available);
        
        if (espnow_queue != NULL) {
            if (xQueueSend(espnow_queue, &message, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE("SensorTask", "Failed to send data to queue");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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