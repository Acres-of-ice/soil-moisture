#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "http_server.h"
#include "wifi_app.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "lcd.h"

// Tag used for ESP serial console messages
static const char TAG[] = "WIFI";

// static bool wifi_icon_state = false;
// static esp_timer_handle_t blink_timer = NULL;

static esp_timer_handle_t shutdown_timer = NULL;

// WiFi application callback
static wifi_connected_event_callback_t wifi_connected_event_cb;

// Used for returning the WiFi configuration
wifi_config_t *wifi_config = NULL;

// netif objects for the station and access point
esp_netif_t *esp_netif_sta = NULL;
esp_netif_t *esp_netif_ap = NULL;

QueueHandle_t wifi_app_queue_handle = NULL;

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static int s_retry_num = 0;
static bool connecting = false;
static bool got_ip = false;
static bool wifi_sta_connected = false;
static bool connection_established = false;
static bool wifi_stopping = false;
static bool wifi_disconnect_requested = false;
static SemaphoreHandle_t timer_mutex = NULL;
static esp_timer_handle_t reconnect_timer = NULL;
static bool reconnect_timer_active = false;
static wifi_ap_record_t ap_records[MAX_NETWORKS];

bool sta_enabled = false;  // WiFi status flag
extern bool wifi_enabled; // WiFi status flag

esp_err_t init_mdns_service(void) {
  esp_err_t err;

  // Stop any existing mDNS service
  mdns_free();
  vTaskDelay(pdMS_TO_TICKS(500)); // Longer wait for cleanup

  // Initialize mDNS with proper config
  err = mdns_init();
  if (err) {
    ESP_LOGE(TAG, "mDNS init failed: %d", err);
    return err;
  }

  // Simplify the mDNS configuration - just set hostname and service
  err = mdns_hostname_set("iceacres");
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "mdns_hostname_set failed: %d", err);
    return err;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // Wait after setting hostname

  // Basic service configuration
  err = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "mdns_service_add failed: %d", err);
    return err;
  }

  ESP_LOGI(TAG, "Server accessible at:");
  ESP_LOGI(TAG, "192.168.4.1");
  ESP_LOGI(TAG, "2. http://iceacres.local (if mDNS is supported)");

  return ESP_OK;
}

static void wifi_reconnect_timer_callback(void *arg) {
  if (!wifi_sta_connected && !connecting) {
    ESP_LOGI(TAG, "Auto-reconnect timer triggered, attempting to reconnect...");
    connecting = true; // Set connecting flag to prevent multiple attempts
    ESP_LOGW(TAG, "Rebooting...");
    vTaskDelay(1000);
    esp_restart();
    wifi_app_send_message(WIFI_APP_MSG_START_STA);
  } else {
    ESP_LOGI(TAG, "Failed Auto-reconnect timer");
  }
}

void start_shutdown_timer(int shutdown_delay_seconds) {
  // Create mutex if it doesn't exist
  if (timer_mutex == NULL) {
    timer_mutex = xSemaphoreCreateMutex();
  }

  // Take mutex with timeout
  if (xSemaphoreTake(timer_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take timer mutex");
    return;
  }

  esp_err_t err = ESP_OK;

  // Safely handle existing timer
  if (shutdown_timer != NULL) {
    // Try to stop timer if it's running
    err = esp_timer_stop(shutdown_timer);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "Failed to stop timer: %s", esp_err_to_name(err));
    }

    // Delete existing timer
    err = esp_timer_delete(shutdown_timer);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to delete timer: %s", esp_err_to_name(err));
    }
    shutdown_timer = NULL;
  }

  // Create new timer
  esp_timer_create_args_t timer_args = {.callback = &wifi_app_shutdown_callback,
                                        .name = "shutdown_timer"};

  err = esp_timer_create(&timer_args, &shutdown_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
    xSemaphoreGive(timer_mutex);
    return;
  }

  // Start timer
  err = esp_timer_start_once(shutdown_timer,
                             (int64_t)shutdown_delay_seconds * 1000000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    esp_timer_delete(shutdown_timer);
    shutdown_timer = NULL;
  } else {
    ESP_LOGI(TAG, "HTTP server will automatically shut down in %d minutes",
             (shutdown_delay_seconds / 60));
  }

  xSemaphoreGive(timer_mutex);
}

void wifi_app_shutdown_callback(void *arg) {
  if (xSemaphoreTake(timer_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take timer mutex in callback");
    return;
  }

  wifi_sta_list_t sta_list;
  esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);

  // Proceed with shutdown if either:
  // 1. We failed to get station list (likely because no clients ever connected)
  // 2. Station list is empty (no active clients)
  if (err != ESP_OK || sta_list.num == 0) {
    ESP_LOGI(
        TAG,
        "Shutdown timer expired, no clients connected. Stopping HTTP server");
    wifi_enabled = false;
    //AUTO_mode = true;
    wifi_app_send_message(WIFI_APP_MSG_STOP_HTTP_SERVER);
  } else {
    ESP_LOGI(TAG, "Clients still connected. Resetting shutdown timer");
    shutdown_timer = NULL; // Reset pointer since timer has expired
    xSemaphoreGive(
        timer_mutex); // Release mutex before calling start_shutdown_timer
    start_shutdown_timer(SERVER_TIMEOUT_S);
    return;
  }

  xSemaphoreGive(timer_mutex);
}

/**
 * WiFi application event handler
 * @param arg data, aside from event data, that is passed to the handler when it
 * is called
 * @param event_base the base id of the event to register the handler for
 * @param event_id the id fo the event to register the handler for
 * @param event_data event data
 */
static void wifi_app_event_handler(void *netif, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGD(TAG, "WIFI_EVENT_STA_START");
      wifi_stopping = false;
      // esp_wifi_connect();
      break;

    case WIFI_EVENT_STA_DISCONNECTED:
      connection_established = false;
      ESP_LOGD(TAG, "WIFI_EVENT_STA_DISCONNECTED");
      xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
      wifi_app_send_message(WIFI_APP_MSG_STA_DISCONNECTED);
      break;

    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGD(TAG, "WIFI_EVENT_STA_CONNECTED");
      break;

    case WIFI_EVENT_AP_START:
      ESP_LOGI(TAG, "AP Started - Starting 5 minute shutdown timer");
      start_shutdown_timer(SERVER_TIMEOUT_S);
      break;

    case WIFI_EVENT_AP_STOP:
      ESP_LOGD(TAG, "WIFI_EVENT_AP_STOP");
      break;

    case WIFI_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG, "Client connected - Resetting timer");
      start_shutdown_timer(SERVER_TIMEOUT_S);
      break;
    }
  } else if (event_base == IP_EVENT) {
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP:
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

      // Convert the received IP to string for LCD display
      char ip_str[16];
      sprintf(ip_str, IPSTR, IP2STR(&event->ip_info.ip));

      // Update LCD with IP address
      // Added: Display IP on LCD
      update_status_message("%s", ip_str);
      xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

      vTaskDelay(pdMS_TO_TICKS(500));

      connection_established = true;
      wifi_sta_connected = true;
      got_ip = true;
      connecting = false;
      s_retry_num = 0;

      // init_mdns_service();
      wifi_app_send_message(WIFI_APP_MSG_STA_CONNECTED);
      break;
    }
  }
}

/**
 * Initializes the WiFi application event handler for WiFi and IP events.
 */
static void wifi_app_event_handler_init(void) {
  // Event loop for the WiFi driver
 // ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Event handler for the connection
  esp_event_handler_instance_t instance_wifi_event;
  esp_event_handler_instance_t instance_ip_event;
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL,
      &instance_wifi_event));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL,
      &instance_ip_event));
}

/**
 * Initializes the TCP stack and default WiFi configuration.
 */
static void wifi_app_default_wifi_init(void) {
  // Initialize the TCP stack
  ESP_ERROR_CHECK(esp_netif_init());
  //printf("\nInside default task1 \n");
  // Default WiFi config - operations must be in this order!
  wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  //printf("\nInside default task2 \n");
  //ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
  //printf("\nInside default task3 \n");
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
 // printf("\nInside default task4 \n");
  // Create default interfaces for both AP and STA
  esp_netif_ap = esp_netif_create_default_wifi_ap();
  //printf("\nInside default task5 \n");
  esp_netif_sta = esp_netif_create_default_wifi_sta();
  //printf("\nInside default task6 \n");
}

void wifi_app_ap_config(void) {
  wifi_config_t ap_config = {
      .ap = {.channel = WIFI_AP_CHANNEL,
             .max_connection = WIFI_AP_MAX_CONNECTIONS,
             .authmode = WIFI_AUTH_WPA2_PSK},
  };

  strlcpy((char *)ap_config.ap.ssid, WIFI_AP_SSID, sizeof(ap_config.ap.ssid));
  strlcpy((char *)ap_config.ap.password, WIFI_AP_PASSWORD,
          sizeof(ap_config.ap.password));

  //ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  //ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_APSTA, &ap_config));
  //ESP_ERROR_CHECK(esp_wifi_start());
  // Configure DNS for AP mode
  esp_netif_dns_info_t dns_info = {
      .ip.u_addr.ip4.addr = ipaddr_addr("192.168.4.1"), // AP's default IP
      .ip.type = IPADDR_TYPE_V4};
  ESP_ERROR_CHECK(
      esp_netif_set_dns_info(esp_netif_ap, ESP_NETIF_DNS_MAIN, &dns_info));

  // Update LCD with IP address
  update_status_message("192.168.4.1");
  ESP_LOGI(TAG, "http://192.168.4.1");
  // Initialize mDNS after WiFi starts
  init_mdns_service();

  ESP_LOGI(TAG, "WiFi AP started with SSID: %s password: %s", WIFI_AP_SSID,
           WIFI_AP_PASSWORD);
}

static void wifi_app_sta_config(void) {
  wifi_config_t sta_config = {0};
  esp_err_t ret;

  // Reset states
  connection_established = false;
  wifi_stopping = false;
  wifi_disconnect_requested = false;
  s_retry_num = 0;

  // Make sure WiFi is stopped and wait a bit longer
  esp_wifi_disconnect();
  esp_wifi_stop();
  vTaskDelay(pdMS_TO_TICKS(500)); // Increased delay to ensure clean state

  // Initialize reconnect timer if needed
  if (reconnect_timer == NULL) {
    esp_timer_create_args_t timer_args = {
        .callback = wifi_reconnect_timer_callback, .name = "wifi_reconnect"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &reconnect_timer));
  }

  // Outer loop - try the full network list up to 5 times
  for (int full_cycle = 0; full_cycle < 5; full_cycle++) {
    ESP_LOGI(TAG, "Starting full network scan cycle %d/5", full_cycle + 1);

    // Set STA mode first
    //ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    //ESP_ERROR_CHECK(esp_wifi_start());
    //vTaskDelay(pdMS_TO_TICKS(500));

    // Try each network once in this cycle
    for (int i = 0; i < MAX_NETWORKS; i++) {
      // Skip empty SSIDs
      if (strlen(known_networks[i].ssid) == 0) {
        continue;
      }

      // Clear previous config
      memset(&sta_config, 0, sizeof(wifi_config_t));

      // Copy SSID and password
      size_t ssid_len = strlen(known_networks[i].ssid);
      size_t pass_len = strlen(known_networks[i].password);

      if (ssid_len > 0 && ssid_len <= 32) {
        memcpy(sta_config.sta.ssid, known_networks[i].ssid, ssid_len);
        memcpy(sta_config.sta.password, known_networks[i].password, pass_len);

        ESP_LOGI(TAG, "Cycle %d/5: Trying network %d/%d: %s", full_cycle + 1,
                 i + 1, MAX_NETWORKS, sta_config.sta.ssid);

        // Stop any ongoing scan
        esp_wifi_scan_stop();
        vTaskDelay(pdMS_TO_TICKS(200));

        // Configure scan
        wifi_scan_config_t scan_config = {
            .ssid = sta_config.sta.ssid, // Changed: Only scan for current SSID
            .bssid = NULL,
            .channel = 0,
            .show_hidden = false};

        // Start scan with error handling
        ret = esp_wifi_scan_start(&scan_config, true);
        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Scan failed with error %d, trying next network...",
                   ret);
          continue;
        }

        // Get scan results
        uint16_t ap_count = 0;
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
        if (ap_count > MAX_NETWORKS) {
          ap_count = MAX_NETWORKS;
        }

        // if (ap_count > 0) {
        //     wifi_ap_record_t *ap_records = malloc(sizeof(wifi_ap_record_t) *
        //     ap_count); if (ap_records == NULL) {
        //         ESP_LOGE(TAG, "Failed to allocate memory for scan results");
        //         continue;
        //     }

        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));

        bool network_found = false;
        for (int j = 0; j < ap_count; j++) {
          if (strcmp((char *)ap_records[j].ssid, (char *)sta_config.sta.ssid) ==
              0) {
            network_found = true;
            break;
          }
        }

        // free(ap_records);

        if (!network_found) {
          ESP_LOGI(TAG, "Network %s not found in scan results",
                   sta_config.sta.ssid);
          continue;
        }

        ESP_LOGI(TAG, "Network %s found, attempting connection",
                 sta_config.sta.ssid);
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));

        ret = esp_wifi_connect();
        if (ret == ESP_OK) {
          // Wait for connection and IP
          EventBits_t bits =
              xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE,
                                  pdFALSE, pdMS_TO_TICKS(5000));

          if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Successfully connected to %s", sta_config.sta.ssid);
            return; // Exit immediately after successful connection
          }
        }
        // }
      }

      vTaskDelay(pdMS_TO_TICKS(500));
    }

    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(100));

    if (full_cycle < 4) {
      ESP_LOGI(TAG, "Completed cycle %d/5, waiting before next cycle...",
               full_cycle + 1);
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }

  ESP_LOGW(TAG,
           "Failed to connect to any known network after 5 complete cycles");
  xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

#ifdef CONFIG_AWS
  if (!reconnect_timer_active && !wifi_stopping) {
    ESP_LOGI(TAG, "Starting auto-reconnect timer (2 minute interval)");
    if (reconnect_timer == NULL) {
      esp_timer_create_args_t timer_args = {
          .callback = wifi_reconnect_timer_callback, .name = "wifi_reconnect"};
      ESP_ERROR_CHECK(esp_timer_create(&timer_args, &reconnect_timer));
    }
    ESP_ERROR_CHECK(esp_timer_start_periodic(
        reconnect_timer, WIFI_RECONNECT_INTERVAL_MS * 1000));
    reconnect_timer_active = true;
    connecting = false;
  }
#endif
}

/**
 * Main task for the WiFi application
 * @param pvParameters parameter which can be passed to the task
 */
void wifi_app_task(void *pvParameters) {
 ESP_LOGI(TAG, "Inside wifi task");
 wifi_app_queue_message_t msg;

 // Initialize the event handler
 wifi_app_event_handler_init();

 // Initialize the TCP/IP stack and WiFi config
 // wifi_app_default_wifi_init();
 wifi_app_espnow_wifi_init();
 ESP_LOGI(TAG,"wifi app task1");
 vTaskDelay(2000);
 wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
 ESP_LOGI(TAG,"wifi app task2");
#ifdef CONFIG_AWS
  vTaskDelay(2000);
  wifi_app_send_message(WIFI_APP_MSG_START_STA);
#endif

  // #ifdef CONFIG_CONDUCTOR
  //   vTaskDelay(2000);
  //   wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
  // #endif
  // if (IS_SITE("Test")){
  //   vTaskDelay(2000);
  //   wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
  //   // wifi_enabled = true;
  // }
 
  for (;;) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    if (xQueueReceive(wifi_app_queue_handle, &msg, portMAX_DELAY)) {
      switch (msg.msgID) {
      
      case WIFI_APP_MSG_START_HTTP_SERVER:
      ESP_LOGI(TAG,"wifi app task3");
        http_server_active = true;
        // stop_wifi_blinking();

        // if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) ==
        //     pdTRUE) { // Changed from portMAX_DELAY to timeout
        //   lcd_put_cur(0, 8);
        //   lcd_send_data(WIFI_SYMBOL_ADDRESS);
        //   xSemaphoreGive(i2c_mutex);
        // } else {
        //   // Log if we couldn't get the mutex
        //   ESP_LOGW(
        //       TAG,
        //       "Could not get i2c mutex for LCD update - continuing shutdown");
        // }
        ESP_LOGI(TAG, "WIFI_APP_MSG_START_HTTP_SERVER");
        //wifi_app_ap_config();

        // Suspend data logging and backup tasks
        // if (adcTaskHandle != NULL) {
        //     vTaskSuspend(adcTaskHandle);
        // }
        // if (dataLoggingTaskHandle != NULL) {
        //   vTaskSuspend(dataLoggingTaskHandle);
        // }
        // if (backupTaskHandle != NULL) {
        //     vTaskSuspend(backupTaskHandle);
        // }
        // // Suspend all SMS-related tasks first and ensure they stay suspended
        // if (smsReceiveTaskHandle != NULL &&
        // eTaskGetState(smsReceiveTaskHandle) != eSuspended) {
        //     vTaskSuspend(smsReceiveTaskHandle);
        // }
        // if (smsTaskHandle != NULL &&
        //     eTaskGetState(smsTaskHandle) != eSuspended) {
        //   vTaskSuspend(smsTaskHandle);
        // }
        // if (smsManagerTaskHandle != NULL &&
        // eTaskGetState(smsManagerTaskHandle) != eSuspended) {
        //     vTaskSuspend(smsManagerTaskHandle);
        // }
        ESP_LOGI(TAG,"wifi app task4");
        vTaskDelay(pdMS_TO_TICKS(500));
        http_server_start();
        break;

      case WIFI_APP_MSG_START_STA:
        ESP_LOGI(TAG, "WIFI_APP_MSG_START_STA");
        // #ifdef CONFIG_CONDUCTOR
        //   start_wifi_blinking();
        // #endif
        wifi_app_sta_config();
        break;

      case WIFI_APP_MSG_STA_CONNECTED:
        connecting = false;
        sta_enabled = true;
        // Stop reconnection timer if it's running
        if (reconnect_timer_active) {
          ESP_ERROR_CHECK(esp_timer_stop(reconnect_timer));
          reconnect_timer_active = false;
        }
        s_retry_num = 0; // Reset retry counter on successful connection
       
        wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
#ifdef CONFIG_CONDUCTOR
        wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
#endif
#ifdef CONFIG_AWS
        wifi_app_send_message(WIFI_APP_MSG_START_MQTT);
#endif
        break;

      case WIFI_APP_MSG_STOP_HTTP_SERVER:
        http_server_active = false;
        ESP_LOGI(TAG, "WIFI_APP_MSG_STOP_HTTP_SERVER");

        // Stop shutdown timer if it exists
        if (shutdown_timer != NULL) {
          esp_timer_stop(shutdown_timer);
          esp_timer_delete(shutdown_timer);
          shutdown_timer = NULL;
        }

        // if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) ==
        //     pdTRUE) { // Changed from portMAX_DELAY to timeout
        //   lcd_put_cur(0, 8);
        //   lcd_send_string(" ");
        //   xSemaphoreGive(i2c_mutex);
        // } else {
        //   // Log if we couldn't get the mutex
        //   ESP_LOGW(
        //       TAG,
        //       "Could not get i2c mutex for LCD update - continuing shutdown");
        // }

        http_server_stop();

        esp_err_t err;

        // err = esp_wifi_stop();
        // if (err != ESP_OK) {
        //   ESP_LOGE(TAG, "Failed to stop Wi-Fi: %s", esp_err_to_name(err));
        // }

        // Set Wi-Fi mode to NONE
        // err = esp_wifi_set_mode(WIFI_MODE_NULL);
        // if (err != ESP_OK) {
        //   ESP_LOGE(TAG, "Failed to set Wi-Fi mode to NULL: %s",
        //            esp_err_to_name(err));
        // }
        mdns_free();

        ESP_LOGI(TAG, "Web server disabled successfully");

        // Resume data logging and backup tasks
        // if (adcTaskHandle != NULL) {
        //     vTaskResume(adcTaskHandle);
        // }
        // if (dataLoggingTaskHandle != NULL) {
        //   vTaskResume(dataLoggingTaskHandle);
        // }
        // if (backupTaskHandle != NULL) {
        //     vTaskResume(backupTaskHandle);
        // }
        // if (smsManagerTaskHandle != NULL) {
        //     vTaskResume(smsManagerTaskHandle);
        // }
        // if (smsTaskHandle != NULL) {
        //   vTaskResume(smsTaskHandle);
        // }
        // if (smsReceiveTaskHandle != NULL) {
        //     vTaskResume(smsReceiveTaskHandle);
        // }

        break;

      case WIFI_APP_MSG_STOP_STA:
        ESP_LOGI(TAG, "WIFI_APP_MSG_STOP_STA");
        wifi_disconnect_requested = true;
        wifi_stopping = true;
        wifi_enabled = false;
        connection_established = false;
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
   
        wifi_app_send_message(WIFI_APP_MSG_STOP_HTTP_SERVER);
#ifdef CONFIG_CONDUCTOR
        wifi_app_send_message(WIFI_APP_MSG_STOP_HTTP_SERVER);
#endif
#ifdef CONFIG_AWS
        wifi_app_send_message(WIFI_APP_MSG_STOP_MQTT);
#endif

        // Wait a bit for MQTT to stop
        vTaskDelay(pdMS_TO_TICKS(1000));

        err = esp_wifi_stop();
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to stop Wi-Fi: %s", esp_err_to_name(err));
        }

        // Set Wi-Fi mode to NONE
        err = esp_wifi_set_mode(WIFI_MODE_NULL);
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to set Wi-Fi mode to NULL: %s",
                   esp_err_to_name(err));
        }
        mdns_free();

        ESP_LOGI(TAG, "Wifi disabled successfully");

        break;

      case WIFI_APP_MSG_STA_DISCONNECTED:

        wifi_sta_connected = false;

#ifdef CONFIG_AWS
        ESP_LOGW(TAG, "Wifi disconnected, shutting down WiFi");
        if (!wifi_stopping && !wifi_disconnect_requested) {
          if (!reconnect_timer_active) {
            ESP_LOGI(TAG, "Starting auto-reconnect timer (2 minute interval)");
            if (reconnect_timer == NULL) {
              esp_timer_create_args_t timer_args = {
                  .callback = wifi_reconnect_timer_callback,
                  .name = "wifi_reconnect"};
              ESP_ERROR_CHECK(esp_timer_create(&timer_args, &reconnect_timer));
            }
            ESP_ERROR_CHECK(esp_timer_start_periodic(
                reconnect_timer, WIFI_RECONNECT_INTERVAL_MS * 1000));
            reconnect_timer_active = true;
          }
        }
        wifi_app_send_message(WIFI_APP_MSG_STOP_STA);
#endif
       
        wifi_app_send_message(WIFI_APP_MSG_STOP_STA);
#ifdef CONFIG_CONDUCTOR
        wifi_app_send_message(WIFI_APP_MSG_STOP_STA);
#endif

        if (!connection_established) {
          sta_enabled = false;
        //   if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        //     lcd_put_cur(0, 8);
        //     lcd_send_string(" ");
        //     xSemaphoreGive(i2c_mutex);
        //   }
#ifdef CONFIG_AWS
          wifi_app_send_message(WIFI_APP_MSG_STOP_MQTT);
#endif
        }
        break;

#ifdef CONFIG_AWS
      case WIFI_APP_MSG_START_MQTT:
        if (mqttTaskHandle == NULL) {
          xTaskCreatePinnedToCore(mqtt_task, "MQTT", MQTT_TASK_STACK_SIZE, NULL,
                                  MQTT_TASK_PRIORITY, &mqttTaskHandle,
                                  MQTT_TASK_CORE_ID);
        } else {
          vTaskResume(mqttTaskHandle);
        }
        break;

      case WIFI_APP_MSG_STOP_MQTT:
        if (mqttTaskHandle != NULL) {
          vTaskSuspend(mqttTaskHandle);
        }
        break;
#endif

      default:
        break;
      }
    }
  }
}

BaseType_t wifi_app_send_message(wifi_app_message_e msgID) {
  wifi_app_queue_message_t msg;
  msg.msgID = msgID;
  // return xQueueSend(wifi_app_queue_handle, &msg, portMAX_DELAY);
  return xQueueSend(wifi_app_queue_handle, &msg, 0);
}

wifi_config_t *wifi_app_get_wifi_config(void) {
  if (wifi_config == NULL) {
    wifi_config = (wifi_config_t *)malloc(sizeof(wifi_config_t));
  }
  return wifi_config;
}

void wifi_app_set_callback(wifi_connected_event_callback_t cb) {
  wifi_connected_event_cb = cb;
}

void wifi_app_call_callback(void) { wifi_connected_event_cb(); }

int8_t wifi_app_get_rssi(void) {
  wifi_ap_record_t wifi_data;

  ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&wifi_data));

  return wifi_data.rssi;
}

void wifi_init(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Allocate memory for the wifi configuration
  wifi_config = (wifi_config_t *)malloc(sizeof(wifi_config_t));
  memset(wifi_config, 0x00, sizeof(wifi_config_t));

  // Create message queue
  wifi_app_queue_handle = xQueueCreate(5, sizeof(wifi_app_queue_message_t));
}

// void wifi_init(void) {
//   // Initialize NVS
//   esp_err_t ret = nvs_flash_init();
//   if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
//       ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//     ESP_ERROR_CHECK(nvs_flash_erase());
//     ret = nvs_flash_init();
//   }
//   ESP_ERROR_CHECK(ret);

//   // Disable default WiFi logging messages

//   // Allocate memory for the wifi configuration
//   wifi_config = (wifi_config_t *)malloc(sizeof(wifi_config_t));
//   memset(wifi_config, 0x00, sizeof(wifi_config_t));

//   ESP_ERROR_CHECK(esp_netif_init());
//   ESP_ERROR_CHECK(esp_event_loop_create_default());
//   //     // Create default AP interface
//     // esp_netif_create_default_wifi_ap();

// //     // Get MAC address for unique SSID
//     uint8_t mac[6];
//     ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
//     char ssid[32];
//     snprintf(ssid, sizeof(ssid), "ESP-NOW-RSSI-%02X%02X", mac[0], mac[1]);
  
// //     // Initialize and configure WiFi
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); // APSTA mode for both AP and ESP-NOW


  
// //     // Set up AP configuration
//     wifi_config_t ap_config = {0};
//     strcpy((char *)ap_config.ap.ssid, ssid);
//     ap_config.ap.ssid_len = strlen(ssid);
//     strcpy((char *)ap_config.ap.password, "12345678");
//     ap_config.ap.channel = CONFIG_ESPNOW_CHANNEL;
//     ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
//     ap_config.ap.max_connection = 4;
  
// //     // Apply config and start WiFi
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
//     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
//     ESP_ERROR_CHECK(esp_wifi_start());


  
// //     // Power saving settings
//     ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
//     ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84)); // Maximum transmission power
//     ESP_ERROR_CHECK(
//         esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
  
//     ESP_LOGI(TAG, "WiFi AP started: SSID=%s, Password=12345678, IP=192.168.4.1",
//              ssid);
//   // Create message queue
//   wifi_app_queue_handle = xQueueCreate(5, sizeof(wifi_app_queue_message_t));
// }

/**
 * Initialize web server network interfaces without reinitializing WiFi
 * Assumes ESP-NOW has already initialized WiFi in APSTA mode
 */
void wifi_app_espnow_wifi_init(void) {
  ESP_LOGI(TAG,
           "Setting up web server with existing ESP-NOW WiFi configuration");

  // Step 1: Create message queue for web server
  if (wifi_app_queue_handle == NULL) {
    wifi_app_queue_handle = xQueueCreate(5, sizeof(wifi_app_queue_message_t));
    if (wifi_app_queue_handle == NULL) {
      ESP_LOGE(TAG, "Failed to create WiFi app queue");
      return;
    }
  }

  // Step 2: Verify WiFi is initialized and in correct mode
  wifi_mode_t current_mode;
  esp_err_t ret = esp_wifi_get_mode(&current_mode);

  if (ret == ESP_ERR_WIFI_NOT_INIT) {
    ESP_LOGW(TAG, "WiFi not initialized - running default initialization");
    wifi_app_default_wifi_init();
    return;
  }

  if (current_mode != WIFI_MODE_APSTA) {
    ESP_LOGW(TAG, "WiFi not in APSTA mode - correcting mode");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  }

  // Step 3: Locate or create network interfaces needed for web server
  // Find existing AP interface
  esp_netif_t *netif = esp_netif_next(NULL);
  while (netif != NULL) {
    if (esp_netif_get_desc(netif) != NULL &&
        strcmp("ap", esp_netif_get_desc(netif)) == 0) {
      ESP_LOGI(TAG, "Found existing AP interface");
      esp_netif_ap = netif;
      break;
    }
    netif = esp_netif_next(netif);
  }

  // Find existing STA interface
  netif = esp_netif_next(NULL);
  while (netif != NULL) {
    if (esp_netif_get_desc(netif) != NULL &&
        strcmp("sta", esp_netif_get_desc(netif)) == 0) {
      ESP_LOGI(TAG, "Found existing STA interface");
      esp_netif_sta = netif;
      break;
    }
    netif = esp_netif_next(netif);
  }

  // Create interfaces if not found
  if (esp_netif_ap == NULL) {
    ESP_LOGI(TAG, "Creating AP interface for web server");
    esp_netif_ap = esp_netif_create_default_wifi_ap();
  }

  if (esp_netif_sta == NULL) {
    ESP_LOGI(TAG, "Creating STA interface for web server");
    esp_netif_sta = esp_netif_create_default_wifi_sta();
  }

  // Step 4: Set up event handlers for web server functionality
  static bool event_handlers_registered = false;
  if (!event_handlers_registered) {
    wifi_app_event_handler_init();
    event_handlers_registered = true;
  }

  ESP_LOGI(TAG, "Web server network interfaces initialized successfully");
}
