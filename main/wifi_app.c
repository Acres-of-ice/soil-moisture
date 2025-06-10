#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "mdns.h"

#include "data.h"
#include "define.h"
#include "http_server.h"
#include "lcd.h"
#include "wifi_app.h"

static const char TAG[] = "WIFI";

static esp_timer_handle_t shutdown_timer = NULL;

// Used for returning the WiFi configuration
wifi_config_t *wifi_config = NULL;

// netif objects for the station and access point
esp_netif_t *esp_netif_sta = NULL;
esp_netif_t *esp_netif_ap = NULL;

QueueHandle_t wifi_app_queue_handle = NULL;

// static EventGroupHandle_t wifi_event_group;
// #define WIFI_CONNECTED_BIT BIT0

static SemaphoreHandle_t timer_mutex = NULL;

void wifi_init(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Disable default WiFi logging messages

  // Allocate memory for the wifi configuration
  wifi_config = (wifi_config_t *)malloc(sizeof(wifi_config_t));
  memset(wifi_config, 0x00, sizeof(wifi_config_t));

  // Create message queue
  wifi_app_queue_handle = xQueueCreate(5, sizeof(wifi_app_queue_message_t));
}

esp_err_t init_mdns_service(void) {
  esp_err_t err;

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
    AUTO_mode = true;
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

// /**
//  * WiFi application event handler
//  * @param arg data, aside from event data, that is passed to the handler when
//  it
//  * is called
//  * @param event_base the base id of the event to register the handler for
//  * @param event_id the id fo the event to register the handler for
//  * @param event_data event data
//  */
// static void wifi_app_event_handler(void *netif, esp_event_base_t event_base,
//                                    int32_t event_id, void *event_data) {
//   if (event_base == WIFI_EVENT) {
//     switch (event_id) {
//
//     case WIFI_EVENT_AP_STACONNECTED:
//       ESP_LOGI(TAG, "Client connected");
//       // start_shutdown_timer(SERVER_TIMEOUT_S);
//       break;
//     }
//   }
// }

// /**
//  * Initializes the WiFi application event handler for WiFi and IP events.
//  */
// static void wifi_app_event_handler_init(void) {
//   // Event handler for the connection
//   esp_event_handler_instance_t instance_wifi_event;
//   esp_event_handler_instance_t instance_ip_event;
//   wifi_event_group = xEventGroupCreate();
//   ESP_ERROR_CHECK(esp_event_handler_instance_register(
//       WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL,
//       &instance_wifi_event));
// }

/**
 * Initializes the TCP stack and default WiFi configuration.
 */
static void wifi_app_default_wifi_init(void) {
  // Initialize the TCP stack
  ESP_ERROR_CHECK(esp_netif_init());

  // Default WiFi config - operations must be in this order!
  wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  // Create default interfaces for both AP and STA
  esp_netif_ap = esp_netif_create_default_wifi_ap();
  esp_netif_sta = esp_netif_create_default_wifi_sta();
}

/**
 * Configure AP for web server with power optimization settings
 */
static void wifi_app_ap_config(void) {
  wifi_config_t ap_config = {
      .ap =
          {                        // .channel = WIFI_AP_CHANNEL,
           .max_connection = 2,    // Reduced from default for power saving
           .beacon_interval = 300, // Increased from default 100ms
           .authmode = WIFI_AUTH_WPA2_PSK},
  };

  ap_config.ap.channel = CONFIG_ESPNOW_CHANNEL;

  strlcpy((char *)ap_config.ap.ssid, WIFI_AP_SSID, sizeof(ap_config.ap.ssid));
  strlcpy((char *)ap_config.ap.password, WIFI_AP_PASSWORD,
          sizeof(ap_config.ap.password));

  // Don't change to AP mode, keep APSTA mode for ESP-NOW
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));

  // Configure DNS for AP mode
  esp_netif_dns_info_t dns_info = {
      .ip.u_addr.ip4.addr = ipaddr_addr("192.168.4.1"), // AP's default IP
      .ip.type = IPADDR_TYPE_V4};
  ESP_ERROR_CHECK(
      esp_netif_set_dns_info(esp_netif_ap, ESP_NETIF_DNS_MAIN, &dns_info));

  // Initialize mDNS after WiFi starts
  init_mdns_service();
}

/**
 * Main task for the WiFi application
 * @param pvParameters parameter which can be passed to the task
 */
void wifi_app_task(void *pvParameters) {
  wifi_app_queue_message_t msg;

  // // Initialize the event handler
  // wifi_app_event_handler_init();

  // Initialize the TCP/IP stack and WiFi config
  // wifi_app_default_wifi_init();
  wifi_app_espnow_wifi_init();
  vTaskDelay(2000);
  wifi_app_ap_config();
  wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);

  for (;;) {
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
      ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }

    if (xQueueReceive(wifi_app_queue_handle, &msg, portMAX_DELAY)) {
      switch (msg.msgID) {

      case WIFI_APP_MSG_START_HTTP_SERVER:

        start_shutdown_timer(SERVER_TIMEOUT_S);
        http_server_active = true;
        // stop_wifi_blinking();

        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) ==
            pdTRUE) { // Changed from portMAX_DELAY to timeout
          lcd_put_cur(0, 8);
          lcd_send_data(WIFI_SYMBOL_ADDRESS);
          xSemaphoreGive(i2c_mutex);
        } else {
          // Log if we couldn't get the mutex
          ESP_LOGW(
              TAG,
              "Could not get i2c mutex for LCD update - continuing shutdown");
        }
        ESP_LOGI(TAG, "WIFI_APP_MSG_START_HTTP_SERVER");

        // Update LCD with IP address
        update_status_message("192.168.4.1");

        ESP_LOGI(TAG, "WiFi AP configured with power-saving settings");
        ESP_LOGI(TAG, "WiFi AP SSID: %s password: %s", WIFI_AP_SSID,
                 WIFI_AP_PASSWORD);

        // Suspend data logging and backup tasks
        if (dataLoggingTaskHandle != NULL) {
          vTaskSuspend(dataLoggingTaskHandle);
        }
        if (smsTaskHandle != NULL &&
            eTaskGetState(smsTaskHandle) != eSuspended) {
          vTaskSuspend(smsTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        http_server_start();
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

        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) ==
            pdTRUE) { // Changed from portMAX_DELAY to timeout
          lcd_put_cur(0, 8);
          lcd_send_string(" ");
          xSemaphoreGive(i2c_mutex);
        } else {
          // Log if we couldn't get the mutex
          ESP_LOGW(
              TAG,
              "Could not get i2c mutex for LCD update - continuing shutdown");
        }

        http_server_stop();

        ESP_LOGI(TAG, "Web server disabled successfully");

        if (dataLoggingTaskHandle != NULL) {
          vTaskResume(dataLoggingTaskHandle);
        }
        if (smsTaskHandle != NULL) {
          vTaskResume(smsTaskHandle);
        }

        break;

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
  esp_netif_t *netif = esp_netif_next_unsafe(NULL);
  while (netif != NULL) {
    if (esp_netif_get_desc(netif) != NULL &&
        strcmp("ap", esp_netif_get_desc(netif)) == 0) {
      ESP_LOGD(TAG, "Found existing AP interface");
      esp_netif_ap = netif;
      break;
    }
    netif = esp_netif_next_unsafe(netif);
  }

  // Find existing STA interface
  netif = esp_netif_next_unsafe(NULL);
  while (netif != NULL) {
    if (esp_netif_get_desc(netif) != NULL &&
        strcmp("sta", esp_netif_get_desc(netif)) == 0) {
      ESP_LOGI(TAG, "Found existing STA interface");
      esp_netif_sta = netif;
      break;
    }
    netif = esp_netif_next_unsafe(netif);
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

  // // Step 4: Set up event handlers for web server functionality
  // static bool event_handlers_registered = false;
  // if (!event_handlers_registered) {
  //   wifi_app_event_handler_init();
  //   event_handlers_registered = true;
  // }

  ESP_LOGD(TAG, "Web server network interfaces initialized successfully");
}
