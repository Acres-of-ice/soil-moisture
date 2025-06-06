#include "Pppos.h"
#include "esp_sntp.h"
#include "mqtt.h"
#include "netdb.h"
#include "rtc.h"
#include "sys/socket.h"
#include "sys/time.h"
#include "time.h"

static const char *TAG = "PPPOS";

#define MODEM_UART_TX_PIN GPIO_NUM_33
#define MODEM_UART_RX_PIN GPIO_NUM_32
#define MODEM_UART_RTS_PIN 0
#define MODEM_UART_CTS_PIN 0
#define MODEM_UART_NUM UART_NUM_2
#define MODEM_UART_RX_BUFFER_SIZE 1024
#define MODEM_UART_TX_BUFFER_SIZE 1024
#define MODEM_UART_EVENT_QUEUE_SIZE 10
#define MODEM_UART_EVENT_TASK_STACK_SIZE 4096
#define MODEM_UART_EVENT_TASK_PRIORITY 10
#define MODEM_PPP_APN "internet"

// Connection timeout configurations
#define PPP_CONNECTION_TIMEOUT_MS (120 * 1000) // 2 minutes
#define TIME_SYNC_TIMEOUT_MS (60 * 1000)       // 1 minute
#define MODEM_INIT_RETRY_COUNT 3
#define CONNECTION_RETRY_COUNT 3

// SNTP configuration
#define SNTP_SERVER_PRIMARY "pool.ntp.org"
#define SNTP_SERVER_SECONDARY "time.google.com"
#define TIME_SYNC_CHECK_INTERVAL_MS 1000

#define ERROR_CHECK_RETURN(err)                                                \
  ({                                                                           \
    esp_err_t __err_rc = (err);                                                \
    if (__err_rc != ESP_OK) {                                                  \
      ESP_LOGE("ERR", "%s, File: %s, Function: %s, Line: %d",                  \
               esp_err_to_name(__err_rc), __FILE__, __FUNCTION__, __LINE__);   \
      return __err_rc;                                                         \
    }                                                                          \
    __err_rc;                                                                  \
  })

// Global variables with proper protection
static esp_modem_dce_t *dce = NULL;
static esp_netif_t *esp_netif_ppp = NULL;
static volatile bool isModemConnectedToPPP = false;
static volatile bool isTimeSync = false;
static SemaphoreHandle_t connection_mutex = NULL;

// Function prototypes
static esp_err_t initialize_sntp_enhanced(void);
static esp_err_t wait_for_time_sync(uint32_t timeout_ms);
static esp_err_t cleanup_pppos_resources(void);
static esp_err_t validate_modem_connection(void);

static void time_sync_notification_cb(struct timeval *tv) {
  ESP_LOGD(TAG, "Time synchronization event received");
  isTimeSync = true;

  // Log the synchronized time
  time_t now = time(NULL);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char strftime_buf[64];
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGD(TAG, "Current time: %s", strftime_buf);
}

static esp_err_t initialize_sntp_enhanced(void) {
  ESP_LOGD(TAG, "Initializing enhanced SNTP configuration for India");

  // Stop SNTP if already running
  if (esp_sntp_enabled()) {
    esp_sntp_stop();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for clean stop
  }

  // Configure SNTP with more robust settings
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

  // Use Indian and reliable NTP servers for better connectivity
  esp_sntp_setservername(0, "in.pool.ntp.org");   // India NTP pool
  esp_sntp_setservername(1, "asia.pool.ntp.org"); // Asia NTP pool
  esp_sntp_setservername(2, "time.nist.gov");     // Global reliable
  esp_sntp_setservername(3, "time.google.com");   // Google time

  // Set Indian Standard Time (IST) - UTC+5:30
  setenv("TZ", "IST-5:30", 1);
  tzset();

  ESP_LOGD(TAG, "Timezone set to Indian Standard Time (IST, UTC+5:30)");

  // Set time sync notification callback
  esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);

  // Set sync mode to smooth for better cellular performance
  esp_sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);

  // Set sync interval (default is too long for initial sync)
  esp_sntp_set_sync_interval(15000); // 15 seconds for initial attempts

  // Initialize and start SNTP
  esp_sntp_init();

  ESP_LOGD(TAG, "SNTP initialized with Indian NTP servers and IST timezone");
  return ESP_OK;
}

esp_err_t wait_for_time_sync(uint32_t timeout_ms) {
  ESP_LOGI(TAG, "Waiting for SNTP time synchronization (timeout: %lu ms)",
           timeout_ms);

  TickType_t start_time = xTaskGetTickCount();
  TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  uint32_t check_count = 0;

  // Reset the sync flag to ensure we wait for actual SNTP sync
  isTimeSync = false;

  // Store the current time before SNTP to detect actual changes
  time_t time_before_sntp = time(NULL);
  ESP_LOGD(TAG, "Time before SNTP sync: %lld", (long long)time_before_sntp);

  // Force SNTP restart to ensure fresh synchronization
  esp_sntp_restart();

  // Give SNTP some time to start
  vTaskDelay(pdMS_TO_TICKS(3000));

  while (!isTimeSync) {
    if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
      ESP_LOGW(TAG, "SNTP synchronization timeout after %lu ms", timeout_ms);

      // Try manual time sync as fallback
      ESP_LOGI(TAG, "Attempting manual SNTP sync restart...");
      esp_sntp_restart();
      vTaskDelay(pdMS_TO_TICKS(5000));

      // Final check - but only accept if time has actually changed from SNTP
      time_t now = time(NULL);
      if (abs((long long)(now - time_before_sntp)) >
          5) { // Time changed by more than 5 seconds
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year > (2020 - 1900)) {
          ESP_LOGI(TAG,
                   "Time appears to have been updated by SNTP (change: %lld "
                   "seconds)",
                   (long long)(now - time_before_sntp));
          isTimeSync = true;
          break;
        }
      }

      return ESP_ERR_TIMEOUT;
    }

    check_count++;

    // Don't accept time just because year > 2020 - wait for actual SNTP
    // callback Only use this as a fallback after significant time passage
    if (check_count > 30) { // After 30 seconds of trying
      time_t now = time(NULL);
      if (abs((long long)(now - time_before_sntp)) >
          10) { // Time changed significantly
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_year > (2020 - 1900)) {
          ESP_LOGI(TAG,
                   "Fallback: Time appears synchronized (year: %d, change: "
                   "%lld sec)",
                   timeinfo.tm_year + 1900,
                   (long long)(now - time_before_sntp));
          isTimeSync = true;
          break;
        }
      }
    }

    // Log progress every 10 seconds
    if (check_count % 10 == 0) {
      time_t current_time = time(NULL);
      ESP_LOGI(
          TAG,
          "Still waiting for SNTP sync... (check %lu, time change: %lld sec)",
          check_count, (long long)(current_time - time_before_sntp));

      // Force another SNTP sync attempt every 20 seconds
      if (check_count % 20 == 0) {
        ESP_LOGI(TAG, "Forcing SNTP restart attempt...");
        esp_sntp_restart();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(TIME_SYNC_CHECK_INTERVAL_MS));
  }

  if (isTimeSync) {
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S",
             &timeinfo);
    ESP_LOGD(TAG, "SNTP synchronization successful: %s (change: %lld sec)",
             strftime_buf, (long long)(now - time_before_sntp));

    // Set longer sync interval after successful initial sync
    esp_sntp_set_sync_interval(604800000); // 7 days
    rtc_update_from_system();

    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

// static esp_err_t wait_for_time_sync(uint32_t timeout_ms) {
//   ESP_LOGI(TAG, "Waiting for time synchronization (timeout: %lu ms)",
//            timeout_ms);
//
//   TickType_t start_time = xTaskGetTickCount();
//   TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
//   uint32_t check_count = 0;
//
//   // First, let's give SNTP some time to start
//   vTaskDelay(pdMS_TO_TICKS(3000));
//
//   while (!isTimeSync) {
//     if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
//       ESP_LOGW(TAG, "Time synchronization timeout after %lu ms", timeout_ms);
//
//       // Try manual time sync as fallback
//       ESP_LOGI(TAG, "Attempting manual SNTP sync...");
//       esp_sntp_restart();
//       vTaskDelay(pdMS_TO_TICKS(5000));
//
//       // Check one more time after restart
//       time_t now = time(NULL);
//       struct tm timeinfo;
//       localtime_r(&now, &timeinfo);
//
//       if (timeinfo.tm_year > (2020 - 1900)) {
//         ESP_LOGI(TAG, "Manual sync successful (year: %d)",
//                  timeinfo.tm_year + 1900);
//         isTimeSync = true;
//         break;
//       }
//
//       return ESP_ERR_TIMEOUT;
//     }
//
//     // Check if we have a valid time (year > 2020) - fallback method
//     time_t now = time(NULL);
//     struct tm timeinfo;
//     localtime_r(&now, &timeinfo);
//
//     check_count++;
//
//     if (timeinfo.tm_year > (2020 - 1900)) {
//       ESP_LOGI(TAG,
//                "Time appears to be synchronized (year: %d) after %lu checks",
//                timeinfo.tm_year + 1900, check_count);
//       isTimeSync = true;
//       break;
//     }
//
//     // Log progress every 10 seconds
//     if (check_count % 10 == 0) {
//       ESP_LOGI(TAG,
//                "Still waiting for time sync... (check %lu, current year:
//                %d)", check_count, timeinfo.tm_year + 1900);
//
//       // Force SNTP sync attempt
//       esp_sntp_restart();
//     }
//
//     vTaskDelay(pdMS_TO_TICKS(TIME_SYNC_CHECK_INTERVAL_MS));
//   }
//
//   if (isTimeSync) {
//     time_t now = time(NULL);
//     struct tm timeinfo;
//     localtime_r(&now, &timeinfo);
//     char strftime_buf[64];
//     strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S",
//              &timeinfo);
//     ESP_LOGI(TAG, "Time synchronization successful: %s", strftime_buf);
//
//     // Set longer sync interval after successful initial sync
//     esp_sntp_set_sync_interval(604800000); // 7 days
//     rtc_update_from_system();
//
//     return ESP_OK;
//   }
//
//   return ESP_ERR_TIMEOUT;
// }

static void on_ppp_changed(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "PPP state changed event %" PRIu32, event_id);

  if (event_id == NETIF_PPP_ERRORUSER) {
    esp_netif_t **p_netif = event_data;
    ESP_LOGD(TAG, "User interrupted event from netif:%p", *p_netif);
  } else if (event_id == NETIF_PPP_ERRORNONE) {
    ESP_LOGD(TAG, "PPP connection established successfully");
  } else if (event_id == NETIF_PPP_ERRORPARAM) {
    ESP_LOGW(TAG, "PPP parameter error");
  } else if (event_id == NETIF_PPP_ERROROPEN) {
    ESP_LOGW(TAG, "PPP open error");
  } else if (event_id == NETIF_PPP_ERRORDEVICE) {
    ESP_LOGW(TAG, "PPP device error");
  } else if (event_id == NETIF_PPP_ERRORALLOC) {
    ESP_LOGW(TAG, "PPP allocation error");
  } else if (event_id == NETIF_PPP_ERRORCONNECT) {
    ESP_LOGW(TAG, "PPP connection error");
  } else if (event_id == NETIF_PPP_ERRORAUTHFAIL) {
    ESP_LOGW(TAG, "PPP authentication error");
  } else if (event_id == NETIF_PPP_ERRORPROTOCOL) {
    ESP_LOGW(TAG, "PPP protocol error");
  } else {
    ESP_LOGW(TAG, "Unknown PPP event: %" PRIu32, event_id);
  }
}

static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "IP event received: %" PRIu32, event_id);

  if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (event_id == IP_EVENT_PPP_GOT_IP) {
      esp_netif_dns_info_t dns_info;
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      esp_netif_t *netif = event->esp_netif;

      ESP_LOGD(TAG, "╔════════════════════════════════════════╗");
      ESP_LOGD(TAG, "║        PPP Connection Established     ║");
      ESP_LOGD(TAG, "╠════════════════════════════════════════╣");
      ESP_LOGI(TAG, "║ IP Address : " IPSTR "              ║",
               IP2STR(&event->ip_info.ip));
      ESP_LOGD(TAG, "║ Netmask    : " IPSTR "              ║",
               IP2STR(&event->ip_info.netmask));
      ESP_LOGD(TAG, "║ Gateway    : " IPSTR "              ║",
               IP2STR(&event->ip_info.gw));

      if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info) ==
          ESP_OK) {
        ESP_LOGD(TAG, "║ DNS Main   : " IPSTR "              ║",
                 IP2STR(&dns_info.ip.u_addr.ip4));
      }
      if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_info) ==
          ESP_OK) {
        ESP_LOGD(TAG, "║ DNS Backup : " IPSTR "              ║",
                 IP2STR(&dns_info.ip.u_addr.ip4));
      }
      ESP_LOGD(TAG, "╚════════════════════════════════════════╝");

      isModemConnectedToPPP = true;

    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
      ESP_LOGW(TAG, "PPP connection lost - IP address removed");
      isModemConnectedToPPP = false;
      isTimeSync = false;

    } else if (event_id == IP_EVENT_GOT_IP6) {
      ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
      ESP_LOGD(TAG, "Got IPv6 address: " IPV6STR, IPV62STR(event->ip6_info.ip));
    }
    xSemaphoreGive(connection_mutex);
  }
}

static esp_err_t validate_modem_connection(void) {
  if (!dce) {
    ESP_LOGE(TAG, "DCE handle is NULL");
    return ESP_ERR_INVALID_STATE;
  }

  // Check modem mode
  esp_modem_dce_mode_t mode = esp_modem_get_mode(dce);
  ESP_LOGD(TAG, "Current modem mode: %d", mode);

  // You can add additional modem validation here
  // Such as signal strength check, network registration, etc.

  return ESP_OK;
}

static esp_err_t cleanup_pppos_resources(void) {
  ESP_LOGD(TAG, "Cleaning up PPPOS resources");

  // Stop SNTP
  if (esp_sntp_enabled()) {
    esp_sntp_stop();
  }

  // Destroy DCE
  if (dce) {
    esp_modem_destroy(dce);
    dce = NULL;
  }

  // Destroy network interface
  if (esp_netif_ppp) {
    esp_netif_destroy(esp_netif_ppp);
    esp_netif_ppp = NULL;
  }

  // Unregister event handlers
  esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event);
  esp_event_handler_unregister(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID,
                               &on_ppp_changed);

  // Clean up mutex
  if (connection_mutex) {
    vSemaphoreDelete(connection_mutex);
    connection_mutex = NULL;
  }

  // Reset state variables
  isModemConnectedToPPP = false;
  isTimeSync = false;

  ESP_LOGI(TAG, "PPPOS resources cleaned up");
  return ESP_OK;
}

esp_err_t iPPPOS_Init(void) {
  esp_err_t ret = ESP_OK;
  int retry_count = 0;

  ESP_LOGD(TAG, "╔════════════════════════════════════════╗");
  ESP_LOGD(TAG, "║          Initializing PPPOS           ║");
  ESP_LOGD(TAG, "╚════════════════════════════════════════╝");

  // Create mutex for connection state protection
  connection_mutex = xSemaphoreCreateMutex();
  if (!connection_mutex) {
    ESP_LOGE(TAG, "Failed to create connection mutex");
    return ESP_ERR_NO_MEM;
  }

  // Reset state variables
  isModemConnectedToPPP = false;
  isTimeSync = false;

  // Initialize network interface and event loop
  ret = esp_netif_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
    cleanup_pppos_resources();
    return ret;
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
    cleanup_pppos_resources();
    return ret;
  }

  // Register event handlers
  ERROR_CHECK_RETURN(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                                &on_ip_event, NULL));
  ERROR_CHECK_RETURN(esp_event_handler_register(
      NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

  // Retry loop for modem initialization
  for (retry_count = 0; retry_count < MODEM_INIT_RETRY_COUNT; retry_count++) {
    ESP_LOGD(TAG, "Modem initialization attempt %d/%d", retry_count + 1,
             MODEM_INIT_RETRY_COUNT);

    // Configure DCE
    esp_modem_dce_config_t dce_config =
        ESP_MODEM_DCE_DEFAULT_CONFIG(MODEM_PPP_APN);

    // Configure network interface
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_ppp = esp_netif_new(&netif_ppp_config);
    if (!esp_netif_ppp) {
      ESP_LOGE(TAG, "Failed to create PPP netif");
      ret = ESP_ERR_NO_MEM;
      if (retry_count == MODEM_INIT_RETRY_COUNT - 1) {
        cleanup_pppos_resources();
        return ret;
      }
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    // Configure DTE (UART)
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.tx_io_num = MODEM_UART_TX_PIN;
    dte_config.uart_config.rx_io_num = MODEM_UART_RX_PIN;
    dte_config.uart_config.rts_io_num = MODEM_UART_RTS_PIN;
    dte_config.uart_config.cts_io_num = MODEM_UART_CTS_PIN;
    dte_config.uart_config.port_num = MODEM_UART_NUM;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_config.uart_config.rx_buffer_size = MODEM_UART_RX_BUFFER_SIZE;
    dte_config.uart_config.tx_buffer_size = MODEM_UART_TX_BUFFER_SIZE;
    dte_config.uart_config.event_queue_size = MODEM_UART_EVENT_QUEUE_SIZE;
    dte_config.task_stack_size = MODEM_UART_EVENT_TASK_STACK_SIZE;
    dte_config.task_priority = MODEM_UART_EVENT_TASK_PRIORITY;
    dte_config.dte_buffer_size = MODEM_UART_RX_BUFFER_SIZE / 2;

    ESP_LOGD(TAG, "Creating modem DCE for SIM800 module...");
    dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config,
                            esp_netif_ppp);
    if (!dce) {
      ESP_LOGE(TAG, "Failed to create modem DCE");
      ret = ESP_ERR_INVALID_STATE;
      if (retry_count == MODEM_INIT_RETRY_COUNT - 1) {
        cleanup_pppos_resources();
        return ret;
      }
      esp_netif_destroy(esp_netif_ppp);
      esp_netif_ppp = NULL;
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    // Set modem mode
    ret = esp_modem_set_mode(dce, ESP_MODEM_MODE_DETECT);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to detect modem mode: %s", esp_err_to_name(ret));
    }

    esp_modem_dce_mode_t mode = esp_modem_get_mode(dce);
    ESP_LOGD(TAG, "Detected modem mode: %d", mode);

    if (mode == ESP_MODEM_MODE_COMMAND) {
      ESP_LOGD(TAG, "Switching modem to data mode...");
      ret = esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set data mode: %s", esp_err_to_name(ret));
        if (retry_count == MODEM_INIT_RETRY_COUNT - 1) {
          cleanup_pppos_resources();
          return ret;
        }
        esp_modem_destroy(dce);
        dce = NULL;
        esp_netif_destroy(esp_netif_ppp);
        esp_netif_ppp = NULL;
        vTaskDelay(pdMS_TO_TICKS(3000));
        continue;
      }
    }

    // Validate modem connection
    ret = validate_modem_connection();
    if (ret == ESP_OK) {
      ESP_LOGD(TAG, "Modem initialization successful");
      break;
    } else {
      ESP_LOGW(TAG, "Modem validation failed: %s", esp_err_to_name(ret));
      if (retry_count == MODEM_INIT_RETRY_COUNT - 1) {
        cleanup_pppos_resources();
        return ret;
      }
      esp_modem_destroy(dce);
      dce = NULL;
      esp_netif_destroy(esp_netif_ppp);
      esp_netif_ppp = NULL;
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
  }

  // Wait for PPP connection with timeout
  ESP_LOGI(TAG, "Waiting for PPP connection (timeout: %d seconds)...",
           PPP_CONNECTION_TIMEOUT_MS / 1000);

  TickType_t connection_start = xTaskGetTickCount();
  TickType_t connection_timeout = pdMS_TO_TICKS(PPP_CONNECTION_TIMEOUT_MS);

  while (!isModemConnectedToPPP) {
    if ((xTaskGetTickCount() - connection_start) > connection_timeout) {
      ESP_LOGE(TAG, "PPP connection timeout after %d ms",
               PPP_CONNECTION_TIMEOUT_MS);
      cleanup_pppos_resources();
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ESP_LOGI(TAG, "PPP connection established successfully");
  // Initialize SNTP and wait for time synchronization
  if (CONFIG_SET_TIME) {
    ret = initialize_sntp_enhanced();
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "SNTP initialization failed: %s", esp_err_to_name(ret));
      // Don't fail the whole initialization for SNTP failure
    } else {
      // Try time sync with increased timeout for cellular connections
      ret = wait_for_time_sync(120000); // 2 minutes timeout
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Time synchronization failed: %s", esp_err_to_name(ret));

        // Try alternative approach - restart SNTP with different servers
        ESP_LOGD(TAG, "Attempting SNTP restart with global servers...");

        esp_sntp_stop();
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Reinitialize with different server configuration
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "time.google.com"); // Google's reliable time
        esp_sntp_setservername(1, "time.cloudflare.com"); // Cloudflare time
        esp_sntp_setservername(2, "pool.ntp.org");        // Global pool
        esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);     // Try immediate mode
        esp_sntp_set_sync_interval(10000);                // 10 second intervals
        esp_sntp_init();

        // Try one more time with shorter timeout
        ESP_LOGI(TAG, "Retry with global servers...");
        wait_for_time_sync(90000); // 1.5 minutes

        // Continue anyway - time sync is not critical for basic operation
      }
    }
  } else {
    ESP_LOGI(TAG, "SNTP time synchronization disabled by configuration");

    // Still initialize SNTP for weekly updates but don't wait for initial sync
    ret = initialize_sntp_enhanced();
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "SNTP initialized for weekly updates only");
    }
  }

  // Initialize MQTT
  ESP_LOGI(TAG, "Initializing MQTT client...");
  ret = iMQTT_Init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "MQTT initialization failed: %s", esp_err_to_name(ret));
    // You may choose to continue without MQTT or fail here
    // cleanup_pppos_resources();
    // return ret;
  } else {
    ESP_LOGI(TAG, "MQTT client initialized successfully");
  }

  ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║       PPPOS Initialization Complete   ║");
  ESP_LOGI(TAG, "║  PPP: %s  Time: %s  MQTT: %s    ║",
           isModemConnectedToPPP ? "✓" : "✗", isTimeSync ? "✓" : "✗",
           ret == ESP_OK ? "✓" : "✗");
  ESP_LOGI(TAG, "╚════════════════════════════════════════╝");

  return ESP_OK;
}

// Additional utility functions

bool isPPPConnected(void) {
  if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    bool connected = isModemConnectedToPPP;
    xSemaphoreGive(connection_mutex);
    return connected;
  }
  return false;
}

bool isTimeSynchronized(void) { return isTimeSync; }

esp_err_t iPPPOS_Disconnect(void) {
  ESP_LOGI(TAG, "Disconnecting PPPOS...");

  if (dce) {
    esp_err_t ret = esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to switch to command mode: %s",
               esp_err_to_name(ret));
    }
  }

  return cleanup_pppos_resources();
}

const char *getPPPStatus(void) {
  if (isPPPConnected()) {
    return isTimeSynchronized() ? "Connected & Synced" : "Connected";
  }
  return "Disconnected";
}
