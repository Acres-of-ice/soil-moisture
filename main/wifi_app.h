#ifndef MAIN_WIFI_APP_H_
#define MAIN_WIFI_APP_H_

#include "define.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "freertos/FreeRTOS.h"
#include "mdns.h"

// Callback typedef
typedef void (*wifi_connected_event_callback_t)(void);

// WiFi application settings
#define WIFI_AP_SSID CONFIG_SITE_NAME              // AP name
#define WIFI_AP_PASSWORD (CONFIG_SITE_NAME "1234") // AP password
#define WIFI_AP_CHANNEL 11          // Changed to 6 for better coverage
#define WIFI_AP_SSID_HIDDEN 0       // AP visibility
#define WIFI_AP_MAX_CONNECTIONS 5   // AP max clients
#define WIFI_AP_BEACON_INTERVAL 100 // Increased to 300ms for power saving
#define WIFI_AP_IP "192.168.0.1"    // AP default IP
#define WIFI_AP_GATEWAY                                                        \
  "192.168.0.1" // AP default Gateway (should be the same as the IP)
#define WIFI_AP_NETMASK "255.255.255.0" // AP netmask
#define WIFI_AP_BANDWIDTH                                                      \
  WIFI_BW_HT40 // AP bandwidth 20 MHz (40 MHz is the other option)
#define WIFI_STA_POWER_SAVE WIFI_PS_NONE // Changed to min modem power save mode
#define MAX_SSID_LENGTH 32               // IEEE standard maximum
#define MAX_PASSWORD_LENGTH 64           // IEEE standard maximum
#define MAX_CONNECTION_RETRIES 5         // Reduced to 3 retries to save power
#define MAX_RETRY_ATTEMPTS 5
#define WIFI_RECONNECT_INTERVAL_MS (5 * 60 * 1000) // 10 minutes in milliseconds
#define INITIAL_RECONNECT_INTERVAL_MS                                          \
  (5 * 1000) // 5 seconds for initial retries

// netif object for the Station and Access Point
extern esp_netif_t *esp_netif_sta;
extern esp_netif_t *esp_netif_ap;

/**
 * Sends a message to the queue
 * @param msgID message ID from the wifi_app_message_e enum.
 * @return pdTRUE if an item was successfully sent to the queue, otherwise
 * pdFALSE.
 * @note Expand the parameter list based on your requirements e.g. how you've
 * expanded the wifi_app_queue_message_t.
 */
BaseType_t wifi_app_send_message(wifi_app_message_e msgID);

/**
 * Starts the WiFi RTOS task
 */
void wifi_app_start(void);
/**
 * Initialize network interfaces assuming ESP-NOW has already initialized WiFi
 */
void wifi_app_espnow_wifi_init(void);

/**
 * Gets the wifi configuration
 */
wifi_config_t *wifi_app_get_wifi_config(void);

/**
 * Sets the callback function.
 */
void wifi_app_set_callback(wifi_connected_event_callback_t cb);

/**
 * Calls the callback function.
 */
void wifi_app_call_callback(void);

/**
 * Gets the RSSI value of the Wifi connection.
 * @return current RSSI level.
 */
int8_t wifi_app_get_rssi(void);

void wifi_app_task(void *pvParameters);
void wifi_init(void);
void start_shutdown_timer(int shutdown_delay_seconds);
void wifi_app_shutdown_callback(void *arg);
void wifi_app_force_shutdown_callback(void *arg);
esp_err_t init_mdns_service(void);

#define MAX_NETWORKS 5

typedef struct {
  char ssid[32];
  char password[64];
} wifi_network_t;

static const wifi_network_t known_networks[MAX_NETWORKS] = {
    {CONFIG_SITE_NAME, (CONFIG_SITE_NAME "1234")},
    {CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD}, // Primary network from menuconfig
    {"AOI", "123456!!"},
    {"Surya12", "Icestupa"},
    {"AOI_Guests", "Acresofice@2024"},
};

#endif /* MAIN_WIFI_APP_H_ */
