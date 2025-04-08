#ifndef MAIN_WIFI_APP_H_
#define MAIN_WIFI_APP_H_

#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "freertos/FreeRTOS.h"
#include "mdns.h"


extern bool http_server_active;

typedef enum wifi_app_message {
    WIFI_APP_MSG_START_HTTP_SERVER = 0,
    WIFI_APP_MSG_STOP_HTTP_SERVER,
    WIFI_APP_MSG_STA_CONNECTED,
    WIFI_APP_MSG_STA_DISCONNECTED,
    WIFI_APP_MSG_START_MQTT,
    WIFI_APP_MSG_STOP_MQTT,
    WIFI_APP_MSG_START_STA,
    WIFI_APP_MSG_STOP_STA,
  } wifi_app_message_e;


  /**
 * Structure for the message queue
 * @note Expand this based on application requirements e.g. add another type and
 * parameter as required
 */
typedef struct wifi_app_queue_message {
    wifi_app_message_e msgID;
  } wifi_app_queue_message_t;


// Callback typedef
typedef void (*wifi_connected_event_callback_t)(void);

#define CONFIG_SERVER_TIMEOUT_M 2  // Timeout in minutes
#define SERVER_TIMEOUT_S (CONFIG_SERVER_TIMEOUT_M * 60)

// WiFi application settings
#define WIFI_AP_SSID                "myssid"     // AP name
#define WIFI_AP_PASSWORD            "mypassword"  // AP password
#define WIFI_AP_CHANNEL             11                   // Changed to 6 for better coverage
#define WIFI_AP_SSID_HIDDEN         0                    // AP visibility
#define WIFI_AP_MAX_CONNECTIONS     5                    // AP max clients
#define WIFI_AP_BEACON_INTERVAL     100                  // Increased to 300ms for power saving
#define WIFI_AP_IP                  "192.168.0.1"        // AP default IP
#define WIFI_AP_GATEWAY             "192.168.0.1"        // AP default Gateway (should be the same as the IP)
#define WIFI_AP_NETMASK             "255.255.255.0"      // AP netmask
#define WIFI_AP_BANDWIDTH           WIFI_BW_HT40         // AP bandwidth 20 MHz (40 MHz is the other option)
#define WIFI_STA_POWER_SAVE         WIFI_PS_NONE         // Changed to min modem power save mode
#define MAX_SSID_LENGTH             32                   // IEEE standard maximum
#define MAX_PASSWORD_LENGTH         64                   // IEEE standard maximum
#define MAX_CONNECTION_RETRIES      5                    // Reduced to 3 retries to save power
#define MAX_RETRY_ATTEMPTS 5
#define WIFI_RECONNECT_INTERVAL_MS (5 * 60 * 1000)  // 10 minutes in milliseconds
#define INITIAL_RECONNECT_INTERVAL_MS (5 * 1000)     // 5 seconds for initial retries

// netif object for the Station and Access Point
extern esp_netif_t* esp_netif_sta;
extern esp_netif_t* esp_netif_ap;


/**
 * Sends a message to the queue
 * @param msgID message ID from the wifi_app_message_e enum.
 * @return pdTRUE if an item was successfully sent to the queue, otherwise pdFALSE.
 * @note Expand the parameter list based on your requirements e.g. how you've expanded the wifi_app_queue_message_t.
 */
BaseType_t wifi_app_send_message(wifi_app_message_e msgID);

/**
 * Starts the WiFi RTOS task
 */
void wifi_app_start(void);

/**
 * Gets the wifi configuration
 */
wifi_config_t* wifi_app_get_wifi_config(void);

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
void wifi_app_shutdown_callback(void* arg);
void wifi_app_force_shutdown_callback(void* arg);
void wifi_app_espnow_wifi_init(void);
esp_err_t init_mdns_service(void);

#define MAX_NETWORKS 5

typedef struct {
    char ssid[32];
    char password[64];
} wifi_network_t;

static const wifi_network_t known_networks[MAX_NETWORKS] = {
    {"myssid", "mypassword"},
    //{CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD},  // Primary network from menuconfig
    {"AOI", "123456!!"},
    {"Surya", "Icestupa"},
    {"AOI_Guests", "Acresofice@2024"},
};

#endif /* MAIN_WIFI_APP_H_ */

