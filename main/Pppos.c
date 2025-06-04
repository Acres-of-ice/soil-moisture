#include "Pppos.h"
#include "MqttParser.h"

#define TAG "[PPPOS]"

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

#define MQTT_TEST_TOPIC "AutoAir/Msg"
// #define MQTT_TEST_DATA "{\"msgId\": 9, \"url\":
// \"http://pcb-bins.s3.us-east-1.amazonaws.com/Stakmo_CONDUCTOR.bin\"}"
#define MQTT_TEST_DATA "HI, MQTT, TEST, DATA"

#define ERROR_CHECK_RETURN(err)                                                \
  ({                                                                           \
    esp_err_t __err_rc = (err);                                                \
    if (__err_rc != ESP_OK) {                                                  \
      ESP_LOGW("ERR", "%s, File: %s, Function: %s, Line: %d",                  \
               esp_err_to_name(__err_rc), __FILE__, __FUNCTION__, __LINE__);   \
      return __err_rc;                                                         \
    }                                                                          \
    __err_rc;                                                                  \
  })

esp_modem_dce_t *dce = NULL;

bool isModemConnectedToPPP = false;
bool isMqttConnected = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);

esp_err_t iMQTT_Init(void);

static void initialize_sntp(void) {
  ESP_LOGI(TAG, "Initializing SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
}

static void on_ppp_changed(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data) {
  ESP_LOGI(TAG, "PPP state changed event %" PRIu32, event_id);
  if (event_id == NETIF_PPP_ERRORUSER) {
    /* User interrupted event from esp-netif */
    esp_netif_t **p_netif = event_data;
    ESP_LOGI(TAG, "User interrupted event from netif:%p", *p_netif);
  }
}

static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "IP event! %" PRIu32, event_id);
  if (event_id == IP_EVENT_PPP_GOT_IP) {
    esp_netif_dns_info_t dns_info;

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    esp_netif_t *netif = event->esp_netif;

    ESP_LOGI(TAG, "Modem Connect to PPP Server");
    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
    ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
    esp_netif_get_dns_info(netif, 0, &dns_info);
    ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
    esp_netif_get_dns_info(netif, 1, &dns_info);
    ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
    ESP_LOGI(TAG, "GOT ip event!!!");
    // iEVENT_SetEventBit(EVENT_MODEM_CONNECTED_TO_PPP);// Sync Time via SNTP
    isModemConnectedToPPP = true;
  } else if (event_id == IP_EVENT_PPP_LOST_IP) {
    ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
    // iEVENT_ClearEventBit(EVENT_MODEM_CONNECTED_TO_PPP);
    isModemConnectedToPPP = false;
  } else if (event_id == IP_EVENT_GOT_IP6) {
    ESP_LOGI(TAG, "GOT IPv6 event!");

    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    ESP_LOGI(TAG, "Got IPv6 address " IPV6STR, IPV62STR(event->ip6_info.ip));
  }
}

esp_err_t iPPPOS_Init(void) {
  /* Init and register system/core components */
  esp_netif_init();
  esp_event_loop_create_default();
  ERROR_CHECK_RETURN(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                                &on_ip_event, NULL));
  ERROR_CHECK_RETURN(esp_event_handler_register(
      NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

  esp_modem_dce_config_t dce_config =
      ESP_MODEM_DCE_DEFAULT_CONFIG(MODEM_PPP_APN);
  esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
  esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
  // ERROR_CHECK_RETURN(CHECK_NULL(esp_netif));
  if (NULL == esp_netif) {
    ERROR_CHECK_RETURN(ESP_ERR_NOT_FOUND);
  }

  /* Configure the DTE */
  esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
  /* setup UART specific configuration based on kconfig options */
  /* TODO: Config for SIM800*/
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

  ESP_LOGI(TAG, "Initializing esp_modem for the SIM800 module...");
  dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config,
                          esp_netif);
  // ERROR_CHECK_RETURN(CHECK_NULL(dce));
  if (NULL == dce) {
    ERROR_CHECK_RETURN(ESP_ERR_NOT_FOUND);
  }

  ERROR_CHECK_RETURN(esp_modem_set_mode(dce, ESP_MODEM_MODE_DETECT));
  esp_modem_dce_mode_t mode = esp_modem_get_mode(dce);
  if (mode == ESP_MODEM_MODE_COMMAND) {
    ERROR_CHECK_RETURN(esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA));
  }

  ESP_LOGI(TAG, "Waiting for PPP connection...");
  while (!isModemConnectedToPPP) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  // time_t now = 0;
  // ESP_LOGI(TAG, "Current time: %lld", time(&now));

  ERROR_CHECK_RETURN(iMQTT_Init());

  return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIu32,
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    msg_id = esp_mqtt_client_subscribe(client, MQTT_TEST_TOPIC, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    // iEVENT_SetEventBit(EVENT_MQTT_CONNECTED);
    isMqttConnected = true;
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    // iEVENT_ClearEventBit(EVENT_MQTT_CONNECTED);
    isMqttConnected = false;
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    msg_id = esp_mqtt_client_publish(client, MQTT_TEST_TOPIC, MQTT_TEST_DATA, 0,
                                     0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    iMqtt_OtaParser(event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "MQTT other event id: %d", event->event_id);
    break;
  }
}

esp_err_t iMQTT_Init(void) {
  ESP_LOGI(TAG, "MQTT Init");
  esp_mqtt_client_config_t mqtt_config = {
      .broker = {
          .address.uri = "mqtt://aoi:4201@44.194.157.172:1883",
      }};
  esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_config);
  ERROR_CHECK_RETURN(esp_mqtt_client_register_event(
      mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
  ERROR_CHECK_RETURN(esp_mqtt_client_start(mqtt_client));

  return ESP_OK;
}
