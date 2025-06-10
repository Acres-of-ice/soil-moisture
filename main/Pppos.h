#ifndef PPPOS_H
#define PPPOS_H

#include "define.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mqtt_client.h"
#include "netdb.h"
#include "sys/socket.h"
#include <sys/time.h>
#include <time.h>

// Main initialization function
esp_err_t iPPPOS_Init(void);

// Utility functions
bool isPPPConnected(void);
bool isTimeSynchronized(void);
esp_err_t iPPPOS_Disconnect(void);
const char *getPPPStatus(void);

#endif // PPPOS_H
