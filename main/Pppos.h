#ifndef PPPOS_H
#define PPPOS_H

#include "define.h"
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_sntp.h"
#include "mqtt_client.h"

esp_err_t iPPPOS_Init(void);

#endif // PPPOS_H
