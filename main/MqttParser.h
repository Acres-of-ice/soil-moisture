#ifndef MQTT_PARSER_H
#define MQTT_PARSER_H

#include "cJSON.h"
#include "define.h"

extern char pcOtaUrl[1000];

esp_err_t iMqtt_OtaParser(char *json_string);

#endif // MQTT_PARSER_H
