#ifndef HEX_DATA_H
#define HEX_DATA_H

#include <stdint.h>
#include "define.h"

bool is_data_available(void);
bool is_hex_available(void);
void encode_data(hex_data_t *data, uint8_t *buffer);
void binary_to_hex(const uint8_t *input, char *output, size_t len);
void hex_data_task(void *pvParameters);
esp_err_t decode_hex_data(const char *hex_string, hex_data_t *data);
char* decode_hex_to_json(const char *hex_string);

// New functions for circular buffer operations
esp_err_t init_payload_buffer(void);
esp_err_t add_payload_to_buffer(const char* payload);
char* get_payload_from_buffer(void);
esp_err_t init_hex_buffer(void);
esp_err_t add_hex_to_buffer(const char* hex);
char* get_hex_from_buffer(void);

#endif // HEX_DATA_H
