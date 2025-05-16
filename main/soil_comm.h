#ifndef SOIL_COMM_H
#define SOIL_COMM_H

#include "define.h"
#include "esp_now.h"
#include "valve_control.h"

#define SENSOR_DATA_QUEUE_SIZE 10
#define ESPNOW_MAX_PAYLOAD_SIZE 250

// Initialize ESP-NOW communication
esp_err_t espnow_init();

// Forward declarations for callback functions
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len);

// Command sending with retries and acknowledgment
bool sendCommandWithRetry(uint8_t deviceAddress, uint8_t command,
                          uint8_t source);
// Queue management
void clearMessageQueue();

// Message processing functions
void processReceivedMessage(comm_t *message);
void serialize_message(const comm_t *message, uint8_t *buffer);
bool deserialize_message(const uint8_t *buffer, size_t size, comm_t *message);
void processValveMessage(comm_t *message);
void processSprayMessage(comm_t *message);
esp_err_t init_hex_buffer(void);

esp_err_t register_loaded_mappings_with_espnow(void);
void espnow_discovery_task(void *pvParameters);
bool nvs_has_valid_mappings(void);
bool verify_device_mappings(void);
esp_err_t load_device_mappings_from_nvs(void);
esp_err_t save_device_mappings_to_nvs(void);

// Task and queue functions
void vTaskESPNOW_RX(void *pvParameters);
void ESPNOW_queueMessage(uint8_t address, uint8_t command, uint8_t source,
                         uint8_t retries);
void reset_acknowledgements();
bool ESPNOW_isQueueEmpty();
#define COMM_isQueueEmpty ESPNOW_isQueueEmpty

// Peer management
bool add_espnow_peer(const uint8_t *mac_addr, uint8_t channel);
bool remove_espnow_peer(const uint8_t *mac_addr);
bool register_device_mac_mapping(uint8_t device_addr, const uint8_t *mac_addr);
bool is_peer_authenticated(uint8_t device_addr);

// MAC address helpers
void get_mac_for_device(uint8_t deviceAddress, uint8_t *mac_addr);
uint8_t get_device_from_mac(const uint8_t *mac_addr);
bool register_device_mac_mapping(uint8_t device_addr, const uint8_t *mac_addr);
void on_data_received(const uint8_t *mac_addr, const uint8_t *data,
                      int data_len, int rssi);
void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void custom_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
void custom_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len,
                    int rssi);
void processMasterMessage(comm_t *message);
void processValveAMessage(comm_t *message, ValveState newState);
void processValveBMessage(comm_t *message, ValveState newState);
void processPumpMessage(comm_t *message);
void clearMessageQueue();

static void wifi_init_for_espnow(void);
const char *get_pcb_name(uint8_t nodeAddress);
uint8_t get_device_from_pcb_name(const char *pcb_name);
void update_device_mappings_from_discovered_peers(void);
esp_err_t espnow_init2(void);
void vTaskESPNOW(void *pvParameters);
void simulation_task(void *pvParameters);

#endif // ESPNOW_COM_H
