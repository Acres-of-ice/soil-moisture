#ifndef ESPNOW_COM_H
#define ESPNOW_COM_H


#include "esp_now.h"



// Initialize ESP-NOW communication
esp_err_t espnow_init();

// Forward declarations for callback functions
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
// Function for checking signal quality (RSSI)
//void check_signal_quality(const esp_now_recv_info_t *recv_info);

// Message serialization and deserialization
//void serialize_message(const comm_t *message, uint8_t *buffer);
//bool deserialize_message(const uint8_t *buffer, size_t size, comm_t *message);

// Command sending with retries and acknowledgment
//bool sendCommandWithRetry(uint8_t deviceAddress, uint8_t command,
  //                        uint8_t source);
//bool sendCommandNoAck(uint8_t deviceAddress, uint8_t command, uint8_t source);

// Queue management
void clearMessageQueue();

// Message processing functions
// void processReceivedMessage(comm_t *message);
// void processConductorMessage(comm_t *message);
// void processDrainNoteMessage(comm_t *message, ValveState newState);
// void processSourceNoteMessage(comm_t *message, ValveState newState);
// void processValveMessage(comm_t *message);
// void processGSMMessage(comm_t *message);

// Task and queue functions
void vTaskESPNOW_TX(void *pvParameters);
void vTaskESPNOW_RX(void *pvParameters);
void ESPNOW_queueMessage(uint8_t address, uint8_t command, uint8_t source,
                         uint8_t retries);
//void reset_acknowledgements();
bool ESPNOW_isQueueEmpty();
// bool receivedConductorMessageRecently();

// Peer management
bool add_espnow_peer(const uint8_t *mac_addr, uint8_t channel);
bool remove_espnow_peer(const uint8_t *mac_addr);

// MAC address helpers
void get_mac_for_device(uint8_t deviceAddress, uint8_t *mac_addr);
uint8_t get_device_from_mac(const uint8_t *mac_addr);
bool register_device_mac_mapping(uint8_t device_addr, const uint8_t *mac_addr);
void on_data_received(const uint8_t *mac_addr, const uint8_t *data,int data_len, int rssi);
void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void wifi_init(void);

#endif // ESPNOW_COM_H