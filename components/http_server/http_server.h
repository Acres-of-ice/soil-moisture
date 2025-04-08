#ifndef MAIN_HTTP_SERVER_H_
#define MAIN_HTTP_SERVER_H_

#define FILE_PATH_MAX 256
#define MAX_SEND_RETRIES 3
#define SEND_RETRY_DELAY_MS 100
#define MAX_CONCURRENT_CLIENTS    2    // Strict limit of 2 devices
#define HTTP_KEEP_ALIVE_IDLE     3    // Reduced from 5 seconds
#define HTTP_KEEP_ALIVE_INTERVAL 3    // Reduced from 5 seconds
#define HTTP_KEEP_ALIVE_COUNT    2    // Reduced from 3 retries
#define HTTP_SERVER_TIMEOUT      30   // Reduced timeout
#define OPTIMAL_CHUNK_SIZE     (1024*0.5)   // 1KB chunks for better memory usage
#define MIN_HEAP_SIZE_HTTP (12 * 1024)  // Minimum 30KB heap required


// Network and Data Tasks - Core 1
#define HTTP_SERVER_TASK_STACK_SIZE (1024 * 8)
#define HTTP_SERVER_TASK_PRIORITY 11
#define HTTP_SERVER_TASK_CORE_ID 1

/**
 * Starts the HTTP server.
 */
void http_server_start(void);

/**
 * Stops the HTTP server.
 */
void http_server_stop(void);

#define OTA_UPDATE_PENDING 		0
#define OTA_UPDATE_SUCCESSFUL	1
#define OTA_UPDATE_FAILED		-1

/**
 * Messages for the HTTP monitor
 */
typedef enum http_server_message
{
	HTTP_MSG_WIFI_CONNECT_INIT = 0,
	HTTP_MSG_WIFI_CONNECT_SUCCESS,
	HTTP_MSG_WIFI_CONNECT_FAIL,
	HTTP_MSG_WIFI_USER_DISCONNECT,
	HTTP_MSG_OTA_UPDATE_SUCCESSFUL,
	HTTP_MSG_OTA_UPDATE_FAILED,
	HTTP_MSG_TIME_SERVICE_INITIALIZED,
} http_server_message_e;

/**
 * Structure for the message queue
 */
typedef struct http_server_queue_message
{
	http_server_message_e msgID;
} http_server_queue_message_t;

/**
 * Sends a message to the queue
 * @param msgID message ID from the http_server_message_e enum.
 * @return pdTRUE if an item was successfully sent to the queue, otherwise pdFALSE.
 * @note Expand the parameter list based on your requirements e.g. how you've expanded the http_server_queue_message_t.
 */
BaseType_t http_server_monitor_send_message(http_server_message_e msgID);

/**
 * Timer callback function which calls esp_restart upon successful firmware update.
 */
void http_server_fw_update_reset_callback(void *arg);
void http_server_fw_update_reset_timer(void);

void http_server_monitor(void *parameter);

//void suspend_tasks(void);
void resume_tasks(void);

#endif /* MAIN_HTTP_SERVER_H_ */
