#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "sys/param.h"

#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "esp_spiffs.h"
#include "esp_vfs.h"

#include "http_server.h"
// #include "modbus.h"
// #include "adc_sensing.h"
// #include "water_temp_sensor.h"
#include "sensor.h"
#include "sdkconfig.h"
#include "wifi_app.h"
//#include "lcd.h"


// Tag used for ESP serial console messages
static const char TAG[] = "SERVER";

// HTTP server task handle
static httpd_handle_t http_server_handle = NULL;

SemaphoreHandle_t file_mutex = NULL;

extern QueueHandle_t wifi_app_queue_handle;
#define SPIFFS_MOUNT_POINT "/spiffs"
// Firmware update status
static int g_fw_update_status = OTA_UPDATE_PENDING;
 #define CONFIG_SITE_NAME "TEST"

 const char *DATA_FILE_HEADER =
    "moisture, Temp, Battery, Time";
                     

 size_t totalBytes = 0;
size_t usedBytes = 0;
char *log_path = SPIFFS_MOUNT_POINT "/log.csv";
char *data_path = SPIFFS_MOUNT_POINT "/data.csv";
//data_statistics_t data_stats = {0};

// HTTP server monitor task handle
// static TaskHandle_t task_http_server_monitor = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t http_server_monitor_queue_handle;

/**
 * ESP32 timer configuration passed to esp_timer_create.
 */
const esp_timer_create_args_t fw_update_reset_args = {
		.callback = &http_server_fw_update_reset_callback,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "fw_update_reset"
};
esp_timer_handle_t fw_update_reset;
static SemaphoreHandle_t ota_semaphore = NULL;
static bool ota_ready = false;
bool http_server_active = false;

static const char favicon[] = {
    // Simple 16x16 transparent favicon - minimal implementation
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x10, 0x00, 0x00, 0x01, 0x00,
    0x20, 0x00, 0x68, 0x04, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00
};

// static esp_err_t http_server_outputVal_handler(httpd_req_t *req)
// {
// 	ESP_LOGD(TAG, "outputVal requested");
//   // Get the output state as a string
// 	static char ValveJSON[64];
//   ValveState nowState = getCurrentState();  // Start with current state
  
//   // Check if the current state is related to SPRAY or DRAIN
//   switch (nowState) {
//         case STATE_SPRAY_START:
//         case STATE_SPRAY_FEEDBACK_DRAIN_NOTE: // Corrected this state
//         case STATE_SPRAY_WAIT_AIR_NOTE:
//         case STATE_SPRAY_WAIT_SOURCE_NOTE:
//         case STATE_SPRAY_CALIBRATION:
//         case STATE_SPRAY_DONE:
//             SPRAY_mode = pdTRUE;
//             DRAIN_mode = pdFALSE;
//             break;
//         case STATE_DRAIN_START:
//         case STATE_DRAIN_FEEDBACK_DRAIN_NOTE: // Added this state
//         case STATE_DRAIN_WAIT_SOURCE_NOTE:
//         case STATE_DRAIN_WAIT_AIR_NOTE:
//         case STATE_DRAIN_DONE:
//         case STATE_ERROR:
//             DRAIN_mode = pdTRUE;
//             SPRAY_mode = pdFALSE;
//             break;
//         case STATE_IDLE:
//             // SPRAY_mode = pdFALSE;
//             // DRAIN_mode = pdFALSE;
//             break;
//         default:
//             ESP_LOGW(TAG, "Unexpected valve state: %s", valveStateToString(nowState));
//             break;
//     }
  
//   sprintf(ValveJSON, "{\"SPRAY\":%s, \"DRAIN\":%s, \"AUTO\":%s, \"STATE\":\"%s\"}",
//           SPRAY_mode ? "true" : "false",
//           DRAIN_mode ? "true" : "false",
//           AUTO_mode ? "true" : "false",
//           valveStateToString(nowState));
  
//   // Send the response with the output state
// 	httpd_resp_set_type(req, "application/json");
//   httpd_resp_send(req, ValveJSON, strlen(ValveJSON));

// 	return ESP_OK;
// }

// static esp_err_t http_server_getInitialState_handler(httpd_req_t *req)
// {
//     ESP_LOGD(TAG, "getInitialState requested");
//     char resp[100];
//     snprintf(resp, sizeof(resp), 
//              "{\"AUTO_mode\": %s, \"SPRAY_mode\": %s, \"DRAIN_mode\": %s}", 
//              AUTO_mode ? "true" : "false",
//              SPRAY_mode ? "true" : "false",
//              DRAIN_mode ? "true" : "false");
    
//     ESP_LOGD(TAG, "Sending initial state: %s", resp);
    
//     httpd_resp_set_type(req, "application/json");
//     httpd_resp_send(req, resp, strlen(resp));
    
//     return ESP_OK;
// }

// static esp_err_t http_server_sensor_readings_handler(httpd_req_t *req)
// {
//     ESP_LOGD(TAG, "Sensor readings requested");
//     static sensor_readings_t http_readings;
//     get_sensor_readings(&http_readings);

//     static char resp[128];  // Increased buffer size
//     snprintf(resp, sizeof(resp), 
//              "{\"moisture\": %.1f, \"temp\": %.1f, \"battery\": %.1f}", 
//              http_readings.soil_moisture, http_readings.temperature, http_readings.battery_level);
    
//     ESP_LOGD(TAG, "Sending sensor readings: %s", resp);
    
//     httpd_resp_set_type(req, "application/json");
//     return httpd_resp_sendstr(req, resp);
// }

// static esp_err_t http_server_site_name_handler(httpd_req_t *req)
// {
//     ESP_LOGD(TAG, "Site name requested");

//     char resp[100];
//     snprintf(resp, sizeof(resp), "{\"siteName\": \"%s\"}", CONFIG_SITE_NAME);
    
//     ESP_LOGD(TAG, "Sending site name: %s", resp);
    
//     httpd_resp_set_type(req, "application/json");
//     httpd_resp_send(req, resp, strlen(resp));
    
//     return ESP_OK;
// }

// static esp_err_t http_server_download_handler(httpd_req_t *req)
// {
//     ESP_LOGI(TAG, "File download requested");
//     const char *filepath = "/spiffs/data.csv";
//     struct stat file_stat;

//     // Try to take mutex with timeout
//     if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
//         ESP_LOGW(TAG, "Failed to acquire file mutex for download");
//         //update_status_message("Download Failed");
//         return ESP_FAIL;
//     }
  
//     if (stat(filepath, &file_stat) == -1) {
//         ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
//         xSemaphoreGive(file_mutex);
//         return ESP_FAIL;
//     }

//     FILE *fd = fopen(filepath, "r");
//     if (!fd) {
//         ESP_LOGE(TAG, "Failed to open file for reading");
//         xSemaphoreGive(file_mutex);
//         return ESP_FAIL;
//     }
  
//     // // Prepare for download
//     // suspend_tasks();

//     ESP_LOGI(TAG, "Starting file transfer: %s (%ld bytes)", filepath, file_stat.st_size);
//     // Set response headers
//     httpd_resp_set_type(req, "text/csv");
//     // Set filename for download
//     char filename[64];
//     char* timeStr = fetchTime();
//     int year, month, day, hour, minute;
//     sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
//     snprintf(filename, sizeof(filename), "%s_data_%d-%d.csv", CONFIG_SITE_NAME, month, day);

//     static char content_disposition[128];
//     snprintf(content_disposition, sizeof(content_disposition), "attachment; filename=\"%s\"", filename);
//     httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);

//     // Send file in chunks
//     char *chunk = heap_caps_malloc(4096, MALLOC_CAP_DMA);
//     if (!chunk) {
//         ESP_LOGE(TAG, "Failed to allocate chunk memory");
//         fclose(fd);
//         xSemaphoreGive(file_mutex);
//         return ESP_FAIL;
//     }

//     size_t bytes_sent = 0;
//     size_t chunk_size;
//     esp_err_t ret = ESP_OK;

//     while ((chunk_size = fread(chunk, 1, 4096, fd)) > 0) {
//         if (httpd_resp_send_chunk(req, chunk, chunk_size) != ESP_OK) {
//             ESP_LOGE(TAG, "File send failed at %zu bytes", bytes_sent);
//             ret = ESP_FAIL;
//             break;
//         }
//         bytes_sent += chunk_size;
//         ESP_LOGD(TAG, "Sent %zu of %ld bytes", bytes_sent, file_stat.st_size);
//     }

//     // Cleanup
//     free(chunk);
//     fclose(fd);
//     xSemaphoreGive(file_mutex);

//     if (ret == ESP_OK) {
//         httpd_resp_send_chunk(req, NULL, 0);
//         ESP_LOGI(TAG, "File sent successfully: %zu bytes", bytes_sent);
//     }

//     return ret;
// }

// static esp_err_t http_server_logs_handler(httpd_req_t *req)
// {
//     ESP_LOGI(TAG, "Logs download requested");
//     const char *filepath = "/spiffs/log.csv";
//     struct stat file_stat;

//     // Try to take mutex with timeout
//     if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
//         ESP_LOGW(TAG, "Failed to acquire file mutex for download");
//         //update_status_message("Download Failed");
//         return ESP_FAIL;
//     }
  
//     if (stat(filepath, &file_stat) == -1) {
//         ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
//         xSemaphoreGive(file_mutex);
//         return ESP_FAIL;
//     }

//     FILE *fd = fopen(filepath, "r");
//     if (!fd) {
//         ESP_LOGE(TAG, "Failed to open file for reading");
//         xSemaphoreGive(file_mutex);
//         return ESP_FAIL;
//     }
  
//     // // Prepare for download
//     // suspend_tasks();

//     ESP_LOGI(TAG, "Starting file transfer: %s (%ld bytes)", filepath, file_stat.st_size);
//     // Set response headers
//     httpd_resp_set_type(req, "text/csv");
//     // Set filename for download
//     char filename[64];
//     char* timeStr = fetchTime();
//     int year, month, day, hour, minute;
//     sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
//     snprintf(filename, sizeof(filename), "%s_logs_%d-%d.csv", CONFIG_SITE_NAME, month, day);

//     char content_disposition[128];
//     snprintf(content_disposition, sizeof(content_disposition), "attachment; filename=\"%s\"", filename);
//     httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);

//     // Send file in chunks
//     char *chunk = heap_caps_malloc(4096, MALLOC_CAP_DMA);
//     if (!chunk) {
//         ESP_LOGE(TAG, "Failed to allocate chunk memory");
//         fclose(fd);
//         xSemaphoreGive(file_mutex);
//         return ESP_FAIL;
//     }

//     size_t bytes_sent = 0;
//     size_t chunk_size;
//     esp_err_t ret = ESP_OK;

//     while ((chunk_size = fread(chunk, 1, 4096, fd)) > 0) {
//         if (httpd_resp_send_chunk(req, chunk, chunk_size) != ESP_OK) {
//             ESP_LOGE(TAG, "File send failed at %zu bytes", bytes_sent);
//             ret = ESP_FAIL;
//             break;
//         }
//         bytes_sent += chunk_size;
//         ESP_LOGD(TAG, "Sent %zu of %ld bytes", bytes_sent, file_stat.st_size);
//     }

//     // Cleanup
//     free(chunk);
//     fclose(fd);
//     xSemaphoreGive(file_mutex);

//     if (ret == ESP_OK) {
//         httpd_resp_send_chunk(req, NULL, 0);
//         ESP_LOGI(TAG, "File sent successfully: %zu bytes", bytes_sent);
//     }

//     return ret;
// }

static esp_err_t send_chunk_with_retry(httpd_req_t *req, const char *chunk, size_t chunksize) {
    int retries = 0;
    esp_err_t err;

    while (retries < MAX_SEND_RETRIES) {
        err = httpd_resp_send_chunk(req, chunk, chunksize);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        
        ESP_LOGW(TAG, "Failed to send chunk, retry %d/%d", retries + 1, MAX_SEND_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(SEND_RETRY_DELAY_MS));
        retries++;
    }
    
    return ESP_FAIL;
}
static esp_err_t http_server_download_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "File download requested");
    const char *filepath = "/spiffs/data.csv";
    struct stat file_stat;
    if (file_mutex == NULL) {
        file_mutex = xSemaphoreCreateMutex();
        if (file_mutex == NULL) {
          ESP_LOGE(TAG, "Failed to create file mutex");
          return ESP_FAIL;
        }
      }
    // Try to take mutex with timeout
    if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire file mutex for download");
        ////update_status_message("Download Failed");
        return ESP_FAIL;
    }
  
    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        xSemaphoreGive(file_mutex);
        return ESP_FAIL;
    }

    FILE *fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        xSemaphoreGive(file_mutex);
        return ESP_FAIL;
    }
  
    // // Prepare for download
    // suspend_tasks();

    ESP_LOGI(TAG, "Starting file transfer: %s (%ld bytes)", filepath, file_stat.st_size);
    // Set response headers
    httpd_resp_set_type(req, "text/csv");
    // Set filename for download
    char filename[64];
    //char* timeStr = fetchTime();
    //int year , month, day, hour, minute;
    //sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
    snprintf(filename, sizeof(filename), "%s_data.csv", CONFIG_SITE_NAME);

    static char content_disposition[128];
    snprintf(content_disposition, sizeof(content_disposition), "attachment; filename=\"%s\"", filename);
    httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);

    // Send file in chunks
    char *chunk = heap_caps_malloc(4096, MALLOC_CAP_DMA);
    if (!chunk) {
        ESP_LOGE(TAG, "Failed to allocate chunk memory");
        fclose(fd);
        xSemaphoreGive(file_mutex);
        return ESP_FAIL;
    }

    size_t bytes_sent = 0;
    size_t chunk_size;
    esp_err_t ret = ESP_OK;

    while ((chunk_size = fread(chunk, 1, 4096, fd)) > 0) {
        if (httpd_resp_send_chunk(req, chunk, chunk_size) != ESP_OK) {
            ESP_LOGE(TAG, "File send failed at %zu bytes", bytes_sent);
            ret = ESP_FAIL;
            break;
        }
        bytes_sent += chunk_size;
        ESP_LOGD(TAG, "Sent %zu of %ld bytes", bytes_sent, file_stat.st_size);
    }

    // Cleanup
    free(chunk);
    fclose(fd);
    xSemaphoreGive(file_mutex);

    if (ret == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
        ESP_LOGI(TAG, "File sent successfully: %zu bytes", bytes_sent);
    }

    return ret;
}

static esp_err_t http_server_logs_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Logs download requested");
    const char *filepath = "/spiffs/log.csv";
    struct stat file_stat;

    // Try to take mutex with timeout
    if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire file mutex for download");
        //update_status_message("Download Failed");
        return ESP_FAIL;
    }
  
    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        xSemaphoreGive(file_mutex);
        return ESP_FAIL;
    }

    FILE *fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        xSemaphoreGive(file_mutex);
        return ESP_FAIL;
    }
  
    // // Prepare for download
    // suspend_tasks();

    ESP_LOGI(TAG, "Starting file transfer: %s (%ld bytes)", filepath, file_stat.st_size);
    // Set response headers
    httpd_resp_set_type(req, "text/csv");
    // Set filename for download
    char filename[64];
    //char* timeStr = fetchTime();
    //int year, month, day, hour, minute;
    //sscanf(timeStr, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute);
    snprintf(filename, sizeof(filename), "%s_logs.csv", CONFIG_SITE_NAME);

    char content_disposition[128];
    snprintf(content_disposition, sizeof(content_disposition), "attachment; filename=\"%s\"", filename);
    httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);

    // Send file in chunks
    char *chunk = heap_caps_malloc(4096, MALLOC_CAP_DMA);
    if (!chunk) {
        ESP_LOGE(TAG, "Failed to allocate chunk memory");
        fclose(fd);
        xSemaphoreGive(file_mutex);
        return ESP_FAIL;
    }

    size_t bytes_sent = 0;
    size_t chunk_size;
    esp_err_t ret = ESP_OK;

    while ((chunk_size = fread(chunk, 1, 4096, fd)) > 0) {
        if (httpd_resp_send_chunk(req, chunk, chunk_size) != ESP_OK) {
            ESP_LOGE(TAG, "File send failed at %zu bytes", bytes_sent);
            ret = ESP_FAIL;
            break;
        }
        bytes_sent += chunk_size;
        ESP_LOGD(TAG, "Sent %zu of %ld bytes", bytes_sent, file_stat.st_size);
    }

    // Cleanup
    free(chunk);
    fclose(fd);
    xSemaphoreGive(file_mutex);

    if (ret == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
        ESP_LOGI(TAG, "File sent successfully: %zu bytes", bytes_sent);
    }

    return ret;
}
static esp_err_t http_server_sensor_readings_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "Sensor readings requested");
    static sensor_readings_t http_readings;
    get_sensor_readings(&http_readings);

    static char resp[128];  // Increased buffer size
    snprintf(resp, sizeof(resp), 
             "{\"temp\": %.1f,  \"humidity\": %.1f, \"battery\": %.1f, }", 
             http_readings.temperature,  http_readings.humidity, http_readings.battery);
    
    ESP_LOGD(TAG, "Sending sensor readings: %s", resp);
    
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

// Initialize SPIFFS
esp_err_t init_spiffs() {
    esp_err_t ret = ESP_OK;
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 10,
        .format_if_mount_failed = true
    };
    
     ret = esp_vfs_spiffs_register(&conf);

    // Check if the file exists and its size
    FILE *dataFile = fopen(data_path, "r");
    if (!dataFile) {
      ESP_LOGI("SYSTEM", "Initialising empty sensor data file");
  
      dataFile = fopen(data_path, "w");
      vTaskDelay(10);
      if (dataFile) {
        fputs(DATA_FILE_HEADER, dataFile);
        fclose(dataFile);
      } else {
        ESP_LOGI("SYSTEM",
                 "Error opening sensor_data.csv for initialization. Errno: %d",
                 errno);
        char err_buf[100];
        strerror_r(errno, err_buf, sizeof(err_buf));
        ESP_LOGE(TAG, "Error details: %s", err_buf);
      }
    } else {
      ESP_LOGI("SYSTEM", "Sensor data file already exists");
      fclose(dataFile);
    }
  
    // Check if the file exists and its size
    FILE *logFile = fopen(log_path, "r");
    if (!logFile) {
      ESP_LOGI("SYSTEM", "Initialising empty log file");
  
      logFile = fopen(log_path, "w");
      vTaskDelay(10);
      if (logFile) {
        const char *header = "Time,Level,Tag,Message\n";
        fputs(header, logFile);
        fclose(logFile);
      } else {
        ESP_LOGE(TAG, "Error opening log.csv for initialization. Errno: %d",
                 errno);
        char err_buf[100];
        strerror_r(errno, err_buf, sizeof(err_buf));
        ESP_LOGE(TAG, "Error details: %s", err_buf);
      }
    } else {
      ESP_LOGI("SYSTEM", "Log file already exists");
      fclose(logFile);
    }
  
    ESP_LOGW("SPIFFS", "Spiffs Partition size: total: %d, used: %d", totalBytes,
             usedBytes);
    return ESP_OK;
  }
static esp_err_t http_server_generic_handler(httpd_req_t *req)
{   

    esp_err_t ret = init_spiffs();
 
    // First check if server is s till active
    if (!http_server_handle || !http_server_active) {
        ESP_LOGW(TAG, "Server inactive, rejecting request");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server shutting down");
        return ESP_FAIL;
    }
    if (uxTaskGetStackHighWaterMark(NULL) < 1000) {
        ESP_LOGE(TAG, "Low stack: %d", uxTaskGetStackHighWaterMark(NULL));
    }
    
    const char *uri = req->uri;
    ESP_LOGD(TAG, "Requested URI: %s", uri);
  //
    // Handle favicon.ico specially
    if (strcmp(uri, "/favicon.ico") == 0) {
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_send(req, (const char *)favicon, sizeof(favicon));
        return ESP_OK;
    }

    // Check for API endpoints first (keep existing API endpoint handling)
    // if (strcmp(uri, "/outputVal") == 0) {
    //     return http_server_outputVal_handler(req);
    // } else 
    if (strcmp(uri, "/download") == 0) {
        return http_server_download_handler(req);
    } 
    else if (strcmp(uri, "/logs") == 0) {
        return http_server_logs_handler(req);
    } 
    //else if (strcmp(uri, "/getInitialState") == 0) {
    //     return http_server_getInitialState_handler(req);
    // } 
    else if (strcmp(uri, "/sensorReadings") == 0) {
        return http_server_sensor_readings_handler(req);
    } 
    //else if (strcmp(uri, "/getSiteName") == 0) {
    //     return http_server_site_name_handler(req);
    // }
   // File serving code
    char filepath[FILE_PATH_MAX];
    if (strcmp(uri, "/") == 0) {
        uri = "/index.html";
    }

    int res = snprintf(filepath, sizeof(filepath), "/spiffs%s", uri);
    if (res < 0 || res >= sizeof(filepath)) {
        ESP_LOGE(TAG, "Filepath too long");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filepath too long");
        return ESP_FAIL;
    }

    struct stat file_stat;
    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGW(TAG, "Failed to stat file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    FILE *fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGW(TAG, "Failed to read file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read file");
        return ESP_FAIL;
    }

    // Set content type
    const char *type = "text/plain";
    if (strstr(uri, ".html")) type = "text/html";
    else if (strstr(uri, ".css")) type = "text/css";
    else if (strstr(uri, ".js")) type = "application/javascript";
    else if (strstr(uri, ".ico")) type = "image/x-icon";
    else if (strstr(uri, ".png")) type = "image/png";
    
    httpd_resp_set_type(req, type);
    
    // Set content length
    char cl_header[32];
    snprintf(cl_header, sizeof(cl_header), "%ld", file_stat.st_size);
    httpd_resp_set_hdr(req, "Content-Length", cl_header);

    // Allocate buffer from SPIRAM if available, otherwise use internal memory
    char *chunk = heap_caps_malloc(OPTIMAL_CHUNK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!chunk) {
        chunk = heap_caps_malloc(OPTIMAL_CHUNK_SIZE, MALLOC_CAP_8BIT);
        if (!chunk) {
            ESP_LOGE(TAG, "Failed to allocate memory for chunk");
            fclose(fd);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
            return ESP_FAIL;
        }
    }

    size_t chunksize;
    size_t bytes_sent = 0;
    
    
    do {
        chunksize = fread(chunk, 1, OPTIMAL_CHUNK_SIZE, fd);
        if (chunksize > 0) {
            if (send_chunk_with_retry(req, chunk, chunksize) != ESP_OK) {
                ESP_LOGW(TAG, "File sending failed at %d bytes", bytes_sent);
                ret = ESP_FAIL;
                break;
            }
            bytes_sent += chunksize;
            
            // Add a small delay between chunks to prevent buffer overflow
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    } while (chunksize > 0 && ret == ESP_OK);

    // Cleanup
    free(chunk);
    fclose(fd);

    if (ret == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
        ESP_LOGI(TAG, "File sent successfully: %s (%d bytes)", filepath, bytes_sent);
    }
    
    return ret;
}

static esp_err_t http_server_setMode_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "setMode request received");

    // Buffer to store request content
    char content[100];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);

    // Read the data for the request
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    // Null-terminate the received data
    content[recv_size] = '\0';

    ESP_LOGD(TAG, "Received content: %s", content);
  
    // Variables to store parsed values
    char mode[10] = {0};
    int state = -1;
  
    // Manual JSON parsing
    char *mode_start = strstr(content, "\"mode\":\"");
    char *state_start = strstr(content, "\"state\":");

    if (mode_start && state_start) {
        mode_start += 8; // Move past "mode":"
        char *mode_end = strchr(mode_start, '"');
        if (mode_end) {
            strncpy(mode, mode_start, mode_end - mode_start);
            mode[mode_end - mode_start] = '\0';
        }

        state_start += 8; // Move past "state":
        state = atoi(state_start);
    }

    if (mode[0] == '\0' || state == -1) {
        ESP_LOGE(TAG, "Invalid JSON format");
        httpd_resp_set_status(req, HTTPD_400);
        httpd_resp_sendstr(req, "Bad Request: Invalid JSON format");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Parsed values: mode=%s, state=%d", mode, state);

    // if (strcmp(mode, "AUTO") == 0) {
    //     AUTO_mode = (state == 1);
    //     // No additional handling for AIR mode
    // } 
    // Handle SPRAY and DRAIN modes
    // else if (strcmp(mode, "SPRAY") == 0 || strcmp(mode, "DRAIN") == 0) {
    //     if (AUTO_mode) {
    //         ESP_LOGW(TAG, "Cannot change SPRAY or DRAIN when AIR mode is enabled");
    //         httpd_resp_set_status(req, HTTPD_400);
    //         httpd_resp_sendstr(req, "Bad Request: AIR mode is enabled");
    //         return ESP_FAIL;
    //     } else {
    //         if (strcmp(valveStateToString(getCurrentState()), "IDLE") == 0) {
    //           if (strcmp(mode, "SPRAY") == 0) {
    //               setCurrentState(STATE_SPRAY_START);
    //           } else {
    //               setCurrentState(STATE_DRAIN_START);
    //           }
    //         }else{
    //           ESP_LOGW(TAG, "Cannot change when State not idle");
    //           httpd_resp_set_status(req, HTTPD_400);
    //           httpd_resp_sendstr(req, "Bad Request: AIR mode is enabled");
    //           return ESP_FAIL;
    //         }
    //     }
    // } else {
    //     ESP_LOGW(TAG, "Unknown mode: %s", mode);
    //     httpd_resp_set_status(req, HTTPD_400);
    //     httpd_resp_sendstr(req, "Bad Request: Unknown mode");
    //     return ESP_FAIL;
    // }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"success\"}");
    return ESP_OK;
}

// void suspend_tasks(void)
// {
//     ESP_LOGI(TAG, "Suspending all tasks");
    
//     // Create semaphore if it doesn't exist
//     if (ota_semaphore == NULL) {
//         ota_semaphore = xSemaphoreCreateBinary();
//     }
    
//     // Safely suspend tasks with null checks
//     if (buttonTaskHandle != NULL) vTaskSuspend(buttonTaskHandle);
//     if (loraTaskHandle != NULL) vTaskSuspend(loraTaskHandle);
//     if (valveTaskHandle != NULL) vTaskSuspend(valveTaskHandle);
//     if (sensorTaskHandle != NULL) vTaskSuspend(sensorTaskHandle);
//     // if (modbusTaskHandle != NULL) vTaskSuspend(modbusTaskHandle);
//     // if (adcTaskHandle != NULL) vTaskSuspend(adcTaskHandle);
//     // if (tempTaskHandle != NULL) vTaskSuspend(tempTaskHandle);
//     if (backupTaskHandle != NULL) vTaskSuspend(backupTaskHandle);
//     if (dataLoggingTaskHandle != NULL) vTaskSuspend(dataLoggingTaskHandle);
    
//     // Give some time for tasks to complete suspension
//     vTaskDelay(pdMS_TO_TICKS(1000));
// }

// void resume_tasks(void)
// {
//     ESP_LOGI(TAG, "Resuming all tasks");
    
//     // Resume tasks with null checks
//     if (buttonTaskHandle != NULL) vTaskResume(buttonTaskHandle);
//     if (loraTaskHandle != NULL) vTaskResume(loraTaskHandle);
//     if (valveTaskHandle != NULL) vTaskResume(valveTaskHandle);
//     if (sensorTaskHandle != NULL) vTaskResume(sensorTaskHandle);
//     // if (modbusTaskHandle != NULL) vTaskResume(modbusTaskHandle);
//     // if (adcTaskHandle != NULL) vTaskResume(adcTaskHandle);
//     // if (tempTaskHandle != NULL) vTaskResume(tempTaskHandle);
//     if (backupTaskHandle != NULL) vTaskResume(backupTaskHandle);
//     if (dataLoggingTaskHandle != NULL) vTaskResume(dataLoggingTaskHandle);
    
//     // Give some time for tasks to resume
//     vTaskDelay(pdMS_TO_TICKS(1000));
    
//     ESP_LOGI(TAG, "All tasks resumed successfully");
// }

/**
 * Receives the .bin file fia the web page and handles the firmware update
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK, otherwise ESP_FAIL if timeout occurs and the update cannot be started.
 */
esp_err_t http_server_OTA_update_handler(httpd_req_t *req)
{
  // Prepare for OTA
  // prepare_for_ota();
  //suspend_tasks();

  // Set OTA ready flag and give semaphore
  ota_ready = true;
  xSemaphoreGive(ota_semaphore);
  
  ESP_LOGI(TAG, "OTA preparation complete");
  // Wait for OTA preparation to complete with timeout
  if (xSemaphoreTake(ota_semaphore, pdMS_TO_TICKS(5000)) != pdTRUE) {
      ESP_LOGE(TAG, "OTA preparation timeout");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA preparation failed");
      return ESP_FAIL;
  }

  if (!ota_ready) {
      ESP_LOGE(TAG, "OTA not ready");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA not ready");
      return ESP_FAIL;
  }

	esp_ota_handle_t ota_handle;
	char ota_buff[1024];
	// char ota_buff[512];
	int content_length = req->content_len;
	int content_received = 0;
	int recv_len;
	bool is_req_body_started = false;
	bool flash_successful = false;

	const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

	do
	{
		// Read the data for the request
		if ((recv_len = httpd_req_recv(req, ota_buff, MIN(content_length, sizeof(ota_buff)))) < 0)
		{
			// Check if timeout occurred
			if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
			{
				ESP_LOGI(TAG, "http_server_OTA_update_handler: Socket Timeout");
				continue; ///> Retry receiving if timeout occurred
			}
			ESP_LOGI(TAG, "http_server_OTA_update_handler: OTA other Error %d", recv_len);
			return ESP_FAIL;
		}
		printf("http_server_OTA_update_handler: OTA RX: %d of %d\r", content_received, content_length);

		// Is this the first data we are receiving
		// If so, it will have the information in the header that we need.
		if (!is_req_body_started)
		{
			is_req_body_started = true;

			// Get the location of the .bin file content (remove the web form data)
			char *body_start_p = strstr(ota_buff, "\r\n\r\n") + 4;
			int body_part_len = recv_len - (body_start_p - ota_buff);

			printf("http_server_OTA_update_handler: OTA file size: %d\r\n", content_length);

			esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
			if (err != ESP_OK)
			{
				printf("http_server_OTA_update_handler: Error with OTA begin, cancelling OTA\r\n");
				return ESP_FAIL;
			}
			else
			{
				printf("http_server_OTA_update_handler: Writing to partition subtype %d at offset 0x%lx\r\n", update_partition->subtype, update_partition->address);
			}

			// Write this first part of the data
			esp_ota_write(ota_handle, body_start_p, body_part_len);
			content_received += body_part_len;
		}
		else
		{
			// Write OTA data
			esp_ota_write(ota_handle, ota_buff, recv_len);
			content_received += recv_len;
		}

	} while (recv_len > 0 && content_received < content_length);

	if (esp_ota_end(ota_handle) == ESP_OK)
	{
		// Lets update the partition
		if (esp_ota_set_boot_partition(update_partition) == ESP_OK)
		{
			const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
			ESP_LOGI(TAG, "http_server_OTA_update_handler: Next boot partition subtype %d at offset 0x%lx", boot_partition->subtype, boot_partition->address);
			flash_successful = true;
		}
		else
		{
			ESP_LOGI(TAG, "http_server_OTA_update_handler: FLASHED ERROR!!!");
		}
	}
	else
	{
		ESP_LOGI(TAG, "http_server_OTA_update_handler: esp_ota_end ERROR!!!");
	}

	// We won't update the global variables throughout the file, so send the message about the status
	// if (flash_successful) { http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_SUCCESSFUL); } else { http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_FAILED); }

	if (flash_successful) { 
    // http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_SUCCESSFUL); 
    ESP_LOGI(TAG, "HTTP_MSG_OTA_UPDATE_SUCCESSFUL");
    g_fw_update_status = OTA_UPDATE_SUCCESSFUL;
    http_server_fw_update_reset_timer();
  } else { 
    // http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_FAILED); 
    ESP_LOGI(TAG, "HTTP_MSG_OTA_UPDATE_FAILED");
    g_fw_update_status = OTA_UPDATE_FAILED;
  }

	return ESP_OK;
}

/**
 * OTA status handler responds with the firmware update status after the OTA update is started
 * and responds with the compile time/date when the page is first requested
 * @param req HTTP request for which the uri needs to be handled
 * @return ESP_OK
 */
esp_err_t http_server_OTA_status_handler(httpd_req_t *req)
{
	char otaJSON[100];

	ESP_LOGI(TAG, "OTAstatus requested");

	sprintf(otaJSON, "{\"ota_update_status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}", g_fw_update_status, __TIME__, __DATE__);

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, otaJSON, strlen(otaJSON));

	return ESP_OK;
}

/**
 * Sets up the default httpd server configuration.
 * @return http server instance handle if successful, NULL otherwise.
 */
static httpd_handle_t http_server_configure(void)
{
	// Generate the default configuration
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.uri_match_fn = httpd_uri_match_wildcard;

	// Create the message queue
	http_server_monitor_queue_handle = xQueueCreate(3, sizeof(http_server_queue_message_t));

	// Create HTTP server monitor task
	// xTaskCreatePinnedToCore(&http_server_monitor, "http_server_monitor", HTTP_SERVER_MONITOR_STACK_SIZE, NULL, HTTP_SERVER_MONITOR_PRIORITY, &task_http_server_monitor, HTTP_SERVER_MONITOR_CORE_ID);

	// The core that the HTTP server will run on
	config.core_id = HTTP_SERVER_TASK_CORE_ID;
	config.task_priority = HTTP_SERVER_TASK_PRIORITY;
	config.stack_size = HTTP_SERVER_TASK_STACK_SIZE;
  // Strict connection limits
  config.max_open_sockets = MAX_CONCURRENT_CLIENTS;  // Only allow 2 connections
  config.backlog_conn = 2;  // Only 1 connection can wait in queue
  config.lru_purge_enable = true;  // Enable LRU to remove stale connections
  
  // Aggressive timeout settings
  config.recv_wait_timeout = HTTP_SERVER_TIMEOUT;
  config.send_wait_timeout = HTTP_SERVER_TIMEOUT;
  
  // Keep-alive settings
  config.keep_alive_enable = true;
  config.keep_alive_idle = HTTP_KEEP_ALIVE_IDLE;
  config.keep_alive_interval = HTTP_KEEP_ALIVE_INTERVAL;
  config.keep_alive_count = HTTP_KEEP_ALIVE_COUNT;
  
  // URI handler settings
  config.max_uri_handlers = 10;     // Reduced from 15
  config.max_resp_headers = 5;      // Reduced from 8

	ESP_LOGD(TAG,
			"http_server_configure: Starting server on port: '%d' with task priority: '%d'",
			config.server_port,
			config.task_priority);

	// Start the httpd server
	if (httpd_start(&http_server_handle, &config) == ESP_OK)
	{
		ESP_LOGD(TAG, "http_server_configure: Registering URI handlers");


    // Register generic handler for HTML files
    httpd_uri_t generic_handler = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = http_server_generic_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(http_server_handle, &generic_handler);

		httpd_uri_t setMode = {
				.uri = "/setMode",
				.method = HTTP_POST,
				.handler = http_server_setMode_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &setMode);

		// register OTAupdate handler
		httpd_uri_t OTA_update = {
				.uri = "/OTAupdate",
				.method = HTTP_POST,
				.handler = http_server_OTA_update_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &OTA_update);

		// register OTAstatus handler
		httpd_uri_t OTA_status = {
				.uri = "/OTAstatus",
				.method = HTTP_POST,
				.handler = http_server_OTA_status_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &OTA_status);

		return http_server_handle;
	}

	return NULL;
}

// void http_server_start(void)
// {
//     if (http_server_handle == NULL) {
//         
//         // Configure and start HTTP server
//         http_server_handle = http_server_configure();
//         // Log available memory
//         ESP_LOGI(TAG, "Free heap: %lu", esp_get_free_heap_size());
//         if (http_server_handle == NULL) {
//             ESP_LOGE(TAG, "Failed to start HTTP server");
//             return;
//         }
//         
//         ESP_LOGI(TAG, "HTTP server started successfully");
//     }
// }

void http_server_start(void)
{
    // Force cleanup before starting server
    // force_heap_cleanup();

    if (http_server_handle != NULL) {
        ESP_LOGW(TAG, "HTTP server already running, stopping first");
        http_server_stop();
        vTaskDelay(pdMS_TO_TICKS(500));  // Give time for cleanup
    }

    // Check available heap memory
    uint32_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Current free heap: %lu bytes", free_heap);
    
    if (free_heap < MIN_HEAP_SIZE_HTTP) {
        ESP_LOGW(TAG, "Insufficient heap for HTTP server (%lu < %d)", 
                 free_heap, MIN_HEAP_SIZE_HTTP);
        ESP_LOGE(TAG, "Cannot start HTTP server - insufficient memory");
        return;
    }
    
    // Configure and start HTTP server
    http_server_handle = http_server_configure();
    if (http_server_handle != NULL) {
        http_server_active = true;
        ESP_LOGI(TAG, "HTTP server started successfully");
        ESP_LOGI(TAG, "Remaining heap: %lu bytes", esp_get_free_heap_size());
    }
}

void http_server_stop(void)
{
	if (http_server_handle)
	{
    vTaskDelay(pdMS_TO_TICKS(500));  // Give time for pending requests to complete
		esp_err_t err = httpd_stop(http_server_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop http server: %d", err);
    }
    
    ESP_LOGI(TAG, "HTTP server stopped");
    http_server_handle = NULL;
    // Clear any remaining resources
    if (wifi_app_queue_handle != NULL) {
        xQueueReset(wifi_app_queue_handle);
    }

    // Wait for cleanup to complete
    vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/**
 * Checks the g_fw_update_status and creates the fw_update_reset timer if g_fw_update_status is true.
 */
void http_server_fw_update_reset_timer(void)
{
	if (g_fw_update_status == OTA_UPDATE_SUCCESSFUL)
	{
		ESP_LOGI(TAG, "http_server_fw_update_reset_timer: FW updated successful starting FW update reset timer");

		// Give the web page a chance to receive an acknowledge back and initialize the timer
		ESP_ERROR_CHECK(esp_timer_create(&fw_update_reset_args, &fw_update_reset));
		ESP_ERROR_CHECK(esp_timer_start_once(fw_update_reset, 8000000));
	}
	else
	{
		ESP_LOGI(TAG, "http_server_fw_update_reset_timer: FW update unsuccessful");
	}
}

void http_server_fw_update_reset_callback(void *arg)
{
	ESP_LOGI(TAG, "http_server_fw_update_reset_callback: Timer timed-out, restarting the device");
	esp_restart();
}

