#ifndef MAIN_TASKS_COMMON_H_
#define MAIN_TASKS_COMMON_H_

// Critical Communication Tasks - Core 0
// These tasks don't run simultaneously (LoRa XOR SMS)
#define COMM_TASK_STACK_SIZE (1024 * 6)
#define COMM_TASK_PRIORITY 12 // Highest priority
#define COMM_TASK_CORE_ID 0

#define SMS_TASK_STACK_SIZE (1024 * 8)
#define SMS_TASK_PRIORITY 10 // Same as LoRa since they're mutually exclusive
#define SMS_TASK_CORE_ID 0

// Critical Control Tasks - Core 0
#define VALVE_TASK_STACK_SIZE (1024 * 6)
#define VALVE_TASK_PRIORITY 11
#define VALVE_TASK_CORE_ID 0

#define BUTTON_TASK_STACK_SIZE (1024 * 4)
#define BUTTON_TASK_PRIORITY 10
#define BUTTON_TASK_CORE_ID 0

// Sensor Tasks - Core 0
#define ADC_TASK_STACK_SIZE (1024 * 4)
#define ADC_TASK_PRIORITY 8
#define ADC_TASK_CORE_ID 0

#define MODBUS_TASK_STACK_SIZE (1024 * 5)
#define MODBUS_TASK_PRIORITY 8
#define MODBUS_TASK_CORE_ID 0

// Replace separate ADC and Modbus definitions with:
#define SENSOR_TASK_STACK_SIZE                                                 \
  (1024 * 7) // Smaller than sum of individual tasks
#define SENSOR_TASK_PRIORITY 8
#define SENSOR_TASK_CORE_ID 0

// Interface Tasks - Core 0
#define LCD_TASK_STACK_SIZE (1024 * 4)
#define LCD_TASK_PRIORITY 6
#define LCD_TASK_CORE_ID 0

#define WIFI_APP_TASK_STACK_SIZE (1024 * 5)
#define WIFI_APP_TASK_PRIORITY 5
#define WIFI_APP_TASK_CORE_ID 0

#define SIMULATION_TASK_STACK_SIZE (1024 * 4)
#define SIMULATION_TASK_PRIORITY 8
#define SIMULATION_TASK_CORE_ID 0

// Network and Data Tasks - Core 1
#define HTTP_SERVER_TASK_STACK_SIZE (1024 * 8)
#define HTTP_SERVER_TASK_PRIORITY 11
#define HTTP_SERVER_TASK_CORE_ID 1

#define DATA_LOG_TASK_STACK_SIZE (1024 * 8)
#define DATA_LOG_TASK_PRIORITY 7
#define DATA_LOG_TASK_CORE_ID 1

#define BACKUP_TASK_STACK_SIZE (1024 * 4)
#define BACKUP_TASK_PRIORITY 6
#define BACKUP_TASK_CORE_ID 1

#define HEX_DATA_TASK_STACK_SIZE (1024 * 4)
#define HEX_DATA_TASK_PRIORITY 6
#define HEX_DATA_TASK_CORE_ID 1

#define MQTT_TASK_STACK_SIZE (1024 * 8)
#define MQTT_TASK_PRIORITY 8
#define MQTT_TASK_CORE_ID 1

#define MQTT_MANAGER_TASK_STACK_SIZE (1024 * 4)
#define MQTT_MANAGER_TASK_PRIORITY 4
#define MQTT_MANAGER_TASK_CORE_ID 1

#define TIMELAPSE_TASK_STACK_SIZE (1024 * 3)
#define TIMELAPSE_TASK_PRIORITY 7
#define TIMELAPSE_TASK_CORE_ID 0

#endif /* MAIN_TASKS_COMMON_H_ */
