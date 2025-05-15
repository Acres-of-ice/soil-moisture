#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define CALIBRATION_DURATION_MS (CONFIG_CALIBRATION_DURATION_M * 60000)
#define ERROR_DURATION_MS (CONFIG_ERROR_DURATION_M * 60000)
#define POLL_INTERVAL_MS (CONFIG_POLL_INTERVAL_S * 1000)

#define CALIBRATION_BUFFER_SIZE (CALIBRATION_DURATION_MS / POLL_INTERVAL_MS)
#define CALIBRATION_RETRY_DELAY_MS (60 * 60 * 1000) // 1 hour in milliseconds

esp_err_t init_calibration(void);
void start_calibration(void);
void check_and_start_calibration(int on_off_counter);
float get_mean_fountain_pressure(void);
bool is_calibration_done(void);

#endif // CALIBRATION_H
