#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    APP_MODE_FULL_POWER = 0,
    APP_MODE_ENTER_SLEEP,
    APP_MODE_SLEEP,
    APP_MODE_WAKE_PUBLISH,
} app_mode_t;

//extern volatile app_mode_t app_mode;

void app_init(void);
void app_process(uint32_t delta_ms);

void app_notify_usb_connected(void);

void app_request_sleep(void);

/* Restore the BNO055 calibration from IMU_CALIB_FILE_PATH (needed after the IMU lost power). */
bool imu_calib_load(void);

void app_sync_gps_log_to_disk(void);

void app_mode_full_pw(void);

#ifdef __cplusplus
}
#endif

#endif