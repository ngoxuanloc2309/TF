#include "app_config.h"
#include "logger.h"
#include "sx_delay.h"
#include "stm32h5xx_hal.h"
#include "usart.h"
#include "i2c.h"
#include "tusb.h"
#include "usb.h"
#include "sx_board.h"

/* ════════════════════════════════════════════════════════════════════════
 * TEST FLAGS — set exactly ONE to 1
 * ════════════════════════════════════════════════════════════════════════ */
#define TEST_CDC        0   /* Read/write USB CDC                          */
#define TEST_MSC        0   /* Virtual disk: create file, read, write      */
#define TEST_GPS        1   /* Read NMEA, parse RMC, print lat/lon         */
#define TEST_MQTT       0   /* Connect broker, publish message             */
#define TEST_EXFLASH    0   /* Create file, write, read on ExFlash         */
#define TEST_IMU        0   /* Read IMU */
#define AT_USB          0

static const char *TAG = "TEST";

/* ════════════════════════════════════════════════════════════════════════
 *  TEST_CDC
 * ════════════════════════════════════════════════════════════════════════
 * Expected:
 *   - Open any serial terminal on the CDC COM port
 *   - Device echoes back everything you type
 *   - Type "hello" → device replies "SynaptiX CDC OK\r\n"
 *   - Logger prints RX byte count each time
 * ════════════════════════════════════════════════════════════════════════ */
#if TEST_CDC

#include "sx_usb_tiny_cdc.h"
#include "sx_user_cdc.h"

static sx_usb_tiny_t        s_usb;
static sx_usb_tiny_config_t s_usb_cfg = {
    .rx_buf_size = 256,
    .tx_buf_size = 256,
};
static uint8_t       s_rx[256];
static uint8_t       s_tx[256];
static sx_user_cdc_t s_cdc;

void app_init(void)
{
    log_info(TAG, "=== TEST CDC ===");

    board_init();
    //dcd_fs_msp_init(0);

    s_usb.rxBuffer = s_rx;
    s_usb.txBuffer = s_tx;
    sx_usb_tiny_init(&s_usb, &s_usb_cfg);
    sx_user_cdc_init(&s_cdc, &s_usb);

    sx_user_msc_init();
    sx_user_msc_create_disk(MSC_DISK_LABEL);

    log_info(TAG, "CDC init done — connect USB and open serial terminal");
}

void app_process(uint32_t timestamp)
{
    (void)timestamp;
    sx_user_cdc_process(&s_cdc);

    if (!sx_user_cdc_connected(&s_cdc)) return;

    if (sx_user_cdc_available(&s_cdc)) {
        uint8_t buf[64];
        int len = sx_user_cdc_read(&s_cdc, buf, sizeof(buf), 10);
        if (len > 0) {
            log_info(TAG, "CDC RX: %d bytes", len);

            /* Echo back */
            sx_user_cdc_write(&s_cdc, buf, len);

            /* Command: "hello" → reply */
            if (len >= 5 && memcmp(buf, "hello", 5) == 0) {
                sx_user_cdc_printf(&s_cdc, "SynaptiX CDC OK\r\n");
                log_info(TAG, "Replied SynaptiX CDC OK");
            }
        }
    }
}

#endif /* TEST_CDC */

/* ════════════════════════════════════════════════════════════════════════
 *  TEST_MSC
 * ════════════════════════════════════════════════════════════════════════
 * Expected:
 *   - PC sees a USB drive named "SynaptiX"
 *   - Drive contains "tracking.csv" with header + 3 test rows
 *   - Logger prints each write/read result
 *   - After 5s, appends 1 more row and remounts → PC sees updated file
 * ════════════════════════════════════════════════════════════════════════ */
#if TEST_MSC

#include "sx_user_msc.h"
#include "sx_usb_tiny_cdc.h"  
#include <stdio.h> 

static uint32_t s_last_append_tick = 0;
static uint8_t  s_append_count     = 0;

static sx_usb_tiny_t        s_usb;
static sx_usb_tiny_config_t s_usb_cfg = {
    .rx_buf_size = 256,
    .tx_buf_size = 256,
};
static uint8_t s_rx[256];
static uint8_t s_tx[256];

#define MSC_APPEND_INTERVAL_MS  10000U
#define MSC_APPEND_MAX          5

void app_init(void)
{
    log_info(TAG, "=== TEST MSC ===");

    board_init();

    s_usb.rxBuffer = s_rx;
    s_usb.txBuffer = s_tx;
    sx_usb_tiny_init(&s_usb, &s_usb_cfg);

    /* Init + create disk */
    sx_user_msc_err_t ret;

    ret = sx_user_msc_init();
    log_info(TAG, "msc_init: %d", ret);

    ret = sx_user_msc_create_disk(MSC_DISK_LABEL);
    log_info(TAG, "create_disk: %d", ret);

    ret = sx_user_msc_create_file(MSC_CSV_PATH);
    log_info(TAG, "create_file: %d", ret);

    /* Write CSV header */
    const char *header = CSV_HEADER;
    ret = sx_user_msc_write(MSC_CSV_PATH,
                            (const uint8_t *)header, strlen(header));
    log_info(TAG, "write header: %d", ret);

    /* Write 3 initial test rows */
    const char *rows[] = {
        "1741234567,21.027763,105.834160\r\n",
        "1741234600,21.027800,105.834200\r\n",
        "1741234633,21.027850,105.834250\r\n",
    };
    for (int i = 0; i < 3; i++) {
        ret = sx_user_msc_append(MSC_CSV_PATH,
                                 (const uint8_t *)rows[i], strlen(rows[i]));
        log_info(TAG, "append row[%d]: %d", i, ret);
    }

    /* Read back and verify */
    uint8_t  rbuf[512] = {0};
    uint32_t rlen      = 0;
    ret = sx_user_msc_read(MSC_CSV_PATH, rbuf, sizeof(rbuf) - 1, &rlen);
    rbuf[rlen] = '\0';
    log_info(TAG, "read back (%lu bytes):\r\n%s", rlen, (char *)rbuf);

    s_last_append_tick = HAL_GetTick();
    log_info(TAG, "MSC init done — connect USB to see drive");
}

void app_process(uint32_t timestamp)
{
    (void)timestamp;
    tud_task();

    /* Every 5s append a new row → trigger remount */
    if (s_append_count < MSC_APPEND_MAX &&
        HAL_GetTick() - s_last_append_tick >= MSC_APPEND_INTERVAL_MS)
    {
        s_last_append_tick = HAL_GetTick();
        s_append_count++;

        char line[64];
        snprintf(line, sizeof(line),
                 "%lu,21.02%04u,105.83%04u\r\n",
                 HAL_GetTick() / 1000UL,
                 s_append_count * 7,
                 s_append_count * 13);

        sx_user_msc_err_t ret = sx_user_msc_update_file(
                MSC_CSV_PATH, (uint8_t *)line, strlen(line));
        log_info(TAG, "update_file[%d]: %d — \"%s\"",
                 s_append_count, ret, line);
    }
}

#endif /* TEST_MSC */

/* ════════════════════════════════════════════════════════════════════════
 *  TEST_GPS
 * ════════════════════════════════════════════════════════════════════════
 * Expected:
 *   - Logger prints raw NMEA sentences received
 *   - When RMC valid → prints lat/lon
 *   - If no fix within GPS_TIMEOUT_MS → prints timeout warning
 * ════════════════════════════════════════════════════════════════════════ */
#if TEST_GPS
 
#include "gps.h"
#include <stdio.h>
 
#define GPS_PRINT_INTERVAL_MS   1000U
#define GPS_WAIT_FIX_LOG_MS     5000U  
 
static uint32_t s_last_print_tick    = 0;
static uint32_t s_last_no_fix_tick   = 0;
 
void app_init(void)
{
    log_info(TAG, "=== TEST GPS ===");
  
    gps_power_on();
    log_info(TAG, "GPS power on — waiting for fix...");
    log_info(TAG, "UART: USART2 | Baud: 9600 | 8N1");
 
    s_last_print_tick  = HAL_GetTick();
    s_last_no_fix_tick = HAL_GetTick();
}
 
void app_process(uint32_t timestamp)
{
    (void)timestamp;
 
    gps_process();
 
    uint32_t now = HAL_GetTick();
 
    if (gps_is_fixed())
    {
        if (now - s_last_print_tick >= GPS_PRINT_INTERVAL_MS)
        {
            s_last_print_tick = now;
 
            const gps_data_t *d = gps_get_data();
 
            log_info(TAG, "--- GPS FIX ---");
            log_info(TAG, "  Lat      : %.7f", d->latitude);
            log_info(TAG, "  Lon      : %.7f", d->longitude);
            log_info(TAG, "  Alt      : %.1f m", d->altitude);
            log_info(TAG, "  Sats     : %d", d->satellites);
            log_info(TAG, "  Fix qual : %d", d->fix_quality);
        }
    }
    else
    {
        if (now - s_last_no_fix_tick >= GPS_WAIT_FIX_LOG_MS)
        {
            s_last_no_fix_tick = now;
 
            const gps_data_t *d = gps_get_data();
            log_warn(TAG, "No fix yet — sats tracked: %d", d->satellites);
        }
    }
}
 
#endif /* TEST_GPS */

/* ════════════════════════════════════════════════════════════════════════
 *  TEST_MQTT
 * ════════════════════════════════════════════════════════════════════════
 * Expected:
 *   - Logger shows SIM76xx init state machine progress
 *   - Logger shows "MQTT connected"
 *   - Every 10s publishes a test message to MQTT_TOPIC
 *   - Verify on broker side (e.g. MQTT Explorer)
 * ════════════════════════════════════════════════════════════════════════ */
#if TEST_MQTT

#include "sx_user_mqtt.h"
#include "gpio.h"
#include "sx_board.h"

static uint32_t s_publish_elapsed = 0;
static uint32_t s_publish_count   = 0;
static uint8_t  s_subscribed      = 0;

#define MQTT_PUBLISH_INTERVAL_MS    10000U

static void _on_connected(void)
{
    log_info(TAG, "MQTT connected!");
    s_subscribed = 0;
}

static void _on_disconnected(void)
{
    log_warn(TAG, "MQTT disconnected");
    s_subscribed = 0;
}

static void _on_publish(int success)
{
    log_info(TAG, "Publish #%lu: %s", s_publish_count, success ? "OK" : "FAIL");
}

static void _on_message(const char *topic, const char *message)
{
    log_info(TAG, "SUB [%s] = %s", topic, message);
}

static sx_user_mqtt_cfg_t s_mqtt_cfg = {
    .apn           = GSM_APN,
    .broker        = MQTT_HOST,
    .port          = MQTT_PORT,
    .client_id     = MQTT_CLIENT_ID,
    .username      = MQTT_USER,
    .password      = MQTT_PASS,
    .keepalive     = MQTT_KEEPALIVE,
    .clean_session = 1,
    .on_connected    = _on_connected,
    .on_disconnected = _on_disconnected,
    .on_message      = _on_message,
    .on_publish      = _on_publish,
};

void app_init(void)
{
    log_info(TAG, "=== TEST MQTT ===");
    sx_user_mqtt_nontls_init(&s_mqtt_cfg);
    log_info(TAG, "MQTT init done — waiting for connection...");
}

void app_process(uint32_t timestamp)
{
    sx_user_mqtt_poll(timestamp);
    s_publish_elapsed += timestamp;

    if (!sx_user_mqtt_is_connected()) {
        s_subscribed = 0;
        return;
    }

    if (!s_subscribed) {
        s_subscribed = 1;
        sx_user_mqtt_subscribe(MQTT_SUB_TOPIC);
        s_publish_elapsed = 0;  
        return;
    }

    if (s_publish_elapsed >= MQTT_PUBLISH_INTERVAL_MS) {
        s_publish_elapsed = 0;
        s_publish_count++;

        char msg[128];
        snprintf(msg, sizeof(msg),
                 "{\"test\":%lu,\"msg\":\"SynaptiX MQTT OK\"}",
                 s_publish_count);

        sx_user_mqtt_publish(MQTT_TOPIC, msg);
        log_info(TAG, "Publishing #%lu: %s", s_publish_count, msg);
    }
}

#endif /* TEST_MQTT */

/* ════════════════════════════════════════════════════════════════════════
 *  TEST_EXFLASH
 * ════════════════════════════════════════════════════════════════════════
 * Expected:
 *   - Logger shows W25Q128 init OK
 *   - Writes a struct to "/test.bin" → reads back → compares
 *   - Writes a string to "/test.txt" → reads back → prints
 *   - Appends lines to "/log.txt" → reads back full content
 *   - Prints free space before and after
 * ════════════════════════════════════════════════════════════════════════ */
#if TEST_EXFLASH

#include "sx_ex_storage.h"

typedef struct {
    uint32_t id;
    float    lat;
    float    lon;
    uint8_t  valid;
} test_record_t;

void app_init(void)
{
    log_info(TAG, "=== TEST EXFLASH ===");

    /* Init */
    sx_storage_err_t ret = sx_storage_init();
    log_info(TAG, "storage_init: %d", ret);
    if (ret != SX_STORAGE_OK) {
        log_error(TAG, "ExFlash init failed — check SPI/CS wiring");
        return;
    }

    log_info(TAG, "Total: %ld bytes | Free: %ld bytes",
             sx_storage_total_space(), sx_storage_free_space());

    /* ── Test 1: binary struct write/read ── */
    test_record_t wr = {
        .id    = 0xDEADBEEF,
        .lat   = 21.027763f,
        .lon   = 105.834160f,
        .valid = 1,
    };
    ret = sx_storage_write("/test.bin", &wr, sizeof(wr));
    log_info(TAG, "[T1] write struct: %d", ret);

    test_record_t rd = {0};
    ret = sx_storage_read("/test.bin", &rd, sizeof(rd));
    log_info(TAG, "[T1] read struct: %d", ret);

    if (rd.id == wr.id && rd.valid == wr.valid) {
        log_info(TAG, "[T1] PASS — id=0x%08lX lat=%.6f lon=%.6f",
                 rd.id, rd.lat, rd.lon);
    } else {
        log_error(TAG, "[T1] FAIL — data mismatch");
    }

    /* ── Test 2: string write/read ── */
    const char *hello = "SynaptiX ExFlash OK";
    ret = sx_storage_write("/test.txt", hello, strlen(hello));
    log_info(TAG, "[T2] write string: %d", ret);

    char rbuf[64] = {0};
    ret = sx_storage_read("/test.txt", rbuf, sizeof(rbuf) - 1);
    log_info(TAG, "[T2] read string: %d → \"%s\"", ret, rbuf);

    if (strcmp(rbuf, hello) == 0)
        log_info(TAG, "[T2] PASS");
    else
        log_error(TAG, "[T2] FAIL");

    /* ── Test 3: append multiple lines ── */
    sx_storage_delete("/log.txt");  /* clean start */

    const char *lines[] = {
        "line1: boot\r\n",
        "line2: GPS OK\r\n",
        "line3: MQTT OK\r\n",
    };
    for (int i = 0; i < 3; i++) {
        ret = sx_storage_append("/log.txt", lines[i], strlen(lines[i]));
        log_info(TAG, "[T3] append line[%d]: %d", i, ret);
    }

    char lbuf[256] = {0};
    ret = sx_storage_read("/log.txt", lbuf, sizeof(lbuf) - 1);
    log_info(TAG, "[T3] read log:\r\n%s", lbuf);

    /* ── Test 4: exists / size / delete ── */
    log_info(TAG, "[T4] exists(/test.bin): %d",  sx_storage_exists("/test.bin"));
    log_info(TAG, "[T4] size(/test.bin):   %ld", sx_storage_size("/test.bin"));

    ret = sx_storage_delete("/test.bin");
    log_info(TAG, "[T4] delete: %d", ret);
    log_info(TAG, "[T4] exists after delete: %d", sx_storage_exists("/test.bin"));

    log_info(TAG, "Free after tests: %ld bytes", sx_storage_free_space());
    log_info(TAG, "=== EXFLASH TEST DONE ===");
}

void app_process(uint32_t timestamp)
{
    (void)timestamp;
    /* ExFlash test is one-shot in app_init — nothing to poll */
}

#endif /* TEST_EXFLASH */

/* ════════════════════════════════════════════════════════════════════════
 *  TEST_IMU
 * ════════════════════════════════════════════════════════════════════════
 * Expected:
 *   - Logger shows BNO055 init OK, chip ID = 0xA0
 *   - Every 100ms prints Euler angles (heading/roll/pitch)
 *   - Every 1s prints calibration status (0-3 per sensor)
 *   - When all calib = 3 → prints "Fully calibrated!"
 * ════════════════════════════════════════════════════════════════════════ */
#if TEST_IMU

#include "bno055.h"

static sx_i2c_t      s_i2c;
static bno055_t      s_bno;
static sx_gpio_t     s_imu_en;
static sx_gpio_t     s_imu_reset;

static sx_gpio_pin_t s_imu_en_pin    = { .port = GPIOB, .pin = GPIO_PIN_4 };
static sx_gpio_pin_t s_imu_reset_pin = { .port = GPIOB, .pin = GPIO_PIN_5 };

static uint32_t s_elapsed_data  = 0;
static uint32_t s_elapsed_calib = 0;

#define IMU_DATA_INTERVAL_MS    100U
#define IMU_CALIB_INTERVAL_MS   1000U

extern sx_i2c_ops_t  sx_i2c_ops;
extern sx_gpio_ops_t sx_gpio_ops;

void bno055_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void app_init(void)
{
    log_info(TAG, "=== TEST IMU ===");

    sx_i2c_init (&s_i2c,       &sx_i2c_ops,  &hi2c1);
    sx_gpio_init(&s_imu_en,    &sx_gpio_ops, &s_imu_en_pin);
    sx_gpio_init(&s_imu_reset, &sx_gpio_ops, &s_imu_reset_pin);
    sx_gpio_write(&s_imu_en, SX_GPIO_LOW);  /* Power on IMU */

    if (bno055_init(&s_bno, &s_i2c, BNO055_I2C_ADDR_DEFAULT,
                    &s_imu_en, &s_imu_reset) != 0) {
        log_error(TAG, "BNO055 init FAIL");
        return;
    }

    log_info(TAG, "BNO055 init OK");
    log_info(TAG, "Move device to calibrate:");
    log_info(TAG, "  Gyro  — keep still for a few seconds");
    log_info(TAG, "  Accel — place in 6 different positions");
    log_info(TAG, "  Mag   — draw figure-8 in the air");
}

void app_process(uint32_t timestamp)
{
    s_elapsed_data  += timestamp;
    s_elapsed_calib += timestamp;

    if (s_elapsed_data >= IMU_DATA_INTERVAL_MS) {
        s_elapsed_data = 0;

        bno055_euler_t euler = {0};
        if (bno055_get_euler(&s_bno, &euler) == 0) {
            log_info(TAG, "Euler → H: %6.2f  R: %6.2f  P: %6.2f",
                     euler.heading / 16.0f,
                     euler.roll    / 16.0f,
                     euler.pitch   / 16.0f);
        }

        bno055_quat_t quat = {0};
        if (bno055_get_quat(&s_bno, &quat) == 0) {
            log_info(TAG, "Quat  → W: %.4f  X: %.4f  Y: %.4f  Z: %.4f",
                     quat.w / 16384.0f,
                     quat.x / 16384.0f,
                     quat.y / 16384.0f,
                     quat.z / 16384.0f);
        }

        bno055_vec3_t accel = {0};
        if (bno055_get_accel(&s_bno, &accel) == 0) {
            log_info(TAG, "Accel → X: %.2f  Y: %.2f  Z: %.2f m/s2",
                     accel.x / 100.0f,
                     accel.y / 100.0f,
                     accel.z / 100.0f);
        }
    }

    /* ── Đọc calibration status mỗi 1s ── */
    if (s_elapsed_calib >= IMU_CALIB_INTERVAL_MS) {
        s_elapsed_calib = 0;

        bno055_calib_stat_t calib = {0};
        if (bno055_get_calib_stat(&s_bno, &calib) == 0) {
            log_info(TAG, "Calib → SYS:%d GYR:%d ACC:%d MAG:%d",
                     calib.sys, calib.gyro, calib.accel, calib.mag);

            if (calib.sys   == 3 && calib.gyro == 3 &&
                calib.accel == 3 && calib.mag  == 3) {
                log_info(TAG, "*** Fully calibrated! ***");
            }
        }
    }
}

#endif /* TEST_IMU */

// #if AT_USB
// // app_at.c
// #include "at_command.h"
// #include "logger.h"

// static const char *TAG = "TEST_AT_USB";

// #define NUMBER_COMMAND  5

// #define AT              0
// #define AT_VPN          1
// #define AT_MQTTCONNECT  2
// #define AT_TIMESLEEP    3
// #define AT_TIMEWAKE     4

// static int _at_execute(AT_Command_t *cmd) {
//     log_info(TAG, "Execute: %s", cmd->command);
//     return 0;
// }

// static int _at_vpn_set(AT_Command_t *cmd, const char *param) {
//     // param = "v-internet","user","pass"
//     log_info(TAG, "VPN set: %s", param ? param : "NULL");
//     return 0;
// }

// static int _at_mqtt_set(AT_Command_t *cmd, const char *param) {
//     // param = "broker.hivemq.com",1883,"user","pass"
//     log_info(TAG, "MQTT set: %s", param ? param : "NULL");
//     return 0;
// }

// static int _at_timesleep_set(AT_Command_t *cmd, const char *param) {
//     // param = "120"
//     log_info(TAG, "TimeSleep set: %s", param ? param : "NULL");
//     return 0;
// }

// static int _at_timewake_set(AT_Command_t *cmd, const char *param) {
//     log_info(TAG, "TimeWake set: %s", param ? param : "NULL");
//     return 0;
// }

// static AT_Command_t s_commands[NUMBER_COMMAND] = {
//     [AT] = {
//         .command = "AT",
//         .handler = {
//             .execute_handler  = _at_execute,
//             .question_handler = NULL,
//             .set_handler      = NULL,
//         }
//     },
//     [AT_VPN] = {
//         .command = "AT+VPN",
//         .handler = {
//             .set_handler      = _at_vpn_set,
//             .question_handler = NULL,
//             .execute_handler  = NULL,
//         }
//     },
//     [AT_MQTTCONNECT] = {
//         .command = "AT+MQTTCONNECT",
//         .handler = {
//             .set_handler      = _at_mqtt_set,
//             .question_handler = NULL,
//             .execute_handler  = NULL,
//         }
//     },
//     [AT_TIMESLEEP] = {
//         .command = "AT+TIMESLEEP",
//         .handler = {
//             .set_handler      = _at_timesleep_set,
//             .question_handler = NULL,
//             .execute_handler  = NULL,
//         }
//     },
//     [AT_TIMEWAKE] = {
//         .command = "AT+TIMEWAKE",
//         .handler = {
//             .set_handler      = _at_timewake_set,
//             .question_handler = NULL,
//             .execute_handler  = NULL,
//         }
//     },
// };

// static AT_Implementation_t s_at_impl;

// void app_at_init(void) {
//     at_init(&s_at_impl, s_commands, NUMBER_COMMAND);
//     log_info(TAG, "AT command init OK");
// }

// void app_at_process(const char *data, size_t len) {
//     at_process_input(&s_at_impl, data, len);
// }
// #endif