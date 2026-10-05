#include "test_at.h"
#include "logger.h"
#include "sx_board.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

static const char *TAG = "TEST_AT_USB";

#define NUMBER_COMMAND  9

#define AT              0
#define AT_VPN          1
#define AT_MQTTCONNECT  2
#define AT_TIMESLEEP    3
#define AT_FLASHPWR     4
#define AT_FLASHTEST    5
#define AT_IMUPWR       6
#define AT_I2CPWR       7
#define AT_BQREAD       8

// const char* at_usb_command[NUMBER_COMMAND] = {"AT", "AT+VPN", "AT+MQTTCONNECT", "AT+TIMESLEEP"};

#define CMD_AT              "AT"
#define CMD_AT_VPN          "AT+VPN"
#define CMD_AT_MQTT         "AT+MQTTCONNECT"
#define CMD_AT_TIMESLEEP    "AT+TIMESLEEP"
#define CMD_AT_FLASHPWR     "AT+FLASHPWR"
#define CMD_AT_FLASHTEST    "AT+FLASHTEST"
#define CMD_AT_IMUPWR       "AT+IMUPWR"
#define CMD_AT_I2CPWR       "AT+I2CPWR"
#define CMD_AT_BQREAD       "AT+BQREAD"

#define AT_RESP_OK      "\r\nOK\r\n"
#define AT_RESP_ERROR   "\r\nERROR\r\n"

static void _respond(const char *resp)
{
    if (&board.usb == NULL) return;
    sx_usb_tiny_write(&board.usb, (const uint8_t *)resp, strlen(resp));
}

static int _at_execute(AT_Command_t *cmd)
{
    log_info(TAG, "Execute: %s", cmd->command);
    _respond(AT_RESP_OK);
    return 0;
}

static int _at_vpn_set(AT_Command_t *cmd, const char *param)
{
    log_info(TAG, "VPN set: %s", param ? param : "NULL");
    if (param == NULL) {
        _respond(AT_RESP_ERROR);
        return -1;
    }
    _respond(AT_RESP_OK);
    return 0;
}

static int _at_mqtt_set(AT_Command_t *cmd, const char *param)
{
    log_info(TAG, "MQTT set: %s", param ? param : "NULL");
    if (param == NULL) {
        _respond(AT_RESP_ERROR);
        return -1;
    }
    _respond(AT_RESP_OK);
    return 0;
}

static int _at_timesleep_set(AT_Command_t *cmd, const char *param)
{
    log_info(TAG, "TimeSleep set: %s", param ? param : "NULL");
    if (param == NULL) {
        _respond(AT_RESP_ERROR);
        return -1;
    }
    _respond(AT_RESP_OK);
    return 0;
}

/* ---------------------------------------------------------------------------
 *  Phase 2 manual tests (flash / IMU / I2C1 power). Not wired into the sleep flow.
 *
 *    AT+FLASHPWR=0|1      cut / restore the flash        AT+FLASHPWR?   state
 *    AT+FLASHTEST=N       N x (power off, write, power off, read back, compare)
 *    AT+IMUPWR=0|1        cut / restore the IMU          AT+IMUPWR?     state
 *    AT+I2CPWR=0|1        DeInit / Init I2C1             AT+I2CPWR?     state
 *    AT+BQREAD            read VBUS_STAT from the BQ (I2C 0x6A)
 *
 *  While I2C1 is off the Phase-0 BQ poll and the RTC reads in app.c will log failures: that is expected.
 * ------------------------------------------------------------------------- */

#define FLASHTEST_PATH      "/pwr_test"
#define FLASHTEST_LEN       32U
#define FLASHTEST_MAX_LOOPS 100

static void _respondf(const char *fmt, ...)
{
    char buf[112];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    _respond("\r\n");
    _respond(buf);
}

static int _parse01(const char *param)
{
    if (param == NULL) return -1;
    if (param[0] == '0' && (param[1] == '\0' || param[1] == '\r' || param[1] == ' ')) return 0;
    if (param[0] == '1' && (param[1] == '\0' || param[1] == '\r' || param[1] == ' ')) return 1;
    return -1;
}

static int _at_flashpwr_q(AT_Command_t *cmd)
{
    _respondf("+FLASHPWR: %d", sx_storage_is_powered() ? 1 : 0);
    _respond(AT_RESP_OK);
    return 0;
}

static int _at_flashpwr_set(AT_Command_t *cmd, const char *param)
{
    int v = _parse01(param);
    if (v < 0) { _respond(AT_RESP_ERROR); return -1; }

    /* Answer first: if the sequence below misbehaves, the terminal has still seen the command arrive. */
    _respondf("+FLASHPWR: going %s", v ? "on" : "off");
    log_info(TAG, "FLASHPWR=%d: start", v);

    if (v == 0) {
        sx_storage_sleep();
    } else if (sx_storage_wake() != SX_STORAGE_OK) {
        _respondf("+FLASHPWR: wake FAILED");
        _respond(AT_RESP_ERROR);
        return -1;
    }
    log_info(TAG, "FLASHPWR=%d: done", v);
    _respondf("+FLASHPWR: %d", sx_storage_is_powered() ? 1 : 0);
    _respond(AT_RESP_OK);
    return 0;
}

static int _at_flashtest_set(AT_Command_t *cmd, const char *param)
{
    int n = param ? atoi(param) : 0;
    if (n < 1 || n > FLASHTEST_MAX_LOOPS) { _respond(AT_RESP_ERROR); return -1; }

    int pass = 0, fail = 0;
    uint8_t wbuf[FLASHTEST_LEN], rbuf[FLASHTEST_LEN];

    for (int i = 0; i < n; i++) {
        for (uint32_t k = 0; k < FLASHTEST_LEN; k++) wbuf[k] = (uint8_t)(i * 7U + k * 3U + 1U);

        /* 1. power off, let the rail decay */
        sx_storage_sleep();
        if (sx_storage_is_powered()) { fail++; continue; }
        sx_delay_ms(20);

        /* 2. the write wakes the flash by itself */
        if (sx_storage_write(FLASHTEST_PATH, wbuf, FLASHTEST_LEN) != SX_STORAGE_OK) { fail++; continue; }

        /* 3. off again, then read back through another automatic wake */
        sx_storage_sleep();
        sx_delay_ms(20);
        memset(rbuf, 0, sizeof(rbuf));
        if (sx_storage_read(FLASHTEST_PATH, rbuf, FLASHTEST_LEN) != SX_STORAGE_OK ||
            memcmp(wbuf, rbuf, FLASHTEST_LEN) != 0) { fail++; continue; }

        pass++;
    }
    sx_storage_delete(FLASHTEST_PATH);

    _respondf("+FLASHTEST: loops=%d pass=%d fail=%d", n, pass, fail);
    _respond(fail == 0 ? AT_RESP_OK : AT_RESP_ERROR);
    return fail == 0 ? 0 : -1;
}

static int _at_imupwr_q(AT_Command_t *cmd)
{
    _respondf("+IMUPWR: %d", sx_board_imu_is_on() ? 1 : 0);
    _respond(AT_RESP_OK);
    return 0;
}

static int _at_imupwr_set(AT_Command_t *cmd, const char *param)
{
    int v = _parse01(param);
    if (v < 0) { _respond(AT_RESP_ERROR); return -1; }

    int rc = (v == 0) ? sx_board_imu_off() : sx_board_imu_on();
    if (v == 1 && rc == 0) {
        bool cal = imu_calib_load();          /* false is normal if no calib file was ever saved */
        _respondf("+IMUPWR: calib=%d", cal ? 1 : 0);
    }
    _respondf("+IMUPWR: %d rc=%d", sx_board_imu_is_on() ? 1 : 0, rc);
    _respond(rc == 0 ? AT_RESP_OK : AT_RESP_ERROR);
    return rc;
}

static int _at_i2cpwr_q(AT_Command_t *cmd)
{
    _respondf("+I2CPWR: %d", sx_board_i2c1_is_on() ? 1 : 0);
    _respond(AT_RESP_OK);
    return 0;
}

static int _at_i2cpwr_set(AT_Command_t *cmd, const char *param)
{
    int v = _parse01(param);
    if (v < 0) { _respond(AT_RESP_ERROR); return -1; }

    int rc = (v == 0) ? sx_board_i2c1_off() : sx_board_i2c1_on();
    _respondf("+I2CPWR: %d rc=%d", sx_board_i2c1_is_on() ? 1 : 0, rc);
    _respond(rc == 0 ? AT_RESP_OK : AT_RESP_ERROR);
    return rc;
}

static int _at_bqread_exec(AT_Command_t *cmd)
{
    if (!sx_board_i2c1_is_on()) {
        _respondf("+BQREAD: I2C1 is off");
        _respond(AT_RESP_ERROR);
        return -1;
    }
    int rc = bq25622_read_status(&board.bq);
    if (rc != 0) {
        _respondf("+BQREAD: FAIL rc=%d", rc);
        _respond(AT_RESP_ERROR);
        return -1;
    }
    _respondf("+BQREAD: VBUS_STAT=%u CHG_STAT=%u", (unsigned)board.bq.vbus_stat, (unsigned)board.bq.chg_stat);
    _respond(AT_RESP_OK);
    return 0;
}

static AT_Command_t s_commands[NUMBER_COMMAND] = {
    [AT] = {
    .command = CMD_AT,
    .handler = {
        .execute_handler  = _at_execute,  
        .question_handler = NULL,
        .set_handler      = NULL,
    }
    },
    [AT_VPN] = {
        .command = CMD_AT_VPN,
        .handler = {
            .set_handler      = _at_vpn_set,  
            .question_handler = NULL,
            .execute_handler  = NULL,
        }
    },
    [AT_MQTTCONNECT] = {
        .command = CMD_AT_MQTT,
        .handler = {
            .set_handler      = _at_mqtt_set, 
            .question_handler = NULL,
            .execute_handler  = NULL,
        }
    },
    [AT_TIMESLEEP] = {
        .command = CMD_AT_TIMESLEEP, 
        .handler = {
            .set_handler      = _at_timesleep_set, 
            .question_handler = NULL,
            .execute_handler  = NULL,
        }
    },
    [AT_FLASHPWR] = {
        .command = CMD_AT_FLASHPWR,
        .handler = { .set_handler = _at_flashpwr_set, .question_handler = _at_flashpwr_q, .execute_handler = NULL }
    },
    [AT_FLASHTEST] = {
        .command = CMD_AT_FLASHTEST,
        .handler = { .set_handler = _at_flashtest_set, .question_handler = NULL, .execute_handler = NULL }
    },
    [AT_IMUPWR] = {
        .command = CMD_AT_IMUPWR,
        .handler = { .set_handler = _at_imupwr_set, .question_handler = _at_imupwr_q, .execute_handler = NULL }
    },
    [AT_I2CPWR] = {
        .command = CMD_AT_I2CPWR,
        .handler = { .set_handler = _at_i2cpwr_set, .question_handler = _at_i2cpwr_q, .execute_handler = NULL }
    },
    [AT_BQREAD] = {
        .command = CMD_AT_BQREAD,
        .handler = { .set_handler = NULL, .question_handler = NULL, .execute_handler = _at_bqread_exec }
    },
};

static AT_Implementation_t s_at_impl;

void app_at_init(void) {
    at_init(&s_at_impl, s_commands, NUMBER_COMMAND);
    log_info(TAG, "AT command init OK");
}

void app_at_process(const char *data, size_t len) {
    at_process_input(&s_at_impl, data, len);
}