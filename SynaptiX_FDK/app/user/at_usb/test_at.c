#include "test_at.h"
#include "logger.h"
#include "sx_board.h"

static const char *TAG = "TEST_AT_USB";

#define NUMBER_COMMAND  4

#define AT              0
#define AT_VPN          1
#define AT_MQTTCONNECT  2
#define AT_TIMESLEEP    3

// const char* at_usb_command[NUMBER_COMMAND] = {"AT", "AT+VPN", "AT+MQTTCONNECT", "AT+TIMESLEEP"};

#define CMD_AT              "AT"
#define CMD_AT_VPN          "AT+VPN"
#define CMD_AT_MQTT         "AT+MQTTCONNECT"
#define CMD_AT_TIMESLEEP    "AT+TIMESLEEP"

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
};

static AT_Implementation_t s_at_impl;

void app_at_init(void) {
    at_init(&s_at_impl, s_commands, NUMBER_COMMAND);
    log_info(TAG, "AT command init OK");
}

void app_at_process(const char *data, size_t len) {
    at_process_input(&s_at_impl, data, len);
}

