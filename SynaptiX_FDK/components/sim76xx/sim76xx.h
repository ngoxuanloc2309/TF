#ifndef SIM70XX_H
#define SIM70XX_H
#ifdef __cplusplus
extern "C" {
#endif

#include "modem.h"
#include <stdint.h>

/*  Retry and timeout config  */
#define SIM76XX_MAX_RETRY           3
#define SIM76XX_TIMEOUT_AT          2500U
#define SIM76XX_TIMEOUT_NETWORK     10000U
#define SIM76XX_TIMEOUT_NETOPEN     10000U
#define SIM76XX_TIMEOUT_CFUN        15000U 

/*  Init state machine  */
typedef enum {
    SIM76XX_STATE_IDLE = 0,
    SIM76XX_STATE_AT,
    SIM76XX_STATE_CGSN,
    SIM76XX_STATE_COPS_QUERY,
    SIM76XX_STATE_COPS_SET,
    SIM76XX_STATE_CSQ,
    SIM76XX_STATE_CFUN,
    SIM76XX_STATE_CGATT,
    SIM76XX_STATE_CGDCONT,
    SIM76XX_STATE_CGAUTH,
    SIM76XX_STATE_CGREG,
    SIM76XX_STATE_NETOPEN,
    SIM76XX_STATE_IPADDR,        
    SIM76XX_STATE_READY,
    SIM76XX_STATE_ERROR,
    SIM76XX_STATE_CGDCONT_QUERY,
} sim76xx_state_t;

typedef struct sim76xx sim76xx_t;

typedef void (*sim76xx_on_ready_cb_t)(sim76xx_t *dce);

typedef void (*sim76xx_on_error_cb_t)(sim76xx_t *dce);

typedef void (*sim76xx_on_urc_cb_t)(sim76xx_t *dce, const char *urc_line);

struct sim76xx
{
    /* data */
    modem_t base;

    sim76xx_state_t state;
    uint8_t retry_count;

    int rssi;
    int ber;
    char imei[16];
    char ip[16];
    char apn[32];

    char username[32];
    char password[32];

    sim76xx_on_ready_cb_t on_ready;
    sim76xx_on_error_cb_t on_error;
    sim76xx_on_urc_cb_t   on_urc;
};

void sim76xx_init(sim76xx_t *dce);
void sim76xx_power_on(sim76xx_t *dce);
void sim76xx_power_off(sim76xx_t *dce);
void sim76xx_reset(sim76xx_t *dce);

int sim76xx_start(sim76xx_t *dce);
void sim76xx_poll(sim76xx_t *dce, uint32_t ts);

int sim76xx_mqtt_stop(sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms);

//Sleep mode
int sim76xx_psm_enable(sim76xx_t *dce, uint32_t tau_sec, uint32_t active_sec);
int sim76xx_psm_disable(sim76xx_t *dce);
void sim76xx_wake(sim76xx_t *dce);
void sim76xx_set_full_apn(sim76xx_t *dce, char* apn, char* user, char* pass);

static inline void sim76xx_set_on_ready(sim76xx_t *dce, sim76xx_on_ready_cb_t cb){
    dce->on_ready = cb;
}

static inline void sim76xx_set_on_error(sim76xx_t *dce, sim76xx_on_error_cb_t cb){
    dce->on_error = cb;
}

static inline void sim76xx_set_on_urc(sim76xx_t *dce, sim76xx_on_urc_cb_t cb){
    dce->on_urc = cb;
}

static inline uint8_t sim76xx_is_ready(sim76xx_t *dce){
    return (dce->state == SIM76XX_STATE_READY);
}

static inline sim76xx_state_t sim76xx_get_state(sim76xx_t *dce){
    return dce->state;
}

static inline const char *sim76xx_get_imei(sim76xx_t *dce){
    return dce->imei;
}

static inline const char *sim76xx_get_ip(sim76xx_t *dce){
    return dce->ip;
}

static inline int sim76xx_get_rssi(sim76xx_t *dce){
    return dce->rssi;
}

static inline void sim76xx_set_apn(sim76xx_t *dce, const char *apn) {
    strncpy(dce->apn, apn, sizeof(dce->apn) - 1);
    dce->apn[sizeof(dce->apn) - 1] = '\0';
}

static inline const char *sim76xx_get_apn(sim76xx_t *dce){
    return dce->apn;
}

/*  API help MQTT   */
int sim76xx_mqtt_start     (sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms);
int sim76xx_mqtt_accq(sim76xx_t *dce, const char *client_id, uint8_t use_ssl, modem_command_response_callback_t cb, uint32_t timeout_ms);
int sim76xx_mqtt_connect   (sim76xx_t *dce, const char *broker, uint16_t port, uint8_t use_ssl, uint16_t keepalive, uint8_t clean_session, const char *username, const char *password, modem_command_response_callback_t cb, uint32_t timeout_ms);
int sim76xx_mqtt_disconnect(sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms);
int sim76xx_mqtt_publish   (sim76xx_t *dce, const char *topic, const char *message, uint8_t qos, uint8_t retain, modem_command_response_callback_t cb, uint32_t timeout_ms);
int sim76xx_mqtt_subscribe (sim76xx_t *dce, const char *topic, uint8_t qos, modem_command_response_callback_t cb, uint32_t timeout_ms);
int sim76xx_mqtt_ssl_cfg (sim76xx_t *dce, uint8_t auth_mode, modem_command_response_callback_t cb, uint32_t timeout_ms);
int sim76xx_mqtt_ssl_bind(sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms);

int sim76xx_mqtt_raw_send(sim76xx_t *dce, const char *data, uint32_t size, const char *res_success,
                               modem_command_response_callback_t cb, uint32_t timeout_ms);

int sim76xx_mqtt_send_raw_cmd(sim76xx_t *dce, const char *cmd_str, const char *res_success,
                               modem_command_response_callback_t cb, uint32_t timeout_ms);

int sim76xx_cert_download(sim76xx_t *dce, const char *filename, const char *content,
                          uint32_t size, modem_command_response_callback_t cb, uint32_t timeout_ms);

int sim76xx_ssl_set_cert(sim76xx_t *dce, const char *ca_cert_name, const char *client_cert_name,
                         const char *client_key_name, modem_command_response_callback_t cb, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif // SIM70XX_H