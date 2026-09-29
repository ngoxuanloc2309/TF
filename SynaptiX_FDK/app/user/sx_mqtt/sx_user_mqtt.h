#ifndef SX_USER_MQTT_H
#define SX_USER_MQTT_H

#ifdef __cplusplus
extern "C"{
#endif

// #include "sx_mqtt.h"
#include <stdint.h>

typedef void (*sx_user_mqtt_on_connected_cb_t)   (void);
typedef void (*sx_user_mqtt_on_disconnected_cb_t)(void);
typedef void (*sx_user_mqtt_on_message_cb_t)     (const char *topic, const char *message);
typedef void (*sx_user_mqtt_on_publish_cb_t)     (int success);

typedef struct {
    /* GSM */
    char apn[20];
    char username_apn[20];
    char password_apn[20];
    /* Broker */
    const char *broker;
    uint16_t    port;
    const char *client_id;
    const char *username;       /* NULL = skip */
    const char *password;       /* NULL = skip */
    uint16_t    keepalive;      /* seconds, 0 = default 60s */
    uint8_t     clean_session;

    /* TLS */
    uint8_t     use_ssl;        /* 0 = TCP, 1 = SSL */
    uint8_t     ssl_auth_mode;  /* 0=no verify, 1=server cert, 2=mutual */

    /* Callbacks */
    sx_user_mqtt_on_connected_cb_t    on_connected;
    sx_user_mqtt_on_disconnected_cb_t on_disconnected;
    sx_user_mqtt_on_message_cb_t      on_message;
    sx_user_mqtt_on_publish_cb_t      on_publish;
} sx_user_mqtt_cfg_t;

/*  Helper  */
const char *sx_user_mqtt_get_ip      (void);
const char *sx_user_mqtt_get_imei    (void);
int sx_user_mqtt_get_rssi    (void);
uint8_t sx_user_mqtt_is_connected(void);

/*  API */
int sx_user_mqtt_nontls_init(const sx_user_mqtt_cfg_t *cfg);
void sx_user_mqtt_publish(const char *topic, const char *message);
void sx_user_mqtt_subscribe(const char *topic);
void sx_user_mqtt_stop(void);
void sx_user_mqtt_poll(uint32_t time_stamp);
uint8_t sx_user_mqtt_is_initialized(void);
void sx_user_mqtt_uart_feed(uint8_t byte);
int sx_user_mqtt_tls_init(const sx_user_mqtt_cfg_t *cfg, char *ca_cert, char *client_cert, char *client_key);
void sx_user_mqtt_force_disconnect(void);
void sx_user_mqtt_stop_service(void);
uint8_t sx_user_mqtt_is_publishing(void);
uint8_t sx_user_mqtt_queue_empty(void);
void sx_user_mqtt_queue_flush(void);

#ifdef __cplusplus
}
#endif

#endif