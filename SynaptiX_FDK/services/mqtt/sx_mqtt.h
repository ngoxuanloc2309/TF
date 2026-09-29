#ifndef __SX_MQTT_H
#define __SX_MQTT_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

#define SX_MQTT_TOPIC_MAX_LEN       128U
#define SX_MQTT_MESSAGE_MAX_LEN     512U
#define SX_MQTT_TIMEOUT_CONF        2000U
#define SX_MQTT_TIMEOUT_ACCQ        2000U
#define SX_MQTT_TIMEOUT_CONN        8000U
#define SX_MQTT_TIMEOUT_DISC        5000U
#define SX_MQTT_TIMEOUT_PUB         3000U
#define SX_MQTT_TIMEOUT_SUB         5000U

typedef enum{
    SX_MQTT_STATE_DISCONNECTED = 0,
    SX_MQTT_STATE_CONFIGURING,   /* AT+CMQTTSTART  */
    SX_MQTT_STATE_CERT_CA,          // upload CA cert
    SX_MQTT_STATE_CERT_CLIENT,      // upload client cert
    SX_MQTT_STATE_CERT_KEY,         // upload client key
    SX_MQTT_STATE_CERT_CONTENT,
    SX_MQTT_STATE_SSL_CFG,
    SX_MQTT_STATE_ACQUIRING,     /* AT+CMQTTACCQ   */
    SX_MQTT_STATE_CONNECTING,    /* AT+CMQTTCONNECT */
    SX_MQTT_STATE_CONNECTED,
    SX_MQTT_STATE_DISCONNECTING,
    SX_MQTT_STATE_ERROR,
} sx_mqtt_state_t;

typedef struct sx_mqtt sx_mqtt_t;

typedef enum {
    URC_IDLE = 0,
    URC_GOT_START,          /* +CMQTTRXSTART received */
    URC_GOT_TOPIC_HDR,      /* +CMQTTRXTOPIC received  */
    URC_GOT_PAYLOAD_HDR,    /* +CMQTTRXPAYLOAD received */
} sx_mqtt_urc_state_t;

typedef void (*sx_mqtt_on_connected_cb_t) (sx_mqtt_t *mqtt);

typedef void (*sx_mqtt_on_disconnected_cb_t) (sx_mqtt_t *mqtt);

typedef void (*sx_mqtt_on_message_cb_t) (sx_mqtt_t *mqtt, const char *topic, const char *message);

typedef void (*sx_mqtt_on_publish_cb_t) (sx_mqtt_t *mqtt, int success);

typedef struct{
    const char *broker;
    uint16_t port;
    const char *client_id;
    const char *username;
    const char *password;
    uint16_t keepalive;
    uint8_t clean_session;
    uint8_t qos;
    uint8_t use_ssl;        //0 : tcp, 1: ssl
    uint8_t ssl_auth_mode; // 0 : no verify, 1 : sever cert, 2 = mutual
    char *ca_cert;        
    char *client_cert;    
    char *client_key;
}sx_mqtt_config_t;

typedef enum{
    CONF_CLIENTID = 0,
    CONF_URL,
    CONF_KEEPTIME,
    CONF_CLEANSS,
    CONF_USERNAME,
    CONF_PASSWORD,
    CONF_DONE,
} sx_mqtt_conf_step_t;

struct sx_mqtt{
    void *dce;
    sx_mqtt_config_t config;
    sx_mqtt_state_t state;
    
    sx_mqtt_urc_state_t urc_state;
    char urc_topic [SX_MQTT_TOPIC_MAX_LEN];
    char urc_payload [SX_MQTT_MESSAGE_MAX_LEN];

    uint32_t reconnect_elapsed;
    uint32_t reconnect_delay;
    uint8_t  reconnect_count;

    /*  Callback    */
    sx_mqtt_on_connected_cb_t on_connected;
    sx_mqtt_on_disconnected_cb_t on_disconnected;
    sx_mqtt_on_message_cb_t on_message;
    sx_mqtt_on_publish_cb_t on_publish;
};

/*  Status helper   */
static inline sx_mqtt_state_t sx_mqtt_get_state(sx_mqtt_t *mqtt){
    return mqtt->state;
}

static inline uint8_t sx_mqtt_is_connected(sx_mqtt_t *mqtt){
    return (mqtt->state == SX_MQTT_STATE_CONNECTED);
}

/*  Callback    */
static inline void sx_mqtt_set_on_connected(sx_mqtt_t *mqtt, sx_mqtt_on_connected_cb_t cb){
    mqtt->on_connected = cb;
}

static inline void sx_mqtt_set_on_disconnected(sx_mqtt_t *mqtt, sx_mqtt_on_disconnected_cb_t cb){
    mqtt->on_disconnected = cb;
}

static inline void sx_mqtt_set_on_publish(sx_mqtt_t *mqtt, sx_mqtt_on_publish_cb_t cb){
    mqtt->on_publish = cb;
}

static inline void sx_mqtt_set_on_message(sx_mqtt_t *mqtt, sx_mqtt_on_message_cb_t cb){
    mqtt->on_message = cb;
}

static inline void sx_mqtt_force_disconnect(sx_mqtt_t *mqtt) {
    mqtt->state           = SX_MQTT_STATE_DISCONNECTED;
    mqtt->reconnect_count = 0;
    mqtt->reconnect_elapsed = 0;
}

/*  API */
void sx_mqtt_init(sx_mqtt_t *mqtt, void *dce);
void sx_mqtt_set_config(sx_mqtt_t *mqtt, const sx_mqtt_config_t *config);
int sx_mqtt_connect(sx_mqtt_t *mqtt);
int sx_mqtt_disconnect(sx_mqtt_t *mqtt);
int sx_mqtt_publish(sx_mqtt_t *mqtt, const char *topic, const char *message, uint8_t qos, uint8_t retain);
int sx_mqtt_subscribe(sx_mqtt_t *mqtt, const char *topic, uint8_t qos);
void sx_mqtt_poll(sx_mqtt_t *mqtt, uint32_t ts);

#ifdef __cplusplus
}
#endif

#endif