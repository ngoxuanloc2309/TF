#include "sx_board.h"
#include "sx_user_mqtt.h"
#include "sx_gpio.h"
#include "sx_uart.h"
#include "sx_mqtt.h"
#include "usart.h"
#include "logger.h"
#include "string.h"
#include "cqueue.h"

static const char *TAG = "USER_MQTT";

#define MQTT_QUEUE_SIZE 16
#define MQTT_PUBLISH_MAX_RETRY 3

volatile sx_mqtt_t s_mqtt;

typedef struct {
    char topic[128];
    char message[256];
} mqtt_queue_item_t;

static mqtt_queue_item_t s_queue_buf[MQTT_QUEUE_SIZE];
static CQueue_t s_queue;

static mqtt_queue_item_t s_current_item;
static uint8_t s_publish_retry = 0;

static sx_user_mqtt_on_connected_cb_t    s_on_connected    = NULL;
static sx_user_mqtt_on_disconnected_cb_t s_on_disconnected = NULL;
static sx_user_mqtt_on_message_cb_t      s_on_message      = NULL;
static sx_user_mqtt_on_publish_cb_t      s_on_publish      = NULL;
static char s_client_id_buf[32];

// Poll timing
static uint32_t s_last_tick = 0;
static uint8_t s_initialized = 0;
static uint8_t s_publishing = 0;

static volatile uint8_t s_stop_done = 0;

static void dispatch_next(void){
    if (!cqueue_is_empty(&s_queue) && sx_mqtt_is_connected(&s_mqtt)) {
        if (cqueue_receive(&s_queue, &s_current_item)) {
            s_publishing = 1;
            sx_mqtt_publish(&s_mqtt, s_current_item.topic, s_current_item.message, 1, 0);
        }
    }
}

static void _on_connected(sx_mqtt_t *mqtt){
    (void)mqtt;
    log_info(TAG, "MQTT connected");
    if (s_on_connected) s_on_connected();
}

static void _on_disconnected(sx_mqtt_t *mqtt){
    (void)mqtt;
    log_warn(TAG, "MQTT disconnected");
    if (s_on_disconnected) s_on_disconnected();
}

static void _on_message(sx_mqtt_t *mqtt, const char *topic, const char *message){
    (void)mqtt;
    log_info(TAG, "MSG [%s] = %s", topic, message);
    if (s_on_message) s_on_message(topic, message);
}

// static void _on_publish(sx_mqtt_t *mqtt, int success){
//     (void)mqtt;
//     s_publishing = 0;
//     log_debug(TAG, "Publish %s", success ? "OK" : "FAIL");
//     if (s_on_publish) s_on_publish(success);

//     if (!success) {
//         s_publish_retry++;
//         log_warn(TAG, "Publish fail %d/%d", s_publish_retry, MQTT_PUBLISH_MAX_RETRY);
//         if (s_publish_retry >= MQTT_PUBLISH_MAX_RETRY) {
//             log_error(TAG, "Max retry — restart modem");
//             s_publish_retry = 0;
//             cqueue_init_static(&s_queue, s_queue_buf, MQTT_QUEUE_SIZE, sizeof(mqtt_queue_item_t));
//             sim76xx_start(&board.sim76xx);
//             return;
//         }
//         s_publishing = 1;
//         sx_mqtt_publish(&s_mqtt, s_current_item.topic, s_current_item.message, 1, 0);
//         return;
//     }

//     s_publish_retry = 0;
//     dispatch_next();
// }

// static void _on_publish(sx_mqtt_t *mqtt, int success){
//     (void)mqtt;
//     s_publishing = 0;
//     log_debug(TAG, "Publish %s", success ? "OK" : "FAIL");

//     if (!success) {
//         s_publish_retry++;
//         log_warn(TAG, "Publish fail %d/%d", s_publish_retry, MQTT_PUBLISH_MAX_RETRY);
//         if (s_publish_retry >= MQTT_PUBLISH_MAX_RETRY) {
//             log_error(TAG, "Max retry — restart modem");
//             s_publish_retry = 0;
//             cqueue_init_static(&s_queue, s_queue_buf, MQTT_QUEUE_SIZE,
//                                sizeof(mqtt_queue_item_t));
//             if (s_on_publish) s_on_publish(0);
//             sim76xx_start(&board.sim76xx);
//             return;
//         }
//         s_publishing = 1;
//         sx_mqtt_publish(&s_mqtt, s_current_item.topic,
//                         s_current_item.message, 1, 0);
//         return;
//     }

//     // Success
//     s_publish_retry = 0;
//     if (s_on_publish) s_on_publish(1);  
//     dispatch_next();
// }

static void _on_publish(sx_mqtt_t *mqtt, int success)
{
    (void)mqtt;
    s_publishing = 0;           
    s_publish_retry = 0;

    log_debug(TAG, "Publish %s", success ? "OK" : "FAIL");

    if (!success) {
        s_publish_retry++;
        log_warn(TAG, "Publish fail %d/%d", s_publish_retry, MQTT_PUBLISH_MAX_RETRY);
        if (s_publish_retry >= MQTT_PUBLISH_MAX_RETRY) {
            log_error(TAG, "Max retry — restart modem");
            s_publish_retry = 0;
            cqueue_init_static(&s_queue, s_queue_buf, MQTT_QUEUE_SIZE,
                               sizeof(mqtt_queue_item_t));
            if (s_on_publish) s_on_publish(0);
            sim76xx_start(&board.sim76xx);
            return;
        }
        s_publishing = 1;
        sx_mqtt_publish(&s_mqtt, s_current_item.topic,
                        s_current_item.message, 1, 0);
        return;
    }

    dispatch_next();                    
    if (s_on_publish) s_on_publish(1);  
}

static void _on_modem_ready(sim76xx_t *dce){
    const char *imei = sim76xx_get_imei(dce);
    size_t imei_len = strlen(imei);
    const char *imei_suffix = imei_len >= 8 ? imei + imei_len - 8 : imei;
    snprintf(s_client_id_buf, sizeof(s_client_id_buf), "stm32h5_%s", imei_suffix);

    s_mqtt.config.client_id = s_client_id_buf;

    log_info(TAG, "Modem ready — IP=%s RSSI=%d client_id=%s",
             sim76xx_get_ip(&board.sim76xx), sim76xx_get_rssi(&board.sim76xx), s_client_id_buf);
    //sx_delay_ms(2000);
    sx_mqtt_connect(&s_mqtt);
    log_info(TAG, "Modem ready — IP=%s RSSI=%d", sim76xx_get_ip(&board.sim76xx), sim76xx_get_rssi(&board.sim76xx));
}

static void _on_modem_error(sim76xx_t *dce){
    (void)dce;
    log_error(TAG, "Modem error");
    if (s_on_disconnected) s_on_disconnected();
}

static int _validate_cfg(const sx_user_mqtt_cfg_t *cfg)
{
    // if (!cfg->apn)       { log_error(TAG, "Missing APN");      return -1; }
    if (!cfg->broker)    { log_error(TAG, "Missing broker");    return -1; }
    if (!cfg->port)      { log_error(TAG, "Missing port");      return -1; }
    if (!cfg->client_id) { log_error(TAG, "Missing client_id"); return -1; }
    return 0;
}

static int _common_init(const sx_user_mqtt_cfg_t *cfg, sx_mqtt_config_t *mqtt_cfg){
    if (_validate_cfg(cfg) != 0) return -1;
    /* save user callbacks */
    cqueue_init_static(&s_queue, s_queue_buf, MQTT_QUEUE_SIZE, sizeof(mqtt_queue_item_t));
    s_on_connected    = cfg->on_connected;
    s_on_disconnected = cfg->on_disconnected;
    s_on_message      = cfg->on_message;
    s_on_publish      = cfg->on_publish;

    // sim76xx_init(&board.sim76xx);
    // sim76xx_power_on(&board.sim76xx);
    sim76xx_set_full_apn(&board.sim76xx, cfg->apn, cfg->username_apn, cfg->password_apn);
    // lte_uart_it_enable();

    /* MQTT config */
    mqtt_cfg->broker        = cfg->broker;
    mqtt_cfg->port          = cfg->port;
    mqtt_cfg->client_id     = cfg->client_id;
    mqtt_cfg->username      = cfg->username;
    mqtt_cfg->password      = cfg->password;
    mqtt_cfg->keepalive     = cfg->keepalive ? cfg->keepalive : 60;
    mqtt_cfg->clean_session = cfg->clean_session;

    sim76xx_set_on_ready (&board.sim76xx, _on_modem_ready);
    sim76xx_set_on_error (&board.sim76xx, _on_modem_error);

    sx_mqtt_init(&s_mqtt, &board.sim76xx);
    sx_mqtt_set_config(&s_mqtt, mqtt_cfg);
    sx_mqtt_set_on_connected   (&s_mqtt, _on_connected);
    sx_mqtt_set_on_disconnected(&s_mqtt, _on_disconnected);
    sx_mqtt_set_on_message     (&s_mqtt, _on_message);
    sx_mqtt_set_on_publish     (&s_mqtt, _on_publish);

    s_last_tick = sx_gettick();
    sim76xx_start(&board.sim76xx);
    log_info(TAG, "Init done — broker=%s:%u ssl=%d", cfg->broker, cfg->port, cfg->use_ssl);

    s_initialized = 1;   //
    return 0;
}

uint8_t sx_user_mqtt_is_connected(void) {
    return sx_mqtt_is_connected(&s_mqtt);
}

int sx_user_mqtt_nontls_init(const sx_user_mqtt_cfg_t *cfg){
    static sx_mqtt_config_t mqtt_cfg;
    memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
    mqtt_cfg.use_ssl = 0;
    return _common_init(cfg, &mqtt_cfg);
}

int sx_user_mqtt_tls_init(const sx_user_mqtt_cfg_t *cfg, char *ca_cert, char *client_cert, char *client_key){
    static sx_mqtt_config_t mqtt_cfg;
    memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
    mqtt_cfg.use_ssl = 1;

    if (client_cert && client_key)
        mqtt_cfg.ssl_auth_mode = 2;
    else if (ca_cert)
        mqtt_cfg.ssl_auth_mode = 1;
    else
        mqtt_cfg.ssl_auth_mode = 0;

    mqtt_cfg.ca_cert     = ca_cert;
    mqtt_cfg.client_cert = client_cert;
    mqtt_cfg.client_key  = client_key;

    return _common_init(cfg, &mqtt_cfg);
}

void sx_user_mqtt_force_disconnect(void) {
    s_mqtt.state           = SX_MQTT_STATE_DISCONNECTED;
    s_mqtt.reconnect_count = 0;
    s_mqtt.reconnect_elapsed = 0;
}

void sx_user_mqtt_publish(const char *topic, const char *message){
    if (!sx_mqtt_is_connected(&s_mqtt)) { log_warn(TAG, "publish: not connected"); return; }

    mqtt_queue_item_t item;
    strncpy(item.topic,   topic,   sizeof(item.topic)   - 1);
    strncpy(item.message, message, sizeof(item.message) - 1);
    item.topic  [sizeof(item.topic)   - 1] = '\0';
    item.message[sizeof(item.message) - 1] = '\0';

    if (!cqueue_send(&s_queue, &item)) {
        log_warn(TAG, "publish queue full, drop: %s", topic);
        return;
    }

    if (!s_publishing) {
        if (cqueue_receive(&s_queue, &s_current_item)) {
            s_publishing = 1;
            sx_mqtt_publish(&s_mqtt, s_current_item.topic, s_current_item.message, 1, 0);
        }
    }
}

void sx_user_mqtt_subscribe(const char *topic){
    if (!sx_mqtt_is_connected(&s_mqtt)) { log_warn(TAG, "subscribe: not connected"); return; }
    if (!topic) { log_warn(TAG, "subscribe: NULL topic"); return; }
    sx_mqtt_subscribe(&s_mqtt, topic, 1);
}

void sx_user_mqtt_stop(void){
    if (sx_mqtt_is_connected(&s_mqtt))
        sx_mqtt_disconnect(&s_mqtt);
}

void sx_user_mqtt_poll(uint32_t time_stamp){
    sx_mqtt_poll(&s_mqtt, time_stamp);
}

uint8_t sx_user_mqtt_is_initialized(void) { return s_initialized; }

static void cb_mqtt_stop_done(modem_t *modem, const char *resp,
                               modem_response_st_t res, void *arg) {
    s_stop_done = 1;
}

static void cb_mqtt_disc_done(modem_t *modem, const char *resp,
                               modem_response_st_t res, void *arg) {
    sim76xx_mqtt_stop(&board.sim76xx, cb_mqtt_stop_done, 3000);
}

void sx_user_mqtt_stop_service(void) {
    s_stop_done = 0;

    if (sx_mqtt_is_connected(&s_mqtt)) {
        sim76xx_mqtt_disconnect(&board.sim76xx, cb_mqtt_disc_done, 5000);
    } else {
        sim76xx_mqtt_stop(&board.sim76xx, cb_mqtt_stop_done, 3000);
    }

    uint32_t wait = 0;
    while (!s_stop_done && wait < 10000) {
        sim76xx_poll(&board.sim76xx, 10);  
        sx_delay_ms(10);
        wait += 10;
    }

    s_mqtt.state             = SX_MQTT_STATE_DISCONNECTED;
    s_mqtt.reconnect_count   = 0;
    s_mqtt.reconnect_elapsed = 0;

    log_info("USER_MQTT", "MQTT service stopped, wait=%lu", wait);
}

uint8_t sx_user_mqtt_is_publishing(void) {
    return s_publishing;
}

uint8_t sx_user_mqtt_queue_empty(void) {
    return cqueue_is_empty(&s_queue);
}

void sx_user_mqtt_queue_flush(void) {
    cqueue_init_static(&s_queue, s_queue_buf, MQTT_QUEUE_SIZE, sizeof(mqtt_queue_item_t));
    s_publishing   = 0;
    s_publish_retry = 0;
    log_info("USER_MQTT", "Queue flushed");
}