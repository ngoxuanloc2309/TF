#include "sx_mqtt.h"
#include "logger.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "sim76xx.h"
#define SX_MQTT_RECONNECT_DELAY_BASE    1000U
#define SX_MQTT_RECONNECT_DELAY_MAX     5000U

static const char *TAG = "SX_MQTT";

static sx_mqtt_t *s_instance = NULL;

static char s_ssl_cmd_buf[128];

static uint8_t s_start_retry = 0;

static void cb_start(modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_version(modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_accq (modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_conn (modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_disc (modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_pub  (modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_sub  (modem_t *modem, const char *resp, modem_response_st_t res, void *arg);

static void cb_sslcfg (modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_sslbind(modem_t *modem, const char *resp, modem_response_st_t res, void *arg);
static void cb_cert_ca_prompt (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_cert_ca_done (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_cert_client_prompt (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_cert_client_done (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_cert_key_prompt (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_cert_key_done (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_ssl_cacert (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_ssl_clientcert (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_ssl_clientkey (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void cb_ssl_authmode (modem_t *m, const char *r, modem_response_st_t res, void *arg);
static void goto_ssl_cfg(sx_mqtt_t *mqtt);
static void cb_start_after_stop(modem_t *modem, const char *resp, modem_response_st_t res, void *arg);

static void urc_handler(sim76xx_t *dce, const char *line);
static void do_error(sx_mqtt_t *mqtt);
/* 
 * URC handler — reassemble +CMQTTRX* multi-line sequence
 *
 *   +CMQTTRXSTART: 0,<topic_len>,<payload_len>
 *   +CMQTTRXTOPIC: 0,<len>\r\n<topic>
 *   +CMQTTRXPAYLOAD: 0,<len>\r\n<payload>
 *   +CMQTTRXEND: 0
 *  */

// static void process_urc_line(sx_mqtt_t *mqtt, const char *line, const char *data_next){
//     if (!line || *line == '\0') return;

//     if (strstr(line, "+CMQTTRXSTART:")) {
//         mqtt->urc_state = URC_GOT_START;
//         memset(mqtt->urc_topic,   0, sizeof(mqtt->urc_topic));
//         memset(mqtt->urc_payload, 0, sizeof(mqtt->urc_payload));
//         return;
//     }

//     if (mqtt->urc_state == URC_GOT_START &&
//         strstr(line, "+CMQTTRXTOPIC:"))
//     {
//         if (data_next) {
//             strncpy(mqtt->urc_topic, data_next, sizeof(mqtt->urc_topic) - 1);
//             // trim \r\n
//             char *end = mqtt->urc_topic + strlen(mqtt->urc_topic) - 1;
//             while (end >= mqtt->urc_topic && (*end == '\r' || *end == '\n'))
//                 *end-- = '\0';
//         }
//         mqtt->urc_state = URC_GOT_TOPIC_HDR;
//         return;
//     }

//     if (mqtt->urc_state == URC_GOT_TOPIC_HDR &&
//         strstr(line, "+CMQTTRXPAYLOAD:"))
//     {
//         if (data_next) {
//             strncpy(mqtt->urc_payload, data_next, sizeof(mqtt->urc_payload) - 1);
//             char *end = mqtt->urc_payload + strlen(mqtt->urc_payload) - 1;
//             while (end >= mqtt->urc_payload && (*end == '\r' || *end == '\n'))
//                 *end-- = '\0';
//         }
//         mqtt->urc_state = URC_GOT_PAYLOAD_HDR;
//         return;
//     }

//     if (mqtt->urc_state == URC_GOT_PAYLOAD_HDR &&
//         strstr(line, "+CMQTTRXEND:"))
//     {
//         log_debug(TAG, "RX topic='%s' payload='%s'",
//                   mqtt->urc_topic, mqtt->urc_payload);
//         if (mqtt->on_message)
//             mqtt->on_message(mqtt, mqtt->urc_topic, mqtt->urc_payload);
//         mqtt->urc_state = URC_IDLE;
//         return;
//     }

//     if (strstr(line, "+CMQTTCONNLOST:")) {
//         log_warn(TAG, "Connection lost");
//         mqtt->state     = SX_MQTT_STATE_DISCONNECTED;
//         mqtt->urc_state = URC_IDLE;
//         if (mqtt->on_disconnected) mqtt->on_disconnected(mqtt);
//         return;
//     }
// }

static void process_urc_line(sx_mqtt_t *mqtt, const char *line, const char *data_next){
    if (!line || *line == '\0') return;

    if (strstr(line, "+CMQTTRXSTART:")) {
        mqtt->urc_state = URC_GOT_START;
        memset(mqtt->urc_topic,   0, sizeof(mqtt->urc_topic));
        memset(mqtt->urc_payload, 0, sizeof(mqtt->urc_payload));
        return;
    }

    if (mqtt->urc_state == URC_GOT_START &&
        strstr(line, "+CMQTTRXTOPIC:"))
    {
        if (data_next) {
            char *end = strstr(data_next, "\r\n");
            size_t len = end ? (size_t)(end - data_next) : strlen(data_next);
            if (len >= sizeof(mqtt->urc_topic)) len = sizeof(mqtt->urc_topic) - 1;
            memcpy(mqtt->urc_topic, data_next, len);
            mqtt->urc_topic[len] = '\0';
        }
        mqtt->urc_state = URC_GOT_TOPIC_HDR;
        return;
    }

    if (mqtt->urc_state == URC_GOT_TOPIC_HDR &&
        strstr(line, "+CMQTTRXPAYLOAD:"))
    {
        if (data_next) {
            char *end = strstr(data_next, "\r\n");
            size_t len = end ? (size_t)(end - data_next) : strlen(data_next);
            if (len >= sizeof(mqtt->urc_payload)) len = sizeof(mqtt->urc_payload) - 1;
            memcpy(mqtt->urc_payload, data_next, len);
            mqtt->urc_payload[len] = '\0';
        }
        mqtt->urc_state = URC_GOT_PAYLOAD_HDR;
        return;
    }

    if (mqtt->urc_state == URC_GOT_PAYLOAD_HDR &&
        strstr(line, "+CMQTTRXEND:"))
    {
        log_debug(TAG, "RX topic='%s' payload='%s'",
                  mqtt->urc_topic, mqtt->urc_payload);
        if (mqtt->on_message)
            mqtt->on_message(mqtt, mqtt->urc_topic, mqtt->urc_payload);
        mqtt->urc_state = URC_IDLE;
        return;
    }

    if (strstr(line, "+CMQTTCONNLOST:")) {
        log_warn(TAG, "Connection lost");
        mqtt->state     = SX_MQTT_STATE_DISCONNECTED;
        mqtt->urc_state = URC_IDLE;
        if (mqtt->on_disconnected) mqtt->on_disconnected(mqtt);
        return;
    }
}

static void urc_handler(sim76xx_t *dce, const char *buf){
    (void)dce;
    if (!s_instance || !buf) return;
    sx_mqtt_t *mqtt = s_instance;

    if (strstr(buf, "+CGEV: ME PDN DEACT") || strstr(buf, "+CGEV: NW PDN DEACT")) {
        log_warn(TAG, "PDN lost — restarting modem");
        mqtt->state = SX_MQTT_STATE_DISCONNECTED;
        mqtt->reconnect_elapsed = 0;
        mqtt->reconnect_count = 0;
        sim76xx_start((sim76xx_t *)mqtt->dce);
        if (mqtt->on_disconnected) mqtt->on_disconnected(mqtt);
        return;
    }

    static char tmp[MODEM_RX_BUFFER_SIZE];
    strncpy(tmp, buf, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    char *line = tmp;
    while (line && *line) {
        char *eol = strstr(line, "\r\n");
        char *data_next = NULL;

        if (eol) {
            *eol      = '\0';
            data_next = eol + 2; 
        }

        process_urc_line(mqtt, line, data_next);

        // Advance - ,if this line is a header with data on the next line
        // skip gain still one line (data processed internal)
        if (data_next) {
            char *next_eol = strstr(data_next, "\r\n");
            if (next_eol) {
                line = next_eol + 2;
            } else {
                break;
            }
        } else {
            break;
        }
    }
}

/* 
 * Connect flow:
 *   cb_start -> cb_accq -> cb_conn
 *  */

static void cb_start_after_stop(modem_t *modem, const char *resp, 
                                 modem_response_st_t res, void *arg) {
    sx_mqtt_t *mqtt = s_instance;
    sim76xx_mqtt_start(mqtt->dce, cb_start, SX_MQTT_TIMEOUT_CONF);
}


static void cb_start(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    log_info(TAG, "CMQTTSTART response: [%s] res=%d",  resp ? resp : "NULL", res);
    if (res != MODEM_RESPONSE_SUCCESS) {
        if (s_start_retry == 0) {
            s_start_retry = 1;
            log_warn(TAG, "CMQTTSTART fail — try STOP and Start again!");
            sim76xx_mqtt_stop(mqtt->dce, cb_start_after_stop, 3000);
            return;
        }
        s_start_retry = 0;
        log_error(TAG, "CMQTTSTART failed sau retry");
        do_error(mqtt);
        return;
    }
    s_start_retry = 0;
    log_debug(TAG, "CMQTTSTART OK");
    if (mqtt->config.use_ssl && mqtt->config.ca_cert) {
        // Upload CA cert trước
        log_info(TAG, "Uploading CA cert...");
        mqtt->state = SX_MQTT_STATE_CERT_CA;
        sim76xx_cert_download(mqtt->dce, "cacert.crt",
                              mqtt->config.ca_cert, strlen(mqtt->config.ca_cert), cb_cert_ca_prompt, SX_MQTT_TIMEOUT_CONF);
    } else if (mqtt->config.use_ssl) {
        // SSL no cert -> enter SSL cfg
        mqtt->state = SX_MQTT_STATE_SSL_CFG;
        sim76xx_mqtt_ssl_cfg(mqtt->dce, mqtt->config.ssl_auth_mode, cb_sslcfg, SX_MQTT_TIMEOUT_CONF);
    } else {
        // TCP 
        mqtt->state = SX_MQTT_STATE_ACQUIRING;
        sim76xx_mqtt_accq(mqtt->dce, mqtt->config.client_id, mqtt->config.use_ssl, cb_accq, SX_MQTT_TIMEOUT_ACCQ);
    }
}

static void cb_version(modem_t *modem, const char *resp, modem_response_st_t res, void *arg) {
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_warn(TAG, "CMQTTCFG version failed — continuing anyway");
    }
    mqtt->state = SX_MQTT_STATE_CONNECTING;
    sim76xx_mqtt_connect(mqtt->dce,
                         mqtt->config.broker, mqtt->config.port,
                         mqtt->config.use_ssl, mqtt->config.keepalive,
                         mqtt->config.clean_session,
                         mqtt->config.username, mqtt->config.password,
                         cb_conn, SX_MQTT_TIMEOUT_CONN);
}

// static void cb_accq(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
//     (void)modem; (void)arg;
//     sx_mqtt_t *mqtt = s_instance;

//     if (res != MODEM_RESPONSE_SUCCESS) {
//         log_error(TAG, "CMQTTACCQ failed");
//         do_error(mqtt);
//         return;
//     }
//     log_debug(TAG, "CMQTTACCQ OK");
//     mqtt->state = SX_MQTT_STATE_CONNECTING;
//     sim76xx_mqtt_connect(mqtt->dce, mqtt->config.broker,mqtt->config.port,mqtt->config.use_ssl, mqtt->config.keepalive,
//                         mqtt->config.clean_session, mqtt->config.username, mqtt->config.password, cb_conn, SX_MQTT_TIMEOUT_CONN);
// }

static void cb_accq(modem_t *modem, const char *resp, modem_response_st_t res, void *arg) {
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "CMQTTACCQ failed");
        do_error(mqtt);
        return;
    }
    log_debug(TAG, "CMQTTACCQ OK — setting MQTT version 3.1.1");
    sim76xx_mqtt_send_raw_cmd(mqtt->dce,
                              "AT+CMQTTCFG=\"version\",0,4\r",
                              "\r\nOK\r\n",
                              cb_version,
                              SX_MQTT_TIMEOUT_CONF);
}

static void cb_conn(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;

        if (res != MODEM_RESPONSE_SUCCESS) {
            if (res == MODEM_RESPONSE_TIMEOUT)
                log_error(TAG, "CMQTTCONNECT TIMEOUT — response: %s", resp ? resp : "NULL");
            else
                log_error(TAG, "CMQTTCONNECT ERROR — response: %s", resp ? resp : "NULL");
        do_error(mqtt);
        return;
    }
    log_info(TAG, "MQTT connected");
    mqtt->state = SX_MQTT_STATE_CONNECTED;
    if (mqtt->on_connected) mqtt->on_connected(mqtt);
}

static void cb_cert_ca_prompt(modem_t *modem, const char *resp,
                               modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "CCERTDOWN CA prompt failed");
        do_error(mqtt);
        return;
    }
    sim76xx_mqtt_raw_send(mqtt->dce,
                          mqtt->config.ca_cert,
                          strlen(mqtt->config.ca_cert),
                          "\r\nOK\r\n",cb_cert_ca_done, SX_MQTT_TIMEOUT_CONF);
}

static void cb_cert_ca_done(modem_t *modem, const char *resp,
                             modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "CA cert upload failed");
        do_error(mqtt);
        return;
    }
    log_info(TAG, "CA cert uploaded OK");

    if (mqtt->config.ssl_auth_mode == 2 && mqtt->config.client_cert) {
        mqtt->state = SX_MQTT_STATE_CERT_CLIENT;
        sim76xx_cert_download(mqtt->dce,
                              "client.crt",
                              mqtt->config.client_cert,
                              strlen(mqtt->config.client_cert),
                              cb_cert_client_prompt,
                              SX_MQTT_TIMEOUT_CONF);
    } else {
        goto_ssl_cfg(mqtt);
    }
}

static void cb_cert_client_prompt(modem_t *modem, const char *resp,
                                   modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "CCERTDOWN client cert prompt failed");
        do_error(mqtt);
        return;
    }
    sim76xx_mqtt_raw_send(mqtt->dce,
                          mqtt->config.client_cert,
                          strlen(mqtt->config.client_cert),
                          "\r\nOK\r\n",
                          cb_cert_client_done,
                          SX_MQTT_TIMEOUT_CONF);
}

static void cb_cert_client_done(modem_t *modem, const char *resp,
                                 modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "Client cert upload failed");
        do_error(mqtt);
        return;
    }
    log_info(TAG, "Client cert uploaded OK");

    mqtt->state = SX_MQTT_STATE_CERT_KEY;
    sim76xx_cert_download(mqtt->dce,
                          "client.key",
                          mqtt->config.client_key,
                          strlen(mqtt->config.client_key),
                          cb_cert_key_prompt,
                          SX_MQTT_TIMEOUT_CONF);
}

static void cb_cert_key_prompt(modem_t *modem, const char *resp,
                                modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "CCERTDOWN key prompt failed");
        do_error(mqtt);
        return;
    }
    sim76xx_mqtt_raw_send(mqtt->dce,
                          mqtt->config.client_key,
                          strlen(mqtt->config.client_key),
                          "\r\nOK\r\n",
                          cb_cert_key_done,
                          SX_MQTT_TIMEOUT_CONF);
}

static void cb_cert_key_done(modem_t *modem, const char *resp,
                              modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "Client key upload failed");
        do_error(mqtt);
        return;
    }
    log_info(TAG, "Client key uploaded OK");
    goto_ssl_cfg(mqtt);
}

static void goto_ssl_cfg(sx_mqtt_t *mqtt)
{
    mqtt->state = SX_MQTT_STATE_SSL_CFG;
    sim76xx_ssl_set_cert(mqtt->dce, mqtt->config.ca_cert ? "cacert.crt" : NULL,
                         mqtt->config.client_cert ? "client.crt" : NULL,
                         mqtt->config.client_key  ? "client.key"  : NULL,
                         cb_ssl_cacert,
                         SX_MQTT_TIMEOUT_CONF);
}

static void cb_ssl_cacert(modem_t *modem, const char *resp,
                           modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "SSL cacert bind failed"); do_error(mqtt); return;
    }
    if (mqtt->config.ssl_auth_mode == 2) {
        snprintf(s_ssl_cmd_buf, sizeof(s_ssl_cmd_buf),
         "AT+CSSLCFG=\"clientcert\",0,\"client.crt\"\r");
        sim76xx_mqtt_send_raw_cmd(mqtt->dce, s_ssl_cmd_buf, "\r\nOK\r\n",
                                  cb_ssl_clientcert, SX_MQTT_TIMEOUT_CONF);
    } else {
        cb_ssl_authmode(NULL, NULL, MODEM_RESPONSE_SUCCESS, NULL);
    }
}

static void cb_ssl_clientcert(modem_t *modem, const char *resp,
                               modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "SSL clientcert bind failed"); do_error(mqtt); return;
    }
    snprintf(s_ssl_cmd_buf, sizeof(s_ssl_cmd_buf), "AT+CSSLCFG=\"clientkey\",0,\"client.key\"\r");
    sim76xx_mqtt_send_raw_cmd(mqtt->dce, s_ssl_cmd_buf, "\r\nOK\r\n", cb_ssl_clientkey, SX_MQTT_TIMEOUT_CONF);
}

static void cb_ssl_clientkey(modem_t *modem, const char *resp,
                              modem_response_st_t res, void *arg)
{
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "SSL clientkey bind failed"); do_error(mqtt); return;
    }
    snprintf(s_ssl_cmd_buf, sizeof(s_ssl_cmd_buf),
             "AT+CSSLCFG=\"authmode\",0,%u\r", mqtt->config.ssl_auth_mode);
    sim76xx_mqtt_send_raw_cmd(mqtt->dce, s_ssl_cmd_buf, "\r\nOK\r\n",
                              cb_ssl_authmode, SX_MQTT_TIMEOUT_CONF);
}

static void cb_ssl_authmode(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "SSL authmode failed"); do_error(mqtt); return;
    }
    sim76xx_mqtt_ssl_bind(mqtt->dce, cb_sslbind, SX_MQTT_TIMEOUT_CONF);
}

/* 
 * Disconnect
 *  */

static void cb_disc(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;

    if (res != MODEM_RESPONSE_SUCCESS)
        log_warn(TAG, "CMQTTDISC failed — forcing disconnect");

    log_info(TAG, "MQTT disconnected");
    mqtt->state = SX_MQTT_STATE_DISCONNECTED;
    if (mqtt->on_disconnected) mqtt->on_disconnected(mqtt);
}

/* 
 * Publish / Subscribe
 *  */

static void cb_pub(modem_t *modem, const char *resp,
                   modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    int ok = (res == MODEM_RESPONSE_SUCCESS);

    if (ok) log_debug(TAG, "Publish OK");
    else    log_error(TAG, "Publish failed");

    if (mqtt->on_publish) mqtt->on_publish(mqtt, ok);
}

static void cb_sub(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    if (res == MODEM_RESPONSE_SUCCESS)
        log_debug(TAG, "Subscribe OK");
    else
        log_error(TAG, "Subscribe failed");
}

/* TLS / Tcp    */
static void cb_sslcfg(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "CSSLCFG failed");
        do_error(mqtt);
        return;
    }
    log_debug(TAG, "CSSLCFG OK");
    sim76xx_mqtt_ssl_bind(mqtt->dce, cb_sslbind, SX_MQTT_TIMEOUT_CONF);
}

static void cb_sslbind(modem_t *modem, const char *resp, modem_response_st_t res, void *arg){
    (void)modem; (void)arg;
    sx_mqtt_t *mqtt = s_instance;
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "CMQTTSSLCFG failed");
        do_error(mqtt);
        return;
    }
    log_debug(TAG, "SSL bind OK");
    mqtt->state = SX_MQTT_STATE_ACQUIRING;
    sim76xx_mqtt_accq(mqtt->dce, mqtt->config.client_id, mqtt->config.use_ssl, cb_accq, SX_MQTT_TIMEOUT_ACCQ);
}

/* 
 * Internal helpers
 *  */

static void do_error(sx_mqtt_t *mqtt){
    mqtt->state = SX_MQTT_STATE_ERROR;
    mqtt->reconnect_count++;
    mqtt->reconnect_delay = SX_MQTT_RECONNECT_DELAY_BASE;
    if (mqtt->reconnect_delay > SX_MQTT_RECONNECT_DELAY_MAX)
        mqtt->reconnect_delay = SX_MQTT_RECONNECT_DELAY_MAX;
    mqtt->reconnect_elapsed = 0;
    if (mqtt->reconnect_count >= 3) {
        log_warn(TAG, "Too many retries — restarting modem init");
        mqtt->reconnect_count = 0;
        mqtt->state = SX_MQTT_STATE_DISCONNECTED;
        sim76xx_power_off((sim76xx_t *)mqtt->dce);
        sim76xx_power_on(((sim76xx_t *)mqtt->dce));
        sim76xx_start((sim76xx_t *)mqtt->dce);  
    }
    log_warn(TAG, "Error — reconnect after: %lu ms (retry #%d)", mqtt->reconnect_delay, mqtt->reconnect_count);
    if (mqtt->on_disconnected) mqtt->on_disconnected(mqtt);
}

/* 
 * Public API
 *  */

void sx_mqtt_init(sx_mqtt_t *mqtt, void *dce){
    memset(mqtt, 0, sizeof(sx_mqtt_t));
    mqtt->dce       = dce;
    mqtt->state     = SX_MQTT_STATE_DISCONNECTED;
    mqtt->urc_state = URC_IDLE;
    s_instance      = mqtt;
    sim76xx_set_on_urc(dce, urc_handler);
    log_info(TAG, "init OK");
}

void sx_mqtt_set_config(sx_mqtt_t *mqtt, const sx_mqtt_config_t *config){
    mqtt->config = *config;
    log_info(TAG, "config: broker=%s port=%u client_id=%s", config->broker, config->port, config->client_id);
}

int sx_mqtt_connect(sx_mqtt_t *mqtt){
    if (!sim76xx_is_ready(mqtt->dce)) {
        log_warn(TAG, "connect: modem not ready");
        return -1;
    }
    if (mqtt->state != SX_MQTT_STATE_DISCONNECTED &&
        mqtt->state != SX_MQTT_STATE_ERROR)
    {
        log_warn(TAG, "connect: already connected or in progress");
        return -2;
    }
    log_info(TAG, "Starting connect flow");
    mqtt->state = SX_MQTT_STATE_CONFIGURING;
    return sim76xx_mqtt_start(mqtt->dce, cb_start, SX_MQTT_TIMEOUT_CONF);
}

int sx_mqtt_disconnect(sx_mqtt_t *mqtt){
    if (!sx_mqtt_is_connected(mqtt)) {
        log_warn(TAG, "disconnect: not connected");
        return -1;
    }
    mqtt->state = SX_MQTT_STATE_DISCONNECTING;
    return sim76xx_mqtt_disconnect(mqtt->dce, cb_disc, SX_MQTT_TIMEOUT_DISC);
}

int sx_mqtt_publish(sx_mqtt_t *mqtt, const char *topic, const char *message, uint8_t qos, uint8_t retain){
    if (!sx_mqtt_is_connected(mqtt)) {
        log_warn(TAG, "publish: not connected");
        return -1;
    }
    if (!topic || !message) return -1;
    return sim76xx_mqtt_publish(mqtt->dce, topic, message, qos, retain,  cb_pub, SX_MQTT_TIMEOUT_PUB);
}

int sx_mqtt_subscribe(sx_mqtt_t *mqtt, const char *topic, uint8_t qos){
    if (!sx_mqtt_is_connected(mqtt)) {
        log_warn(TAG, "subscribe: not connected");
        return -1;
    }
    if (!topic) return -1;
    return sim76xx_mqtt_subscribe(mqtt->dce, topic, qos, cb_sub, SX_MQTT_TIMEOUT_SUB);
}

void sx_mqtt_poll(sx_mqtt_t *mqtt, uint32_t ts){
    sim76xx_poll(mqtt->dce, ts);
    if ((mqtt->state == SX_MQTT_STATE_ERROR || mqtt->state == SX_MQTT_STATE_DISCONNECTED) && sim76xx_is_ready((sim76xx_t *)mqtt->dce) && mqtt->reconnect_count > 0)
    {
        mqtt->reconnect_elapsed += ts;
        if (mqtt->reconnect_elapsed >= mqtt->reconnect_delay) {
            mqtt->reconnect_elapsed = 0;
            mqtt->reconnect_count++;
            log_info(TAG, "Reconnecting... (retry #%d)", mqtt->reconnect_count);
            sx_mqtt_connect(mqtt);
        }
    }

    if (mqtt->state == SX_MQTT_STATE_CONNECTED) {
        mqtt->reconnect_count = 0;
        mqtt->reconnect_delay = SX_MQTT_RECONNECT_DELAY_BASE;
    }
}