#include <stdint.h>
#include "sim76xx.h"
#include "sx_delay.h"
#include "logger.h"
#include "string.h"
#include "stdio.h"
#include "sx_board.h"

#define pModem(dce) ((modem_t *)&dce->base)
#define pDCE(modem) ((sim76xx_t *)modem)

static const char *TAG = "SIM76XX";
static uint8_t  s_pub_retain;
static char s_urc_buf[MODEM_RX_BUFFER_SIZE];
static uint16_t s_urc_buf_id = 0;

#define CMD_AT          0
#define CMD_CGSN        1
#define CMD_COPS_QUERY  2
#define CMD_COPS_SET    3
#define CMD_CSQ         4
#define CMD_CFUN        5
#define CMD_CGATT       6
#define CMD_CGREG       7
#define CMD_NETOPEN     8
#define CMD_IPADDR      9
#define CMD_CGDCONT_QUERY   10
#define CMD_CSSLCFG     11
#define CMD_CMQTTSSLCFG 12
#define CMD_DYNAMIC     13

modem_command_t command[] = {
    [CMD_AT]         = {.cmd = "AT\r\n",         .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_CGSN]       = {.cmd = "AT+CGSN\r\n",    .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_COPS_QUERY] = {.cmd = "AT+COPS?\r\n",   .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_COPS_SET]   = {.cmd = "AT+COPS=0\r\n",  .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_CSQ]        = {.cmd = "AT+CSQ\r\n",     .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_CFUN]       = {.cmd = "AT+CFUN=1\r\n",  .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_CGATT]      = {.cmd = "AT+CGATT=1\r\n", .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_CGREG]      = {.cmd = "AT+CGREG=1\r\n", .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_NETOPEN]    = {.cmd = "AT+NETOPEN\r\n", .res_success = "+NETOPEN: 0",   .res_fail = "\r\nERROR\r\n"},
    [CMD_IPADDR]     = {.cmd = "AT+IPADDR\r\n",  .res_success = "\r\nOK\r\n",    .res_fail = "\r\nERROR\r\n"},
    [CMD_CGDCONT_QUERY] = {.cmd = "AT+CGDCONT?\r\n", .res_success = "\r\nOK\r\n", .res_fail = "\r\nERROR\r\n"},
    [CMD_CSSLCFG]     = {0},
    [CMD_CMQTTSSLCFG] = {0},
    [CMD_DYNAMIC]    = {0},
};

static char s_dyn_cmd_buf[256];

static char            s_mqtt_cmd_buf[256];
static char            s_mqtt_resp_buf[512];
static modem_command_t s_mqtt_cmd;

static char     s_pub_topic[128];
static char     s_pub_payload[256];
static uint8_t  s_pub_qos;
static modem_command_response_callback_t s_pub_final_cb;
static uint32_t s_pub_timeout_ms;

static char     s_sub_topic[128];
static uint8_t  s_sub_qos;
static modem_command_response_callback_t s_sub_final_cb;
static uint32_t s_sub_timeout_ms;

static const char *s_cert_content;
static uint32_t    s_cert_size;

static void send_cmd    (sim76xx_t *dce, int idx, modem_command_response_callback_t cb, uint32_t timeout_ms);
static void send_dynamic(sim76xx_t *dce, const char *cmd_str, const char *res_success, modem_command_response_callback_t cb, uint32_t timeout_ms);
static void restart_at  (sim76xx_t *dce);

static void cb_at      (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cgsn    (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cops_q  (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cops_set(modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_csq     (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cfun    (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cgatt   (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cgdcont (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cgauth (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cgdcont_query(modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_cgreg   (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_netopen (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_ipaddr  (modem_t *modem, const char *response, modem_response_st_t res, void *arg);

static void cb_pub_topic_prompt  (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_pub_topic_sent    (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_pub_payload_prompt(modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_pub_payload_sent  (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_pub_done          (modem_t *modem, const char *response, modem_response_st_t res, void *arg);

static void cb_sub_topic_prompt(modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_sub_done        (modem_t *modem, const char *response, modem_response_st_t res, void *arg);


static void map_operator_to_apn(sim76xx_t *dce, const char *mccmnc);
#if SIM76XX_TIME_PROBE
static void time_probe_start(sim76xx_t *dce);
static void cb_tp_cgmm     (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_tp_ctzu_test(modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_tp_ctzu_set (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_tp_ctzu_q   (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
static void cb_tp_cclk     (modem_t *modem, const char *response, modem_response_st_t res, void *arg);
#endif

/*  Helper  */
static void send_cmd(sim76xx_t *dce, int idx, modem_command_response_callback_t cb, uint32_t timeout_ms)
{
    sx_uart_flush(&dce->base.uart);  
    memset(dce->base.buff, 0x00, MODEM_RX_BUFFER_SIZE);
    dce->base.buff_id = 0;
    command[idx].callback      = cb;
    command[idx].arg           = dce;
    // lte_uart_it_enable();
    // sx_uart_flush(&dce->base.uart);
    log_debug(TAG, "CMD : %s", command[idx].cmd);
    modem_send_command(&dce->base, &command[idx], timeout_ms);
}

static void send_dynamic(sim76xx_t *dce, const char *cmd_str, const char *res_success, modem_command_response_callback_t cb, uint32_t timeout_ms)
{
    modem_t *m = pModem(dce);
    memset(m->buff, 0x00, MODEM_RX_BUFFER_SIZE);
    command[CMD_DYNAMIC].cmd           = cmd_str;
    command[CMD_DYNAMIC].res_success   = res_success ? res_success : "\r\nOK\r\n";
    command[CMD_DYNAMIC].res_fail      = "\r\nERROR\r\n";
    command[CMD_DYNAMIC].callback      = cb;
    command[CMD_DYNAMIC].arg           = dce;
    command[CMD_DYNAMIC].fail_on_cme   = 0;
    log_debug(TAG, "CMD : %s", cmd_str);
    modem_send_command(m, &command[CMD_DYNAMIC], timeout_ms);
}

static int mqtt_send_cmd(sim76xx_t *dce, const char *cmd_str,
                          const char *res_success,
                          modem_command_response_callback_t cb,
                          uint32_t timeout_ms)
{
    if (!sim76xx_is_ready(dce)) return -1;
    memset(pModem(dce)->buff, 0x00, MODEM_RX_BUFFER_SIZE);
    pModem(dce)->buff_id = 0;
    memset(s_mqtt_resp_buf, 0x00, sizeof(s_mqtt_resp_buf));
    s_mqtt_cmd.cmd           = cmd_str;
    s_mqtt_cmd.res_success   = res_success ? res_success : "\r\nOK\r\n";
    s_mqtt_cmd.res_fail      = "\r\nERROR\r\n";
    s_mqtt_cmd.callback      = cb;
    s_mqtt_cmd.arg           = dce;
    log_debug(TAG, "MQTT CMD: %s", cmd_str);
    return modem_send_command(pModem(dce), &s_mqtt_cmd, timeout_ms);
}

static void restart_at(sim76xx_t *dce)
{
    dce->state = SIM76XX_STATE_AT;
    send_cmd(dce, CMD_AT, cb_at, SIM76XX_TIMEOUT_AT);
}

static void encode_timer_t3412(uint32_t seconds, char *out)
{
    uint8_t val;
    if      (seconds % 3600 == 0) { val = (0b001 << 5) | (seconds / 3600);  }  // 1h unit
    else if (seconds % 60   == 0) { val = (0b101 << 5) | (seconds / 60);    }  // 1min unit
    else if (seconds % 30   == 0) { val = (0b100 << 5) | (seconds / 30);    }  // 30s unit
    else                          { val = (0b011 << 5) | (seconds / 2);      }  // 2s unit
    // format thành chuỗi 8 bit
    for (int i = 7; i >= 0; i--)
        out[7 - i] = ((val >> i) & 1) ? '1' : '0';
    out[8] = '\0';
}

static void encode_timer_t3324(uint32_t seconds, char *out)
{
    uint8_t val;
    if      (seconds % 60 == 0) { val = (0b101 << 5) | (seconds / 60); }   // 1min unit
    else if (seconds % 2  == 0) { val = (0b000 << 5) | (seconds / 2);  }   // 2s unit
    else                        { val = (0b000 << 5) | 1;               }   // fallback 2s
    for (int i = 7; i >= 0; i--)
        out[7 - i] = ((val >> i) & 1) ? '1' : '0';
    out[8] = '\0';
}

/*  API */

void sim76xx_init(sim76xx_t *dce)
{
    modem_init(pModem(dce));
    dce->state = SIM76XX_STATE_IDLE;
    dce->retry_count = 0;
    dce->rssi = 0;
    dce->ber = 0;
    dce->pwr_step = SIM76XX_PWR_IDLE;
    dce->pwr_deadline = 0;
    dce->pwr_then_on = 0;
    dce->start_pending = 0;
    dce->boot_restarts = 0;
    memset(dce->imei, 0, sizeof(dce->imei));
    memset(dce->ip,   0, sizeof(dce->ip));
    memset(dce->model, 0, sizeof(dce->model));
    dce->clk_valid = 0;
    dce->clk_tz_q  = 0;
    dce->clk_utc   = 0;
    // modem_init(pModem(dce));
}
/*
 * Power sequencing — NON-BLOCKING.
 * power_on/power_off/reset only arm a small state machine and return at once;
 * sim76xx_poll() advances it using HAL ticks, so the main loop (and therefore
 * tud_task) keeps running while the modem boots.
 * sim76xx_start() called during the sequence is deferred until boot wait ends,
 * so existing "power_on(); start();" call sites keep working unchanged.
 */
static void pwr_set_deadline(sim76xx_t *dce, uint32_t ms)
{
    dce->pwr_deadline = sx_gettick() + ms;
}

static uint8_t pwr_deadline_reached(sim76xx_t *dce)
{
    return ((int32_t)(sx_gettick() - dce->pwr_deadline) >= 0);
}

static void pwr_abort_modem_io(sim76xx_t *dce)
{
    modem_t *m = pModem(dce);
    m->isBusy   = 0;
    m->buff_id  = 0;
    m->elapsed  = 0;
    memset(m->buff, 0, MODEM_RX_BUFFER_SIZE);
    s_urc_buf_id = 0;
    dce->state  = SIM76XX_STATE_POWERING;
}

/* Modem boot banner: A76xx prints "*ATREADY: 1", SIM7600 prints "SMS DONE"/"PB DONE".
 * AT sent BEFORE this point is silently dropped by the modem. */
static char    s_boot_buf[96];
static uint8_t s_boot_len;

static uint8_t has_boot_banner(const char *s)
{
    return (s != NULL) && (strstr(s, "ATREADY") || strstr(s, "SMS DONE") || strstr(s, "PB DONE"));
}

static uint8_t pwr_scan_banner(sim76xx_t *dce)
{
    int avail = sx_uart_available(&dce->base.uart);
    while (avail > 0) {
        uint8_t tmp[32];
        int n = (avail > (int)sizeof(tmp)) ? (int)sizeof(tmp) : avail;
        n = sx_uart_read(&dce->base.uart, tmp, n, 10);
        if (n <= 0) break;
        avail -= n;
        for (int i = 0; i < n; i++) {
            if (s_boot_len >= sizeof(s_boot_buf) - 1) {          /* rolling window */
                memmove(s_boot_buf, s_boot_buf + sizeof(s_boot_buf) / 2, sizeof(s_boot_buf) / 2);
                s_boot_len = sizeof(s_boot_buf) / 2 - 1;
            }
            s_boot_buf[s_boot_len++] = (char)tmp[i];
        }
        s_boot_buf[s_boot_len] = '\0';
    }
    return has_boot_banner(s_boot_buf);
}

static void pwr_begin_on(sim76xx_t *dce)
{
    log_info(TAG, "Power On (async)");
    pwr_abort_modem_io(dce);
    s_boot_len = 0;
    s_boot_buf[0] = '\0';
    sx_gpio_write(&dce->base.powerPin, 0);
    sx_gpio_write(&dce->base.pwrPin, 1);
    dce->pwr_step = SIM76XX_PWR_ON_SETTLE;
    pwr_set_deadline(dce, SIM76XX_PWRKEY_SETTLE_MS);
}

static void pwr_begin_off(sim76xx_t *dce, uint8_t then_on)
{
    log_info(TAG, "Power Off (async)%s", then_on ? " + power on" : "");
    pwr_abort_modem_io(dce);
    dce->start_pending = 0;
    dce->pwr_then_on   = then_on;
    sx_gpio_write(&dce->base.pwrPin, 0);
    dce->pwr_step = SIM76XX_PWR_OFF_PULSE;
    pwr_set_deadline(dce, SIM76XX_PWROFF_PULSE_MS);
}

static void pwr_process(sim76xx_t *dce)
{
    if (dce->pwr_step == SIM76XX_PWR_ON_BOOT && pwr_scan_banner(dce)) {
        log_info(TAG, "Boot banner seen");
        dce->pwr_step = SIM76XX_PWR_ON_BANNER;
        pwr_set_deadline(dce, SIM76XX_BANNER_SETTLE_MS);
        return;
    }
    if (!pwr_deadline_reached(dce)) return;

    switch (dce->pwr_step) {
    case SIM76XX_PWR_ON_SETTLE:
        sx_gpio_write(&dce->base.pwrPin, 0);
        dce->pwr_step = SIM76XX_PWR_ON_PULSE;
        pwr_set_deadline(dce, SIM76XX_PWRKEY_PULSE_MS);
        break;

    case SIM76XX_PWR_ON_PULSE:
        sx_gpio_write(&dce->base.pwrPin, 1);
        dce->pwr_step = SIM76XX_PWR_ON_BOOT;
        pwr_set_deadline(dce, SIM76XX_BOOT_MAX_MS);
        break;

    case SIM76XX_PWR_ON_BOOT:       /* timeout: no banner (URC disabled?) — go on anyway */
        log_warn(TAG, "No boot banner within %u ms — continuing", (unsigned)SIM76XX_BOOT_MAX_MS);
        /* fall through */
    case SIM76XX_PWR_ON_BANNER:
        dce->pwr_step = SIM76XX_PWR_IDLE;
        dce->state    = SIM76XX_STATE_IDLE;
        sx_uart_flush(&dce->base.uart);          /* drop boot banner (RDY, +CPIN...) */
        log_info(TAG, "Power On OK!");
        if (dce->start_pending) {
            dce->start_pending = 0;
            sim76xx_start(dce);
        }
        break;

    case SIM76XX_PWR_OFF_PULSE:
        sx_gpio_write(&dce->base.powerPin, 1);   /* cut supply */
        if (dce->pwr_then_on) {
            dce->pwr_step = SIM76XX_PWR_OFF_GAP;
            pwr_set_deadline(dce, SIM76XX_RESET_GAP_MS);
        } else {
            dce->pwr_step = SIM76XX_PWR_IDLE;
            dce->state    = SIM76XX_STATE_IDLE;
            log_info(TAG, "Power Off OK!");
        }
        break;

    case SIM76XX_PWR_OFF_GAP:
        dce->pwr_then_on = 0;
        pwr_begin_on(dce);
        break;

    default:
        dce->pwr_step = SIM76XX_PWR_IDLE;
        break;
    }
}

void sim76xx_power_on(sim76xx_t *dce)
{
    switch (dce->pwr_step) {
    case SIM76XX_PWR_ON_SETTLE:
    case SIM76XX_PWR_ON_PULSE:
    case SIM76XX_PWR_ON_BOOT:
        return;                     /* already powering on: a 2nd PWRKEY pulse would switch it OFF */
    case SIM76XX_PWR_OFF_PULSE:
    case SIM76XX_PWR_OFF_GAP:
        dce->pwr_then_on = 1;       /* queue: power on when the off phase is done */
        return;
    default:
        pwr_begin_on(dce);
        return;
    }
}

void sim76xx_power_off(sim76xx_t *dce)
{
    if (dce->pwr_step == SIM76XX_PWR_OFF_PULSE || dce->pwr_step == SIM76XX_PWR_OFF_GAP) {
        dce->pwr_then_on   = 0;
        dce->start_pending = 0;
        return;
    }
    pwr_begin_off(dce, 0);
}

void sim76xx_reset(sim76xx_t *dce)
{
    log_info(TAG, "SIM Reset — power cycle (async)");
    if (dce->pwr_step == SIM76XX_PWR_OFF_PULSE || dce->pwr_step == SIM76XX_PWR_OFF_GAP) {
        dce->pwr_then_on   = 1;
        dce->start_pending = 0;
        return;
    }
    pwr_begin_off(dce, 1);
}

/* Busy-wait version, kept ONLY for entering STOP mode where the supply must be
 * really off before the MCU sleeps and nobody polls the modem afterwards. */
void sim76xx_power_off_blocking(sim76xx_t *dce)
{
    log_info(TAG, "Power Off (blocking)");
    dce->pwr_step      = SIM76XX_PWR_IDLE;
    dce->pwr_then_on   = 0;
    dce->start_pending = 0;
    dce->base.isBusy   = 0;
    sx_gpio_write(&dce->base.pwrPin, 0);
    sx_delay_ms(SIM76XX_PWROFF_PULSE_MS);
    sx_gpio_write(&dce->base.powerPin, 1);
    dce->state = SIM76XX_STATE_IDLE;
    log_info(TAG, "Power Off OK!");
}

int sim76xx_start(sim76xx_t *dce){
    if (dce->pwr_step != SIM76XX_PWR_IDLE) {
        dce->start_pending = 1;
        log_info(TAG, "start deferred until power sequence ends");
        return 0;
    }
    if(pModem(dce)->isBusy){
        log_warn(TAG, "start: modem busy");
        return -1;
    }
    log_info(TAG, "Starting init flow");
    
    dce->state = SIM76XX_STATE_AT;
    dce->retry_count = 0;
    dce->boot_restarts = 0;
    send_cmd(dce, CMD_AT, cb_at, SIM76XX_TIMEOUT_AT);
    // log_debug(TAG, "Callback at cmd");
    return 0;
}

void sim76xx_poll(sim76xx_t *dce, uint32_t ts){

    if (dce->pwr_step != SIM76XX_PWR_IDLE) {
        pwr_process(dce);
        return;                       /* modem is off/booting: no AT, no URC */
    }
    modem_poll(&dce->base, ts);
    if (dce->on_urc && !dce->base.isBusy) {
        int avail = sx_uart_available(&dce->base.uart);
        if (avail > 0) {
            int n_read = avail < (int)(sizeof(s_urc_buf) - s_urc_buf_id - 1)
                       ? avail
                       : (int)(sizeof(s_urc_buf) - s_urc_buf_id - 1);
            int n = sx_uart_read(&dce->base.uart,
                                 (uint8_t *)s_urc_buf + s_urc_buf_id,
                                 n_read, 1);
            if (n > 0) {
                s_urc_buf_id += n;
                s_urc_buf[s_urc_buf_id] = '\0';

                /* DIAG: while no MQTT RX message is being reassembled, log every complete
                 * unsolicited line (e.g. *ATREADY, +CMQTTCONNLOST, +CGEV, +CPSMSTATUS) and
                 * drop it from the buffer. Previously these were swallowed silently. */
                if (!strstr(s_urc_buf, "+CMQTTRXSTART")) {
                    char *start = s_urc_buf;
                    char *nl;
                    while ((nl = strchr(start, '\n')) != NULL) {
                        *nl = '\0';
                        char *e = nl;
                        while (e > start && (e[-1] == '\r')) { *--e = '\0'; }
                        if (e > start) log_warn(TAG, "URC: %s", start);
                        start = nl + 1;
                    }
                    if (start != s_urc_buf) {
                        size_t rest = strlen(start);
                        memmove(s_urc_buf, start, rest + 1);
                        s_urc_buf_id = rest;
                    }
                }

                if (strstr(s_urc_buf, "+CMQTTRXEND:")) {
                    dce->on_urc(dce, s_urc_buf);
                    s_urc_buf_id = 0;
                    memset(s_urc_buf, 0, sizeof(s_urc_buf));
                }
            }
        }
    }
}

int sim76xx_psm_enable(sim76xx_t *dce, uint32_t tau_sec, uint32_t active_sec)
{
    char tau_str[9];
    char active_str[9];
    encode_timer_t3412(tau_sec,    tau_str);
    encode_timer_t3324(active_sec, active_str);

    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf),
             "AT+CPSMS=1,,,\"%s\",\"%s\"\r", tau_str, active_str);

    log_info(TAG, "PSM enable: TAU=%lus (%s) Active=%lus (%s)",
             tau_sec, tau_str, active_sec, active_str);

    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "\r\nOK\r\n",
                         NULL, SIM76XX_TIMEOUT_AT);
}

int sim76xx_psm_disable(sim76xx_t *dce)
{
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CPSMS=0\r");
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "\r\nOK\r\n",
                         NULL, SIM76XX_TIMEOUT_AT);
}

void sim76xx_wake(sim76xx_t *dce){
    const uint8_t wake_char = 0xFF;
    sx_uart_write(&dce->base.uart, &wake_char, 1);
    sx_delay_ms(50);
    send_cmd(dce, CMD_AT, cb_at, SIM76XX_TIMEOUT_AT);
}

/*CALL BACK*/
static void cb_at(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        dce->retry_count = 0;
        dce->boot_restarts = 0;
        dce->state = SIM76XX_STATE_CGSN;
        send_cmd(dce, CMD_CGSN, cb_cgsn, SIM76XX_TIMEOUT_AT);
        return;
    }
    if (has_boot_banner(modem->buff) && dce->boot_restarts < SIM76XX_MAX_BOOT_RESTARTS) {
        /* The modem was still booting when AT went out (banner arrived during the wait):
         * resend without burning a retry. Capped so a modem stuck in a reset loop still escalates. */
        dce->boot_restarts++;
        log_warn(TAG, "modem still booting — resend AT (%u/%u, not counted)",
                 dce->boot_restarts, (unsigned)SIM76XX_MAX_BOOT_RESTARTS);
        dce->state = SIM76XX_STATE_AT;
        send_cmd(dce, CMD_AT, cb_at, SIM76XX_TIMEOUT_AT);
        return;
    }
    dce->retry_count++;
    log_error(TAG, "AT failed (retry %d/%d)", dce->retry_count, SIM76XX_MAX_RETRY);
    if (dce->retry_count >= SIM76XX_MAX_RETRY) {
        dce->retry_count = 0;
        dce->state = SIM76XX_STATE_ERROR;
        sim76xx_reset(dce);
        if (dce->on_error) dce->on_error(dce);
        sim76xx_start(dce);
        return;
    }
    dce->state = SIM76XX_STATE_AT;
    // sx_delay_ms(2000);
    send_cmd(dce, CMD_AT, cb_at, SIM76XX_TIMEOUT_AT);
}

static void cb_cgsn(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS && response) {
        // Skip qua echo line "AT+CGSN\r\n"
        const char *p = strstr(response, "AT+CGSN");
        if (p) {
            p = strstr(p, "\r\n");  
            if (p) p += 2;          
        } else {
            p = response;           
        }
        while (p && (*p == '\r' || *p == '\n')) p++;

        size_t i = 0;
        while (i < sizeof(dce->imei) - 1 && p[i] != '\r' && p[i] != '\n' && p[i] != '\0') {
            dce->imei[i] = p[i];
            i++;
        }
        dce->imei[i] = '\0';
        log_debug(TAG, "IMEI: %s", dce->imei);
        dce->state = SIM76XX_STATE_COPS_QUERY;
        send_cmd(dce, CMD_COPS_QUERY, cb_cops_q, SIM76XX_TIMEOUT_AT);
        return;
    }
    restart_at(dce);
}

// static void cb_cops_q(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
//     sim76xx_t *dce = pDCE(arg);
//     if (res == MODEM_RESPONSE_SUCCESS) {
//         // Parse: +COPS: 0,2,"45204",7
//         const char *p = strstr(response, "+COPS:");
//         if (p) {
//             p = strchr(p, '"');
//             if (p) {
//                 p++; 
//                 char mccmnc[8] = {0};
//                 size_t i = 0;
//                 while (i < sizeof(mccmnc) - 1 && *p && *p != '"') {
//                     mccmnc[i++] = *p++;
//                 }
//                 mccmnc[i] = '\0';
//                 //map_operator_to_apn(dce, mccmnc);
//             }
//         }
//         //dce->state = SIM76XX_STATE_COPS_SET;
//         dce->state = SIM76XX_STATE_CSQ;
//         send_cmd(dce, CMD_CSQ, cb_csq, SIM76XX_TIMEOUT_NETWORK);
//         return;
//     }
//     restart_at(dce);
// }

static void cb_cops_q(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        dce->state = SIM76XX_STATE_CSQ;
        send_cmd(dce, CMD_COPS_SET, cb_cops_set, SIM76XX_TIMEOUT_NETWORK);
        return;
    }
    restart_at(dce);
}

// static void map_operator_to_apn(sim76xx_t *dce, const char *mccmnc){
//     if      (strstr(mccmnc, "45204")) strncpy(dce->apn, "internet",  sizeof(dce->apn) - 1);
//     else if (strstr(mccmnc, "45201")) strncpy(dce->apn, "m-wap",     sizeof(dce->apn) - 1);
//     else if (strstr(mccmnc, "45205")) strncpy(dce->apn, "m3-world",  sizeof(dce->apn) - 1);
//     else if (strstr(mccmnc, "45207")) strncpy(dce->apn, "v-internet", sizeof(dce->apn) - 1);
//     else                              strncpy(dce->apn, "internet",  sizeof(dce->apn) - 1);
//     dce->apn[sizeof(dce->apn) - 1] = '\0';
//     log_info(TAG, "Operator: %s → APN: %s", mccmnc, dce->apn);
// }

static void cb_cops_set(modem_t *modem, const char *response, modem_response_st_t res, void *arg) {
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        dce->state = SIM76XX_STATE_CSQ;
        send_cmd(dce, CMD_CSQ, cb_csq, SIM76XX_TIMEOUT_AT);
        return;
    }
    restart_at(dce);
}

static void cb_csq(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS && response) {
        const char *p = strstr(response, "+CSQ:");
        if (p) {
            p += 5;
            while (*p == ' ') p++;
            dce->rssi = (int)strtol(p, NULL, 10);
            p = strchr(p, ',');
            if (p) dce->ber = (int)strtol(p + 1, NULL, 10);
        }
        log_info(TAG, "RSSI: %d  BER: %d", dce->rssi, dce->ber);
        
        if(dce->rssi < 15 || dce->rssi == 99){
            dce->state = SIM76XX_STATE_CGSN;
            send_cmd(dce, CMD_CGSN, cb_cgsn, SIM76XX_TIMEOUT_AT);
        }
        else{
            dce->state = SIM76XX_STATE_CFUN;
            send_cmd(dce, CMD_CFUN, cb_cfun, SIM76XX_TIMEOUT_AT);
        }
        return;
    }
    restart_at(dce);
}

static void cb_cfun(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        dce->state = SIM76XX_STATE_CGATT;
        //sx_delay_ms(3000);
        send_cmd(dce, CMD_CGATT, cb_cgatt, SIM76XX_TIMEOUT_CFUN);
        return;
    }
    restart_at(dce);
}

// static void cb_cgatt(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
//     sim76xx_t *dce = pDCE(arg);
//     if (res == MODEM_RESPONSE_SUCCESS) {
//         dce->state = SIM76XX_STATE_CGDCONT;
//         snprintf(s_dyn_cmd_buf, sizeof(s_dyn_cmd_buf),
//                  "AT+CGDCONT=1,\"IP\",\"%s\"\r", dce->apn);
//         send_dynamic(dce, s_dyn_cmd_buf, "\r\nOK\r\n",
//                      cb_cgdcont, SIM76XX_TIMEOUT_NETWORK);
//         return;
//     }
//     restart_at(dce);
// }

// static void cb_cgatt(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
//     sim76xx_t *dce = pDCE(arg);
//     if (res == MODEM_RESPONSE_SUCCESS) {
//         dce->state = SIM76XX_STATE_CGDCONT;

//         if (dce->username[0] != '\0' && dce->password[0] != '\0') {
//             snprintf(s_dyn_cmd_buf, sizeof(s_dyn_cmd_buf),
//                      "AT+CGDCONT=1,\"IP\",\"%s\",\"%s\",\"%s\"\r", dce->apn, dce->username, dce->password);
//         } else {
//             snprintf(s_dyn_cmd_buf, sizeof(s_dyn_cmd_buf), "AT+CGDCONT=1,\"IP\",\"%s\"\r", dce->apn);
//         }

//         send_dynamic(dce, s_dyn_cmd_buf, "\r\nOK\r\n", cb_cgdcont, SIM76XX_TIMEOUT_NETWORK);
//         return;
//     }
//     restart_at(dce);
// }

static void cb_cgatt(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        dce->state = SIM76XX_STATE_CGDCONT;
        snprintf(s_dyn_cmd_buf, sizeof(s_dyn_cmd_buf),
                 "AT+CGDCONT=1,\"IP\",\"%s\"\r", dce->apn);
        send_dynamic(dce, s_dyn_cmd_buf, "\r\nOK\r\n",
                     cb_cgdcont, SIM76XX_TIMEOUT_NETWORK);
        return;
    }
    restart_at(dce);
}

static void cb_cgdcont(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        dce->state = SIM76XX_STATE_CGAUTH;

        if (dce->username[0] != '\0' && dce->password[0] != '\0') {
            // PAP both username and pass
            snprintf(s_dyn_cmd_buf, sizeof(s_dyn_cmd_buf),
                     "AT+CGAUTH=1,1,\"%s\",\"%s\"\r",
                     dce->password, dce->username);
            send_dynamic(dce, s_dyn_cmd_buf, "\r\nOK\r\n",
                         cb_cgauth, SIM76XX_TIMEOUT_NETWORK);
        } else if (dce->password[0] != '\0') {
            // CHAP — onlyu password
            snprintf(s_dyn_cmd_buf, sizeof(s_dyn_cmd_buf),
                     "AT+CGAUTH=1,2,\"%s\"\r",
                     dce->password);
            send_dynamic(dce, s_dyn_cmd_buf, "\r\nOK\r\n",
                         cb_cgauth, SIM76XX_TIMEOUT_NETWORK);
        } else {
            // step over when you don't have auth
            dce->state = SIM76XX_STATE_CGREG;
            send_cmd(dce, CMD_CGREG, cb_cgreg, SIM76XX_TIMEOUT_AT);
        }
        return;
    }
    restart_at(dce);
}

static void cb_cgauth(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        log_info(TAG, "CGAUTH OK — user=%s", dce->username);
        dce->state = SIM76XX_STATE_CGREG;
        send_cmd(dce, CMD_CGREG, cb_cgreg, SIM76XX_TIMEOUT_AT);
        return;
    }
    restart_at(dce);
}

static void cb_cgdcont_query(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS && response) {
        // Response: +CGDCONT: 1,"IP","internet","6.234.97.3",0,0
        const char *p = strstr(response, "+CGDCONT:");
        if (p) {
            p = strchr(p, '"'); if (!p) goto done; p++; // 
            p = strchr(p, '"'); if (!p) goto done; p++; // 
            p = strchr(p, '"'); if (!p) goto done; p++; // 
            size_t i = 0;
            while (i < sizeof(dce->apn) - 1 && *p && *p != '"') {
                dce->apn[i] = *p;
                i++; p++;
            }
            dce->apn[i] = '\0';
        }
    }
done:
    log_info("SIM76XX", "APN: %s", dce->apn);
    dce->state = SIM76XX_STATE_READY;
    dce->retry_count = 0;
    if (dce->on_ready) dce->on_ready(dce);
}

static void cb_cgreg(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS) {
        dce->state = SIM76XX_STATE_NETOPEN;
        send_cmd(dce, CMD_NETOPEN, cb_netopen, SIM76XX_TIMEOUT_NETWORK);
        return;
    }
    restart_at(dce);
}

static void cb_netopen(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS ||
        (response && strstr(response, "+NETOPEN: 1")) ||
        (response && strstr(response, "already opened"))) // add
    {
        dce->state = SIM76XX_STATE_IPADDR;
        send_cmd(dce, CMD_IPADDR, cb_ipaddr, SIM76XX_TIMEOUT_NETOPEN);
        return;
    }
    restart_at(dce);
}

static void cb_ipaddr(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS && response) {
        const char *p = strstr(response, "+IPADDR:");
        if (p) {
            p += 8;
            while (*p == ' ') p++;
            size_t i = 0;
            while (i < sizeof(dce->ip) - 1 && *p && *p != '\r' && *p != '\n') {
                dce->ip[i] = *p;
                i++; p++;
            }
            dce->ip[i] = '\0';
        }
    }
    /* whether you have an IP address or not, it will still switch to READY. */
    log_info(TAG, "Network ready - IP: %s", dce->ip);
#if SIM76XX_TIME_PROBE
    time_probe_start(dce);          /* CGMM / CTZU / CCLK, then continues with CGDCONT? */
#else
    dce->state       = SIM76XX_STATE_CGDCONT_QUERY;
    send_cmd(dce, CMD_CGDCONT_QUERY, cb_cgdcont_query, SIM76XX_TIMEOUT_AT);
#endif
}

#if SIM76XX_TIME_PROBE
/* ======================================================================
 *  Phase 1 — network time (NITZ) probe
 *  AT+CGMM -> AT+CTZU=? -> AT+CTZU=1 -> AT+CTZU? -> AT+CCLK?
 *  Result is logged ("TIME ..." lines) and, if CCLK? looks valid, stored in
 *  dce->clk_utc (unix seconds, UTC). Nothing here is fatal: any FAIL/TIMEOUT is
 *  logged and the init flow continues to CGDCONT? / READY.
 *  CCLK? returns LOCAL time + zone in quarters of an hour (manual 3.2.10),
 *  so UTC = local - zz*15 min.
 * ====================================================================== */

/* ---- pure helpers (no HW access) : PROBE_HELPERS_BEGIN ---- */
static int cclk_num2(const char **pp, int *out)
{
    const char *p = *pp;
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return 0;
    *out = (p[0] - '0') * 10 + (p[1] - '0');
    *pp = p + 2;
    return 1;
}

/* days since 1970-01-01 for a civil date (proleptic Gregorian) */
static int32_t days_from_civil(int y, int m, int d)
{
    y -= (m <= 2);
    int32_t era = (y >= 0 ? y : y - 399) / 400;
    int32_t yoe = y - era * 400;
    int32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

static void civil_from_days(int32_t z, int *y, int *m, int *d)
{
    z += 719468;
    int32_t era = (z >= 0 ? z : z - 146096) / 146097;
    int32_t doe = z - era * 146097;
    int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int32_t mp  = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp < 10 ? mp + 3 : mp - 9;
    *y = yoe + era * 400 + (*m <= 2);
}

/* Parse   +CCLK: "yy/MM/dd,hh:mm:ss+zz"   -> unix seconds UTC and zone (quarters).
 * Returns 1 if the string was parsed (range-checked); plausibility (year) is checked by the caller. */
static int parse_cclk_utc(const char *resp, int *year, int64_t *utc, int *tz_q)
{
    int yy, mo, dd, hh, mi, ss, tz;
    char sign;
    if (!resp) return 0;
    const char *p = strstr(resp, "+CCLK:");
    if (!p) return 0;
    p = strchr(p, '"');
    if (!p) return 0;
    p++;
    if (!cclk_num2(&p, &yy) || *p++ != '/') return 0;
    if (!cclk_num2(&p, &mo) || *p++ != '/') return 0;
    if (!cclk_num2(&p, &dd) || *p++ != ',') return 0;
    if (!cclk_num2(&p, &hh) || *p++ != ':') return 0;
    if (!cclk_num2(&p, &mi) || *p++ != ':') return 0;
    if (!cclk_num2(&p, &ss)) return 0;
    sign = *p++;
    if (sign != '+' && sign != '-') return 0;
    if (!cclk_num2(&p, &tz)) return 0;
    if (sign == '-') tz = -tz;

    if (mo < 1 || mo > 12 || dd < 1 || dd > 31 || hh > 23 || mi > 59 || ss > 60 || tz < -96 || tz > 96) return 0;

    int y = 2000 + yy;
    int64_t local = (int64_t)days_from_civil(y, mo, dd) * 86400 + hh * 3600 + mi * 60 + ss;
    *year = y;
    *tz_q = tz;
    *utc  = local - (int64_t)tz * 900;
    return 1;
}

/* Copy the first non-empty line after the echoed command (e.g. the answer of AT+CGMM). */
static void first_line_after_echo(const char *resp, const char *echo, char *out, size_t n)
{
    out[0] = '\0';
    if (!resp || n == 0) return;
    const char *p = strstr(resp, echo);
    if (p) {
        p = strstr(p, "\r\n");
        if (p) p += 2;
    } else {
        p = resp;
    }
    while (p && (*p == '\r' || *p == '\n')) p++;
    size_t i = 0;
    while (p && i < n - 1 && *p && *p != '\r' && *p != '\n') out[i++] = *p++;
    out[i] = '\0';
}
/* ---- PROBE_HELPERS_END ---- */

static void log_probe(const char *cmd, modem_response_st_t res, const char *resp)
{
    char line[112];
    size_t n = 0;
    int sp = 1;
    const char *r = resp ? resp : "";
    for (; *r && n < sizeof(line) - 1; r++) {
        char ch = *r;
        if (ch == '\r' || ch == '\n') ch = ' ';
        if (ch == ' ' && sp) continue;      /* collapse blanks, drop leading */
        sp = (ch == ' ');
        line[n++] = ch;
    }
    line[n] = '\0';
    log_info(TAG, "TIME %s -> %s | %s", cmd,
             res == MODEM_RESPONSE_SUCCESS ? "OK" : (res == MODEM_RESPONSE_FAIL ? "FAIL" : "TIMEOUT"),
             line);
}

/* Like send_dynamic(), but "+CME ERROR" ends the command at once (otherwise an unsupported
 * command would just sit until the 9 s timeout). */
static void send_probe(sim76xx_t *dce, const char *cmd_str, modem_command_response_callback_t cb)
{
    modem_t *m = pModem(dce);
    memset(m->buff, 0x00, MODEM_RX_BUFFER_SIZE);
    command[CMD_DYNAMIC].cmd           = cmd_str;
    command[CMD_DYNAMIC].res_success   = "\r\nOK\r\n";
    command[CMD_DYNAMIC].res_fail      = "\r\nERROR\r\n";
    command[CMD_DYNAMIC].callback      = cb;
    command[CMD_DYNAMIC].arg           = dce;
    command[CMD_DYNAMIC].fail_on_cme   = 1;
    log_debug(TAG, "CMD : %s", cmd_str);
    modem_send_command(m, &command[CMD_DYNAMIC], SIM76XX_TIMEOUT_TIME);
}

static void cb_tp_cgmm(modem_t *modem, const char *response, modem_response_st_t res, void *arg)
{
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS && response) {
        first_line_after_echo(response, "AT+CGMM", dce->model, sizeof(dce->model));
        log_info(TAG, "TIME model (CGMM): %s", dce->model[0] ? dce->model : "(empty)");
    } else {
        log_probe("AT+CGMM", res, modem->buff);
    }
    send_probe(dce, "AT+CTZU=?\r\n", cb_tp_ctzu_test);
}

static void cb_tp_ctzu_test(modem_t *modem, const char *response, modem_response_st_t res, void *arg)
{
    sim76xx_t *dce = pDCE(arg);
    log_probe("AT+CTZU=?", res, response ? response : modem->buff);
    send_probe(dce, "AT+CTZU=1\r\n", cb_tp_ctzu_set);
}

static void cb_tp_ctzu_set(modem_t *modem, const char *response, modem_response_st_t res, void *arg)
{
    sim76xx_t *dce = pDCE(arg);
    log_probe("AT+CTZU=1", res, response ? response : modem->buff);
    send_probe(dce, "AT+CTZU?\r\n", cb_tp_ctzu_q);
}

static void cb_tp_ctzu_q(modem_t *modem, const char *response, modem_response_st_t res, void *arg)
{
    sim76xx_t *dce = pDCE(arg);
    log_probe("AT+CTZU?", res, response ? response : modem->buff);
    send_probe(dce, "AT+CCLK?\r\n", cb_tp_cclk);
}

static void cb_tp_cclk(modem_t *modem, const char *response, modem_response_st_t res, void *arg)
{
    sim76xx_t *dce = pDCE(arg);
    int year = 0, tz_q = 0;
    int64_t utc = 0;

    dce->clk_valid = 0;
    log_probe("AT+CCLK?", res, response ? response : modem->buff);

    if (res == MODEM_RESPONSE_SUCCESS && parse_cclk_utc(response, &year, &utc, &tz_q)) {
        int uy, um, ud;
        int32_t days = (int32_t)(utc / 86400);
        int32_t secs = (int32_t)(utc % 86400);
        if (secs < 0) { secs += 86400; days--; }
        civil_from_days(days, &uy, &um, &ud);
        log_info(TAG, "TIME parsed: zone=%d quarters (offset %d min) -> UTC %04d-%02d-%02d %02d:%02d:%02d (unix %lu)",
                 tz_q, tz_q * 15, uy, um, ud, secs / 3600, (secs % 3600) / 60, secs % 60, (unsigned long)utc);
        if (year >= SIM76XX_CLK_MIN_YEAR && utc > 0) {
            dce->clk_valid = 1;
            dce->clk_tz_q  = (int16_t)tz_q;
            dce->clk_utc   = (uint32_t)utc;
            log_info(TAG, "TIME valid (year %d >= %d)", year, SIM76XX_CLK_MIN_YEAR);
        } else {
            log_warn(TAG, "TIME NOT valid: year %d < %d (modem clock probably not synced by network yet)",
                     year, SIM76XX_CLK_MIN_YEAR);
        }
    } else if (res == MODEM_RESPONSE_SUCCESS) {
        log_warn(TAG, "TIME: could not parse +CCLK line");
    }

    dce->state = SIM76XX_STATE_CGDCONT_QUERY;
    send_cmd(dce, CMD_CGDCONT_QUERY, cb_cgdcont_query, SIM76XX_TIMEOUT_AT);
}

static void time_probe_start(sim76xx_t *dce)
{
    dce->state = SIM76XX_STATE_TIME_PROBE;
    dce->clk_valid = 0;
    log_info(TAG, "TIME probe start (CGMM, CTZU, CCLK)");
    send_probe(dce, "AT+CGMM\r\n", cb_tp_cgmm);
}
#endif /* SIM76XX_TIME_PROBE */

/*  MQTT API    */
int sim76xx_mqtt_start(sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms){
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTSTART\r");
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "+CMQTTSTART: 0", cb, timeout_ms);
}

int sim76xx_mqtt_accq(sim76xx_t *dce, const char *client_id, uint8_t use_ssl, modem_command_response_callback_t cb, uint32_t timeout_ms){
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTACCQ=0,\"%s\",%u\r", client_id, use_ssl ? 1 : 0);
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "\r\nOK\r\n", cb, timeout_ms);
}

int sim76xx_mqtt_ssl_cfg(sim76xx_t *dce, uint8_t auth_mode, modem_command_response_callback_t cb, uint32_t timeout_ms){
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CSSLCFG=\"authmode\",0,%u\r", auth_mode);
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "\r\nOK\r\n", cb, timeout_ms);
}

int sim76xx_mqtt_ssl_bind(sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms){
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTSSLCFG=0,0\r");
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "\r\nOK\r\n", cb, timeout_ms);
}

// int sim76xx_mqtt_connect(sim76xx_t *dce, const char *broker, uint16_t port, uint8_t use_ssl, uint16_t keepalive, 
//                             uint8_t clean_session, const char *username, const char *password, modem_command_response_callback_t cb,
//                             uint32_t timeout_ms)
// {   const char *scheme = use_ssl ? "ssl" : "tcp";

//     if(username && password) {
//         snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTCONNECT=0,\"%s://%s:%u\",%u,%u,\"%s\",\"%s\"\r",
//                  scheme, broker, port, keepalive, clean_session, username, password);
//     } else {
//         snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTCONNECT=0,\"%s://%s:%u\",%u,%u\r",
//                  scheme, broker, port, keepalive, clean_session);
//     }
//     return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "+CMQTTCONNECT: 0,0", cb, timeout_ms);
// }

int sim76xx_mqtt_connect(sim76xx_t *dce, const char *broker, uint16_t port, uint8_t use_ssl, uint16_t keepalive, 
                            uint8_t clean_session, const char *username, const char *password, modem_command_response_callback_t cb,
                            uint32_t timeout_ms)
{
    const char *scheme = use_ssl ? "ssl" : "tcp";

    uint8_t has_auth = (username && password &&
                        username[0] != '\0' &&
                        password[0] != '\0' &&
                        strcmp(username, "null") != 0 &&
                        strcmp(password, "null") != 0);

    if (has_auth) {
        snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf),
                 "AT+CMQTTCONNECT=0,\"%s://%s:%u\",%u,%u,\"%s\",\"%s\"\r",
                 scheme, broker, port, keepalive, clean_session, username, password);
    } else {
        snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf),
                 "AT+CMQTTCONNECT=0,\"%s://%s:%u\",%u,%u\r",
                 scheme, broker, port, keepalive, clean_session);
    }

    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "+CMQTTCONNECT: 0,0", cb, timeout_ms);
}

int sim76xx_mqtt_disconnect(sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms){
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTDISC=0,120\r");
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "+CMQTTDISC: 0,0", cb, timeout_ms);
    //return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "\r\nOK\r\n", cb, timeout_ms);
}

int sim76xx_mqtt_subscribe(sim76xx_t *dce, const char *topic, uint8_t qos, modem_command_response_callback_t cb, uint32_t timeout_ms){
    if (!sim76xx_is_ready(dce)) return -1;
    strncpy(s_sub_topic, topic, sizeof(s_sub_topic) - 1);
    s_sub_topic[sizeof(s_sub_topic) - 1] = '\0';
    s_sub_qos = qos;
    s_sub_final_cb = cb;
    s_sub_timeout_ms = timeout_ms;

    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTSUB=0,%u,%u\r", (unsigned)strlen(s_sub_topic), s_sub_qos);
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, ">", cb_sub_topic_prompt, timeout_ms);
}

static void cb_sub_topic_prompt(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "MQTT sub: CMQTTSUB prompt failed");
        if (s_sub_final_cb) s_sub_final_cb(modem, response, res, dce);
        return;
    }
    mqtt_send_cmd(dce, s_sub_topic, "+CMQTTSUB: 0,0", cb_sub_done, s_sub_timeout_ms);
}

static void cb_sub_done(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res == MODEM_RESPONSE_SUCCESS)
        log_info(TAG, "MQTT subscribe OK: %s", s_sub_topic);
    else
        log_error(TAG, "MQTT subscribe failed");
    if (s_sub_final_cb) s_sub_final_cb(modem, response, res, dce);
}

static void cb_pub_topic_prompt(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "MQTT pub: CMQTTTOPIC failed res=%d(1=FAIL,2=TIMEOUT) response=[%s]", res, response ? response : "NULL");
        if (s_pub_final_cb) s_pub_final_cb(modem, response, res, dce);
        return;
    }
    mqtt_send_cmd(dce, s_pub_topic, "\r\nOK\r\n", cb_pub_topic_sent, s_pub_timeout_ms);
}

static void cb_pub_topic_sent(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    log_debug(TAG, "topic_sent res=%d response=[%s]", res, response ? response : "NULL");
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "MQTT pub: topic write failed");
        if (s_pub_final_cb) s_pub_final_cb(modem, response, res, dce);
        return;
    }
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTPAYLOAD=0,%u\r", (unsigned)strlen(s_pub_payload));
    mqtt_send_cmd(dce, s_mqtt_cmd_buf, ">", cb_pub_payload_prompt, s_pub_timeout_ms);
}

static void cb_pub_payload_prompt(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "MQTT pub: CMQTTPAYLOAD failed res=%d(1=FAIL,2=TIMEOUT) response=[%s]", res, response ? response : "NULL");
        if (s_pub_final_cb) s_pub_final_cb(modem, response, res, dce);
        return;
    }
    mqtt_send_cmd(dce, s_pub_payload, "\r\nOK\r\n", cb_pub_payload_sent, s_pub_timeout_ms);
}

static void cb_pub_payload_sent(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    if (res != MODEM_RESPONSE_SUCCESS) {
        log_error(TAG, "MQTT pub: payload write failed");
        if (s_pub_final_cb) s_pub_final_cb(modem, response, res, dce);
        return;
    }
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTPUB=0,%u,60,%u\r", s_pub_qos, s_pub_retain);
    mqtt_send_cmd(dce, s_mqtt_cmd_buf, "+CMQTTPUB: 0,0", cb_pub_done, s_pub_timeout_ms);
}

static void cb_pub_done(modem_t *modem, const char *response, modem_response_st_t res, void *arg){
    sim76xx_t *dce = pDCE(arg);
    log_debug(TAG, "PUB response: [%s] res=%d", response ? response : "NULL", res);
    if (res == MODEM_RESPONSE_SUCCESS)
        log_info(TAG, "MQTT publish OK");
    else
        log_error(TAG, "MQTT publish failed");
    if (s_pub_final_cb) s_pub_final_cb(modem, response, res, dce);
}

int sim76xx_mqtt_publish(sim76xx_t *dce, const char *topic, const char *message, uint8_t qos, uint8_t retain, modem_command_response_callback_t cb, uint32_t timeout_ms){
    if (!sim76xx_is_ready(dce)) return -1;
    strncpy(s_pub_topic,   topic,   sizeof(s_pub_topic)   - 1);
    strncpy(s_pub_payload, message, sizeof(s_pub_payload) - 1);
    s_pub_topic  [sizeof(s_pub_topic)   - 1] = '\0';
    s_pub_payload[sizeof(s_pub_payload) - 1] = '\0';
    s_pub_retain = retain;
    s_pub_qos = qos;
    s_pub_final_cb = cb;
    s_pub_timeout_ms = timeout_ms;
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTTOPIC=0,%u\r", (unsigned)strlen(s_pub_topic));
    log_info(TAG, "topic len=%d topic=[%s]", strlen(s_pub_topic), s_pub_topic);
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, ">", cb_pub_topic_prompt, timeout_ms);
}

int sim76xx_cert_download(sim76xx_t *dce, const char *filename, const char *content, uint32_t size, modem_command_response_callback_t cb, uint32_t timeout_ms){
    s_cert_content = content;
    s_cert_size    = size;

    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CCERTDOWN=\"%s\",%lu\r", filename, size);
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, ">", cb, timeout_ms);
}

int sim76xx_ssl_set_cert(sim76xx_t *dce, const char *ca_cert_name, const char *client_cert_name, const char *client_key_name,
                         modem_command_response_callback_t cb, uint32_t timeout_ms){
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CSSLCFG=\"cacert\",0,\"%s\"\r", ca_cert_name);
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "\r\nOK\r\n", cb, timeout_ms);
}

int sim76xx_mqtt_raw_send(sim76xx_t *dce, const char *data, uint32_t size, const char *res_success,
                               modem_command_response_callback_t cb, uint32_t timeout_ms){
    return mqtt_send_cmd(dce, data, res_success, cb, timeout_ms);                            
}

int sim76xx_mqtt_send_raw_cmd(sim76xx_t *dce, const char *cmd_str, const char *res_success,
                               modem_command_response_callback_t cb, uint32_t timeout_ms){
    return mqtt_send_cmd(dce, cmd_str, res_success, cb, timeout_ms);
}

int sim76xx_mqtt_stop(sim76xx_t *dce, modem_command_response_callback_t cb, uint32_t timeout_ms){
    snprintf(s_mqtt_cmd_buf, sizeof(s_mqtt_cmd_buf), "AT+CMQTTSTOP\r");
    return mqtt_send_cmd(dce, s_mqtt_cmd_buf, "+CMQTTSTOP: 0", cb, timeout_ms);
}

void sim76xx_set_full_apn(sim76xx_t *dce, char* apn, char* username, char* password){
    strncpy(dce->apn,      apn      ? apn      : "", sizeof(dce->apn)-1);
    strncpy(dce->username, username ? username : "", sizeof(dce->username)-1);
    strncpy(dce->password, password ? password : "", sizeof(dce->password)-1);

    dce->apn[sizeof(dce->apn)-1] = '\0';
    dce->username[sizeof(dce->username)-1] = '\0';
    dce->password[sizeof(dce->password)-1] = '\0';

    log_info(TAG, "APN config: %s | User: %s | Pass: %s", 
             dce->apn, dce->username[0]?dce->username:"(none)", 
             dce->password[0]?dce->password:"(none)");
}