#include "app.h"
#include "logger.h"
#include "sx_board.h"
#include "sx_user_mqtt.h"
#include "sx_sleep_manager.h"
#include "sx_sleep.h"
#include "sx_ex_storage.h"
#include "app_config.h"
#include "ff.h"
#include "sx_fatfs.h"
#include "sx_diskio.h"
#include "sx_user_msc.h"
#include "cJSON.h"
#include "bno055.h"
#include "test_at.h"
#include "tusb.h"
#include "sx_usb_tiny_msc.h"
#include <time.h>

static const char *TAG = "App";

typedef struct TrackingApp
{
    // Add any app-level state here if needed
    app_mode_t app_mode;

    sx_sleep_t sleep;
    sx_sleep_manager_t sleep_mgr;

    uint32_t publish_elapsed;
    uint8_t subscribed ;
    uint8_t first_pub_pending;   /* publish once right after MQTT (re)connect, then follow time_publish */
    uint8_t last_publish_done;
    uint8_t enter_sleep_published;
    uint8_t mqtt_stopped;

    uint8_t publish_count;

    uint32_t enter_sleep_elapsed_ms;
    uint8_t sleep_requested;

    uint8_t s_wake_publish_started;//

    uint8_t usb_connect_pending;

    Board_t *board;
    /*Parse JSon*/
    const char *json_str;
    /*RTC*/
    rx8130ce_time_t time;
    /*Synch gps*/
    uint8_t s_sync_disk_pending;

    /*IMU*/
    bno055_euler_t euler;
    bno055_vec3_t accel;
    bno055_vec3_t gyro;
    bno055_vec3_t l_accel;
    bno055_quat_t quat;
    bno055_calib_stat_t calib;
    bool calib_saved;
    float linear_accel;
    float vel;
    float pos;

    // last gps
    float last_lat;
    float last_lon;
    float last_alt;  
    float last_spd;  
    int   last_sat;  

    char last_time_str[12];
    char last_date_str[12];
    
    char device_name[64];
} TrackingApp_t;

volatile TrackingApp_t g_app;

typedef struct 
{
    /* data */
    uint8_t  s_cfg_buf[512];
    char     s_mqtt_host[64];
    char     s_mqtt_client_id[64];
    char     s_mqtt_user[32];
    char     s_mqtt_pass[32];
    char     s_apn_name[32];
    uint32_t time_sleep_ms;
    uint32_t time_wake_ms;

    char    s_device_name[64];

    // TIME IMU
    uint32_t s_elapsed_data;
    uint32_t s_elapsed_calib;
    uint32_t s_time_publish;
}config_json_t;

/* s_time_publish must have a non-zero default: if config.json has no "time_publish"
 * key, a value of 0 makes the FULL_POWER publish condition always true, so GSM/GPS
 * are published on every loop iteration (log spam). */
static config_json_t config_json = { .s_time_publish = TIME_PUBLISH_FULL_PW_MODE_MS };

/* ======================================================================
 *  External RTC (RX8130CE)
 *
 *  - RTC holds Vietnam local time = UTC + APP_RTC_TZ_OFFSET_S (same zone as gps.c, which adds +7 h).
 *  - rx8130ce_time_t.month is 1..12 (driver rejects 0). struct tm uses tm_mon 0..11: convert at the edges.
 *    (Old code stored tm_mon 0..11 directly: January was rejected and every other month was off by one.)
 *  - week is a one-hot bitmask (RX8130CE_WEEK_xxx), not a number.
 *  - Sources: network time (AT+CCLK?) has priority; GPS RMC (only with a valid fix) is a fallback.
 *    GPS writes the RTC only while the current modem session has no network time applied
 *    (see rtc_net_time_current()). A new CCLK sample always overwrites whatever GPS wrote earlier.
 * ====================================================================== */
#define APP_RTC_TZ_OFFSET_S     (7 * 3600)
#define APP_RTC_MIN_YEAR        2024
#define APP_RTC_MAX_YEAR        2099

/* days since 1970-01-01 for a civil date; linear in d, so tm_mday overflow (32) normalises itself */
static int32_t rtc_days_from_civil(int y, int m, int d)
{
    y -= (m <= 2);
    int32_t era = (y >= 0 ? y : y - 399) / 400;
    int32_t yoe = y - era * 400;
    int32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

static int64_t rtc_tm_to_secs(const struct tm *t)
{
    return (int64_t)rtc_days_from_civil(t->tm_year + 1900, t->tm_mon + 1, t->tm_mday) * 86400
         + t->tm_hour * 3600 + t->tm_min * 60 + t->tm_sec;
}

/* Write "local seconds since 1970" into the RTC. Returns RX8130CE_OK or an RX8130CE_ERR_* code. */
static int rtc_set_local_secs(int64_t local)
{
    int64_t days = local / 86400;
    int32_t rem  = (int32_t)(local % 86400);
    if (rem < 0) { rem += 86400; days--; }

    int64_t z   = days + 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    int32_t doe = (int32_t)(z - era * 146097);
    int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int32_t mp  = (5 * doy + 2) / 153;
    int     d   = doy - (153 * mp + 2) / 5 + 1;
    int     m   = mp < 10 ? mp + 3 : mp - 9;
    int     y   = (int)(yoe + era * 400) + (m <= 2);

    if (y < APP_RTC_MIN_YEAR || y > APP_RTC_MAX_YEAR) return RX8130CE_ERR_PARAM;

    int wday = (int)(((days % 7) + 11) % 7);   /* 1970-01-01 = Thursday; 0 = Sunday */

    rx8130ce_time_t r;
    r.sec   = (uint8_t)(rem % 60);
    r.min   = (uint8_t)((rem / 60) % 60);
    r.hour  = (uint8_t)(rem / 3600);
    r.week  = (uint8_t)(1U << wday);
    r.day   = (uint8_t)d;
    r.month = (uint8_t)m;                       /* 1..12 */
    r.year  = (uint8_t)(y - 2000);

    int rc = rx8130ce_set_time(&board.rtc, &r);
    if (rc == RX8130CE_OK) g_app.time = r;
    return rc;
}

/* Network time priority. s_net_ok_utc = clk_utc of the CCLK sample that was successfully written to the RTC.
 * sim76xx clears clk_valid whenever the modem (re)starts, so every modem session starts "not applied" and
 * GPS can act as the fallback until a network sample is written. */
static uint32_t s_net_ok_utc = 0;

static int rtc_net_time_current(void)
{
    sim76xx_t *m = &g_app.board->sim76xx;
    return m->clk_valid && m->clk_utc == s_net_ok_utc;
}

/* GPS -> RTC (fallback only). gps->tim stays stale after an invalid RMC, so require a fix. */
static void rtc_sync_from_gps(void)
{
    sx_gps_t *gps = &g_app.board->gps;
    if (gps->latitude == 0.0f || gps->longtitude == 0.0f) return;
    if (gps->tim.tm_mday < 1) return;
    if (rtc_net_time_current()) return;      /* network time already applied this session: it wins */
    /* gps->tim is already UTC+7 (gps.c); mday may be 32 at month end, rtc_tm_to_secs() normalises it */
    int rc = rtc_set_local_secs(rtc_tm_to_secs(&gps->tim));
    if (rc != RX8130CE_OK) log_warn(TAG, "RTC sync from GPS failed (rc=%d)", rc);
}

/* Network time (sim76xx) -> RTC. Called every loop; each new CCLK sample is applied once. */
static void app_sync_rtc_from_modem(void)
{
    static uint32_t s_applied_snap = 0;
    static uint8_t  s_tries = 0;
    sim76xx_t *m = &g_app.board->sim76xx;
    uint32_t utc;

    if (!m->clk_valid || m->clk_utc == s_applied_snap) return;
    if (!sim76xx_get_utc_now(m, &utc)) return;

    int rc = rtc_set_local_secs((int64_t)utc + APP_RTC_TZ_OFFSET_S);
    if (rc == RX8130CE_OK) {
        s_applied_snap = m->clk_utc;
        s_net_ok_utc   = m->clk_utc;         /* from now on GPS must not overwrite this session */
        s_tries = 0;
        log_info(TAG, "RTC synced from network time: %02d:%02d:%02d %02d/%02d/20%02d (UTC+7)",
                 g_app.time.hour, g_app.time.min, g_app.time.sec,
                 g_app.time.day, g_app.time.month, g_app.time.year);
    } else if (++s_tries >= 3) {
        log_warn(TAG, "RTC sync from network time failed (rc=%d) - giving up on this sample", rc);
        s_applied_snap = m->clk_utc;         /* s_net_ok_utc stays unchanged: GPS fallback remains allowed */
        s_tries = 0;
    }
}

static void get_time_exrtc(void){
    rx8130ce_get_time(&board.rtc, &g_app.time);
    log_debug("RTC", "%02d:%02d:%02d %02d/%02d/20%02d",
         g_app.time.hour, g_app.time.min, g_app.time.sec,
         g_app.time.day, g_app.time.month, g_app.time.year);
}

void app_mode_full_pw(void){
    g_app.app_mode = APP_MODE_FULL_POWER;
}

void app_mode_enter_sleep(void){
    g_app.app_mode = APP_MODE_ENTER_SLEEP;
}

void app_mode_sleep(void){
    g_app.app_mode = APP_MODE_SLEEP;
}

static void write_gps_log(const char *event)
{
    static uint32_t s_log_count = 0;
    char line[192];
    sx_gps_t *gps = &g_app.board->gps;

    rx8130ce_get_time(&g_app.board->rtc, &g_app.time);

    if (gps->latitude != 0.0f && gps->longtitude != 0.0f)
    {
        snprintf(line, sizeof(line),
                 "[%s] fix=1 lat=%.6f lon=%.6f rssi=%d alt=%.6f spd=%.6f sat=%d time=%02d:%02d:%02d date=%02d/%02d/20%02d\n",
                 event,
                 gps->latitude, gps->longtitude,
                 sim76xx_get_rssi(&g_app.board->sim76xx),
                 gps->altitude, gps->speed, gps->satellites,
                 g_app.time.hour, g_app.time.min, g_app.time.sec,
                 g_app.time.day,  g_app.time.month, g_app.time.year);
    }
    else
    {
        snprintf(line, sizeof(line),
                 "[%s] fix=0 lat=0.000000 lon=0.000000 rssi=%d alt=0.000000 spd=0.000000 sat=0 time=%02d:%02d:%02d date=%02d/%02d/20%02d\n",
                 event,
                 sim76xx_get_rssi(&g_app.board->sim76xx),
                 g_app.time.hour, g_app.time.min, g_app.time.sec,
                 g_app.time.day,  g_app.time.month, g_app.time.year);
    }

    sx_storage_delete(GPS_LOG_FILE_PATH);
    sx_storage_append(GPS_LOG_FILE_PATH, line, strlen(line));
    s_log_count++;
    log_info(TAG, "GPS log written: %s", line);
}

static void write_calib_imu_data(void){
    bno055_calib_data_t cal = {0};
    if(bno055_get_calib_data(&board.imu, &cal)!=0)return;
    char line[128];
    snprintf(line, sizeof(line),
             "acc_x=%d acc_y=%d acc_z=%d "
             "mag_x=%d mag_y=%d mag_z=%d "
             "gyr_x=%d gyr_y=%d gyr_z=%d "
             "acc_r=%d mag_r=%d\n",
             cal.acc_x, cal.acc_y, cal.acc_z,
             cal.mag_x, cal.mag_y, cal.mag_z,
             cal.gyr_x, cal.gyr_y, cal.gyr_z,
             cal.acc_radius, cal.mag_radius);
    sx_storage_write(IMU_CALIB_FILE_PATH, line, strlen(line));
    log_info(TAG, "calib saved: %s", line);
}

bool imu_calib_load(void)
{
    if (!sx_storage_exists(IMU_CALIB_FILE_PATH)) return false;

    char line[128] = {0};
    if (sx_storage_read(IMU_CALIB_FILE_PATH, line, sizeof(line) - 1) != SX_STORAGE_OK) return false;

    bno055_calib_data_t cal = {0};
    int parsed = sscanf(line,
                        "acc_x=%hd acc_y=%hd acc_z=%hd "
                        "mag_x=%hd mag_y=%hd mag_z=%hd "
                        "gyr_x=%hd gyr_y=%hd gyr_z=%hd "
                        "acc_r=%hd mag_r=%hd",
                        &cal.acc_x, &cal.acc_y, &cal.acc_z,
                        &cal.mag_x, &cal.mag_y, &cal.mag_z,
                        &cal.gyr_x, &cal.gyr_y, &cal.gyr_z,
                        &cal.acc_radius, &cal.mag_radius);

    if (parsed != 11) {
        log_error(TAG, "calib parse failed: %d/11", parsed);
        return false;
    }

    bno055_set_calib_data(&board.imu, &cal);
    log_debug(TAG, "calib loaded OK");
    return true;
}

void app_notify_usb_connected(void)
{
    g_app.usb_connect_pending = 1;
    g_app.app_mode = APP_MODE_FULL_POWER;

    /* A connect cancels any sleep that was requested but not finished (ENTER_SLEEP). Without this the
     * flag stays set and the next unplug is ignored by app_request_sleep(). */
    g_app.sleep_requested        = 0;
    g_app.enter_sleep_elapsed_ms = 0;
    g_app.enter_sleep_published  = 0;
}

static void _apply_default_config(sx_user_mqtt_cfg_t *cfg){
    log_warn("AppCfg", "Using default hardcoded config");
}

static void _sync_gps_log_to_disk(void);

static void _handle_usb_connected(void)
{
    g_app.sleep.wake_reason = WAKE_REASON_VBUS;

    log_info(TAG, "=== USB connected — restarting ===");

    /* The USB cable was plugged while the MCU was in STOP (USB IRQ off, clock stopped), so the host's
     * enumeration got no answer and it gave up ("unknown USB device"). tud_mounted() cannot be trusted
     * here: the stack never saw the unplug (no VBUS sensing, IRQ off in STOP), so it still reports the old
     * configuration (log shows "USB tiny resumed"). Always toggle the D+ pull-up so the host sees a fresh
     * plug and enumerates CDC + MSC again. */
    log_debug(TAG, "USB re-enumerate (tud_mounted=%d tud_connected=%d)", tud_mounted() ? 1 : 0, tud_connected() ? 1 : 0);
    sx_usb_tiny_msc_disconnect();
    sx_delay_ms(500);

    /* Refresh log_gps.csv on the MSC disk while the host cannot see the drive (D+ pull-up is off), so the
     * host reads the new FAT when it enumerates again. Writing after the host mounted the disk is not
     * reliable: the host keeps its own cache of the FAT/directory and is not told that the media changed. */
    _sync_gps_log_to_disk();

    sx_usb_tiny_msc_connect();

    g_app.last_publish_done = 0;
    g_app.publish_count = 0;
    g_app.enter_sleep_published = 0;
    g_app.enter_sleep_elapsed_ms = 0;
    g_app.sleep_requested = 0;
    g_app.mqtt_stopped = 0;
    g_app.subscribed = 0;
    g_app.publish_elapsed = 0;

    sx_sleep_manager_reset_wake(&g_app.sleep_mgr);
    sx_user_mqtt_force_disconnect();

    g_app.board->sim76xx.base.isBusy = 0;
    g_app.board->sim76xx.base.buff_id = 0;
    memset(g_app.board->sim76xx.base.buff, 0, MODEM_RX_BUFFER_SIZE);
    g_app.board->sim76xx.state = SIM76XX_STATE_IDLE;

    sx_board_uart_resume_it();

    sim76xx_power_off(&g_app.board->sim76xx);
    sim76xx_power_on(&g_app.board->sim76xx);
    sim76xx_start(&g_app.board->sim76xx);

    gps_power_on(&g_app.board->gps);
    gps_it_handle();
}

/*  MQTT callbacks  */
static void _on_connected(void)
{
    log_info(TAG, "MQTT connected");
    g_app.subscribed = 0;
    g_app.first_pub_pending = 1;
}

static void _on_disconnected(void)
{
    log_warn(TAG, "MQTT disconnected");
    g_app.subscribed = 0;

    //
    if (g_app.app_mode == APP_MODE_WAKE_PUBLISH && !g_app.last_publish_done) {
        g_app.sleep_mgr.published = 0;
    }
}

static void _on_message(const char *topic, const char *message)
{
    if (!message || message[0] == '\0')
        return;
    log_debug(TAG, "SUB [%s] = %s", topic, message);
}

static void _on_publish(int success)
{
    log_debug(TAG, "Publish %s", success ? "OK" : "FAIL");

    if (g_app.app_mode == APP_MODE_WAKE_PUBLISH)
    {
        g_app.publish_count++;
        log_debug(TAG, "publish_count = %d", g_app.publish_count);
        if (g_app.publish_count >= 2)
        {
            g_app.last_publish_done = 1;
            g_app.publish_count     = 0;
        }
    }

    if (g_app.app_mode == APP_MODE_FULL_POWER)
        g_app.publish_elapsed = 0;
}

static sx_user_mqtt_cfg_t s_mqtt_cfg = {
    .apn = APN,
    .username_apn = USERNAME_APN,
    .password_apn = PASSWORD_APN,
    .broker = MQTT_HOST,
    .port = MQTT_PORT,
    .client_id = MQTT_CLIENT_ID,
    .username = MQTT_USER,
    .password = MQTT_PASS,
    .keepalive = MQTT_KEEPALIVE,
    .clean_session = 0,
    .on_connected = _on_connected,
    .on_disconnected = _on_disconnected,
    .on_message = _on_message,
    .on_publish = _on_publish,
};

static void read_last_gps(void);

/*  Publish helpers  */
static void publish_gsm(char *mode)
{
    char msg[256];
    char topic[128];
    
    rx8130ce_get_time(&g_app.board->rtc, &g_app.time);

    snprintf(topic, sizeof(topic), "%s%s", MQTT_TOPIC_GSM, g_app.device_name);

    snprintf(msg, sizeof(msg),
             "{\"mode\":\"%s\",\"rssi\":%d,\"ip\":\"%s\",\"imei\":\"%s\",\"apn\":\"%s\",\"vbat\":%f,"
             "\"time\":\"%02d:%02d:%02d\",\"date\":\"%02d/%02d/20%02d\"}",
             mode,
             sim76xx_get_rssi(&g_app.board->sim76xx),
             sim76xx_get_ip(&g_app.board->sim76xx),
             sim76xx_get_imei(&g_app.board->sim76xx),
             sim76xx_get_apn(&g_app.board->sim76xx), g_app.board->voltage.v_bat,
             g_app.time.hour, g_app.time.min, g_app.time.sec,
             g_app.time.day, g_app.time.month, g_app.time.year);

    sx_user_mqtt_publish(topic, msg);
    log_debug(TAG, "GSM: %s", msg);
}

// static void publish_gps(char *mode)
// {
//     char msg[256];
//     char topic[128];
//     sx_gps_t *gps = &g_app.board->gps;
//     set_time_exrtc(board.gps.tim.tm_sec, board.gps.tim.tm_min, board.gps.tim.tm_hour, (board.gps.tim.tm_mday / 7 +1), board.gps.tim.tm_mday,
//                             board.gps.tim.tm_mon, board.gps.tim.tm_year - 100);
    
//     rx8130ce_get_time(&g_app.board->rtc, &g_app.time);

//     snprintf(topic, sizeof(topic), "%s%s", MQTT_TOPIC_GPS, g_app.device_name);

//     if (gps->latitude != 0.0f && gps->longtitude != 0.0f) {
//         g_app.last_lat = gps->latitude;
//         g_app.last_lon = gps->longtitude;
//         snprintf(msg, sizeof(msg),
//                  "{\"mode\":\"%s\",\"fix\":1,\"lat\":%.6f,\"lon\":%.6f,\"Vbat\":%f"
//                  "\"time\":\"%02d:%02d:%02d\",\"date\":\"%02d/%02d/20%02d\"}",
//                  mode, gps->latitude, gps->longtitude, g_app.board->voltage.v_bat,
//                  g_app.time.hour, g_app.time.min, g_app.time.sec,
//                  g_app.time.day, g_app.time.month, g_app.time.year);
//     } else {
//         read_last_gps();
//         snprintf(msg, sizeof(msg),
//                  "{\"mode\":\"%s\",\"fix\":0,\"lat\":%.6f,\"lon\":%.6f,\"Vbat\":%f"
//                  "\"time\":\"%02d:%02d:%02d\",\"date\":\"%02d/%02d/20%02d\"}",
//                  mode, g_app.last_lat, g_app.last_lon, g_app.board->voltage.v_bat,
//                  g_app.time.hour, g_app.time.min, g_app.time.sec,
//                  g_app.time.day, g_app.time.month, g_app.time.year);
//     }
//     sx_user_mqtt_publish(topic, msg);
//     log_info(TAG, "GPS: %s", msg);
// }

static void publish_gps(char *mode)
{
    char v_str[128];
    char msg[256];
    char topic[128];
    sx_gps_t *gps = &g_app.board->gps;
    struct tm t = {0};
    rx8130ce_get_time(&g_app.board->rtc, &g_app.time);
    t.tm_year = g_app.time.year + 100;  
    t.tm_mon  = (g_app.time.month >= 1) ? (g_app.time.month - 1) : 0;   /* RTC 1..12 -> tm_mon 0..11 */
    t.tm_mday = g_app.time.day;
    t.tm_hour = g_app.time.hour;
    t.tm_min  = g_app.time.min;
    t.tm_sec  = g_app.time.sec;
    time_t unix_ts = mktime(&t);
    snprintf(topic, sizeof(topic), "%s%s", MQTT_TOPIC_GPS, g_app.device_name);
    float lat = 0.0f, lon = 0.0f, alt = 0.0f, spd = 0.0f;
    int sat = 0;
    int fix = 0; 

    if (gps->latitude != 0.0f && gps->longtitude != 0.0f)
    {
        lat = gps->latitude;
        lon = gps->longtitude;
        alt = gps->altitude;
        spd = gps->speed;
        sat = gps->satellites;
        fix = 1; 
        rtc_sync_from_gps();
    }
    else
    {
        fix = 0;
        read_last_gps();
        lat = g_app.last_lat;
        lon = g_app.last_lon;
    }

    // Build inner JSON string (escaped) - thêm \"fix\"
    snprintf(v_str, sizeof(v_str),
             "{\\\"lat\\\":%.6f,\\\"lon\\\":%.6f,\\\"alt\\\":%.6f,\\\"spd\\\":%.6f,\\\"sat\\\":%d,\\\"fix\\\":%d}",
             lat, lon, alt, spd, sat, fix); // <-- thêm fix vào cuối

    // Build outer JSON
    snprintf(msg, sizeof(msg),
             "{\"v\":\"%s\",\"time\":%ld}",
             v_str, (long)unix_ts);
    sx_user_mqtt_publish(topic, msg);
    log_debug(TAG, "GPS: %s", msg);
}

static void read_last_gps(void)
{
    int32_t file_size = sx_storage_size(GPS_LOG_FILE_PATH);
    if (file_size <= 0) {
        log_info(TAG, "NO DATA FROM EXFLASH!");
        return;
    }

    static uint8_t s_tail_buf[256];
    uint32_t read_len = (file_size < (int32_t)sizeof(s_tail_buf))
                            ? (uint32_t)file_size
                            : (uint32_t)sizeof(s_tail_buf) - 1;
    uint32_t offset = (uint32_t)file_size - read_len;

    sx_storage_err_t err = sx_storage_read_partial(
        GPS_LOG_FILE_PATH, s_tail_buf, offset, read_len);
    if (err != SX_STORAGE_OK) {
        log_error(TAG, "Flash read failed");
        return;
    }
    s_tail_buf[read_len] = '\0';

    int32_t end = (int32_t)read_len - 1;
    while (end >= 0 && (s_tail_buf[end] == '\n' || s_tail_buf[end] == '\r'))
        end--;

    int32_t start = end;
    while (start > 0 && s_tail_buf[start - 1] != '\n')
        start--;

    if (start > end) {
        log_warn(TAG, "Cannot find last line");
        return;
    }

    s_tail_buf[end + 1] = '\0';
    char *last_line = (char *)(s_tail_buf + start);
    log_debug(TAG, "Last log line: %s", last_line);

    // Parse
    int   fix = 0, rssi = 0;
    char  lat_str[16] = {0};
    char  lon_str[16] = {0};
    char  alt_str[16] = {0};
    char  spd_str[16] = {0};
    char  sat_str[16] = {0};
    char  time_str[12] = {0};
    char  date_str[12] = {0};

    int parsed = sscanf(last_line,
                        "%*s fix=%d lat=%15s lon=%15s rssi=%d alt=%15s spd=%15s sat=%15s time=%11s date=%11s",
                        &fix, lat_str, lon_str, &rssi, alt_str, spd_str, sat_str, time_str, date_str);

    if (parsed >= 3 && fix == 1) {
        g_app.last_lat = strtof(lat_str, NULL);
        g_app.last_lon = strtof(lon_str, NULL);
        g_app.last_alt = (parsed >= 5) ? strtof(alt_str, NULL) : 0.0f;
        g_app.last_spd = (parsed >= 6) ? strtof(spd_str, NULL) : 0.0f;
        g_app.last_sat = (parsed >= 7) ? (int)strtol(sat_str, NULL, 10) : 0;
        snprintf(g_app.last_time_str, sizeof(g_app.last_time_str), "%s", time_str);
        snprintf(g_app.last_date_str, sizeof(g_app.last_date_str), "%s", date_str);
        log_info(TAG, "Last GPS loaded: lat=%.6f lon=%.6f alt=%.2f spd=%.2f sat=%d time=%s date=%s",
                g_app.last_lat, g_app.last_lon, g_app.last_alt, g_app.last_spd, g_app.last_sat,
                g_app.last_time_str, g_app.last_date_str);
    } else {
        log_warn(TAG, "No valid fix in last log");
    }
}

// static void _sync_gps_log_to_disk(void)
// {
//     log_info(TAG, "Syncing last GPS log entry to disk...");

//     if (!sx_storage_exists(GPS_LOG_FILE_PATH))
//     {
//         log_warn(TAG, "No GPS log on flash");
//         sx_user_msc_write(GPS_CSV_FILE_PATH,
//                           (const uint8_t *)"no data\n",
//                           strlen("no data\n"));
//         sx_user_msc_remount_disk();
//         return;
//     }

//     int32_t file_size = sx_storage_size(GPS_LOG_FILE_PATH);
//     if (file_size <= 0)
//     {
//         log_warn(TAG, "GPS log empty");
//         sx_user_msc_write(GPS_CSV_FILE_PATH,
//                           (const uint8_t *)"no data\n",
//                           strlen("no data\n"));
//         sx_user_msc_remount_disk();
//         return;
//     }

//     static uint8_t s_tail_buf[256];
//     uint32_t read_len = (file_size < (int32_t)sizeof(s_tail_buf))
//                             ? (uint32_t)file_size
//                             : (uint32_t)sizeof(s_tail_buf) - 1;
//     uint32_t offset = (uint32_t)file_size - read_len;

//     sx_storage_err_t err = sx_storage_read_partial(
//         GPS_LOG_FILE_PATH,
//         s_tail_buf,
//         offset,
//         read_len);
//     if (err != SX_STORAGE_OK)
//     {
//         log_error(TAG, "Flash read failed");
//         sx_user_msc_write(GPS_CSV_FILE_PATH,
//                           (const uint8_t *)"no data\n",
//                           strlen("no data\n"));
//         sx_user_msc_remount_disk();
//         return;
//     }

//     s_tail_buf[read_len] = '\0';

//     int32_t end = (int32_t)read_len - 1;
//     while (end >= 0 && (s_tail_buf[end] == '\n' || s_tail_buf[end] == '\r'))
//         end--;

//     int32_t start = end;
//     while (start > 0 && s_tail_buf[start - 1] != '\n')
//         start--;

//     if (start > end)
//     {
//         log_warn(TAG, "Cannot find last line");
//         sx_user_msc_write(GPS_CSV_FILE_PATH,
//                           (const uint8_t *)"no data\n",
//                           strlen("no data\n"));
//         sx_user_msc_remount_disk();
//         return;
//     }

//     s_tail_buf[end + 1] = '\0';
//     char *last_line = (char *)(s_tail_buf + start);
//     log_info(TAG, "Last log line: %s", last_line);

//     /* --- Parse --- */
//     int  fix  = 0;
//     int  rssi = 0;
//     char lat_str[16]  = {0};
//     char lon_str[16]  = {0};
//     char time_str[12] = {0};
//     char date_str[12] = {0};

//     static char s_csv_line[128];

//     int parsed = sscanf(last_line,
//                         "%*s fix=%d lat=%15s lon=%15s rssi=%d time=%11s date=%11s",
//                         &fix, lat_str, lon_str, &rssi, time_str, date_str);

//     log_info(TAG, "parsed=%d fix=%d lat=%s lon=%s rssi=%d t=%s d=%s",
//              parsed, fix, lat_str, lon_str, rssi, time_str, date_str);

//     if (parsed == 6 && fix == 1)
//     {
//         snprintf(s_csv_line, sizeof(s_csv_line),
//                  "%s1,%s,%s,%d,%s,%s",
//                  GPS_CSV_HEADER, lat_str, lon_str, rssi, time_str, date_str);
//     }
//     else
//     {
//         fix = 0; rssi = 0;
//         memset(time_str, 0, sizeof(time_str));
//         memset(date_str, 0, sizeof(date_str));

//         parsed = sscanf(last_line,
//                         "%*s fix=%d rssi=%d time=%11s date=%11s",
//                         &fix, &rssi, time_str, date_str);

//         log_info(TAG, "fix=0 parsed=%d rssi=%d t=%s d=%s",
//                  parsed, rssi, time_str, date_str);

//         snprintf(s_csv_line, sizeof(s_csv_line),
//                  "%s0,0.000000,0.000000,%d,%s,%s",
//                  GPS_CSV_HEADER,
//                  rssi,
//                  time_str[0] ? time_str : "N/A",
//                  date_str[0] ? date_str : "N/A");
//     }

//     sx_user_msc_write(GPS_CSV_FILE_PATH,
//                       (const uint8_t *)s_csv_line,
//                       strlen(s_csv_line));

//     snprintf(lat_str, sizeof(lat_str), "%.6f", g_app.last_lat);
//     snprintf(lon_str, sizeof(lon_str), "%.6f", g_app.last_lon);
//     sx_user_msc_remount_disk();
//     log_info(TAG, "GPS CSV written: %s", s_csv_line);
// }

static void _sync_gps_log_to_disk(void)
{
    log_debug(TAG, "Syncing last GPS log entry to disk...");

    if (!sx_storage_exists(GPS_LOG_FILE_PATH))
    {
        log_warn(TAG, "No GPS log on flash");
        sx_user_msc_write(GPS_CSV_FILE_PATH,
                          (const uint8_t *)"no data\n",
                          strlen("no data\n"));
        sx_user_msc_remount_disk();
        return;
    }

    read_last_gps();

    static char s_csv_line[192];

    if (g_app.last_lat != 0.0f && g_app.last_lon != 0.0f)
    {
        snprintf(s_csv_line, sizeof(s_csv_line),
                "%s1,%.6f,%.6f,%.6f,%.6f,%d,%s,%s",
                GPS_CSV_HEADER,
                g_app.last_lat, g_app.last_lon,
                g_app.last_alt, g_app.last_spd, g_app.last_sat,
                g_app.last_time_str, g_app.last_date_str);
    }
    else
    {
        snprintf(s_csv_line, sizeof(s_csv_line),
                "%s0,0.000000,0.000000,0.000000,0.000000,0,N/A,N/A",
                GPS_CSV_HEADER);
    }

    sx_user_msc_write(GPS_CSV_FILE_PATH,
                      (const uint8_t *)s_csv_line,
                      strlen(s_csv_line));
    sx_user_msc_remount_disk();
    log_info(TAG, "GPS CSV written: %s", s_csv_line);
}

void app_sync_gps_log_to_disk(void)
{
    g_app.s_sync_disk_pending = 1;
}

void app_request_sleep(void)
{
    if (g_app.sleep_requested)
        return;
    g_app.sleep_requested = 1;
    g_app.enter_sleep_elapsed_ms = 0;
    g_app.enter_sleep_published = 0;
    g_app.last_publish_done = 0;
    g_app.publish_count = 0;
    g_app.mqtt_stopped = 0;
    g_app.app_mode = APP_MODE_ENTER_SLEEP;
    log_info(TAG, "Sleep requested");
}

static void app_read_config_file(TrackingApp_t *app)
{
    // Read config file to initialize MQTT config (e.g. broker address, credentials)
    // For simplicity, we use hardcoded config in this example, but in real application, you can read from flash or other storage
    // and populate s_mqtt_cfg accordingly.

    int ret = sx_fatfs_remount();
    sx_fatfs_debug_list();
    if (ret != 0) {
        log_warn("AppCfg", "FatFS remount failed (%d) — using default config", ret);
        _apply_default_config(&s_mqtt_cfg);
        return;
    }

    FILINFO fno;
    FRESULT fres = f_stat("0:/config.json", &fno);
    if (fres != FR_OK) {
        log_warn("AppCfg", "config.json not found (f_stat=%d) — using default config", fres);
        _apply_default_config(&s_mqtt_cfg);
        return;
    }
    log_debug("AppCfg", "config.json found, size=%lu bytes", fno.fsize);

    /*  3. read content file  */
    uint32_t out_len = 0;
    sx_user_msc_err_t err = sx_user_msc_read(
        "0:/config.json",
        config_json.s_cfg_buf,
        sizeof(config_json.s_cfg_buf) - 1,
        &out_len
    );
    if (err != SX_USER_MSC_OK || out_len == 0) {
        log_error("AppCfg", "Read config.json failed (err=%d) — using default config", err);
        _apply_default_config(&s_mqtt_cfg);
        return;
    }
    config_json.s_cfg_buf[out_len] = '\0';
    log_debug("AppCfg", "cfg.json content: %s", (char *)config_json.s_cfg_buf);

    /*  4. Parse JSON  */
    cJSON *root = cJSON_Parse((const char *)config_json.s_cfg_buf);
    if (root == NULL) {
        log_error("AppCfg", "JSON parse failed — using default config");
        _apply_default_config(&s_mqtt_cfg);
        return;
    }

    /*  5. APN  */
    cJSON *apn = cJSON_GetObjectItem(root, "apn");
    if (apn) {
        cJSON *name = cJSON_GetObjectItem(apn, "name");
        cJSON *user = cJSON_GetObjectItem(apn, "user");
        cJSON *pass = cJSON_GetObjectItem(apn, "password");

        log_debug("AppCfg", "APN name    : %s", (!name || cJSON_IsNull(name))   ? "NULL" : name->valuestring);
        log_debug("AppCfg", "APN user    : %s", (!user || cJSON_IsNull(user))   ? "NULL" : user->valuestring);
        log_debug("AppCfg", "APN pass    : %s", (!pass || cJSON_IsNull(pass))   ? "NULL" : pass->valuestring);

        if (name && !cJSON_IsNull(name) && name->valuestring) {
            strncpy(config_json.s_apn_name, name->valuestring, sizeof(config_json.s_apn_name) - 1);
            strncpy(s_mqtt_cfg.apn, config_json.s_apn_name, sizeof(s_mqtt_cfg.apn) - 1);
        }
    } else {
        log_warn("AppCfg", "No 'apn' field in config");
    }

    /*  6. MQTT  */
    cJSON *mqtt = cJSON_GetObjectItem(root, "mqtt");
    if (mqtt) {
        cJSON *host      = cJSON_GetObjectItem(mqtt, "host");
        cJSON *port      = cJSON_GetObjectItem(mqtt, "port");
        cJSON *client_id = cJSON_GetObjectItem(mqtt, "client_id");
        cJSON *user      = cJSON_GetObjectItem(mqtt, "user_name");
        cJSON *pass      = cJSON_GetObjectItem(mqtt, "password");

        log_debug("AppCfg", "MQTT host     : %s", (host && host->valuestring)           ? host->valuestring      : "NULL");
        log_debug("AppCfg", "MQTT port     : %d", port                                  ? port->valueint         : 0);
        log_debug("AppCfg", "MQTT client_id: %s", (client_id && client_id->valuestring) ? client_id->valuestring : "NULL");
        log_debug("AppCfg", "MQTT user     : %s", (user && user->valuestring)            ? user->valuestring      : "NULL");
        log_debug("AppCfg", "MQTT pass     : %s", (pass && pass->valuestring)            ? pass->valuestring      : "NULL");

        if (host && host->valuestring) {
            strncpy(config_json.s_mqtt_host, host->valuestring, sizeof(config_json.s_mqtt_host) - 1);
            s_mqtt_cfg.broker = config_json.s_mqtt_host;
        }
        if (port) {
            s_mqtt_cfg.port = (uint16_t)port->valueint;
        }
        if (client_id && client_id->valuestring) {
            strncpy(config_json.s_mqtt_client_id, client_id->valuestring, sizeof(config_json.s_mqtt_client_id) - 1);
            s_mqtt_cfg.client_id = config_json.s_mqtt_client_id;
        }
        if (user && user->valuestring) {
            strncpy(config_json.s_mqtt_user, user->valuestring, sizeof(config_json.s_mqtt_user) - 1);
            s_mqtt_cfg.username = config_json.s_mqtt_user;
        }
        if (pass && pass->valuestring) {
            strncpy(config_json.s_mqtt_pass, pass->valuestring, sizeof(config_json.s_mqtt_pass) - 1);
            s_mqtt_cfg.password = config_json.s_mqtt_pass;
        }
    } else {
        log_warn("AppCfg", "No 'mqtt' field in config — using default MQTT config");
    }

    /* 7. time_sleeps  */
    cJSON *sleep_t = cJSON_GetObjectItem(root, "time_sleeps");
    if (sleep_t && cJSON_IsNumber(sleep_t) && sleep_t->valueint > 0) {
        config_json.time_sleep_ms = (uint32_t)sleep_t->valueint * 1000U;
        log_debug("AppCfg", "time_sleeps loaded: %lu ms", config_json.time_sleep_ms);
    } else {
        log_warn("AppCfg", "No valid 'time_sleeps' — using default %lu ms",
                config_json.time_sleep_ms);
    }

    /* 8. Device ID */
    cJSON *device_t = cJSON_GetObjectItem(root, "device_id");
    log_debug("AppCfg", "Device ID     : %s", (device_t && device_t->valuestring) ? device_t->valuestring : "NULL");
    if (device_t && device_t->valuestring) {
        strncpy(config_json.s_device_name, device_t->valuestring, sizeof(config_json.s_device_name) - 1);
        strncpy(g_app.device_name, config_json.s_device_name, sizeof(g_app.device_name) - 1);
    }

    cJSON *time_pub_t = cJSON_GetObjectItem(root, "time_publish");
    if (time_pub_t && cJSON_IsNumber(time_pub_t) && time_pub_t->valueint > 0) {
        config_json.s_time_publish = (uint32_t)time_pub_t->valueint * 1000U;
        log_debug("AppCfg", "time_publish loaded: %lu ms", config_json.s_time_publish);
    } else {
        log_warn("AppCfg", "No valid 'time_publish' — using default %lu ms",
                config_json.s_time_publish);
    }

    cJSON_Delete(root);
    log_info("AppCfg", "Config loaded OK");
    
    /*
    Crete a default config file if it doesn't exist, or if parsing failed. This is optional and can be implemented as needed.
    create_default_config_file:
        // Create a JSON object with default config values
        // goto application init again to read the newly created config file
    */
}

/*  Init  */
void app_init(void)
{
    log_info(TAG, "App init");
    config_json.time_sleep_ms = SX_TIME_IN_SLEEP;
    config_json.time_wake_ms  = SX_TIME_IN_WAKE;
    // application_init:
    g_app.board = &board;
    g_app.app_mode = APP_MODE_FULL_POWER;
    g_app.publish_elapsed = 0;
    g_app.subscribed = 0;
    g_app.first_pub_pending = 0;
    g_app.last_publish_done = 0;
    g_app.enter_sleep_published = 0;
    g_app.mqtt_stopped = 0;
    g_app.publish_count = 0;
    g_app.enter_sleep_elapsed_ms = 0;
    g_app.sleep_requested = 0;
    g_app.usb_connect_pending = 0;
    
    log_debug(TAG, "Read Config file...");

    app_read_config_file(&g_app);
    
    sx_sleep_init(&g_app.sleep, &sx_sleep_ops, &hrtc);

    sx_sleep_manager_init(&g_app.sleep_mgr, &g_app.sleep,
                          &g_app.board->sim76xx, &g_app.board->gps);

    g_app.sleep_mgr.sleep_ms        = config_json.time_sleep_ms;
    g_app.sleep_mgr.wake_timeout_ms = config_json.time_wake_ms;
    log_info(TAG, "Sleep config applied — sleep=%lu ms, wake=%lu ms", g_app.sleep_mgr.sleep_ms, g_app.sleep_mgr.wake_timeout_ms);


    sx_user_mqtt_nontls_init(&s_mqtt_cfg);
    read_last_gps();

    /* Boot with USB already plugged: log_gps.csv on the MSC disk still holds the previous content. Update it,
     * then re-enumerate so the host reads the new FAT instead of its cached copy (and also recovers if it gave up
     * enumerating while the long init above was running). Skipped on battery: nobody is looking at the drive. */
    _sync_gps_log_to_disk();
    if (g_app.board->bq.present) {
        sx_usb_tiny_msc_disconnect();
        sx_delay_ms(500);
        sx_usb_tiny_msc_connect();
    }

    gps_it_handle();

    publish_gps("init");
    publish_gsm("init");

    app_at_init();
    
    log_info(TAG, "App init done");
    return;
}

/* ---- Phase 0: observe BQ25622 VBUS_STAT (read-only, no action taken) ---- */
#ifndef BQ_PHASE0_DEBUG
#define BQ_PHASE0_DEBUG            1
#endif
#define BQ_PHASE0_READ_MS          500U    /* I2C read period            */
#define BQ_PHASE0_HEARTBEAT_MS     5000U   /* print even if unchanged    */

#if BQ_PHASE0_DEBUG
static void bq_phase0_debug(uint32_t delta_ms)
{
    static uint32_t hb_acc = 0;
    static int16_t  last_vbus = -1;         /* -1 = nothing printed yet   */

    hb_acc += delta_ms;

    bq25622_t *bq = &g_app.board->bq;
    if (!bq->valid)
        return;

    /* Passive: bq25622_poll() (app_vbus_process) does the I2C read, this only prints it. */
    if (bq->vbus_stat != last_vbus || hb_acc >= BQ_PHASE0_HEARTBEAT_MS) {
        log_debug(TAG, "BQ VBUS_STAT=%u%u%u CHG_STAT=%u -> %s",
                 (bq->vbus_stat >> 2) & 1, (bq->vbus_stat >> 1) & 1, bq->vbus_stat & 1,
                 bq->chg_stat,
                 bq->vbus_stat == BQ25622_VBUS_NONE ? "on battery" :
                 bq->vbus_stat == BQ25622_VBUS_OTG  ? "OTG" : "VBUS present");
        last_vbus = bq->vbus_stat;
        hb_acc = 0;
    }
}
#endif

/* ---- Phase 3b: USB presence from the charger (VBUS_STAT over I2C1) ---------------------------
 * No GPIO on v1.4. bq25622_poll() reads every BQ25622_POLL_INTERVAL_MS, debounces, and never treats
 * an I2C error as "unplugged". Sleep is requested here, not by the USB stack (see tud_umount_cb). */
#ifndef SX_SLEEP_ON_BATTERY
#define SX_SLEEP_ON_BATTERY   1      /* 1 = boot without USB goes to the sleep flow; 0 = stay in FULL_POWER */
#endif
#ifndef SX_BOOT_SLEEP_MAX_WAIT_MS
#define SX_BOOT_SLEEP_MAX_WAIT_MS  120000U   /* boot on battery: wait this long for MQTT before sleeping anyway */
#endif

static void app_vbus_process(uint32_t delta_ms)
{
    static uint8_t  s_boot_checked       = 0;
    static uint8_t  s_boot_sleep_pending = 0;
    static uint32_t s_boot_wait_ms       = 0;
    bq25622_t *bq = &g_app.board->bq;

    bq25622_event_t ev = bq25622_poll(bq, delta_ms);

    if (!s_boot_checked && bq->valid) {
        s_boot_checked = 1;
        log_info(TAG, "Power source at boot: %s", bq->present ? "USB" : "battery");
        if (!bq->present && SX_SLEEP_ON_BATTERY)
            s_boot_sleep_pending = 1;
        return;
    }

    /* Booted on battery: let the modem connect and publish once (ENTER_SLEEP publishes "enter sleep"),
     * then sleep. Requesting sleep right away would make ENTER_SLEEP give up on MQTT after 5 s, before
     * the modem has even registered. */
    if (s_boot_sleep_pending)
    {
        s_boot_wait_ms += delta_ms;
        if (bq->present) {
            s_boot_sleep_pending = 0;               /* USB showed up meanwhile */
        } else if (g_app.app_mode == APP_MODE_FULL_POWER &&
                   (sx_user_mqtt_is_connected() || s_boot_wait_ms >= SX_BOOT_SLEEP_MAX_WAIT_MS)) {
            s_boot_sleep_pending = 0;
            log_info(TAG, "Boot on battery - starting sleep flow (mqtt=%d, waited %lu ms)",
                     sx_user_mqtt_is_connected() ? 1 : 0, (unsigned long)s_boot_wait_ms);
            app_request_sleep();
        }
    }

    if (ev == BQ25622_EVT_UNPLUGGED)
    {
        if (g_app.app_mode == APP_MODE_FULL_POWER)
            app_request_sleep();
    }
    else if (ev == BQ25622_EVT_PLUGGED)
    {
        if (g_app.app_mode == APP_MODE_ENTER_SLEEP)
        {
            /* modem and MQTT are still running: just cancel the pending sleep */
            log_info(TAG, "USB plugged during ENTER_SLEEP - back to FULL_POWER");
            g_app.sleep_requested        = 0;
            g_app.enter_sleep_elapsed_ms = 0;
            g_app.enter_sleep_published  = 0;
            g_app.app_mode               = APP_MODE_FULL_POWER;
        }
        else if (g_app.app_mode == APP_MODE_WAKE_PUBLISH)
        {
            /* the SIM may not even be powered yet at this point: use the full restart path */
            log_info(TAG, "USB plugged during WAKE_PUBLISH - full restart");
            app_notify_usb_connected();
        }
    }
}

/*  Process  */
void app_process(uint32_t timestamp)
{

    if (g_app.usb_connect_pending)
    {
        g_app.usb_connect_pending = 0;
        _handle_usb_connected();
    }

    sx_usb_tiny_process(&g_app.board->usb);
    sx_diskio_process();
    if (g_app.s_sync_disk_pending) {
        g_app.s_sync_disk_pending = 0;
        _sync_gps_log_to_disk();  
    }

    if (sx_usb_tiny_available(&g_app.board->usb)) {
        uint8_t buf[64];
        int len = sx_usb_tiny_read(&g_app.board->usb, buf, sizeof(buf), 10);
        log_debug(TAG, "USB Tiny received %d bytes: %.*s", len, len, buf);
        if (len > 0) {
            app_at_process((const char *)buf, len);
        }
    }

    app_sync_rtc_from_modem();   /* network time (CCLK?) -> external RTC, once per new sample */
    app_vbus_process(timestamp);          /* BQ VBUS_STAT -> plug/unplug events (no GPIO on v1.4) */
    check_charge();
#if BQ_PHASE0_DEBUG
    bq_phase0_debug(timestamp);
#endif

    switch (g_app.app_mode)
    {
    /* ---------------------------------------------------------------- */
    case APP_MODE_FULL_POWER:
        gps_process(&g_app.board->gps, timestamp);
        sx_user_mqtt_poll(timestamp);
        g_app.publish_elapsed += timestamp;

        if (!sx_user_mqtt_is_connected())
        {
            g_app.subscribed = 0;
            break;
        }
        if (!g_app.subscribed)
        {
            g_app.subscribed = 1;
            sx_user_mqtt_subscribe(MQTT_SUB_TOPIC);
            g_app.publish_elapsed = 0;
            break;
        }

        /* First publish right after (re)connect. Wait until the modem is idle
         * (the subscribe command may still be in flight) and no publish is running,
         * then send GSM + GPS once. The periodic timer below takes over afterwards. */
        if (g_app.first_pub_pending && !sx_user_mqtt_is_publishing() &&
            !g_app.board->sim76xx.base.isBusy)
        {
            publish_gsm("full pw");
            publish_gps("full pw");
            if (sx_user_mqtt_is_publishing()) {   /* accepted by the modem */
                g_app.first_pub_pending = 0;
                g_app.publish_elapsed = 0;
            }
            break;
        }

        if (g_app.publish_elapsed >= config_json.s_time_publish && !sx_user_mqtt_is_publishing())
        {
            g_app.publish_elapsed = 0;  // reset 
            publish_gsm("full pw");
            publish_gps("full pw");    
        }
        break;

    /* ---------------------------------------------------------------- */
    case APP_MODE_ENTER_SLEEP:
        if (sx_usb_tiny_connected(&g_app.board->usb) || g_app.board->bq.present)
        {
            g_app.sleep_requested = 0;
            g_app.enter_sleep_elapsed_ms = 0;
            g_app.app_mode = APP_MODE_FULL_POWER;
            break;
        }

        g_app.enter_sleep_elapsed_ms += timestamp;
        gps_process(&g_app.board->gps, timestamp);
        sx_user_mqtt_poll(timestamp);

        if (!g_app.enter_sleep_published && sx_user_mqtt_is_connected())
        {
            g_app.enter_sleep_published = 1;
            publish_gsm("enter sleep");
            publish_gps("enter sleep");
        }

        uint8_t pub_done = g_app.last_publish_done;
        uint8_t mqtt_down = !sx_user_mqtt_is_connected() &&
                            g_app.enter_sleep_elapsed_ms >= 5000U;
        uint8_t timed_out = g_app.enter_sleep_elapsed_ms >= ENTER_SLEEP_TIMEOUT_MS;

        if (!pub_done && !mqtt_down && !timed_out)
            break;

        if (pub_done)
            log_info(TAG, "Enter sleep — publish OK");
        else if (mqtt_down)
            log_warn(TAG, "Enter sleep — MQTT unavailable");
        else
            log_warn(TAG, "Enter sleep — timeout");

        //write_gps_log("enter_sleep");
        g_app.sleep_requested = 0;
        g_app.enter_sleep_elapsed_ms = 0;
        g_app.last_publish_done = 0;
        g_app.enter_sleep_published = 0;
        g_app.publish_count = 0;
        g_app.mqtt_stopped = 0;
        g_app.app_mode = APP_MODE_SLEEP;
        break;

    /* ---------------------------------------------------------------- */
    case APP_MODE_SLEEP:
        sx_user_mqtt_force_disconnect();
        sx_user_mqtt_queue_flush();
        g_app.last_publish_done = 0;
        g_app.publish_count     = 0;
        sx_sleep_manager_enter(&g_app.sleep_mgr);

        {
            wake_reason_t wake_reason = sx_sleep_manager_get_wake_reason(&g_app.sleep_mgr);
            log_debug(TAG, "[SLEEP] wake_reason=%d after stop", wake_reason);

            if (wake_reason == WAKE_REASON_RTC)
            {
                log_info(TAG, "Woke by RTC timer");
                g_app.app_mode = APP_MODE_WAKE_PUBLISH;
            }
            else if (wake_reason == WAKE_REASON_VBUS)
            {
                log_info(TAG, "Woke by VBUS (BQ check)");
                app_notify_usb_connected();
            }
            else {
                log_warn(TAG, "Spurious wake — reason=%d", wake_reason);
                if (bq25622_refresh(&g_app.board->bq)) {
                    log_info(TAG, "VBUS present (BQ) after unknown wake — handling as USB connect");
                    app_notify_usb_connected();
                } else {
                    log_info(TAG, "No VBUS (BQ) after unknown wake — re-entering sleep");
                    g_app.app_mode = APP_MODE_SLEEP;
                }
            }
        }

        sx_sleep_manager_reset_wake(&g_app.sleep_mgr);
        break;
    /* ---------------------------------------------------------------- */
    case APP_MODE_WAKE_PUBLISH:
        gps_process(&g_app.board->gps, timestamp);
        sx_sleep_manager_wake_process(&g_app.sleep_mgr, timestamp);

        if (g_app.sleep_mgr.wake_step >= SX_WAKE_STEP_UART_RESUME) {
            sx_user_mqtt_poll(timestamp);
        }

        if (sx_sleep_manager_wake_tick(&g_app.sleep_mgr, timestamp))
        {
            log_warn(TAG, "Wake timeout — force sleep");
            g_app.last_publish_done = 0;
            g_app.publish_count = 0;
            g_app.app_mode = APP_MODE_SLEEP;
            break;
        }

        if (!sx_sleep_manager_is_wake_done(&g_app.sleep_mgr))
            break;

        if (!g_app.sleep_mgr.published && sx_user_mqtt_is_connected())
        {
            g_app.sleep_mgr.published = 1;
            g_app.last_publish_done   = 0;
            g_app.publish_count       = 0;
            rtc_sync_from_gps();
            publish_gsm("wake up");
            publish_gps("wake up");
        }

        if (g_app.sleep_mgr.published && g_app.last_publish_done)
        {
            write_gps_log("enter_sleep");
            rtc_sync_from_gps();
            g_app.last_publish_done = 0;
            g_app.publish_count = 0;
            g_app.app_mode = APP_MODE_SLEEP;
        }
        break;

    default:
        break;
    }
}