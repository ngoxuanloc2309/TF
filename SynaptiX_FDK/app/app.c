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

static const char *TAG = "App";

typedef struct TrackingApp
{
    // Add any app-level state here if needed
    app_mode_t app_mode;

    sx_sleep_t sleep;
    sx_sleep_manager_t sleep_mgr;

    uint32_t publish_elapsed;
    uint8_t subscribed ;
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

static config_json_t config_json;

static void set_time_exrtc(uint8_t sec, uint8_t min, uint8_t hour, uint8_t week, uint8_t day, uint8_t month, uint8_t year){
    g_app.time.sec = sec; 
    g_app.time.min = min;
    g_app.time.hour = hour;
    g_app.time.week = week;
    g_app.time.day = day;
    g_app.time.month = month;
    g_app.time.year = year;
    rx8130ce_set_time(&board.rtc, &g_app.time);
}

static void get_time_exrtc(void){
    rx8130ce_get_time(&board.rtc, &g_app.time);
    log_info("RTC", "%02d:%02d:%02d %02d/%02d/20%02d",
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
                 g_app.time.day,  g_app.time.month+1, g_app.time.year);
    }
    else
    {
        snprintf(line, sizeof(line),
                 "[%s] fix=0 lat=0.000000 lon=0.000000 rssi=%d alt=0.000000 spd=0.000000 sat=0 time=%02d:%02d:%02d date=%02d/%02d/20%02d\n",
                 event,
                 sim76xx_get_rssi(&g_app.board->sim76xx),
                 g_app.time.hour, g_app.time.min, g_app.time.sec,
                 g_app.time.day,  g_app.time.month+1, g_app.time.year);
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

static bool imu_calib_load(void)
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
    log_info(TAG, "calib loaded OK");
    return true;
}

void app_notify_usb_connected(void)
{
    g_app.usb_connect_pending = 1;
    g_app.app_mode = APP_MODE_FULL_POWER;
}

static void _apply_default_config(sx_user_mqtt_cfg_t *cfg){
    log_warn("AppCfg", "Using default hardcoded config");
}

static void _handle_usb_connected(void)
{
    g_app.sleep.wake_reason = WAKE_REASON_EXTI;

    log_info(TAG, "=== USB connected — restarting ===");

    g_app.last_publish_done = 0;
    g_app.publish_count = 0;
    g_app.enter_sleep_published = 0;
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
    log_info(TAG, "SUB [%s] = %s", topic, message);
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
             g_app.time.day, g_app.time.month+1, g_app.time.year);

    sx_user_mqtt_publish(topic, msg);
    log_info(TAG, "GSM: %s", msg);
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
//                  g_app.time.day, g_app.time.month+1, g_app.time.year);
//     } else {
//         read_last_gps();
//         snprintf(msg, sizeof(msg),
//                  "{\"mode\":\"%s\",\"fix\":0,\"lat\":%.6f,\"lon\":%.6f,\"Vbat\":%f"
//                  "\"time\":\"%02d:%02d:%02d\",\"date\":\"%02d/%02d/20%02d\"}",
//                  mode, g_app.last_lat, g_app.last_lon, g_app.board->voltage.v_bat,
//                  g_app.time.hour, g_app.time.min, g_app.time.sec,
//                  g_app.time.day, g_app.time.month+1, g_app.time.year);
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
    t.tm_mon  = g_app.time.month;
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
        set_time_exrtc(board.gps.tim.tm_sec, board.gps.tim.tm_min, board.gps.tim.tm_hour,
                       (board.gps.tim.tm_mday / 7 + 1), board.gps.tim.tm_mday,
                       board.gps.tim.tm_mon, board.gps.tim.tm_year - 100);
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
    log_info(TAG, "GPS: %s", msg);
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
    log_info(TAG, "Last log line: %s", last_line);

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
    log_info(TAG, "Syncing last GPS log entry to disk...");

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
    log_info("AppCfg", "config.json found, size=%lu bytes", fno.fsize);

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
    log_info("AppCfg", "cfg.json content: %s", (char *)config_json.s_cfg_buf);

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

        log_info("AppCfg", "APN name    : %s", (!name || cJSON_IsNull(name))   ? "NULL" : name->valuestring);
        log_info("AppCfg", "APN user    : %s", (!user || cJSON_IsNull(user))   ? "NULL" : user->valuestring);
        log_info("AppCfg", "APN pass    : %s", (!pass || cJSON_IsNull(pass))   ? "NULL" : pass->valuestring);

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

        log_info("AppCfg", "MQTT host     : %s", (host && host->valuestring)           ? host->valuestring      : "NULL");
        log_info("AppCfg", "MQTT port     : %d", port                                  ? port->valueint         : 0);
        log_info("AppCfg", "MQTT client_id: %s", (client_id && client_id->valuestring) ? client_id->valuestring : "NULL");
        log_info("AppCfg", "MQTT user     : %s", (user && user->valuestring)            ? user->valuestring      : "NULL");
        log_info("AppCfg", "MQTT pass     : %s", (pass && pass->valuestring)            ? pass->valuestring      : "NULL");

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
        log_info("AppCfg", "time_sleeps loaded: %lu ms", config_json.time_sleep_ms);
    } else {
        log_warn("AppCfg", "No valid 'time_sleeps' — using default %lu ms",
                config_json.time_sleep_ms);
    }

    /* 8. Device ID */
    cJSON *device_t = cJSON_GetObjectItem(root, "device_id");
    log_info("AppCfg", "Device ID     : %s", (device_t && device_t->valuestring) ? device_t->valuestring : "NULL");
    if (device_t && device_t->valuestring) {
        strncpy(config_json.s_device_name, device_t->valuestring, sizeof(config_json.s_device_name) - 1);
        strncpy(g_app.device_name, config_json.s_device_name, sizeof(g_app.device_name) - 1);
    }

    cJSON *time_pub_t = cJSON_GetObjectItem(root, "time_publish");
    if (time_pub_t && cJSON_IsNumber(time_pub_t) && time_pub_t->valueint > 0) {
        config_json.s_time_publish = (uint32_t)time_pub_t->valueint * 1000U;
        log_info("AppCfg", "time_publish loaded: %lu ms", config_json.s_time_publish);
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
    g_app.last_publish_done = 0;
    g_app.enter_sleep_published = 0;
    g_app.mqtt_stopped = 0;
    g_app.publish_count = 0;
    g_app.enter_sleep_elapsed_ms = 0;
    g_app.sleep_requested = 0;
    g_app.usb_connect_pending = 0;
    
    log_info(TAG, "Read Config file...");

    app_read_config_file(&g_app);
    
    sx_sleep_init(&g_app.sleep, &sx_sleep_ops, &hrtc);

    sx_sleep_manager_init(&g_app.sleep_mgr, &g_app.sleep,
                          &g_app.board->sim76xx, &g_app.board->gps);

    g_app.sleep_mgr.sleep_ms        = config_json.time_sleep_ms;
    g_app.sleep_mgr.wake_timeout_ms = config_json.time_wake_ms;
    log_info(TAG, "Sleep config applied — sleep=%lu ms, wake=%lu ms", g_app.sleep_mgr.sleep_ms, g_app.sleep_mgr.wake_timeout_ms);


    sx_user_mqtt_nontls_init(&s_mqtt_cfg);
    read_last_gps();
    gps_it_handle();

    publish_gps("init");
    publish_gsm("init");

    app_at_init();
    
    log_info(TAG, "App init done");
    return;
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
        log_info(TAG, "USB Tiny received %d bytes: %.*s", len, len, buf);
        if (len > 0) {
            app_at_process((const char *)buf, len);
        }
    }

    read_vol_pin(timestamp);
    check_charge();

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

        if (g_app.publish_elapsed >= config_json.s_time_publish && !sx_user_mqtt_is_publishing())
        {
            g_app.publish_elapsed = 0;  // reset 
            publish_gsm("full pw");
            publish_gps("full pw");    
        }
        break;

    /* ---------------------------------------------------------------- */
    case APP_MODE_ENTER_SLEEP:
        if (sx_usb_tiny_connected(&g_app.board->usb))
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
            else if (wake_reason == WAKE_REASON_EXTI)
            {
                log_info(TAG, "Woke by VBUS interrupt");
                app_notify_usb_connected();
            }
            else {
                log_warn(TAG, "Spurious wake — reason=%d", wake_reason);
                if (HAL_GPIO_ReadPin(VBUS_PORT, VBUS_PIN) == GPIO_PIN_SET) {
                    log_info(TAG, "VBUS is HIGH after unknown wake — handling as USB connect");
                    app_notify_usb_connected();
                } else {
                    log_info(TAG, "VBUS is LOW after unknown wake — re-entering sleep");
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
            set_time_exrtc(board.gps.tim.tm_sec, board.gps.tim.tm_min, board.gps.tim.tm_hour, (board.gps.tim.tm_mday / 7 +1), board.gps.tim.tm_mday,
                            board.gps.tim.tm_mon, board.gps.tim.tm_year - 100);
            publish_gsm("wake up");
            publish_gps("wake up");
        }

        if (g_app.sleep_mgr.published && g_app.last_publish_done)
        {
            write_gps_log("enter_sleep");
            set_time_exrtc(board.gps.tim.tm_sec, board.gps.tim.tm_min, board.gps.tim.tm_hour, (board.gps.tim.tm_mday / 7 +1), board.gps.tim.tm_mday,
                            board.gps.tim.tm_mon, board.gps.tim.tm_year - 100);
            g_app.last_publish_done = 0;
            g_app.publish_count = 0;
            g_app.app_mode = APP_MODE_SLEEP;
        }
        break;

    default:
        break;
    }
}