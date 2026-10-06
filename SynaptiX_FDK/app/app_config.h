#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define USER_DISK_LABEL_CREATE         "VD GPS"   

#define MQTT_HOST               "iot.coreflux.cloud"//"broker.emqx.io"//"test.mosquitto.org"//"103.226.248.21" "broker.hivemq.com"
#define MQTT_PORT               1883//1993
#define MQTT_CLIENT_ID          "stm32h5_001"
#define MQTT_USER               NULL
#define MQTT_PASS               NULL
#define MQTT_KEEPALIVE          60          /* seconds */
#define MQTT_TOPIC_GSM              "vindynamic/tracking/gsm/"
#define MQTT_TOPIC_GPS               "vindynamic/tracking/gps/"
#define MQTT_SUB_TOPIC          "stm32/cmd/#"
#define APN                     "v-internet"

#define USERNAME_APN            ""
#define PASSWORD_APN            ""

#define SX_TIME_IN_SLEEP    60000U     
#define SX_TIME_IN_WAKE     160000U   

/* Wake-fake period (ms): during sleep the MCU wakes this often, turns on ONLY I2C1 and reads VBUS_STAT
 * from the BQ to see if USB was plugged in. Worst-case delay before a plugged USB is noticed. */
#define SX_TIME_WAKE_FAKE               10000U

#define T_ALIVE_MS                      30000U      /* MCU stay-alive after wake   */
#define GPS_TIMEOUT_MS                  130000U      /* GPS fix timeout (ms)        */

#define TIME_PUBLISH_FULL_PW_MODE_MS    60000U

#define ENTER_SLEEP_TIMEOUT_MS          10000U

/* Battery discharge cut-off (volts). While running on battery only (no USB) the firmware reads VBAT from the
 * BQ25628 ADC every wake-fake period; when it reads this value or lower (several readings in a row) it sends
 * BATFET shutdown and the board loses power after BATFET_DLY (12.5 s). Plug USB to start it again.
 * Keep it >= ~2.9 V: below VBAT_LOWV (2.8 V falling) the chip ADC refuses to run, so a lower value never triggers. */
#define VBAT_CUT_OFF                    3.0


#define DELTA_T                         100U

/* ------------------------------------------------------------------ */
/*  W25Q128 Partition Table (16MB total) */
/* ------------------------------------------------------------------ */

#define FLASH_TOTAL_SIZE            (16U * 1024U * 1024U)   /* 16MB */

#define PART_BOOTLOADER_OFFSET      (0x000000U)
#define PART_BOOTLOADER_SIZE        (1U * 1024U * 1024U)    /* 1MB  */

#define PART_MQTT_CONFIG_OFFSET     (PART_BOOTLOADER_OFFSET + PART_BOOTLOADER_SIZE)
#define PART_MQTT_CONFIG_SIZE       (2U * 1024U * 1024U)    /* 2MB  */

#define PART_MISC_OFFSET            (PART_MQTT_CONFIG_OFFSET + PART_MQTT_CONFIG_SIZE)
#define PART_MISC_SIZE              (1U * 1024U * 1024U)    /* 1MB  */

#define PART_MSC_DISK_WIN           (PART_MISC_OFFSET + PART_MISC_SIZE)
#define PART_MSC_DISK_SIZE          (3U * 1024U * 1024U)    /*  3MB */

#define PART_GPS_LOG_OFFSET         (PART_MSC_DISK_WIN + PART_MSC_DISK_SIZE)
#define PART_GPS_LOG_SIZE           (FLASH_TOTAL_SIZE - PART_GPS_LOG_OFFSET) /* ~9MB */

/* GPS Log file path */
#define GPS_LOG_FILE_PATH           "/log_gps"
#define IMU_CALIB_FILE_PATH         "/imu_calib"

#define GPS_CSV_FILE_PATH    "log_gps.csv"   
// #define GPS_CSV_HEADER       "fix,lat,lon,rssi,time,date\n"
#define GPS_CSV_HEADER  "fix,lat,lon,alt,spd,sat,time,date\n"
#define GPS_LOG_READ_CHUNK   256U 

// #define GPS_LOG_MAX_ENTRIES 10

/* ================================================================== */
/*  ADC Voltage Reader                                                 */
/* ================================================================== */

#define ADC_VREF            3.3f
#define ADC_RESOLUTION      4095.0f
#define VBAT_DIVIDER_RATIO  1.47f
#define SAMPLE_PERIOD_MS    100U
#define LOG_PERIOD_MS       10000U

/* Kalman tuning
 *  Q small -> trust model, smooth but slow to follow real changes
 *  R large -> trust measurement less (noisy ADC)
 *  Adjust Q upward if Vbat changes are missed; adjust R upward for more smoothing */
/* Kalman tuning for battery voltage:
 * Q small = smooth, R large = trust measurement less */
#define VBAT_KALMAN_Q   0.0001f
#define VBAT_KALMAN_R   0.1f
#define VBAT_LPF_ALPHA  0.05f   

#define OVERSAMPLE_COUNT    12U

#endif /* APP_CONFIG_H */