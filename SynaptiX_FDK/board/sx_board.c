#include "sx_board.h"
#include <stdio.h>
#include <stddef.h>
#include "stm32h5xx_hal.h"
#include "tusb.h"
#include "usb.h"
#include "sx_user_cdc.h"
#include "sx_user_msc.h"
#include "logger.h"
#include "sx_delay.h"
#include "sx_gpio.h"
#include "app_config.h"
#include "app.h"
#include "sx_sleep.h"

static const char *TAG = "BOARD";

Board_t board;

#define UART_LTE 0
#define UART_GPS 1
#define UART_LOG 2

static UART_HandleTypeDef *hal_uart[3] = {&huart1, &huart2, &huart3}; // lte, gps, log
static sx_uart_t *bsp_uart[3];
static uint8_t uart_rx_char[3];

static void set_enter_sleep_mode(void);
static void set_enter_full_mode(void);

void dcd_fs_msp_init(uint8_t rhport)
{
    (void)rhport;
    log_info(TAG, "dcd_fs_msp_init called");
    hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
    HAL_PCD_MspInit(&hpcd_USB_DRD_FS);
    HAL_Delay(100);
    log_info(TAG, "MSP init done!");
}

void USB_DRD_FS_IRQHandler(void)
{
    tud_int_handler(0);
}

/* ------------------------------------------------------------------ */
/*  GPIO Define                                                       */
/* ------------------------------------------------------------------ */

static sx_gpio_pin_t s_lte_power_pin = {.pin = LTE_EN_PW_Pin, .port = LTE_EN_PW_Port};
static sx_gpio_pin_t s_lte_pwrkey_pin = {.pin = LTE_PWRKEY_Pin, .port = LTE_PWRKEY_PW_Port};
static sx_gpio_pin_t s_gps_power_pin = {.pin = GPS_EN_PW_Pin, .port = GPS_EN_PW_Port};

static sx_gpio_t s_charge;
static sx_gpio_pin_t s_charge_pin = {.pin = EN_BAT_CHARGE_PIN, .port = EN_BAT_CHARGE_Port};

static sx_gpio_t s_dis_charge;
static sx_gpio_pin_t s_dis_charge_pin = {.pin = EN_BAT_DISCHARGE_PIN, .port = EN_BAT_DISCHARGE_Port};

static sx_gpio_pin_t s_rtc_pwr_pin = {.pin = RTC_EN_PW_GPIO_Pin, .port = RTC_EN_PW_GPIO_Port};
static sx_gpio_t     s_rtc_pwr;

static sx_gpio_pin_t s_imu_en_pin    = {.pin = IMU_EN_PW_GPIO_Pin,  .port = IMU_EN_PW_GPIO_Port};
static sx_gpio_pin_t s_imu_reset_pin = {.pin = IMU_RESET_Pin,  .port = IMU_RESET_Port};
static sx_gpio_t     s_imu_en;
static sx_gpio_t     s_imu_reset;

/*  SPI */
// static sx_gpio_t s_spi_cs;
// static sx_gpio_t s_spi_pwr;
// static sx_spi_t storage_spi;

static sx_gpio_pin_t s_spi_cs_pin = {.pin = SPI_CS_Pin, .port = SPI_CS_Port};
static sx_gpio_pin_t s_spi_pw_pin = {.pin = SPI_PW_PIN, .port = SPI_PW_PORT};

static void spi_storage_init(void){
    board.storage_cfg.cs_pin = s_spi_cs_pin;
    board.storage_cfg.pwr_pin = s_spi_pw_pin;
    board.storage_cfg.hspi = &hspi1;

    sx_storage_init(&board.storage_cfg);
}

// USART define

/* ------------------------------------------------------------------ */
/*  Board                                                               */
/* ------------------------------------------------------------------ */

static void log_print(const char *str)
{
    sx_uart_write(&board.log_uart, (const uint8_t *)str, strlen(str));
}

/* BQ25628 start-up (after a successful bq25622_init): read-only snapshot of the POR values, then the project
 * config. The discharge cut-off comes from VBAT_CUT_OFF (V, app_config.h); the driver keeps it in mV. */
static void bq_start(void)
{
    bq25622_dump_regs(&board.bq);                /* read-only, FLAG regs clear on read: boot only */
    board.bq.cfg.cutoff = (bq25622_cutoff_t)(unsigned)(VBAT_CUT_OFF * 1000.0 + 0.5);
    bq25622_config_apply(&board.bq);             /* WATCHDOG off FIRST, then VREG/ICHG/ITERM/IBAT_PK/ADC mask */
    log_info(TAG, "VBAT cut-off = %u mV", (unsigned)board.bq.cfg.cutoff);
}

/* ---- Phase 0 diagnostic: I2C1 scan (turn off with -DI2C_SCAN_DEBUG=0) ---- */
#ifndef I2C_SCAN_DEBUG
#define I2C_SCAN_DEBUG 1
#endif
#if I2C_SCAN_DEBUG
static void i2c1_scan_debug(void)
{
    /* Idle levels: with pull-ups present and no device holding the bus, both must read 1. */
    log_info(TAG, "I2C1 idle: SCL(PB6)=%d SDA(PB7)=%d (expect 1/1)",
             (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
             (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));

    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (sx_i2c_is_device_ready(&board.i2c1, (uint16_t)(a << 1), 1, 10) == 0) {
            log_info(TAG, "I2C1 scan: ACK at 0x%02X%s", a,
                     a == 0x6A ? "  <- BQ25628" :
                     a == 0x32 ? "  <- RX8130CE" :
                     (a == 0x28 || a == 0x29) ? "  <- BNO055" : "");
            found++;
        }
    }
    log_info(TAG, "I2C1 scan done: %u device(s)", found);
    if (!board.bq.online) {
        log_info(TAG, "BQ25628 retry init after scan...");
        if (bq25622_init(&board.bq, &board.i2c1) == 0)
            bq_start();
    }
}
#endif

void sx_board_init(void)
{
    /* v1.4 has no GPIO for USB detection. CubeMX still sets PC1 up as an EXTI input; a floating pin would
     * wake the MCU from STOP, so put it back to analog and switch the EXTI1 line off. */
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);
    // Initialize Logger
    static sx_uart_config_t uart_config[3];
    uart_config[UART_LOG].pDriver = hal_uart[UART_LOG];
    uart_config[UART_LOG].baudrate = 115200;
    uart_config[UART_LOG].bits = 8;
    uart_config[UART_LOG].parity = 0;
    uart_config[UART_LOG].stopbits = 1;

    bsp_uart[UART_LOG] = &board.log_uart;

    sx_uart_init(&board.log_uart, &uart_config[UART_LOG], 512, 512);
    logger_init(LOGGER_INFO, log_print);
    log_info(TAG, "Board init start");

    sx_gpio_init(&s_charge, &sx_gpio_ops, &s_charge_pin);
    sx_gpio_init(&s_dis_charge, &sx_gpio_ops, &s_dis_charge_pin);
    
    sx_gpio_write(&s_charge, SX_GPIO_HIGH);
    sx_gpio_write(&s_dis_charge, SX_GPIO_HIGH);
    
    /*  USB  */ 
    #if BOARD_USE_MSC
    sx_user_msc_init();
    sx_user_msc_create_disk(USER_DISK_LABEL_CREATE);
    log_info(TAG, "MSC disk created");
    #endif
    board.usb_cfg.rx_buf_size = 256;
    board.usb_cfg.tx_buf_size = 256;
    dcd_fs_msp_init(0);
    sx_usb_tiny_init(&board.usb, &board.usb_cfg);
    log_info(TAG, "USB init done");

    uart_config[UART_LTE].pDriver = hal_uart[UART_LTE];
    uart_config[UART_LTE].baudrate = 115200;
    uart_config[UART_LTE].bits = 8;
    uart_config[UART_LTE].parity = 0;
    uart_config[UART_LTE].stopbits = 1;

    uart_config[UART_GPS].pDriver = hal_uart[UART_GPS];
    uart_config[UART_GPS].baudrate = 9600;
    uart_config[UART_GPS].bits = 8;
    uart_config[UART_GPS].parity = 0;
    uart_config[UART_GPS].stopbits = 1;

    bsp_uart[UART_LTE] = &board.sim76xx.base.uart;
    bsp_uart[UART_GPS] = &board.gps.comm;

    // I2C
    sx_i2c_init(&board.i2c1, &sx_i2c_ops, &hi2c1);
    // BQ25628: probe, read-only snapshot of the POR values, then apply the project config
    if (bq25622_init(&board.bq, &board.i2c1) == 0)
        bq_start();
    // RTC
    sx_gpio_init(&s_rtc_pwr,   &sx_gpio_ops, &s_rtc_pwr_pin);
    rx8130ce_init(&board.rtc,  &board.i2c1, &s_rtc_pwr);
    // IMU
    sx_gpio_init(&s_imu_en,    &sx_gpio_ops, &s_imu_en_pin);
    sx_gpio_init(&s_imu_reset, &sx_gpio_ops, &s_imu_reset_pin);
    // Initialize LTE
    sx_gpio_init(&board.sim76xx.base.powerPin, &sx_gpio_ops, &s_lte_power_pin);
    sx_gpio_init(&board.sim76xx.base.pwrPin, &sx_gpio_ops, &s_lte_pwrkey_pin);
    sx_uart_init(&board.sim76xx.base.uart, &uart_config[UART_LTE], 512, 512);
    sim76xx_init(&board.sim76xx);
    HAL_UART_Receive_IT(hal_uart[UART_LTE], &uart_rx_char[UART_LTE], 1);
    sim76xx_power_on(&board.sim76xx);
    gps_init(&board.gps, &uart_config[UART_GPS], &sx_gpio_ops, &s_gps_power_pin, NULL);
    HAL_UART_Receive_IT(hal_uart[UART_GPS], &uart_rx_char[UART_GPS], 1);
    sim76xx_start(&board.sim76xx);
    // Initialize GPS
    spi_storage_init();
    bno055_power_on(&board.imu);
    sx_gpio_write(&s_imu_en, SX_GPIO_LOW);
    bno055_init(&board.imu, &board.i2c1, BNO055_I2C_ADDR_DEFAULT, &s_imu_en, &s_imu_reset);
#if I2C_SCAN_DEBUG
    i2c1_scan_debug();
#endif
}

/* ------------------------------------------------------------------ */
/*  Phase 2: on-demand power for the IMU and for I2C1                   */
/* ------------------------------------------------------------------ */

static uint8_t s_imu_active = 1;       /* 1 = NORMAL/NDOF running, 0 = SUSPEND */
static uint8_t s_i2c1_on = 1;
int sx_board_i2c1_scan(char *out, size_t n);

/* The IMU supply must NOT be cut: measured on board v1.4, with IMU_EN_PW high SCL and SDA are pulled to 0
 * (the unpowered BNO055 clamps the shared bus), and BQ (0x6A) and RTC (0x32) stop answering. So the IMU
 * stays powered and goes into the BNO055 SUSPEND power mode instead. */

/* IMU -> SUSPEND (all sensors and the internal MCU sleep; registers are not updated). */
int sx_board_imu_suspend(void)
{
    if (!s_imu_active) return 0;

    int rc = bno055_set_pwr_mode(&board.imu, BNO055_PWR_MODE_SUSPEND);   /* goes to CONFIG first */
    if (rc != 0) {
        log_error("BOARD", "IMU suspend failed (rc=%d)", rc);
        return rc;
    }
    board.imu.initialized = false;           /* reads are refused while suspended */
    s_imu_active = 0;
    log_info("BOARD", "IMU suspended");
    return 0;
}

/* IMU -> NORMAL power mode and NDOF fusion again. No reset pulse and no re-init: the chip never lost power.
 * Whether SUSPEND keeps the calibration is not confirmed; the caller may restore it with imu_calib_load(). */
int sx_board_imu_resume(void)
{
    if (s_imu_active) return 0;

    int rc = bno055_set_pwr_mode(&board.imu, BNO055_PWR_MODE_NORMAL);
    if (rc == 0) {
        sx_delay_ms(10);
        rc = bno055_set_opr_mode(&board.imu, BNO055_OPR_MODE_NDOF);
    }
    if (rc != 0) {
        char diag[96];
        sx_board_i2c1_scan(diag, sizeof(diag));
        log_error("BOARD", "IMU resume failed (rc=%d) | bus: %s", rc, diag);
        return rc;
    }
    board.imu.initialized = true;
    s_imu_active = 1;
    log_info("BOARD", "IMU resumed");
    return 0;
}

uint8_t sx_board_imu_is_active(void)
{
    return s_imu_active;
}

/* I2C1 (BQ, RX8130CE, BNO055 share it). DeInit puts PB6/PB7 in analog mode; the external pull-ups stay. */

int sx_board_i2c1_off(void)
{
    if (!s_i2c1_on) return 0;
    if (HAL_I2C_DeInit(&hi2c1) != HAL_OK) return -1;
    s_i2c1_on = 0;
    return 0;
}

int sx_board_i2c1_on(void)
{
    if (s_i2c1_on) return 0;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) return -1;
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) return -1;
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) return -1;
    s_i2c1_on = 1;
    return 0;
}

/* Bus health check for the Phase 2 tests: line levels + ACK list. Writes e.g. "SCL=1 SDA=1 ACK: 0x29 0x32 0x6A".
 * With I2C1 DeInit'd the pins are analog and read 0, so the levels are only meaningful while I2C1 is on. */
int sx_board_i2c1_scan(char *out, size_t n)
{
    size_t k = 0;
    if (!out || n < 32) return -1;

    k += (size_t)snprintf(out + k, n - k, "SCL=%d SDA=%d ACK:",
                          (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
                          (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));
    int found = 0;
    if (s_i2c1_on) {
        for (uint8_t a = 0x08; a < 0x78 && k + 6 < n; a++) {
            if (sx_i2c_is_device_ready(&board.i2c1, (uint16_t)(a << 1), 1, 10) == 0) {
                k += (size_t)snprintf(out + k, n - k, " 0x%02X", a);
                found++;
            }
        }
    }
    if (!found) snprintf(out + k, n - k, " none");
    return found;
}

uint8_t sx_board_i2c1_is_on(void)
{
    return s_i2c1_on;
}

static void sx_sim76_uart_abort(void) {
    HAL_UART_Abort(hal_uart[UART_LTE]);
}

static void sx_gps_uart_abort(){
    HAL_UART_Abort(hal_uart[UART_GPS]);
}

void gps_it_handle(){
    HAL_UART_Receive_IT(hal_uart[UART_GPS], &uart_rx_char[UART_GPS], 1);
}

void sim_it_handle(){
    HAL_UART_Receive_IT(hal_uart[UART_LTE], &uart_rx_char[UART_LTE], 1);
}

void board_gps_uart_resume_it(void){
    sx_gps_uart_abort();
    gps_it_handle();
}

void board_sim_uart_resume_it(void){
    sx_sim76_uart_abort();
    sim_it_handle();
}
void sx_board_uart_resume_it(void) {
    // sx_sim76_uart_abort();
    // sx_gps_uart_abort();
    // HAL_UART_Receive_IT(hal_uart[UART_LTE], &uart_rx_char[UART_LTE], 1);
    // HAL_UART_Receive_IT(hal_uart[UART_GPS], &uart_rx_char[UART_GPS], 1);
    board_gps_uart_resume_it();
    board_sim_uart_resume_it();
}

static void set_enter_sleep_mode(void) {
    sx_gpio_write(&s_dis_charge, SX_GPIO_HIGH); 
    sx_gpio_write(&s_charge, SX_GPIO_LOW);
    log_info(TAG, "Enter sleep POWER");
}

// static void set_enter_full_mode(void) {
//     sx_gpio_write(&s_charge, SX_GPIO_HIGH);
//     sx_gpio_write(&s_dis_charge, SX_GPIO_HIGH);
//     log_info(TAG, "Enter full POWER");
// }

/* USB IT CB    */
void tud_mount_cb(void) {
    log_info(TAG, "USB tiny connected");
    
    //app_sync_gps_log_to_disk();
    // set_enter_full_mode();
    //app_mode = APP_MODE_FULL_POWER;
    //app_notify_usb_connected();
}

void tud_umount_cb(void) {
    //(void)remote_wakeup_en;
    sx_gpio_write(&s_charge, SX_GPIO_LOW);
    sx_gpio_write(&s_dis_charge, SX_GPIO_HIGH);
    log_info(TAG,"USB discharge");
    /* No sleep request here: VBUS_STAT of the BQ decides (a host reset/unmount while VBUS is still present must not sleep) */
    log_info(TAG, "USB tiny disconnected");
    // set_enter_sleep_mode();
    // app_request_sleep();
    
}

void tud_suspend_cb(bool remote_wakeup_en) {
    sx_gpio_write(&s_dis_charge, SX_GPIO_HIGH);
    // (void)remote_wakeup_en;
    sx_gpio_write(&s_charge, SX_GPIO_LOW);
    // log_info(TAG,"USB discharge");
    // app_request_sleep();
    // log_info(TAG, "USB tiny suspended"); 
}

void tud_resume_cb(void) {
    log_info(TAG, "USB tiny resumed");
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
    //set_enter_full_mode();
    // app_mode = APP_MODE_FULL_POWER;
    // app_notify_usb_connected();
}

void check_charge(void){
    uint8_t ret = board.bq.present;      /* debounced VBUS_STAT, updated by bq25622_poll() in app.c */
    (ret == 1)?(sx_gpio_write(&s_charge, SX_GPIO_HIGH)):(sx_gpio_write(&s_charge, SX_GPIO_LOW));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == hal_uart[UART_LTE]){
        sx_uart_rx_callback(bsp_uart[UART_LTE], &uart_rx_char[UART_LTE], 1);
        HAL_UART_Receive_IT(hal_uart[UART_LTE], &uart_rx_char[UART_LTE], 1);
    } else if(huart == hal_uart[UART_GPS]){
        sx_uart_rx_callback(bsp_uart[UART_GPS], &uart_rx_char[UART_GPS], 1);
        HAL_UART_Receive_IT(hal_uart[UART_GPS], &uart_rx_char[UART_GPS], 1);
    } else if(huart == hal_uart[UART_LOG]){
        sx_uart_rx_callback(bsp_uart[UART_LOG], &uart_rx_char[UART_LOG], 1);
        HAL_UART_Receive_IT(hal_uart[UART_LOG], &uart_rx_char[UART_LOG], 1);
    }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    log_info("USB", "CDC line state: dtr=%d rts=%d", dtr, rts);
}