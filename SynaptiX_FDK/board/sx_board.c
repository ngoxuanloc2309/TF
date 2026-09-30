#include "sx_board.h"
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

static ADC_HandleTypeDef *hal_adc = &hadc1;

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
                     a == 0x6B ? "  <- BQ25622" :
                     a == 0x32 ? "  <- RX8130CE" :
                     (a == 0x28 || a == 0x29) ? "  <- BNO055" : "");
            found++;
        }
    }
    log_info(TAG, "I2C1 scan done: %u device(s)", found);
    if (!board.bq.online) {
        log_info(TAG, "BQ25622 retry init after scan...");
        bq25622_init(&board.bq, &board.i2c1);
    }
}
#endif

void sx_board_init(void)
{
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
    // BQ25622 (read-only, Phase 0)
    bq25622_init(&board.bq, &board.i2c1);
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

    HAL_ADCEx_Calibration_Start(hal_adc, ADC_SINGLE_ENDED);
    HAL_ADC_Start(hal_adc);
    sx_adc_reader_init(&board.s_adc_reader);
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

void read_vol_pin(uint32_t time_stamp) {
    sx_adc_reader_process(&board.s_adc_reader, hal_adc, time_stamp);
    board.voltage.v_bat = board.s_adc_reader.v_bat_filtered;
}

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
    app_request_sleep();
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
    uint8_t ret = HAL_GPIO_ReadPin(VBUS_PORT, VBUS_PIN);
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

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == VBUS_PIN){
        
        /* Set EXTI wake reason for sleep manager if MCU is waking from sleep */
        sx_sleep_set_exti_wake();
        
        sx_gpio_write(&s_dis_charge, SX_GPIO_HIGH);
        sx_gpio_write(&s_charge, SX_GPIO_HIGH);

        app_sync_gps_log_to_disk();
        app_mode_full_pw();
        app_notify_usb_connected();
        log_info(TAG,"USB charge");
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == VBUS_PIN){
        sx_gpio_write(&s_dis_charge, SX_GPIO_HIGH);
        sx_gpio_write(&s_charge, SX_GPIO_LOW);
        
        app_request_sleep();
        log_info(TAG,"USB discharge");
    }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    log_info("USB", "CDC line state: dtr=%d rts=%d", dtr, rts);
}