#ifndef SX_BOARD_H
#define SX_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "sx_spi.h"
#include "sx_gpio.h"
#include "sx_uart.h"
#include "gps.h"
#include "sim76xx.h"
#include "sx_delay.h"
#include "sx_W25Q128.h"
#include "sx_ex_storage.h"
#include "sx_usb_tiny_cdc.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "sx_ex_rtc.h"
#include "bno055.h"
#include "sx_filter.h"
#include "sx_read_bat.h"

typedef struct {
    volatile uint32_t raw_adc;
    volatile float v_adc;
    volatile float v_bat;
}voltage_t;

typedef struct Board{
    sim76xx_t sim76xx;
    sx_gps_t gps;
    sx_uart_t log_uart;
    voltage_t voltage;
    sx_usb_tiny_t usb;
    sx_W25Q128_t  q128;
    sx_usb_tiny_config_t usb_cfg;
    sx_storage_cfg_t    storage_cfg;
    sx_i2c_t            i2c1;
    rx8130ce_t  rtc;
    sx_i2c_t    rtc_i2c;
    bno055_t    imu;
    sx_adc_reader_t s_adc_reader;
}Board_t;

/*  MODE USB    */
#define BOARD_USE_CDC    1
#define BOARD_USE_MSC    1

/*  LTE GPIO    */
#define LTE_EN_PW_Port      LTE_PWR_GPIO_Port
#define LTE_EN_PW_Pin       LTE_PWR_Pin
#define LTE_PWRKEY_PW_Port  LTE_PWR_Key_GPIO_Port
#define LTE_PWRKEY_Pin      LTE_PWR_Key_Pin

/*  GPS GPIO    */
#define GPS_EN_PW_Port      GPS_PWR_GPIO_Port
#define GPS_EN_PW_Pin       GPS_PWR_Pin
// #define GPS_RESET_Port      GPS_RESET_GPIO_Port
// #define GPS_RESET_PIN       GPS_RESET_Pin
// #define GPS_WAKEUP_Port     GPS_WAKEUP_GPIO_Port
// #define GPS_WAKEUP_PIN      GPS_WAKEUP_Pin
// #define GPS_EINT_OUT_Port   GPS_EINT_OUT_GPIO_Port
// #define GPS_EINT_OUT_PIN    GPS_EINT_OUT_Pin
// #define GPS_TIME_MAKE_Port  GPS_TIME_MAKE_GPIO_Port
// #define GPS_TIME_MAKE_PIN   GPS_TIME_MAKE_Pin

/*  TEST PIN    */
#define EN_BAT_CHARGE_Port       EN_CHARGE_GPIO_Port
#define EN_BAT_CHARGE_PIN        EN_CHARGE_Pin

#define EN_BAT_DISCHARGE_Port    EN_DISCHARGE_GPIO_Port
#define EN_BAT_DISCHARGE_PIN     EN_DISCHARGE_Pin

#define VBUS_PORT                GPIOC
#define VBUS_PIN                 GPIO_PIN_1

/*  SPI */
#define SPI_CS_Port             GPIOA
#define SPI_CS_Pin              GPIO_PIN_4

#define SPI_PW_PORT             GPIOC
#define SPI_PW_PIN              GPIO_PIN_4

#define RTC_EN_PW_GPIO_Port   GPIOB
#define RTC_EN_PW_GPIO_Pin    GPIO_PIN_3

#define IMU_EN_PW_GPIO_Port   GPIOB
#define IMU_EN_PW_GPIO_Pin    GPIO_PIN_4

#define IMU_RESET_Pin         GPIO_PIN_5
#define IMU_RESET_Port        GPIOB

/* USB */
void dcd_fs_msp_init(uint8_t rhport);
void USB_DRD_FS_IRQHandler(void);

/* Board */
void sx_board_init(void);
void read_vol_pin(uint32_t time_stamp);
void gps_it_handle(void);
void sim_it_handle(void);
void board_gps_uart_resume_it(void);
void board_sim_uart_resume_it(void);

void sx_board_uart_abort(void);
void sx_board_uart_resume_it(void);
void check_charge(void);

extern Board_t board;

#ifdef __cplusplus
}
#endif

#endif // SX_BOARD_H