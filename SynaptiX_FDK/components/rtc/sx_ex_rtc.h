#ifndef SX_EX_RTC_H
#define SX_EX_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sx_i2c.h"
#include "sx_gpio.h"

#define SX_I2C_MEMADD_SIZE_8BIT     0x0001U
#define SX_I2C_MEMADD_SIZE_16BIT    0x0010U

/* ------------------------------------------------------------------ */
/*  I2C Address                                                        */
/* ------------------------------------------------------------------ */
#define RX8130CE_ADDR           (0x32 << 1)     /* 8-bit: 0x64 write  */

/* ------------------------------------------------------------------ */
/*  Register Map                                                       */
/* ------------------------------------------------------------------ */
#define RX8130CE_REG_SEC        0x10
#define RX8130CE_REG_MIN        0x11
#define RX8130CE_REG_HOUR       0x12
#define RX8130CE_REG_WEEK       0x13
#define RX8130CE_REG_DAY        0x14
#define RX8130CE_REG_MONTH      0x15
#define RX8130CE_REG_YEAR       0x16

#define RX8130CE_REG_MIN_ALARM  0x17
#define RX8130CE_REG_HOUR_ALARM 0x18
#define RX8130CE_REG_WEEK_ALARM 0x19

#define RX8130CE_REG_TIMER0     0x1A
#define RX8130CE_REG_TIMER1     0x1B
#define RX8130CE_REG_EXT        0x1C
#define RX8130CE_REG_FLAG       0x1D
#define RX8130CE_REG_CTRL0      0x1E
#define RX8130CE_REG_CTRL1      0x1F

#define RX8130CE_REG_OFFSET     0x30
#define RX8130CE_REG_EXT1       0x31

/* ------------------------------------------------------------------ */
/*  Register Bit Masks                                                 */
/* ------------------------------------------------------------------ */

/* Flag Register (0x1D) */
#define RX8130CE_FLAG_VLF       (1 << 1)    /* Voltage Low Flag      */
#define RX8130CE_FLAG_VLF_VBFF  (1 << 0)    /* VBAT Full Flag        */
#define RX8130CE_FLAG_AF        (1 << 3)    /* Alarm Flag            */
#define RX8130CE_FLAG_TF        (1 << 4)    /* Timer Flag            */
#define RX8130CE_FLAG_UF        (1 << 5)    /* Update Flag           */
#define RX8130CE_FLAG_VBLF      (1 << 7)    /* VBAT Low Flag         */

/* Control Register 0 (0x1E) */
#define RX8130CE_CTRL0_STOP     (1 << 6)    /* Stop timekeeping      */
#define RX8130CE_CTRL0_AIE      (1 << 3)    /* Alarm Interrupt En    */
#define RX8130CE_CTRL0_TIE      (1 << 4)    /* Timer Interrupt En    */
#define RX8130CE_CTRL0_UIE      (1 << 5)    /* Update Interrupt En   */
#define RX8130CE_CTRL0_TEST     (1 << 7)    /* TEST bit (keep 0)     */

/* Control Register 1 (0x1F) */
#define RX8130CE_CTRL1_INIEN    (1 << 4)    /* Power switch enable   */
#define RX8130CE_CTRL1_CHGEN    (1 << 5)    /* Charge enable         */

/* Extension Register (0x1C) */
#define RX8130CE_EXT_TE         (1 << 4)    /* Timer Enable          */

/* ------------------------------------------------------------------ */
/*  Week bit mapping                                                   */
/* ------------------------------------------------------------------ */
#define RX8130CE_WEEK_SUN       (1 << 0)
#define RX8130CE_WEEK_MON       (1 << 1)
#define RX8130CE_WEEK_TUE       (1 << 2)
#define RX8130CE_WEEK_WED       (1 << 3)
#define RX8130CE_WEEK_THU       (1 << 4)
#define RX8130CE_WEEK_FRI       (1 << 5)
#define RX8130CE_WEEK_SAT       (1 << 6)

/* ------------------------------------------------------------------ */
/*  Return codes                                                       */
/* ------------------------------------------------------------------ */
#define RX8130CE_OK             0
#define RX8130CE_ERR_I2C        (-1)
#define RX8130CE_ERR_NOT_INIT   (-2)
#define RX8130CE_ERR_PARAM      (-3)

/* ------------------------------------------------------------------ */
/*  Data structures                                                    */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t sec;    /* 0 - 59                                        */
    uint8_t min;    /* 0 - 59                                        */
    uint8_t hour;   /* 0 - 23  (24-hour format)                      */
    uint8_t week;   /* bitmask: RX8130CE_WEEK_xxx                    */
    uint8_t day;    /* 1 - 31                                        */
    uint8_t month;  /* 1 - 12                                        */
    uint8_t year;   /* 0 - 99  (e.g. 25 = 2025)                     */
} rx8130ce_time_t;

typedef struct {
    sx_i2c_t  *i2c;
    sx_gpio_t *pwr_pin;     /* PB3 — LOW = power ON (P-ch MOSFET)   */
    bool       initialized;
} rx8130ce_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int rx8130ce_init(rx8130ce_t *dev, sx_i2c_t *i2c, sx_gpio_t *pwr_pin);

int rx8130ce_power_on(rx8130ce_t *dev);
int rx8130ce_power_off(rx8130ce_t *dev);

int rx8130ce_set_time(rx8130ce_t *dev, const rx8130ce_time_t *t);

int rx8130ce_get_time(rx8130ce_t *dev, rx8130ce_time_t *t);

int rx8130ce_is_time_valid(rx8130ce_t *dev, bool *valid);

#ifdef __cplusplus
}
#endif

#endif /* SX_EX_H */