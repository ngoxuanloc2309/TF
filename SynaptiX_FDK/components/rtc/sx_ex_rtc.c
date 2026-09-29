#include "sx_ex_rtc.h"
#include "sx_delay.h"   
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static inline uint8_t bcd_to_dec(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10U) + (bcd & 0x0FU));
}

static inline uint8_t dec_to_bcd(uint8_t dec)
{
    return (uint8_t)(((dec / 10U) << 4) | (dec % 10U));
}

static int _reg_write(rx8130ce_t *dev, uint8_t reg, uint8_t val)
{
    return sx_i2c_mem_write(dev->i2c,
                            RX8130CE_ADDR,
                            reg,
                            SX_I2C_MEMADD_SIZE_8BIT,
                            &val, 1);
}

static int _reg_read(rx8130ce_t *dev, uint8_t reg, uint8_t *buf, uint32_t len)
{
    return sx_i2c_mem_read(dev->i2c,
                           RX8130CE_ADDR,
                           reg,
                           SX_I2C_MEMADD_SIZE_8BIT,
                           buf, len);
}

/* ------------------------------------------------------------------ */
/*  Power control                                                      */
/* ------------------------------------------------------------------ */

int rx8130ce_power_on(rx8130ce_t *dev)
{
    if (!dev || !dev->pwr_pin) return RX8130CE_ERR_PARAM;
    /* P-channel MOSFET: gate LOW → transistor ON → 3.3 V to RTC */
    return sx_gpio_write(dev->pwr_pin, SX_GPIO_LOW);
}

int rx8130ce_power_off(rx8130ce_t *dev)
{
    if (!dev || !dev->pwr_pin) return RX8130CE_ERR_PARAM;
    return sx_gpio_write(dev->pwr_pin, SX_GPIO_HIGH);
}

/* ------------------------------------------------------------------ */
/*  Software Reset sequence (datasheet §18.2)                         */
/* ------------------------------------------------------------------ */

static int _software_reset(rx8130ce_t *dev)
{
    uint8_t dummy;

    /* Steps 4-10 from datasheet software-reset flow */
    if (_reg_write(dev, 0x1E, 0x00) != 0) return RX8130CE_ERR_I2C;
    if (_reg_write(dev, 0x1E, 0x80) != 0) return RX8130CE_ERR_I2C;
    if (_reg_write(dev, 0x50, 0x6C) != 0) return RX8130CE_ERR_I2C;
    if (_reg_write(dev, 0x53, 0x01) != 0) return RX8130CE_ERR_I2C;
    if (_reg_write(dev, 0x66, 0x03) != 0) return RX8130CE_ERR_I2C;
    if (_reg_write(dev, 0x6B, 0x02) != 0) return RX8130CE_ERR_I2C;
    if (_reg_write(dev, 0x6B, 0x01) != 0) return RX8130CE_ERR_I2C;

    /* Step 12: wait 125 ms for reset to complete */
    sx_delay_ms(125);

    /* Ignore return — dummy read after reset */
    (void)_reg_read(dev, RX8130CE_REG_FLAG, &dummy, 1);

    return RX8130CE_OK;
}

/* ------------------------------------------------------------------ */
/*  Initialisation                                                     */
/* ------------------------------------------------------------------ */

int rx8130ce_init(rx8130ce_t *dev, sx_i2c_t *i2c, sx_gpio_t *pwr_pin)
{
    if (!dev || !i2c || !pwr_pin) return RX8130CE_ERR_PARAM;

    dev->i2c         = i2c;
    dev->pwr_pin     = pwr_pin;
    dev->initialized = false;

    /* 1. Power ON */
    if (rx8130ce_power_on(dev) != 0) return RX8130CE_ERR_I2C;

    /* 2. Wait ≥ 30 ms (datasheet: oscillation start + stable) */
    sx_delay_ms(40);

    /* 3. Dummy read — ignore ACK/NACK */
    {
        uint8_t dummy;
        (void)_reg_read(dev, RX8130CE_REG_FLAG, &dummy, 1);
    }

    /* 4. Check VLF bit */
    uint8_t flag = 0;
    if (_reg_read(dev, RX8130CE_REG_FLAG, &flag, 1) != 0)
        return RX8130CE_ERR_I2C;

    if (flag & RX8130CE_FLAG_VLF) {
        /* Oscillation was stopped — perform software reset */
        if (_software_reset(dev) != 0) return RX8130CE_ERR_I2C;
    }

    /* 5. Configure Control Register 1 (0x1F):
     *      INIEN = 1  (power switchover enabled)
     *      CHGEN = 0  (VBAT is hardwired 3.3 V — no charge needed)
     */
    uint8_t ctrl1 = 0;
    if (_reg_read(dev, RX8130CE_REG_CTRL1, &ctrl1, 1) != 0)
        return RX8130CE_ERR_I2C;

    ctrl1 |=  RX8130CE_CTRL1_INIEN;
    ctrl1 &= ~RX8130CE_CTRL1_CHGEN;
    if (_reg_write(dev, RX8130CE_REG_CTRL1, ctrl1) != 0)
        return RX8130CE_ERR_I2C;

    /* 6. Configure Control Register 0 (0x1E):
     *      TEST = 0, AIE = 0, TIE = 0, UIE = 0
     *      STOP = 0  (timekeeping running)
     */
    if (_reg_write(dev, RX8130CE_REG_CTRL0, 0x00) != 0)
        return RX8130CE_ERR_I2C;

    /* 7. Clear TE bit in Extension Register (0x1C) — timer not used */
    uint8_t ext = 0;
    if (_reg_read(dev, RX8130CE_REG_EXT, &ext, 1) != 0)
        return RX8130CE_ERR_I2C;

    ext &= ~RX8130CE_EXT_TE;
    if (_reg_write(dev, RX8130CE_REG_EXT, ext) != 0)
        return RX8130CE_ERR_I2C;

    /* 8. Clear VLF bit */
    flag &= ~RX8130CE_FLAG_VLF;
    if (_reg_write(dev, RX8130CE_REG_FLAG, flag) != 0)
        return RX8130CE_ERR_I2C;

    dev->initialized = true;
    return RX8130CE_OK;
}

/* ------------------------------------------------------------------ */
/*  Set time                                                           */
/* ------------------------------------------------------------------ */

int rx8130ce_set_time(rx8130ce_t *dev, const rx8130ce_time_t *t)
{
    if (!dev || !t)                return RX8130CE_ERR_PARAM;
    if (!dev->initialized)         return RX8130CE_ERR_NOT_INIT;

    /* Validate ranges */
    if (t->sec   > 59 || t->min   > 59 || t->hour > 23 ||
        t->day   < 1  || t->day   > 31 ||
        t->month < 1  || t->month > 12 || t->year  > 99)
        return RX8130CE_ERR_PARAM;

    /* Set STOP = 1 to freeze counters during write */
    uint8_t ctrl0 = 0;
    if (_reg_read(dev, RX8130CE_REG_CTRL0, &ctrl0, 1) != 0)
        return RX8130CE_ERR_I2C;
    ctrl0 |= RX8130CE_CTRL0_STOP;
    if (_reg_write(dev, RX8130CE_REG_CTRL0, ctrl0) != 0)
        return RX8130CE_ERR_I2C;

    /* Write 7 consecutive registers starting at REG_SEC */
    uint8_t buf[7];
    buf[0] = dec_to_bcd(t->sec);
    buf[1] = dec_to_bcd(t->min);
    buf[2] = dec_to_bcd(t->hour);
    buf[3] = t->week;                   /* already a bitmask, not BCD */
    buf[4] = dec_to_bcd(t->day);
    buf[5] = dec_to_bcd(t->month);
    buf[6] = dec_to_bcd(t->year);

    int ret = sx_i2c_mem_write(dev->i2c,
                               RX8130CE_ADDR,
                               RX8130CE_REG_SEC,
                               SX_I2C_MEMADD_SIZE_8BIT,
                               buf, 7);

    /* Clear STOP = 0 (restart clock) regardless of write result */
    ctrl0 &= ~RX8130CE_CTRL0_STOP;
    (void)_reg_write(dev, RX8130CE_REG_CTRL0, ctrl0);

    return (ret == 0) ? RX8130CE_OK : RX8130CE_ERR_I2C;
}

/* ------------------------------------------------------------------ */
/*  Get time                                                           */
/* ------------------------------------------------------------------ */

int rx8130ce_get_time(rx8130ce_t *dev, rx8130ce_time_t *t)
{
    if (!dev || !t)        return RX8130CE_ERR_PARAM;
    if (!dev->initialized) return RX8130CE_ERR_NOT_INIT;

    uint8_t buf[7];
    if (sx_i2c_mem_read(dev->i2c,
                        RX8130CE_ADDR,
                        RX8130CE_REG_SEC,
                        SX_I2C_MEMADD_SIZE_8BIT,
                        buf, 7) != 0)
        return RX8130CE_ERR_I2C;

    /* Mask unused bits before BCD conversion (datasheet table 12) */
    t->sec   = bcd_to_dec(buf[0] & 0x7F);
    t->min   = bcd_to_dec(buf[1] & 0x7F);
    t->hour  = bcd_to_dec(buf[2] & 0x3F);
    t->week  = buf[3] & 0x7F;           /* bitmask, not BCD          */
    t->day   = bcd_to_dec(buf[4] & 0x3F);
    t->month = bcd_to_dec(buf[5] & 0x1F);
    t->year  = bcd_to_dec(buf[6]);

    return RX8130CE_OK;
}

/* ------------------------------------------------------------------ */
/*  VLF check                                                          */
/* ------------------------------------------------------------------ */

int rx8130ce_is_time_valid(rx8130ce_t *dev, bool *valid)
{
    if (!dev || !valid)    return RX8130CE_ERR_PARAM;
    if (!dev->initialized) return RX8130CE_ERR_NOT_INIT;

    uint8_t flag = 0;
    if (_reg_read(dev, RX8130CE_REG_FLAG, &flag, 1) != 0)
        return RX8130CE_ERR_I2C;

    *valid = !(flag & RX8130CE_FLAG_VLF);
    return RX8130CE_OK;
}