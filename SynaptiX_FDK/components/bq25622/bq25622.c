#include "bq25622.h"
#include "logger.h"

static const char *TAG = "BQ25622";

static uint8_t vbus_stat_is_present(uint8_t stat)
{
    /* 000 = not powered from VBUS, 111 = we are the OTG source (no external USB) */
    return (stat != BQ25622_VBUS_NONE && stat != BQ25622_VBUS_OTG);
}

static int reg_read8(bq25622_t *dev, uint8_t reg, uint8_t *val)
{
    return sx_i2c_mem_read(dev->i2c, BQ25622_I2C_ADDR, reg,
                           BQ25622_I2C_MEMADD_8BIT, val, 1);
}

int bq25622_read_status(bq25622_t *dev)
{
    uint8_t r = 0;
    if (reg_read8(dev, BQ25622_REG_CHARGER_STATUS_1, &r) != 0)
        return -1;
    /* Reads outside the register map return 0xFF; treat that as a bus/device fault. */
    if (r == 0xFF)
        return -1;

    dev->vbus_stat = r & BQ25622_VBUS_STAT_MASK;
    dev->chg_stat  = (r & BQ25622_CHG_STAT_MASK) >> BQ25622_CHG_STAT_SHIFT;
    return 0;
}

int bq25622_init(bq25622_t *dev, sx_i2c_t *i2c)
{
    dev->i2c          = i2c;
    dev->vbus_stat    = 0;
    dev->chg_stat     = 0;
    dev->part_number  = 0;
    dev->present      = 0;
    dev->valid        = 0;
    dev->online       = 0;
    dev->debounce_cnt = 0;
    dev->fail_cnt     = 0;
    dev->elapsed_ms   = 0;

    uint8_t pi = 0;
    if (reg_read8(dev, BQ25622_REG_PART_INFO, &pi) != 0) {
        log_error(TAG, "not found on I2C (addr 0x%02X)", BQ25622_I2C_ADDR >> 1);
        return -1;
    }
    dev->online      = 1;
    dev->part_number = (pi & BQ25622_PN_MASK) >> BQ25622_PN_SHIFT;
    log_info(TAG, "found: PN=%u (%s) rev=%u", dev->part_number,
             dev->part_number == BQ25622_PN_BQ25622 ? "BQ25622" :
             dev->part_number == BQ25622_PN_BQ25620 ? "BQ25620" : "unknown",
             pi & 0x07U);

    if (bq25622_read_status(dev) == 0) {
        dev->present = vbus_stat_is_present(dev->vbus_stat);
        dev->valid   = 1;
        log_info(TAG, "initial VBUS_STAT=%u -> USB %s", dev->vbus_stat,
                 dev->present ? "present" : "absent");
    }
    return 0;
}

bq25622_event_t bq25622_poll(bq25622_t *dev, uint32_t ts_ms)
{
    uint32_t interval = (dev->fail_cnt >= BQ25622_FAIL_BACKOFF_COUNT)
                        ? BQ25622_FAIL_BACKOFF_MS : BQ25622_POLL_INTERVAL_MS;

    dev->elapsed_ms += ts_ms;
    if (dev->elapsed_ms < interval)
        return BQ25622_EVT_NONE;
    dev->elapsed_ms = 0;

    if (bq25622_read_status(dev) != 0) {
        /* Fail-safe: an I2C error is NOT "USB removed". Keep the last known state. */
        if (dev->fail_cnt < 255) dev->fail_cnt++;
        if (dev->fail_cnt == BQ25622_FAIL_BACKOFF_COUNT)
            log_warn(TAG, "I2C read failing — keeping last state, polling slower");
        dev->debounce_cnt = 0;
        return BQ25622_EVT_NONE;
    }
    if (dev->fail_cnt >= BQ25622_FAIL_BACKOFF_COUNT)
        log_info(TAG, "I2C recovered");
    dev->fail_cnt = 0;
    dev->online   = 1;

    uint8_t raw = vbus_stat_is_present(dev->vbus_stat);

    if (!dev->valid) {                 /* first ever sample: adopt it, no event */
        dev->present      = raw;
        dev->valid        = 1;
        dev->debounce_cnt = 0;
        return BQ25622_EVT_NONE;
    }

    if (raw == dev->present) {
        dev->debounce_cnt = 0;
        return BQ25622_EVT_NONE;
    }

    if (++dev->debounce_cnt < BQ25622_DEBOUNCE_COUNT)
        return BQ25622_EVT_NONE;

    dev->debounce_cnt = 0;
    dev->present      = raw;
    log_info(TAG, "VBUS_STAT=%u -> USB %s", dev->vbus_stat, raw ? "PLUGGED" : "REMOVED");
    return raw ? BQ25622_EVT_PLUGGED : BQ25622_EVT_UNPLUGGED;
}

uint8_t bq25622_refresh(bq25622_t *dev)
{
    if (bq25622_read_status(dev) == 0) {
        dev->present      = vbus_stat_is_present(dev->vbus_stat);
        dev->valid        = 1;
        dev->debounce_cnt = 0;
        dev->fail_cnt     = 0;
        dev->elapsed_ms   = 0;
    }
    return dev->present;
}