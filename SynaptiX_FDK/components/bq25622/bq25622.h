#ifndef BQ25622_H
#define BQ25622_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sx_i2c.h"

/*
 * BQ25620/BQ25622 — I2C controlled 1-cell buck charger with NVDC power path.
 *
 * This driver is READ-ONLY on purpose: a write to any register would move the
 * charger from "default mode" to "host mode" and start the I2C watchdog. We only
 * need to know whether VBUS (USB) is present, so the charger keeps running
 * autonomously with its default settings.
 *
 * Presence is taken from VBUS_STAT (REG0x1E[2:0]), NOT from VBUS_ADC:
 *   - ADC is disabled by default (ADC_EN=0) and VBUS_ADC keeps its last
 *     measurement instead of dropping to 0, so "VBUS_ADC == 0" is not a
 *     reliable "unplugged" indicator.
 *   - VBUS_STAT == 000b means "not powered from VBUS" and needs no ADC.
 */

#define BQ25622_I2C_ADDR                (0x6B << 1)     /* 7-bit 0x6B, HAL wants 8-bit */
#define BQ25622_I2C_MEMADD_8BIT         0x0001U         /* == I2C_MEMADD_SIZE_8BIT     */

#define BQ25622_REG_CHARGER_STATUS_0    0x1D
#define BQ25622_REG_CHARGER_STATUS_1    0x1E            /* CHG_STAT[4:3], VBUS_STAT[2:0] */
#define BQ25622_REG_PART_INFO           0x38            /* PN[5:3], DEV_REV[2:0]         */

#define BQ25622_VBUS_STAT_MASK          0x07U
#define BQ25622_CHG_STAT_MASK           0x18U
#define BQ25622_CHG_STAT_SHIFT          3
#define BQ25622_PN_MASK                 0x38U
#define BQ25622_PN_SHIFT                3
#define BQ25622_PN_BQ25620              0
#define BQ25622_PN_BQ25622              1

/* VBUS_STAT values */
#define BQ25622_VBUS_NONE               0x0     /* not powered from VBUS          */
#define BQ25622_VBUS_UNKNOWN_ADAPTER    0x4     /* BQ25622: adapter present       */
#define BQ25622_VBUS_OTG                0x7     /* boost OTG: WE are the source   */

/* Tunables (override with -D or before including) */
#ifndef BQ25622_POLL_INTERVAL_MS
#define BQ25622_POLL_INTERVAL_MS        250U
#endif
#ifndef BQ25622_DEBOUNCE_COUNT
#define BQ25622_DEBOUNCE_COUNT          2U      /* consecutive differing reads before an event */
#endif
#ifndef BQ25622_FAIL_BACKOFF_COUNT
#define BQ25622_FAIL_BACKOFF_COUNT      3U      /* after this many I2C errors, slow the polling */
#endif
#ifndef BQ25622_FAIL_BACKOFF_MS
#define BQ25622_FAIL_BACKOFF_MS         2000U
#endif

typedef enum {
    BQ25622_EVT_NONE = 0,
    BQ25622_EVT_PLUGGED,        /* VBUS absent  -> present */
    BQ25622_EVT_UNPLUGGED,      /* VBUS present -> absent  */
} bq25622_event_t;

typedef struct {
    sx_i2c_t *i2c;

    uint8_t  vbus_stat;         /* last raw VBUS_STAT[2:0]                 */
    uint8_t  chg_stat;          /* last raw CHG_STAT[1:0]                  */
    uint8_t  part_number;       /* PN field, valid if online               */

    uint8_t  present;           /* debounced: 1 = USB power present        */
    uint8_t  valid;             /* 1 once a first status sample was taken  */
    uint8_t  online;            /* 1 if the IC answered on I2C at init     */
    uint8_t  debounce_cnt;
    uint8_t  fail_cnt;

    uint32_t elapsed_ms;
} bq25622_t;

/* Probe the IC, read part info + first status sample (no event generated).
 * Returns 0 if the IC answered, -1 otherwise (driver stays usable, poll keeps retrying). */
int bq25622_init(bq25622_t *dev, sx_i2c_t *i2c);

/* Read REG0x1E now. Returns 0 on success, -1 on I2C error (state left untouched). */
int bq25622_read_status(bq25622_t *dev);

/* Call every main-loop iteration with the elapsed ms. Reads the charger every
 * BQ25622_POLL_INTERVAL_MS and returns a debounced plug/unplug event. */
bq25622_event_t bq25622_poll(bq25622_t *dev, uint32_t ts_ms);

/* Immediate read (no debounce, no event) — use right after waking from STOP.
 * Returns the resulting "present" flag; on I2C error returns the last known value. */
uint8_t bq25622_refresh(bq25622_t *dev);

static inline uint8_t bq25622_vbus_present(const bq25622_t *dev){
    return dev->present;
}

static inline uint8_t bq25622_vbus_stat(const bq25622_t *dev){
    return dev->vbus_stat;
}

#ifdef __cplusplus
}
#endif
#endif /* BQ25622_H */