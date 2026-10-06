#ifndef BQ25622_H
#define BQ25622_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "sx_i2c.h"

/*
 * BQ25628 (also BQ25629) — I2C controlled 1-cell, 2 A buck charger with NVDC power path.
 * The file/symbol prefix "bq25622" is historical; the register map below is the BQ25628 one.
 *
 * Two kinds of API live here:
 *
 *  1) READ-ONLY presence detection (init/poll/refresh/read_status).
 *     Presence is taken from VBUS_STAT (REG0x1E[2:0]), NOT from VBUS_ADC:
 *       - ADC is disabled by default (ADC_EN=0) and the ADC registers keep
 *         their last measurement instead of dropping to 0, so "VBUS_ADC == 0"
 *         is not a reliable "unplugged" indicator.
 *       - VBUS_STAT == 000b means "not powered from VBUS" and needs no ADC.
 *
 *  2) WRITE APIs (config_apply, measure, check_vbat_cutoff, battery_disconnect).
 *     A write to ANY register moves the charger from "default mode" to "host
 *     mode" and starts the I2C watchdog (default 50 s). bq25622_config_apply() therefore
 *     writes WATCHDOG=00 (disabled) in REG0x16 FIRST, so the watchdog never
 *     runs and the settings are not reverted (ICHG halved etc.).
 *     All write APIs refuse to run unless PART_INFO.PN is BQ25628 (2) or BQ25629 (6).
 *
 *  3) READ-ONLY diagnostic dump: bq25622_dump_regs(). NOTE it reads the FLAG registers
 *     (0x20..0x22), which CLEAR ON READ - call it rarely (boot), never from a poll loop.
 *
 * Datasheet: SLUSEG4C (BQ25628/BQ25629), Rev. C.
 * POR values (chip, before any write): ICHG 320 mA, VREG 4.2 V, ITERM 20 mA, IPRECHG 30 mA,
 * IINDPM 3.2 A (then set by D+/D- detection), WATCHDOG 50 s, IBAT_PK 12 A, VBAT_UVLO 2.2 V.
 */

/* 7-bit address: 0x6A for BQ25628/BQ25629 (datasheet SLUSEG4C section 8.6). Matches the
 * v1.4 board scan (only 0x6A ACKs) and PART_INFO.PN = 2. */
#ifndef BQ25622_I2C_ADDR7
#define BQ25622_I2C_ADDR7               0x6A
#endif
#define BQ25622_I2C_ADDR                (BQ25622_I2C_ADDR7 << 1)   /* HAL wants 8-bit */
#define BQ25622_I2C_MEMADD_8BIT         0x0001U         /* == I2C_MEMADD_SIZE_8BIT     */

#define BQ25622_REG_ICHG                0x02            /* 16-bit, ICHG[10:5],  40 mA/step, 40..2000 mA   */
#define BQ25622_REG_VREG                0x04            /* 16-bit, VREG[11:3],  10 mV/step                */
#define BQ25622_REG_IINDPM              0x06            /* 16-bit, IINDPM[11:4], 20 mA/step               */
#define BQ25622_REG_IPRECHG             0x10            /* 16-bit, IPRECHG[7:3], 10 mA/step               */
#define BQ25622_REG_ITERM               0x12            /* 16-bit, ITERM[7:2],   5 mA/step, 5..310 mA     */
#define BQ25622_REG_CHARGE_CONTROL      0x14            /* Q1_FULLON Q4_FULLON ITRICKLE TOPOFF_TMR EN_TERM ... */
#define BQ25622_REG_CHARGER_CTRL_0      0x16            /* EN_CHG[5] EN_HIZ[4] WD_RST[2] WATCHDOG[1:0]    */
#define BQ25622_REG_CHARGER_CTRL_2      0x18            /* BATFET_CTRL_WVBUS[3] BATFET_DLY[2] BATFET_CTRL[1:0] */
#define BQ25622_REG_CHARGER_CTRL_3      0x19            /* IBAT_PK[7:6] VBAT_UVLO[5] VBAT_OTG_MIN[4] ... */
#define BQ25622_REG_NTC_CONTROL_0       0x1A            /* TS_IGNORE[7] ...                               */
#define BQ25622_REG_CHARGER_STATUS_0    0x1D
#define BQ25622_REG_CHARGER_STATUS_1    0x1E            /* CHG_STAT[4:3], VBUS_STAT[2:0] */
#define BQ25622_REG_FAULT_STATUS_0      0x1F            /* VBUS/BAT/SYS/OTG fault, TSHUT, TS_STAT[2:0] */
#define BQ25622_REG_CHARGER_FLAG_0      0x20            /* ADC_DONE_FLAG[6], WD_FLAG[0]; clear-on-read */
#define BQ25622_REG_CHARGER_FLAG_1      0x21            /* clear-on-read */
#define BQ25622_REG_FAULT_FLAG_0        0x22            /* clear-on-read */
#define BQ25622_REG_ADC_CONTROL         0x26            /* ADC_EN[7] ADC_RATE[6] ADC_SAMPLE[5:4] */
#define BQ25622_REG_ADC_DISABLE_0       0x27            /* per-channel disable bits */
#define BQ25622_REG_VBUS_ADC            0x2C            /* 16-bit, [14:2], 3.97 mV/LSB */
#define BQ25622_REG_VBAT_ADC            0x30            /* 16-bit, [12:1], 1.99 mV/LSB */
#define BQ25622_REG_PART_INFO           0x38            /* PN[5:3], DEV_REV[2:0]         */

/* REG0x16 (Charger Control 0) */
#define BQ25622_CTRL0_EN_CHG            0x20U
#define BQ25622_CTRL0_WATCHDOG_MASK     0x03U           /* 00 = disabled, 01 = 50 s (POR) */
/* REG0x18 (Charger Control 2) */
#define BQ25622_CTRL2_BATFET_CTRL_MASK  0x03U
#define BQ25622_BATFET_NORMAL           0x00U
#define BQ25622_BATFET_SHUTDOWN         0x01U           /* only enters when VBUS absent */
#define BQ25622_CTRL2_BATFET_DLY        0x04U           /* 1 = 12.5 s delay (POR), 0 = 25 ms */
/* REG0x19 (Charger Control 3) - read-only use for now */
#define BQ25622_CTRL3_IBAT_PK_SHIFT     6               /* 10b = 6 A, 11b = 12 A (POR) */
#define BQ25622_CTRL3_IBAT_PK_MASK      0xC0U           /* REG0x19[7:6]; 00b/01b are reserved */
#define BQ25622_CTRL3_VBAT_UVLO         0x20U           /* 0 = 2.2 V (POR), 1 = 1.8 V   */
/* REG0x20 / REG0x26 */
#define BQ25622_FLAG0_ADC_DONE          0x40U
#define BQ25622_ADC_EN                  0x80U
#define BQ25622_ADC_ONESHOT             0x40U
#define BQ25622_ADC_SAMPLE_12BIT        0x00U           /* 30 ms per channel */
/* REG0x27 bits */
#define BQ25622_ADC_DIS_IBUS            0x80U
#define BQ25622_ADC_DIS_IBAT            0x40U
#define BQ25622_ADC_DIS_VBUS            0x20U
#define BQ25622_ADC_DIS_VBAT            0x10U
#define BQ25622_ADC_DIS_VSYS            0x08U
#define BQ25622_ADC_DIS_TS              0x04U
#define BQ25622_ADC_DIS_TDIE            0x02U
#define BQ25622_ADC_DIS_VPMID           0x01U

#define BQ25622_VBUS_STAT_MASK          0x07U
#define BQ25622_CHG_STAT_MASK           0x18U
#define BQ25622_CHG_STAT_SHIFT          3
#define BQ25622_PN_MASK                 0x38U
#define BQ25622_PN_SHIFT                3
#define BQ25622_PN_BQ25628              2
#define BQ25622_PN_BQ25629              6

/* VBUS_STAT values */
#define BQ25622_VBUS_NONE               0x0     /* not powered from VBUS          */
#define BQ25622_VBUS_UNKNOWN_ADAPTER    0x4     /* BQ25628: adapter present (BQ25629 also has 001..011, 101) */
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

/* ---- Charge / cutoff configuration (project decisions) -------------------- */
/*
 * Enum values are the physical value (mV / mA) so they read naturally in code and
 * log output. Only values that the register can represent EXACTLY are listed:
 *   VREG  : 10 mV/step      ICHG : 40 mA/step (40..2000 mA)
 *   ITERM :  5 mA/step      IINDPM: 20 mA/step
 * e.g. 500 mA is NOT a legal ICHG step, hence 480 / 520 mA.
 */
typedef enum {
    BQ25622_VREG_4100MV = 4100,
    BQ25622_VREG_4200MV = 4200,             /* chip default */
    BQ25622_VREG_4350MV = 4350,
    BQ25622_VREG_4400MV = 4400,
} bq25622_vreg_t;

typedef enum {
    BQ25622_ICHG_240MA  = 240,
    BQ25622_ICHG_320MA  = 320,              /* chip default (POR) */
    BQ25622_ICHG_400MA  = 400,
    BQ25622_ICHG_480MA  = 480,              /* closest step below 500 mA */
    BQ25622_ICHG_520MA  = 520,              /* closest step above 500 mA */
    BQ25622_ICHG_640MA  = 640,
    BQ25622_ICHG_800MA  = 800,
    BQ25622_ICHG_1040MA = 1040,
    BQ25622_ICHG_2000MA = 2000,             /* chip maximum */
} bq25622_ichg_t;

typedef enum {
    BQ25622_ITERM_20MA  = 20,               /* chip default (POR) */
    BQ25622_ITERM_40MA  = 40,
    BQ25622_ITERM_60MA  = 60,
    BQ25622_ITERM_80MA  = 80,
    BQ25622_ITERM_100MA = 100,
} bq25622_iterm_t;

typedef enum {
    BQ25622_IINDPM_KEEP   = 0,              /* do not write, leave the chip value */
    BQ25622_IINDPM_500MA  = 500,
    BQ25622_IINDPM_900MA  = 900,
    BQ25622_IINDPM_1000MA = 1000,
    BQ25622_IINDPM_1500MA = 1500,
    BQ25622_IINDPM_2000MA = 2000,
    BQ25622_IINDPM_3200MA = 3200,           /* chip default, also restored on every adapter removal */
} bq25622_iindpm_t;

/* Firmware discharge cutoff. The chip has no such threshold (only UVLO 2.2 / 1.8 V). */
typedef enum {
    BQ25622_CUTOFF_2800MV = 2800,
    BQ25622_CUTOFF_2900MV = 2900,
    BQ25622_CUTOFF_3000MV = 3000,
    BQ25622_CUTOFF_3100MV = 3100,
} bq25622_cutoff_t;

/* Delay between the shutdown command and the BATFET opening. */
typedef enum {
    BQ25622_BATFET_DLY_25MS  = 0,
    BQ25622_BATFET_DLY_12S5  = 1,           /* chip default (POR): time to flush logs */
} bq25622_batfet_dly_t;

/* Battery discharging peak-current protection (REG0x19[7:6], fast ~100 us trip).
 * Only 6 A and 12 A exist. This is a short-circuit / overload protection, NOT a load limit.
 * BATFET_OCP (6 A, ~50 ms) is a separate fixed threshold and cannot be changed. */
typedef enum {
    BQ25622_IBAT_PK_6A  = 6,                /* 10b */
    BQ25622_IBAT_PK_12A = 12,               /* 11b, chip default (POR) */
} bq25622_ibat_pk_t;

typedef struct {
    bq25622_vreg_t       vreg;
    bq25622_ichg_t       ichg;
    bq25622_iterm_t      iterm;
    bq25622_iindpm_t     iindpm;
    bq25622_ibat_pk_t    ibat_pk;           /* discharge peak-current protection */
    uint8_t              en_chg;            /* 1 = charging enabled */

    bq25622_cutoff_t     cutoff;
    uint8_t              cutoff_confirm;    /* consecutive low readings before cutting */
    /* ADC refuses to run when VBAT <= VBAT_LOWV (2.7..2.9 V). 1 = count a refused ADC
     * (no VBUS) as a "low" reading. 0 until verified on the board: a wrong cut is only
     * recoverable by plugging USB, UVLO (2.2 V) is the last resort. */
    uint8_t              cutoff_on_adc_refused;
    bq25622_batfet_dly_t batfet_dly;
} bq25622_cfg_t;

/* Project defaults (user decision): VREG 4.2 V, ICHG 320 mA, ITERM 20 mA, IBAT_PK 12 A,
 * cutoff 2.9 V. VREG/ICHG/ITERM/IBAT_PK equal the chip POR values, so config_apply() only
 * has to write what differs from POR (WATCHDOG off, ADC channel mask).
 * Applied from sx_board_init() right after bq25622_init() succeeds. */
#define BQ25622_CFG_DEFAULT {                       \
    .vreg                  = BQ25622_VREG_4200MV,   \
    .ichg                  = BQ25622_ICHG_320MA,    \
    .iterm                 = BQ25622_ITERM_20MA,    \
    .iindpm                = BQ25622_IINDPM_KEEP,   \
    .ibat_pk               = BQ25622_IBAT_PK_12A,   \
    .en_chg                = 1,                     \
    .cutoff                = BQ25622_CUTOFF_2900MV, \
    .cutoff_confirm        = 3,                     \
    .cutoff_on_adc_refused = 0,                     \
    .batfet_dly            = BQ25622_BATFET_DLY_12S5, \
}

/* Only VBUS and VBAT are measured; TS is disabled so the ADC keeps working
 * down to VBAT_LOWV instead of stopping at 3.2 V. */
#ifndef BQ25622_ADC_DIS_MASK
#define BQ25622_ADC_DIS_MASK            (BQ25622_ADC_DIS_IBUS | BQ25622_ADC_DIS_IBAT | \
                                         BQ25622_ADC_DIS_VSYS | BQ25622_ADC_DIS_TS   | \
                                         BQ25622_ADC_DIS_TDIE | BQ25622_ADC_DIS_VPMID)
#endif
#ifndef BQ25622_ADC_TIMEOUT_MS
#define BQ25622_ADC_TIMEOUT_MS          200U
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

    bq25622_cfg_t cfg;          /* edit after init(), before config_apply() */

    uint16_t vbat_mv;           /* last successful ADC reading (0 = none)  */
    uint16_t vbus_mv;
    uint8_t  cutoff_cnt;        /* consecutive low-VBAT readings           */
    uint8_t  cfg_ok;            /* 1 once config_apply() succeeded         */

    uint32_t elapsed_ms;
} bq25622_t;

/* Probe the IC, read part info + first status sample (no event generated).
 * Returns 0 if the IC answered, -1 otherwise (driver stays usable, poll keeps retrying). */
int bq25622_init(bq25622_t *dev, sx_i2c_t *i2c);

/* READ-ONLY dump + decode of PART_INFO, status 0/1, FAULT_STATUS_0, the 3 FLAG registers and the
 * current charge configuration (ICHG/VREG/IINDPM/IPRECHG/ITERM, CTRL regs, IBAT_PK, VBAT_UVLO,
 * TS_IGNORE). Does NOT write, so the chip stays in default mode. The FLAG registers (0x20..0x22)
 * clear on read: the first call after power-up shows everything that happened since POR.
 * Returns 0 if every register was read, -1 if any read failed. */
int bq25622_dump_regs(bq25622_t *dev);

/* Read REG0x1E now. Returns 0 on success, -1 on I2C error (state left untouched). */
int bq25622_read_status(bq25622_t *dev);

/* Call every main-loop iteration with the elapsed ms. Reads the charger every
 * BQ25622_POLL_INTERVAL_MS and returns a debounced plug/unplug event. */
bq25622_event_t bq25622_poll(bq25622_t *dev, uint32_t ts_ms);

/* Immediate read (no debounce, no event) — use right after waking from STOP.
 * Returns the resulting "present" flag; on I2C error returns the last known value. */
uint8_t bq25622_refresh(bq25622_t *dev);

/* ---- Write APIs ------------------------------------------------------- */

/* Disable the watchdog, apply dev->cfg: VREG / ICHG / ITERM (/ IINDPM unless KEEP), EN_CHG,
 * and restrict the ADC channels. Each field is read first and only written when
 * different, then read back to verify. Call at boot and after every USB plug
 * (IINDPM returns to 3.2 A whenever the adapter is removed).
 * Returns 0 on success, -1 on I2C error or verify mismatch. */
int bq25622_config_apply(bq25622_t *dev);

/* One-shot ADC measurement of VBAT and VBUS in mV (either pointer may be NULL).
 * Blocks up to BQ25622_ADC_TIMEOUT_MS.
 * Returns 0 ok, -1 I2C error/timeout, -2 ADC refused to start (VBAT/VBUS too low). */
int bq25622_measure(bq25622_t *dev, uint16_t *vbat_mv, uint16_t *vbus_mv);

/* Enable / disable charging (EN_CHG, REG0x16 bit5), read-modify-write. */
int bq25622_set_charge_enable(bq25622_t *dev, uint8_t enable);

/* Open the BATFET (BATFET_CTRL = shutdown). Refused (-2) while VBUS is present,
 * because the chip ignores it in that case. After success the board LOSES POWER
 * after BATFET_DLY (12.5 s default) when running on battery. Only a USB plug wakes it.
 * Returns 0 if the command was accepted, -1 on I2C error, -2 if VBUS present. */
int bq25622_battery_disconnect(bq25622_t *dev);

/* Call on each long wake with GPS/SIM OFF. On battery only: measures VBAT and, after
 * cfg.cutoff_confirm consecutive readings below cfg.cutoff, calls
 * bq25622_battery_disconnect(). Never cuts on an I2C/ADC error.
 * Returns 1 if a disconnect was commanded, 0 if no action, <0 on error. */
int bq25622_check_vbat_cutoff(bq25622_t *dev);

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