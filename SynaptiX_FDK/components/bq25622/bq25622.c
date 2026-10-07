#include "bq25622.h"
#include "logger.h"
#include "sx_delay.h"

static const char *TAG = "BQ25628";

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
    dev->vbat_mv      = 0;
    dev->vbus_mv      = 0;
    dev->cutoff_cnt   = 0;
    dev->cfg_ok       = 0;
    dev->cfg          = (bq25622_cfg_t)BQ25622_CFG_DEFAULT;

    uint8_t pi = 0;
    if (reg_read8(dev, BQ25622_REG_PART_INFO, &pi) != 0) {
        log_error(TAG, "not found on I2C (addr 0x%02X)", BQ25622_I2C_ADDR >> 1);
        return -1;
    }

    /* Diagnostic dump (read-only). If the chip at this address is NOT a BQ25628/9 these
     * values are meaningless, so do not trust PN/VBUS_STAT below until they look sane. */
    {
        uint8_t s0 = 0xEE, s1 = 0xEE;
        int e0 = reg_read8(dev, BQ25622_REG_CHARGER_STATUS_0, &s0);
        int e1 = reg_read8(dev, BQ25622_REG_CHARGER_STATUS_1, &s1);
        log_debug(TAG, "raw @0x%02X: PART_INFO(0x38)=0x%02X STATUS0(0x1D)=0x%02X%s STATUS1(0x1E)=0x%02X%s",
                 BQ25622_I2C_ADDR >> 1, pi, s0, e0 ? "(err)" : "", s1, e1 ? "(err)" : "");
    }
    dev->online      = 1;
    dev->part_number = (pi & BQ25622_PN_MASK) >> BQ25622_PN_SHIFT;
    log_info(TAG, "found: PN=%u (%s) rev=%u", dev->part_number,
             dev->part_number == BQ25622_PN_BQ25628 ? "BQ25628" :
             dev->part_number == BQ25622_PN_BQ25629 ? "BQ25629" : "unknown",
             pi & 0x07U);
    if (!(dev->part_number == BQ25622_PN_BQ25628 || dev->part_number == BQ25622_PN_BQ25629))
        log_warn(TAG, "PN=%u is not BQ25628/BQ25629: register WRITES are disabled", dev->part_number);

    if (bq25622_read_status(dev) == 0) {
        dev->present = vbus_stat_is_present(dev->vbus_stat);
        dev->valid   = 1;
        log_info(TAG, "initial VBUS_STAT=%u -> USB %s", dev->vbus_stat,
                 dev->present ? "present" : "absent");
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Low-level write helpers                                                    */
/* ------------------------------------------------------------------------ */

static int reg_write8(bq25622_t *dev, uint8_t reg, uint8_t val)
{
    return sx_i2c_mem_write(dev->i2c, BQ25622_I2C_ADDR, reg,
                            BQ25622_I2C_MEMADD_8BIT, &val, 1);
}

/* 16-bit registers are little-endian, low byte at the lower address. */
static int reg_read16(bq25622_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t b[2];
    if (sx_i2c_mem_read(dev->i2c, BQ25622_I2C_ADDR, reg,
                        BQ25622_I2C_MEMADD_8BIT, b, 2) != 0)
        return -1;
    *val = (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
    return 0;
}

static int reg_write16(bq25622_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t b[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return sx_i2c_mem_write(dev->i2c, BQ25622_I2C_ADDR, reg,
                            BQ25622_I2C_MEMADD_8BIT, b, 2);
}

/* Read-modify-write, skipped when the field already has the wanted value,
 * then read back to verify. */
static int reg8_update(bq25622_t *dev, uint8_t reg, uint8_t mask, uint8_t val,
                       const char *name)
{
    uint8_t cur = 0;
    if (reg_read8(dev, reg, &cur) != 0) {
        log_error(TAG, "%s: read 0x%02X failed", name, reg);
        return -1;
    }
    if ((cur & mask) == val)
        return 0;

    uint8_t nv = (uint8_t)((cur & ~mask) | val);
    if (reg_write8(dev, reg, nv) != 0) {
        log_error(TAG, "%s: write 0x%02X failed", name, reg);
        return -1;
    }
    if (reg_read8(dev, reg, &cur) != 0 || (cur & mask) != val) {
        log_error(TAG, "%s: verify 0x%02X failed (got 0x%02X, want field 0x%02X)",
                  name, reg, cur, val);
        return -1;
    }
    log_debug(TAG, "%s set (reg 0x%02X = 0x%02X)", name, reg, cur);
    return 0;
}

static int reg16_update(bq25622_t *dev, uint8_t reg, uint16_t mask, uint16_t val,
                        const char *name)
{
    uint16_t cur = 0;
    if (reg_read16(dev, reg, &cur) != 0) {
        log_error(TAG, "%s: read 0x%02X failed", name, reg);
        return -1;
    }
    if ((cur & mask) == val)
        return 0;

    uint16_t nv = (uint16_t)((cur & ~mask) | val);
    if (reg_write16(dev, reg, nv) != 0) {
        log_error(TAG, "%s: write 0x%02X failed", name, reg);
        return -1;
    }
    if (reg_read16(dev, reg, &cur) != 0 || (cur & mask) != val) {
        log_error(TAG, "%s: verify 0x%02X failed (got 0x%04X, want field 0x%04X)",
                  name, reg, cur, val);
        return -1;
    }
    log_debug(TAG, "%s set (reg 0x%02X = 0x%04X)", name, reg, cur);
    return 0;
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Every API that WRITES goes through this: the register map in this driver is the BQ25628/9 one. */
static int part_writable(const bq25622_t *dev, const char *who)
{
    if (dev->online &&
        (dev->part_number == BQ25622_PN_BQ25628 || dev->part_number == BQ25622_PN_BQ25629))
        return 1;
    log_warn(TAG, "%s refused: chip not identified as BQ25628/9 (online=%u PN=%u)",
             who, dev->online, dev->part_number);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Configuration                                                              */
/* ------------------------------------------------------------------------ */

int bq25622_config_apply(bq25622_t *dev)
{
    int rc = 0;
    dev->cfg_ok = 0;
    if (!part_writable(dev, "config_apply")) return -1;

    /* 1) FIRST write: watchdog off (+ EN_CHG). The first write of any register
     *    enters host mode and would start the 50 s watchdog; writing WATCHDOG=00
     *    here means it never runs. WD_RST (bit2) is left at 0. */
    uint8_t ctrl0_mask = (uint8_t)(BQ25622_CTRL0_EN_CHG | BQ25622_CTRL0_WATCHDOG_MASK);
    uint8_t ctrl0_val  = dev->cfg.en_chg ? BQ25622_CTRL0_EN_CHG : 0U;
    rc |= reg8_update(dev, BQ25622_REG_CHARGER_CTRL_0, ctrl0_mask, ctrl0_val,
                      "WATCHDOG=off,EN_CHG");

    /* 2) VREG = mV/10, bits [11:3]. 4200 mV -> 0x1A4 -> reg 0x0D20 (= POR). */
    uint32_t vreg = clamp_u32((uint32_t)dev->cfg.vreg, 3500U, 4800U) / 10U;
    rc |= reg16_update(dev, BQ25622_REG_VREG, 0x0FF8U, (uint16_t)(vreg << 3), "VREG");

    /* 3) ICHG = mA/40, bits [10:5] (BQ25628: 40 mA..2000 mA). Bits [15:11] and [4:0] are reserved. */
    uint32_t ichg = clamp_u32((uint32_t)dev->cfg.ichg, 40U, 2000U) / 40U;
    rc |= reg16_update(dev, BQ25622_REG_ICHG, 0x07E0U, (uint16_t)(ichg << 5), "ICHG");

    /* 4) ITERM = mA/5, bits [7:2] (BQ25628: 5 mA..310 mA). */
    uint32_t iterm = clamp_u32((uint32_t)dev->cfg.iterm, 5U, 310U) / 5U;
    rc |= reg16_update(dev, BQ25622_REG_ITERM, 0x00FCU, (uint16_t)(iterm << 2), "ITERM");

    /* 5) IINDPM = mA/20, bits [11:4] — only when configured. It falls back to
     *    3.2 A on every adapter removal, so call config_apply() after each plug. */
    if (dev->cfg.iindpm != BQ25622_IINDPM_KEEP) {
        uint32_t iin = clamp_u32((uint32_t)dev->cfg.iindpm, 100U, 3200U) / 20U;
        rc |= reg16_update(dev, BQ25622_REG_IINDPM, 0x0FF0U, (uint16_t)(iin << 4), "IINDPM");
    }

    /* 6) IBAT_PK (REG0x19[7:6]): 10b = 6 A, 11b = 12 A (POR). Skipped by reg8_update()
     *    when the chip already holds the wanted value. */
    {
        uint8_t pk_bits = (dev->cfg.ibat_pk == BQ25622_IBAT_PK_6A) ? 0x2U : 0x3U;
        rc |= reg8_update(dev, BQ25622_REG_CHARGER_CTRL_3, BQ25622_CTRL3_IBAT_PK_MASK,
                          (uint8_t)(pk_bits << BQ25622_CTRL3_IBAT_PK_SHIFT), "IBAT_PK");
    }

    /* 7) ADC channels: keep VBUS + VBAT only, TS off (ADC then works down to
     *    VBAT_LOWV (2.7..2.9 V) instead of stopping at 3.2 V). */
    rc |= reg8_update(dev, BQ25622_REG_ADC_DISABLE_0, 0xFFU,
                      (uint8_t)BQ25622_ADC_DIS_MASK, "ADC channels");

    if (rc != 0) {
        log_error(TAG, "config_apply FAILED");
        return -1;
    }
    dev->cfg_ok = 1;
    log_info(TAG, "config OK: VREG=%u mV ICHG=%u mA ITERM=%u mA IBAT_PK=%u A",
             (unsigned)(vreg * 10U), (unsigned)(ichg * 40U), (unsigned)(iterm * 5U),
             (unsigned)dev->cfg.ibat_pk);
    return 0;
}

int bq25622_set_charge_enable(bq25622_t *dev, uint8_t enable)
{
    if (!part_writable(dev, "set_charge_enable")) return -1;
    return reg8_update(dev, BQ25622_REG_CHARGER_CTRL_0, BQ25622_CTRL0_EN_CHG,
                       enable ? BQ25622_CTRL0_EN_CHG : 0U, "EN_CHG");
}

/* ------------------------------------------------------------------------ */
/* ADC                                                                        */
/* ------------------------------------------------------------------------ */

int bq25622_measure(bq25622_t *dev, uint16_t *vbat_mv, uint16_t *vbus_mv)
{
    uint8_t r = 0;
    if (!part_writable(dev, "measure")) return -1;     /* it writes ADC_CONTROL */

    /* Reading FLAG0 clears ADC_DONE_FLAG, so a later "1" means a NEW conversion.
     * (The ADC result registers never clear — a stale value looks valid.) */
    if (reg_read8(dev, BQ25622_REG_CHARGER_FLAG_0, &r) != 0)
        return -1;

    if (reg_write8(dev, BQ25622_REG_ADC_CONTROL,
                   BQ25622_ADC_EN | BQ25622_ADC_ONESHOT | BQ25622_ADC_SAMPLE_12BIT) != 0)
        return -1;

    /* If VBUS/VBAT are too low the chip forces ADC_EN back to 0 right away. */
    if (reg_read8(dev, BQ25622_REG_ADC_CONTROL, &r) != 0)
        return -1;
    if (!(r & BQ25622_ADC_EN)) {
        uint8_t f = 0;
        if (reg_read8(dev, BQ25622_REG_CHARGER_FLAG_0, &f) == 0 &&
            (f & BQ25622_FLAG0_ADC_DONE))
            goto done;                       /* it simply finished already */
        return -2;
    }

    uint32_t waited = 0;
    for (;;) {
        sx_delay_ms(5);
        waited += 5;
        if (reg_read8(dev, BQ25622_REG_CHARGER_FLAG_0, &r) != 0)
            return -1;
        if (r & BQ25622_FLAG0_ADC_DONE)
            break;
        if (waited >= BQ25622_ADC_TIMEOUT_MS) {
            log_warn(TAG, "ADC one-shot timeout");
            return -1;
        }
    }

done:;
    uint16_t raw = 0;
    if (reg_read16(dev, BQ25622_REG_VBAT_ADC, &raw) != 0)
        return -1;
    uint16_t bat = (uint16_t)((((uint32_t)(raw >> 1) & 0x0FFFU) * 199U) / 100U);   /* 1.99 mV */

    uint16_t bus = 0;
    if (vbus_mv != NULL || !(BQ25622_ADC_DIS_MASK & BQ25622_ADC_DIS_VBUS)) {
        if (reg_read16(dev, BQ25622_REG_VBUS_ADC, &raw) != 0)
            return -1;
        bus = (uint16_t)((((uint32_t)(raw >> 2) & 0x1FFFU) * 397U) / 100U);        /* 3.97 mV */
    }

    if (bat == 0)                 /* conversion "done" but nothing latched: do not trust */
        return -1;

    dev->vbat_mv = bat;
    dev->vbus_mv = bus;
    if (vbat_mv) *vbat_mv = bat;
    if (vbus_mv) *vbus_mv = bus;
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Discharge cutoff                                                           */
/* ------------------------------------------------------------------------ */

int bq25622_battery_disconnect(bq25622_t *dev)
{
    if (!part_writable(dev, "battery_disconnect")) return -1;
    /* Shutdown is only accepted when VBUS is absent; with an adapter the chip
     * ignores it and clears BATFET_CTRL back to 00. Check first. */
    if (bq25622_read_status(dev) != 0)
        return -1;
    if (vbus_stat_is_present(dev->vbus_stat)) {
        log_warn(TAG, "disconnect refused: VBUS present (VBUS_STAT=%u)", dev->vbus_stat);
        return -2;
    }

    uint8_t mask = (uint8_t)(BQ25622_CTRL2_BATFET_CTRL_MASK | BQ25622_CTRL2_BATFET_DLY);
    uint8_t val  = (uint8_t)(BQ25622_BATFET_SHUTDOWN |
                             (dev->cfg.batfet_dly == BQ25622_BATFET_DLY_12S5
                              ? BQ25622_CTRL2_BATFET_DLY : 0U));
    uint8_t cur = 0;
    if (reg_read8(dev, BQ25622_REG_CHARGER_CTRL_2, &cur) != 0)
        return -1;

    log_warn(TAG, "BATTERY DISCONNECT (BATFET_CTRL=shutdown, delay %s) — board will lose power",
             dev->cfg.batfet_dly == BQ25622_BATFET_DLY_12S5 ? "12.5 s" : "25 ms");

    /* No read-back: with a 25 ms delay the chip (and I2C) may already be gone. */
    if (reg_write8(dev, BQ25622_REG_CHARGER_CTRL_2, (uint8_t)((cur & ~mask) | val)) != 0)
        return -1;
    return 0;
}

int bq25622_check_vbat_cutoff(bq25622_t *dev)
{
    /* Unknown power source (I2C error) must never lead to a cut. */
    if (bq25622_read_status(dev) != 0)
        return -1;

    if (vbus_stat_is_present(dev->vbus_stat)) {      /* on USB: nothing to protect */
        dev->cutoff_cnt = 0;
        return 0;
    }

    uint16_t vbat = 0;
    int m = bq25622_measure(dev, &vbat, NULL);
    uint8_t low = 0;

    if (m == 0) {
        low = (vbat <= (uint16_t)dev->cfg.cutoff);
        if (low)
            log_info(TAG, "VBAT=%u mV (cutoff %u mV) LOW", vbat, (unsigned)dev->cfg.cutoff);
        else
            log_debug(TAG, "VBAT=%u mV (cutoff %u mV)", vbat, (unsigned)dev->cfg.cutoff);
    } else if (m == -2) {
        /* ADC refused to start with no VBUS: VBAT is at/below VBAT_LOWV (2.7..2.9 V). */
        log_warn(TAG, "ADC refused (VBAT probably <= VBAT_LOWV)");
        low = dev->cfg.cutoff_on_adc_refused ? 1U : 0U;
        if (!low)
            return -2;
    } else {
        return -1;
    }

    if (!low) {
        dev->cutoff_cnt = 0;
        return 0;
    }
    if (dev->cutoff_cnt < 255) dev->cutoff_cnt++;
    if (dev->cutoff_cnt < dev->cfg.cutoff_confirm)
        return 0;

    return (bq25622_battery_disconnect(dev) == 0) ? 1 : -1;
}

/* ------------------------------------------------------------------------ */
/* Read-only diagnostic dump                                                  */
/* ------------------------------------------------------------------------ */

int bq25622_dump_regs(bq25622_t *dev)
{
    uint8_t  pi = 0xEE, s0 = 0xEE, s1 = 0xEE, f0 = 0xEE, fl0 = 0xEE, fl1 = 0xEE, ffl = 0xEE;
    uint8_t  chg = 0xEE, c0 = 0xEE, c2 = 0xEE, c3 = 0xEE, ntc = 0xEE;
    uint16_t ich = 0xEEEE, vrg = 0xEEEE, iin = 0xEEEE, ipc = 0xEEEE, itm = 0xEEEE;
    int bad = 0;

    bad |= reg_read8(dev, BQ25622_REG_PART_INFO,      &pi);
    bad |= reg_read8(dev, BQ25622_REG_CHARGER_STATUS_0, &s0);
    bad |= reg_read8(dev, BQ25622_REG_CHARGER_STATUS_1, &s1);
    bad |= reg_read8(dev, BQ25622_REG_FAULT_STATUS_0, &f0);
    /* FLAG registers clear on read: this is the one-time snapshot of what happened since POR. */
    bad |= reg_read8(dev, BQ25622_REG_CHARGER_FLAG_0, &fl0);
    bad |= reg_read8(dev, BQ25622_REG_CHARGER_FLAG_1, &fl1);
    bad |= reg_read8(dev, BQ25622_REG_FAULT_FLAG_0,   &ffl);
    bad |= reg_read16(dev, BQ25622_REG_ICHG,    &ich);
    bad |= reg_read16(dev, BQ25622_REG_VREG,    &vrg);
    bad |= reg_read16(dev, BQ25622_REG_IINDPM,  &iin);
    bad |= reg_read16(dev, BQ25622_REG_IPRECHG, &ipc);
    bad |= reg_read16(dev, BQ25622_REG_ITERM,   &itm);
    bad |= reg_read8(dev, BQ25622_REG_CHARGE_CONTROL, &chg);
    bad |= reg_read8(dev, BQ25622_REG_CHARGER_CTRL_0, &c0);
    bad |= reg_read8(dev, BQ25622_REG_CHARGER_CTRL_2, &c2);
    bad |= reg_read8(dev, BQ25622_REG_CHARGER_CTRL_3, &c3);
    bad |= reg_read8(dev, BQ25622_REG_NTC_CONTROL_0,  &ntc);

    log_debug(TAG, "dump: PART_INFO=0x%02X PN=%u REV=%u", pi,
             (unsigned)((pi & BQ25622_PN_MASK) >> BQ25622_PN_SHIFT), (unsigned)(pi & 0x07U));
    log_debug(TAG, "dump: STATUS0=0x%02X STATUS1=0x%02X (CHG_STAT=%u VBUS_STAT=%u) FAULT0=0x%02X "
                  "(VBUS_F=%u BAT_F=%u SYS_F=%u OTG_F=%u TSHUT=%u TS_STAT=%u)",
             s0, s1,
             (unsigned)((s1 & BQ25622_CHG_STAT_MASK) >> BQ25622_CHG_STAT_SHIFT),
             (unsigned)(s1 & BQ25622_VBUS_STAT_MASK), f0,
             (unsigned)((f0 >> 7) & 1U), (unsigned)((f0 >> 6) & 1U), (unsigned)((f0 >> 5) & 1U),
             (unsigned)((f0 >> 4) & 1U), (unsigned)((f0 >> 3) & 1U), (unsigned)(f0 & 0x07U));
    log_debug(TAG, "dump: FLAG0=0x%02X FLAG1=0x%02X FAULT_FLAG0=0x%02X (cleared by this read; WD_FLAG=%u)",
             fl0, fl1, ffl, (unsigned)(fl0 & 1U));
    log_debug(TAG, "dump: ICHG=%u mA VREG=%u mV IINDPM=%u mA IPRECHG=%u mA ITERM=%u mA",
             (unsigned)(((ich >> 5) & 0x3FU) * 40U), (unsigned)(((vrg >> 3) & 0x1FFU) * 10U),
             (unsigned)(((iin >> 4) & 0xFFU) * 20U), (unsigned)(((ipc >> 3) & 0x1FU) * 10U),
             (unsigned)(((itm >> 2) & 0x3FU) * 5U));
    {
        uint8_t pk = (uint8_t)(c3 >> BQ25622_CTRL3_IBAT_PK_SHIFT);
        log_debug(TAG, "dump: CHG_CTRL=0x%02X CTRL0=0x%02X (EN_CHG=%u WATCHDOG=%u) CTRL2=0x%02X "
                      "CTRL3=0x%02X (IBAT_PK=%s VBAT_UVLO=%s) NTC0=0x%02X (TS_IGNORE=%u)",
                 chg, c0, (unsigned)((c0 >> 5) & 1U), (unsigned)(c0 & BQ25622_CTRL0_WATCHDOG_MASK),
                 c2, c3,
                 pk == 3U ? "12A" : (pk == 2U ? "6A" : "rsvd"),
                 (c3 & BQ25622_CTRL3_VBAT_UVLO) ? "1.8V" : "2.2V",
                 ntc, (unsigned)((ntc >> 7) & 1U));
    }
    if (bad) log_warn(TAG, "dump: some register reads FAILED (shown as 0xEE/0xEEEE)");
    return bad ? -1 : 0;
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