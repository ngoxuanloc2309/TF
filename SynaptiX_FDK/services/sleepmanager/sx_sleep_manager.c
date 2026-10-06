#include "sx_sleep_manager.h"
#include "sx_user_mqtt.h"
#include "sx_board.h"
#include "sx_ex_storage.h"
#include "bq25622.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>

static const char *TAG = "SX_SLEEP_MGR";

void sx_sleep_manager_init(sx_sleep_manager_t *mgr,
                           sx_sleep_t         *sleep,
                           sim76xx_t          *sim,
                           sx_gps_t           *gps)
{
    mgr->sleep              = sleep;
    mgr->module.sim         = sim;
    mgr->module.gps         = gps;
    mgr->wake_step          = SX_WAKE_STEP_IDLE;
    mgr->published          = 0;
    mgr->elapsed.wake_elapsed_ms = 0;
    mgr->elapsed.gps_elapsed_ms  = 0;
    mgr->elapsed.sim_elapsed_ms  = 0;
    mgr->sleep_ms        = 0;   
    mgr->check_ms        = 0;
    mgr->wake_timeout_ms = 0;

    log_info(TAG, "init OK — waiting for config values");
}

/* Keep the debug log in the wake-fake loop (needs the log UART awake). Set 0 when measuring sleep current. */
#ifndef SX_WAKE_FAKE_LOG
#define SX_WAKE_FAKE_LOG   1
#endif

/* Seconds from the internal RTC calendar (only differences are used, so the epoch does not matter).
 * Used to count the publish period from real time instead of summing nominal periods. */
static uint64_t _rtc_secs(void)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);      /* must follow GetTime (shadow registers) */

    uint32_t y = 2000U + d.Year, m = d.Month, dd = d.Date;
    y -= (m <= 2U) ? 1U : 0U;
    uint32_t era = y / 400U;
    uint32_t yoe = y - era * 400U;
    uint32_t doy = (153U * ((m > 2U) ? (m - 3U) : (m + 9U)) + 2U) / 5U + dd - 1U;
    uint32_t doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    uint64_t days = (uint64_t)era * 146097U + doe;

    return days * 86400ULL + (uint64_t)t.Hours * 3600U + (uint64_t)t.Minutes * 60U + t.Seconds;
}

/* Sleep for sleep_ms in total, waking every check_ms ("wake-fake") to read VBUS_STAT.
 *
 *  - Before the loop: GPS, SIM, flash (power cut), IMU (SUSPEND) are all put to sleep.
 *  - Wake-fake: I2C1 is the ONLY thing turned on. Read VBUS_STAT, then I2C1 off and back to STOP.
 *  - VBUS present  -> wake-real: wake_reason = WAKE_REASON_VBUS (the app treats it as "USB connected"), IMU resumed, I2C1 left on.
 *  - Period elapsed -> wake_reason = WAKE_REASON_RTC (publish wake), I2C1 left on (RTC write needs it);
 *                     the other modules are started by the WAKE_PUBLISH steps / on demand.
 */
void sx_sleep_manager_enter(sx_sleep_manager_t *mgr)
{
    uint32_t sleep_ms  = (mgr->sleep_ms > 0) ? mgr->sleep_ms : SX_TIME_IN_SLEEP;
    uint32_t sleep_sec = sleep_ms / 1000U;
    if (sleep_sec == 0) sleep_sec = 1;

    uint32_t check_sec = ((mgr->check_ms > 0) ? mgr->check_ms : SX_TIME_WAKE_FAKE) / 1000U;
    if (check_sec == 0)         check_sec = 1;
    if (check_sec > sleep_sec)  check_sec = sleep_sec;

    log_info(TAG, "Entering sleep for %lu ms (%lu sec), VBUS check every %lu sec",
             sleep_ms, sleep_sec, check_sec);

    gps_power_off(mgr->module.gps);
    sx_delay_ms(100);
    mgr->module.gps->latitude   = 0.0f;
    mgr->module.gps->longtitude = 0.0f;
    sim76xx_power_off_blocking(mgr->module.sim);
    sx_delay_ms(500);

    sx_storage_sleep();          /* wait WIP, cut flash rail, SPI DeInit. sx_storage_* wake it again on demand */
    sx_board_imu_suspend();      /* needs I2C1 on: do it before the first I2C1 off. No-op if already suspended */

    uint64_t start_s   = _rtc_secs();
    uint32_t acc_s     = 0;      /* sum of completed RTC periods (fallback if the RTC calendar is changed) */
    uint32_t elapsed_s = 0;
    wake_reason_t result = WAKE_REASON_RTC;

    for (;;)
    {
        uint32_t remaining = sleep_sec - elapsed_s;
        uint32_t period    = (remaining < check_sec) ? remaining : check_sec;
        if (period == 0) period = 1;

        sx_board_i2c1_off();                         /* PB6/PB7 analog, external pull-ups stay */
        sx_sleep_set_rtc_wake(mgr->sleep, period);

#if SX_WAKE_FAKE_LOG
        log_info(TAG, ">>> Entering STOP mode NOW (%lu s, elapsed %lu/%lu s)", period, elapsed_s, sleep_sec);
        sx_delay_ms(10);                             /* let the log UART drain */
#endif
        sx_sleep_enter_stop(mgr->sleep);
        sx_sleep_cancel_rtc(mgr->sleep);

        /* Only a real RTC wake counts as a completed period; a spurious wake just loops again. */
        if (mgr->sleep->wake_reason == WAKE_REASON_RTC)
            acc_s += period;

        uint64_t now_s    = _rtc_secs();
        uint32_t rtc_el_s = (now_s > start_s) ? (uint32_t)(now_s - start_s) : 0U;
        elapsed_s = (rtc_el_s > acc_s) ? rtc_el_s : acc_s;   /* RTC time, but never behind the sum */

        if (elapsed_s >= sleep_sec)
        {
            sx_board_i2c1_on();                      /* publish wake: RTC write and BQ need I2C1 */
            result = WAKE_REASON_RTC;
            break;
        }

        /* ---- wake-fake: only I2C1 + BQ ---- */
        if (sx_board_i2c1_on() != 0) {
            log_warn(TAG, "wake-fake: I2C1 init failed — keep sleeping");
            continue;                                /* never treat an I2C error as "USB plugged" */
        }

        if (bq25622_refresh(&board.bq))
        {
            log_info(TAG, "wake-fake: VBUS_STAT=%u -> USB present, wake-real", board.bq.vbus_stat);
            sx_board_imu_resume();                   /* I2C1 is on; calib after suspend is unverified (handoff 12.4) */
            result = WAKE_REASON_VBUS;
            break;
        }
#if SX_WAKE_FAKE_LOG
        log_info(TAG, "wake-fake: VBUS_STAT=%u -> no USB", board.bq.vbus_stat);
#endif
    }

    mgr->sleep->wake_reason = result;
    log_info(TAG, "<<< Woke from STOP mode (%s after %lu s)",
             (result == WAKE_REASON_VBUS) ? "VBUS" : "RTC", elapsed_s);
}

void sx_sleep_manager_wake_process(sx_sleep_manager_t *mgr, uint32_t delta_ms)
{
    switch (mgr->wake_step)
    {
    case SX_WAKE_STEP_IDLE:
        mgr->wake_step = SX_WAKE_STEP_GPS_ON_FIRST;
        break;

    case SX_WAKE_STEP_GPS_ON_FIRST:
        board_gps_uart_resume_it();
        log_info(TAG, "Power on GPS first");
        gps_power_on(mgr->module.gps);
        //board_gps_uart_resume_it();
        mgr->elapsed.gps_elapsed_ms = 0;
        mgr->wake_step = SX_WAKE_STEP_GPS_WAIT;
        break;

    case SX_WAKE_STEP_GPS_WAIT:
        mgr->elapsed.gps_elapsed_ms += delta_ms;

        if (mgr->module.gps->latitude  != 0.0f &&
            mgr->module.gps->longtitude != 0.0f)
        {
            log_info(TAG, "GPS fix OK after %lu ms — now starting SIM",
                     mgr->elapsed.gps_elapsed_ms);
            mgr->wake_step = SX_WAKE_STEP_UART_RESUME;
        }
        else if (mgr->elapsed.gps_elapsed_ms >= GPS_TIMEOUT_MS)
        {
            log_warn(TAG, "GPS timeout — proceed without fix, starting SIM");
            mgr->wake_step = SX_WAKE_STEP_UART_RESUME;
        }
        break;

    case SX_WAKE_STEP_UART_RESUME:
        log_info(TAG, "Resume UART + Power on SIM");

        sx_board_uart_resume_it();

        mgr->module.sim->base.isBusy  = 0;
        mgr->module.sim->base.buff_id = 0;
        memset(mgr->module.sim->base.buff, 0, MODEM_RX_BUFFER_SIZE);
        mgr->module.sim->state = SIM76XX_STATE_IDLE;

        sim76xx_power_on(mgr->module.sim);
        sim76xx_start(mgr->module.sim);

        mgr->elapsed.sim_elapsed_ms = 0;
        mgr->wake_step = SX_WAKE_STEP_SIM_WAKE;
        break;

    case SX_WAKE_STEP_SIM_WAKE:
        mgr->elapsed.sim_elapsed_ms += delta_ms;
        if (sim76xx_is_ready(mgr->module.sim)) {
            log_info(TAG, "SIM ready after wake");
            mgr->wake_step = SX_WAKE_STEP_DONE;
        } else if (mgr->elapsed.sim_elapsed_ms >= 90000U) {
            log_warn(TAG, "SIM timeout — reset");
            mgr->elapsed.sim_elapsed_ms = 0;
            sim76xx_reset(mgr->module.sim);
            sim76xx_start(mgr->module.sim);
        }
        break;

    case SX_WAKE_STEP_DONE:
        break;

    default:
        break;
    }
}

uint8_t sx_sleep_manager_is_wake_done(sx_sleep_manager_t *mgr)
{
    return (mgr->wake_step == SX_WAKE_STEP_DONE);
}

uint8_t sx_sleep_manager_wake_tick(sx_sleep_manager_t *mgr, uint32_t delta_ms)
{
    uint32_t timeout_ms = (mgr->wake_timeout_ms > 0)
                          ? mgr->wake_timeout_ms
                          : SX_TIME_IN_WAKE;

    mgr->elapsed.wake_elapsed_ms += delta_ms;
    
    if (mgr->elapsed.wake_elapsed_ms >= timeout_ms) {  
        return 1;
    }
    return 0;
}

void sx_sleep_manager_reset_wake(sx_sleep_manager_t *mgr)
{
    mgr->wake_step                   = SX_WAKE_STEP_IDLE;
    mgr->published                   = 0;
    mgr->elapsed.wake_elapsed_ms     = 0;
    mgr->elapsed.gps_elapsed_ms      = 0;
    mgr->elapsed.sim_elapsed_ms      = 0;
}