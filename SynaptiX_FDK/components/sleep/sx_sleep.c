#include "sx_sleep.h"
#include "sx_config.h"
#include "rtc.h"
#include "usart.h"
#include "logger.h"

#if SX_PLATFORM == SX_PLATFORM_STM32H5
    #include "stm32h5xx_hal.h"
    #include "stm32h5xx_hal_pwr_ex.h"
    #include "stm32h5xx_hal_rtc_ex.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32H7
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_hal_pwr_ex.h"
    #include "stm32h7xx_hal_rtc_ex.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F4
    #include "stm32f4xx_hal.h"
    #include "stm32f4xx_hal_pwr_ex.h"
    #include "stm32f4xx_hal_rtc_ex.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F1
    #include "stm32f1xx_hal.h"
    #include "stm32f1xx_hal_pwr.h"
    #include "stm32f1xx_hal_rtc_ex.h"
#endif

#define SX_SUSPEND_TICS()          HAL_SuspendTick()
#define SX_RESUME_TICS()           HAL_ResumeTick()

static const char *TAG = "SX_SLEEP";
static sx_sleep_t *s_instance = NULL;

typedef void (*pwr_enter_stop_fn_t) (uint32_t Regulator, uint8_t STOPEntry);
typedef HAL_StatusTypeDef (*rtc_set_wakeup_fn_t)(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock, uint32_t WakeUpAutoClr);
typedef HAL_StatusTypeDef (*rtc_cancel_wakeup_fn_t)(RTC_HandleTypeDef *hrtc);

static pwr_enter_stop_fn_t    s_enter_stop    = HAL_PWR_EnterSTOPMode;
static rtc_set_wakeup_fn_t    s_set_wakeup    = HAL_RTCEx_SetWakeUpTimer_IT;
static rtc_cancel_wakeup_fn_t s_cancel_wakeup = HAL_RTCEx_DeactivateWakeUpTimer;

static void _enter_stop  (sx_sleep_t *mgr);
static void _set_rtc_wake(sx_sleep_t *mgr, uint32_t period_sec);
static void _cancel_rtc  (sx_sleep_t *mgr);

sx_sleep_ops_t sx_sleep_ops = {
    .enter_stop   = _enter_stop,
    .set_rtc_wake = _set_rtc_wake,
    .cancel_rtc   = _cancel_rtc,
};

// static void _enter_stop(sx_sleep_t *mgr){
//     s_instance = mgr;
//     SX_SUSPEND_TICS();
//     s_enter_stop(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//     extern void SystemClock_Config(void);
//     SystemClock_Config();
//     SX_RESUME_TICS();
// }

static void _enter_stop(sx_sleep_t *mgr)
{
    s_instance = mgr;
    if (s_instance) {
        s_instance->wake_reason = WAKE_REASON_UNKNOWN;
    }

    HAL_UART_Abort(&huart1);   // LTE
    HAL_UART_Abort(&huart2);   // GPS
    __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);

    HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);
    HAL_NVIC_ClearPendingIRQ(USB_DRD_FS_IRQn);

    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    __DSB();
    __ISB();

    SX_SUSPEND_TICS();
    s_enter_stop(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* ── Sau wake: restore ── */
    extern void SystemClock_Config(void);
    SystemClock_Config();
    SX_RESUME_TICS();

    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
}
static void _set_rtc_wake(sx_sleep_t *mgr, uint32_t period_sec){
    // (void)mgr;
    // s_cancel_wakeup(&hrtc);
    // s_set_wakeup(&hrtc, period_sec - 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
    (void)mgr;
    HAL_StatusTypeDef ret1, ret2;
    
    ret1 = HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    ret2 = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, period_sec - 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
    
    log_info("RTC", "DeactivateWakeUpTimer ret=%d", ret1);  
    log_info("RTC", "SetWakeUpTimer ret=%d counter=%lu",    
             ret2, period_sec - 1);
}

static void _cancel_rtc(sx_sleep_t *mgr){
    (void)mgr;
    s_cancel_wakeup(&hrtc);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_cb){
    (void)hrtc_cb;
    log_debug(TAG, "RTC WakeUp callback triggered!");
    if (s_instance) {
        s_instance->wake_reason = WAKE_REASON_RTC;
        log_info(TAG, "[RTC CB] Set wake_reason = WAKE_REASON_RTC");
    } else {
        log_error(TAG, "[RTC CB] ERROR: s_instance is NULL!");
    }
}


sx_sleep_t* sx_sleep_get_instance(void)
{
    return s_instance;
}


void sx_sleep_set_exti_wake(void)
{
    log_debug(TAG, "sx_sleep_set_exti_wake() called!");
    if (s_instance) {
        s_instance->wake_reason = WAKE_REASON_EXTI;
        log_info(TAG, "[EXTI] Set wake_reason = WAKE_REASON_EXTI");
    } else {
        log_error(TAG, "[EXTI] ERROR: s_instance is NULL!");
    }
}