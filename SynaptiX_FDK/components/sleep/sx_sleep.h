#ifndef SX_SLEEP_H
#define SX_SLEEP_H

#ifdef __cplusplus
extern "C" {
#endif
 
#include <stdint.h>

typedef enum {
    WAKE_REASON_UNKNOWN = 0,
    WAKE_REASON_RTC,
    WAKE_REASON_EXTI,
} wake_reason_t;

typedef struct sx_sleep sx_sleep_t;

typedef struct sx_sleep_ops {
    void (*enter_stop)  (sx_sleep_t *mgr);
    void (*set_rtc_wake)(sx_sleep_t *mgr, uint32_t period_sec);
    void (*cancel_rtc)  (sx_sleep_t *mgr);
} sx_sleep_ops_t;

struct sx_sleep {
    sx_sleep_ops_t *ops;
    void                   *pDriver;
    wake_reason_t           wake_reason;
};

static inline void sx_sleep_init(sx_sleep_t *mgr,
                                          sx_sleep_ops_t *ops,
                                          void *pDriver)
{
    mgr->ops         = ops;
    mgr->pDriver     = pDriver;
    mgr->wake_reason = WAKE_REASON_UNKNOWN;
}

static inline void sx_sleep_enter_stop(sx_sleep_t *mgr)
{
    if (mgr->ops && mgr->ops->enter_stop)
        mgr->ops->enter_stop(mgr);
}

static inline void sx_sleep_set_rtc_wake(sx_sleep_t *mgr,
                                                   uint32_t period_sec)
{
    if (mgr->ops && mgr->ops->set_rtc_wake)
        mgr->ops->set_rtc_wake(mgr, period_sec);
}

static inline void sx_sleep_cancel_rtc(sx_sleep_t *mgr)
{
    if (mgr->ops && mgr->ops->cancel_rtc)
        mgr->ops->cancel_rtc(mgr);
}

static inline wake_reason_t sx_sleep_get_wake_reason(sx_sleep_t *mgr)
{
    return mgr->wake_reason;
}

/* Global sleep instance accessor for interrupt handlers to set wake reason */
sx_sleep_t* sx_sleep_get_instance(void);
void sx_sleep_set_exti_wake(void);

extern sx_sleep_ops_t sx_sleep_ops;

#ifdef __cplusplus
}
#endif

#endif