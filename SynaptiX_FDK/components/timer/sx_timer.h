#ifndef SX_TIMER_H
#define SX_TIMER_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct sx_timer sx_timer_t;

typedef void (*sx_timer_callback_t)(void *arg);

typedef struct sx_timer_ops{
    int (*start)(sx_timer_t *timer);
    int (*stop)(sx_timer_t *timer);
    int (*set_period)(sx_timer_t *timer, uint32_t timeout_ms);
} sx_timer_ops_t;

struct sx_timer{
    uint32_t timeout_ms;
    sx_timer_callback_t callback;
    void *arg;
    sx_timer_ops_t *ops;
};

void sx_timer_init(sx_timer_t *_timer, sx_timer_ops_t *ops, uint32_t _timeout_ms, sx_timer_callback_t _callback, void *_arg);
static inline int sx_timer_start(sx_timer_t *_timer){
    if(_timer->ops && _timer->ops->start){
        return _timer->ops->start(_timer);
    }
    return -1;
}
static inline int sx_timer_stop(sx_timer_t *_timer){
    if(_timer->ops && _timer->ops->stop){
        return _timer->ops->stop(_timer);
    }
    return -1;
}
static inline int sx_timer_set_period(sx_timer_t *_timer, uint32_t _timeout_ms){
    if(_timer->ops && _timer->ops->set_period){
        return _timer->ops->set_period(_timer, _timeout_ms);
    }
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif // SX_TIMER_H