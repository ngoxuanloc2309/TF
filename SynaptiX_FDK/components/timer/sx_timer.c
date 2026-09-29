#include "sx_timer.h"

void sx_timer_init(sx_timer_t *timer, sx_timer_ops_t *ops, uint32_t timeout_ms, sx_timer_callback_t callback, void *arg){
    timer->ops = ops;
    timer->timeout_ms = timeout_ms;
    timer->callback = callback;
    timer->arg = arg;
}