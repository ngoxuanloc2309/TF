#ifndef SX_OS_H
#define SX_OS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "sx_config.h"

// #if (SX_USE_FREERTOS == 1)

#include <stdint.h>

typedef void* os_mbox_t ;
typedef void* os_mutex_t ;
typedef void* os_sem_t ;
typedef void* os_thread_t;
typedef void* os_timer_t;

typedef void (*os_thread_fn)(void *arg);
typedef void (*os_timer_callback_fn)(os_timer_t _timer_handle);
void sx_os_init(void);
uint32_t sx_os_now(void);
uint32_t sx_os_jiffies(void);
void sx_os_msleep(uint32_t delay_ms);
int sx_os_mutex_new(os_mutex_t mutex);
void sx_os_mutex_lock(os_mutex_t mutex);
void sx_os_mutex_unlock(os_mutex_t mutex);
void sx_os_mutex_free(os_mutex_t mutex);
int sx_os_sem_new(os_sem_t sem, uint8_t count);
void sx_os_sem_wait(os_sem_t sem,uint32_t timeout_ms);
void sx_os_sem_signal(os_sem_t sem);
void sx_os_sem_free(os_sem_t sem);
int sx_os_mbox_new(os_mbox_t mbox, int size);
int sx_os_mbox_trypost(os_mbox_t mbox, void *msg);
int sx_os_mbox_trypost_fromisr(os_mbox_t mbox, void *msg);
int sx_os_mbox_fetch(os_mbox_t mbox, void **msg, uint32_t timeout_ms);
int sx_os_mbox_tryfetch(os_mbox_t mbox, void **msg);
void sx_os_mbox_free(os_mbox_t mbox);
int sx_os_thread_new(const char *name, os_thread_fn thread_fn, void *arg, int stacksize, int prio,os_thread_t *thread);
void sx_os_thread_exit(os_thread_t thread);
void sx_os_thread_yield(void);
void sx_os_enter_critical(void);
void sx_os_exit_critical(void);
os_timer_t sx_os_timer_create(const char *name, uint32_t period_ms, uint8_t auto_reload, void *const pvTimerID, os_timer_callback_fn callback);
uint32_t sx_os_timer_is_activate(os_timer_t timer);
uint32_t sx_os_timer_start(os_timer_t timer);
uint32_t sx_os_timer_stop(os_timer_t timer);
uint32_t sx_os_timer_change_period(os_timer_t timer, uint32_t period_ms);
uint32_t sx_os_timer_delete(os_timer_t timer);
uint32_t sx_os_timer_reset(os_timer_t timer);
uint32_t sx_os_timer_start_from_isr(os_timer_t timer);
uint32_t sx_os_timer_stop_from_isr(os_timer_t timer);
uint32_t sx_os_timer_reset_from_isr(os_timer_t timer);
uint32_t sx_os_timer_change_period_from_isr(os_timer_t timer, uint32_t period_ms);

// #endif /* SX_USE_FREERTOS == 1 */
#ifdef __cplusplus
}
#endif

#endif // SX_OS_H