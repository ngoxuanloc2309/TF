#include "sx_os.h"

#ifdef SX_FREERTOS_ENABLED
#include <assert.h>
/** Set this to 1 if you want the stack size passed to sys_thread_new() to be
 * interpreted as number of stack words (FreeRTOS-like).
 * Default is that they are interpreted as byte count (SX-like).
 */
#ifndef SX_FREERTOS_THREAD_STACKSIZE_IS_STACKWORDS
#define SX_FREERTOS_THREAD_STACKSIZE_IS_STACKWORDS  0
#endif

/** Set this to 1 to use a mutex for SYS_ARCH_PROTECT() critical regions.
 * Default is 0 and locks interrupts/scheduler for SYS_ARCH_PROTECT().
 */
#ifndef SX_FREERTOS_SYS_ARCH_PROTECT_USES_MUTEX
#define SX_FREERTOS_SYS_ARCH_PROTECT_USES_MUTEX     0
#endif

/** Set this to 1 to include a sanity check that SYS_ARCH_PROTECT() and
 * SYS_ARCH_UNPROTECT() are called matching.
 */
#ifndef SX_FREERTOS_SYS_ARCH_PROTECT_SANITY_CHECK
#define SX_FREERTOS_SYS_ARCH_PROTECT_SANITY_CHECK   0
#endif

/** Set this to 1 to let sys_mbox_free check that queues are empty when freed */
#ifndef SX_FREERTOS_CHECK_QUEUE_EMPTY_ON_FREE
#define SX_FREERTOS_CHECK_QUEUE_EMPTY_ON_FREE       0
#endif

/** Set this to 1 to enable core locking check functions in this port.
 * For this to work, you'll have to define SX_ASSERT_CORE_LOCKED()
 * and SX_MARK_TCPIP_THREAD() correctly in your SXopts.h! */
#ifndef SX_FREERTOS_CHECK_CORE_LOCKING
#define SX_FREERTOS_CHECK_CORE_LOCKING              0
#endif

/** Set this to 0 to implement sys_now() yourself, e.g. using a hw timer.
 * Default is 1, where FreeRTOS ticks are used to calculate back to ms.
 */
#ifndef SX_FREERTOS_SYS_NOW_FROM_FREERTOS
#define SX_FREERTOS_SYS_NOW_FROM_FREERTOS           1
#endif

#if !configSUPPORT_DYNAMIC_ALLOCATION
# error "SX FreeRTOS port requires configSUPPORT_DYNAMIC_ALLOCATION"
#endif
#if !INCLUDE_vTaskDelay
# error "SX FreeRTOS port requires INCLUDE_vTaskDelay"
#endif
#if !INCLUDE_vTaskSuspend
# error "SX FreeRTOS port requires INCLUDE_vTaskSuspend"
#endif
#if SX_FREERTOS_SYS_ARCH_PROTECT_USES_MUTEX || !SX_COMPAT_MUTEX
#if !configUSE_MUTEXES
# error "SX FreeRTOS port requires configUSE_MUTEXES"
#endif
#endif

/* Initialize this module (see description in sys.h) */
void sx_os_init(void)
{
  
}

#if configUSE_16_BIT_TICKS == 1
#error This port requires 32 bit ticks or timer overflow will fail
#endif

#if SX_FREERTOS_SYS_NOW_FROM_FREERTOS
uint32_t
sx_os_now(void)
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
#endif

uint32_t
sx_os_jiffies(void)
{
  return xTaskGetTickCount();
}

void
sx_os_msleep(uint32_t delay_ms)
{
  TickType_t delay_ticks = delay_ms / portTICK_RATE_MS;
  vTaskDelay(delay_ticks);
}

int
sx_os_mutex_new(os_mutex_t mutex)
{
  mutex = xSemaphoreCreateMutex();
  if(mutex == NULL) {
    return -1;
  }
  SYS_STATS_INC_USED(mutex);
  return 0;
}

void
sx_os_mutex_lock(os_mutex_t mutex)
{
    assert(mutex != NULL);
    xSemaphoreTake(mutex, portMAX_DELAY);
}

void
sx_os_mutex_unlock(os_mutex_t mutex)
{
  assert(mutex != NULL);
  xSemaphoreGive(mutex);
}

void
sx_os_mutex_free(os_mutex_t mutex)
{
  assert(mutex != NULL);
  vSemaphoreDelete(mutex);
  muxtex = NULL;
}

int sx_os_sem_new(os_sem_t sem, uint8_t count)
{
  sem = xSemaphoreCreateCounting(count, 0);
  if(sem == NULL) {
    return -1;
  }
  return 0;
}

int sx_os_sem_wait(os_sem_t sem,uint32_t timeout_ms)
{
  if(!timeout_ms) {
    timeout_ms = portMAX_DELAY;
  } else {
    timeout_ms /= portTICK_RATE_MS;
  }
  assert(sem != NULL);
  BaseType_t ret = xSemaphoreTake(sem, timeout_ms);
  return (ret == pdTRUE) ? 0 : -1;
}

void sx_os_sem_signal(os_sem_t sem)
{
  assert(sem != NULL);
  xSemaphoreGive(sem);
}

void sx_os_sem_free(os_sem_t sem)
{
  assert(sem != NULL);
  vSemaphoreDelete(sem);
  sem = NULL;
}

int sx_os_mbox_new(os_mbox_t mbox, int size)
{
  mbox = xQueueCreate(size, sizeof(void *));
  if(mbox == NULL) {
    return -1;
  }
  return 0;
}

void sx_os_mbox_post(os_mbox_t mbox, void *msg)
{
  assert(mbox != NULL);
  xQueueSendToBack(mbox, &msg, portMAX_DELAY);
}

int sx_os_mbox_trypost(os_mbox_t mbox, void *msg)
{
  assert(mbox != NULL);
  BaseType_t ret = xQueueSendToBack(mbox, &msg, 0);
  return (ret == pdTRUE) ? 0 : -1;
}

int sx_os_mbox_trypost_fromisr(os_mbox_t mbox, void *msg)
{
  assert(mbox != NULL);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t ret = xQueueSendToBackFromISR(mbox, &msg, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE) {
    if (xHigherPriorityTaskWoken == pdTRUE) {
      return -2;
    }
    return 0;
  } else {
    LWIP_ASSERT("mbox trypost failed", ret == errQUEUE_FULL);
    SYS_STATS_INC(mbox.err);
    return -1;
  }
}

int sx_os_mbox_fetch(os_mbox_t mbox, void **msg, uint32_t timeout_ms)
{
  if(!timeout_ms) {
    timeout_ms = portMAX_DELAY;
  } else {
    timeout_ms /= portTICK_RATE_MS;
  }
  assert(mbox != NULL);
  BaseType_t ret = xQueueReceive(mbox, msg, timeout_ms);
  return (ret == pdTRUE) ? 0 : -1;
}

int sx_os_mbox_tryfetch(os_mbox_t mbox, void **msg)
{
  assert(mbox != NULL);
  BaseType_t ret = xQueueReceive(mbox, msg, 0);
  return (ret == pdTRUE) ? 0 : -1;
}

void sx_os_mbox_free(os_mbox_t mbox)
{
  assert(mbox != NULL);
  vQueueDelete(mbox);
  mbox = NULL;
}

int sx_os_thread_new(const char *name, os_thread_fn thread_fn, void *arg, int stacksize, int prio,os_thread_t *thread)
{
  TaskHandle_t thread;
  BaseType_t ret;
  size_t rtos_stacksize;
#if SX_FREERTOS_THREAD_STACKSIZE_IS_STACKWORDS
  rtos_stacksize = stacksize;
#else
    rtos_stacksize = stacksize / sizeof(StackType_t);
#endif
    ret = xTaskCreate(thread_fn, name, rtos_stacksize, arg, prio,thread);
    if (ret != pdPASS) {
        return -1;
    }
    return 0;
}

void sx_os_thread_exit(os_thread_t thread)
{
  vTaskDelete(thread);
}

void sx_os_thread_yield(void)
{
  taskYIELD();
}

void sx_os_enter_critical(void){
  taskENTER_CRITICAL();
}
void sx_os_exit_critical(void){
  taskEXIT_CRITICAL();
}
os_timer_t sx_os_timer_create(const char *name, uint32_t period_ms, uint8_t auto_reload, void *const pvTimerID, os_timer_callback_fn callback){
  TimerHandle_t timer;
  timer = xTimerCreate(name, pdMS_TO_TICKS(period_ms), auto_reload, pvTimerID, callback);
  return timer;
}
uint32_t sx_os_timer_is_activate(os_timer_t timer){
  BaseType_t ret = xTimerIsTimerActive(timer);
  return (ret == pdPASS) ? 0 : -1;
}
uint32_t sx_os_timer_start(os_timer_t timer){
  BaseType_t ret = xTimerStart(timer, 0);
  return (ret == pdPASS) ? 0 : -1;
}
uint32_t sx_os_timer_stop(os_timer_t timer){
  BaseType_t ret = xTimerStop(timer, 0);
  return (ret == pdPASS) ? 0 : -1;
}
uint32_t sx_os_timer_change_period(os_timer_t timer, uint32_t period_ms){
  BaseType_t ret = xTimerChangePeriod(timer, pdMS_TO_TICKS(period_ms), 0);
  return (ret == pdPASS) ? 0 : -1;
}
uint32_t sx_os_timer_delete(os_timer_t timer){
  BaseType_t ret = xTimerDelete(timer, 0);
  return (ret == pdPASS) ? 0 : -1;
}
uint32_t sx_os_timer_reset(os_timer_t timer){
  BaseType_t ret = xTimerReset(timer, 0);
  return (ret == pdPASS) ? 0 : -1;
}
uint32_t sx_os_timer_start_from_isr(os_timer_t timer){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t ret = xTimerStartFromISR(timer, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE) {
    if (xHigherPriorityTaskWoken == pdTRUE) {
      taskYIELD();
    }
    return 0;
  } else {
    return -1;
  }
}
uint32_t sx_os_timer_stop_from_isr(os_timer_t timer){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t ret = xTimerStopFromISR(timer, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE) {
    if (xHigherPriorityTaskWoken == pdTRUE) {
      taskYIELD();
    }
    return 0;
  } else {
    return -1;
  }
}
uint32_t sx_os_timer_reset_from_isr(os_timer_t timer){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t ret = xTimerResetFromISR(timer, &xHigherPriorityTaskWoken);
  if (ret == pdTRUE) {
    if (xHigherPriorityTaskWoken == pdTRUE) {
      taskYIELD();
    }
    return 0;
  } else {
    return -1;
  }
}
uint32_t sx_os_timer_change_period_from_isr(os_timer_t timer, uint32_t period_ms){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t ret = xTimerChangePeriodFromISR(timer, pdMS_TO_TICKS(period_ms), &xHigherPriorityTaskWoken);
  if (ret == pdTRUE) {
    if (xHigherPriorityTaskWoken == pdTRUE) {
      taskYIELD();
    }
    return 0;
  } else {
    return -1;
  }
}
#endif /* SX_FREERTOS_ENABLED */