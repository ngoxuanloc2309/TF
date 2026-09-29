#ifndef SX_SLEEP_MANAGER_H
#define SX_SLEEP_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rtc.h"
#include "sx_sleep.h"
#include "sim76xx.h"
#include "gps.h"
#include "sx_W25Q128.h"

typedef enum {
    SX_WAKE_STEP_IDLE = 0,
    SX_WAKE_STEP_GPS_ON_FIRST,   
    SX_WAKE_STEP_GPS_WAIT,       
    SX_WAKE_STEP_UART_RESUME,    
    SX_WAKE_STEP_SIM_WAKE,
    SX_WAKE_STEP_DONE,
} sx_wake_step_t;

typedef struct {
    sim76xx_t     *sim;
    sx_gps_t      *gps;
    /*  add another */
    sx_W25Q128_t *ex_flash;
}module_t;

typedef struct {
    uint32_t wake_elapsed_ms;
    uint32_t gps_elapsed_ms;
    uint32_t sim_elapsed_ms; 
} sx_wake_elapsed_t;

typedef struct {
    sx_sleep_t    *sleep;
    
    module_t module;

    sx_wake_elapsed_t   elapsed;   

    sx_wake_step_t wake_step;
    uint8_t        published;          
    uint32_t       sleep_ms;        
    uint32_t       wake_timeout_ms;
} sx_sleep_manager_t;

void sx_sleep_manager_init (sx_sleep_manager_t *mgr,
                            sx_sleep_t         *sleep,
                            sim76xx_t          *sim,
                            sx_gps_t           *gps);

void sx_sleep_manager_enter(sx_sleep_manager_t *mgr);

void sx_sleep_manager_wake_process(sx_sleep_manager_t *mgr, uint32_t delta_ms);

uint8_t sx_sleep_manager_is_wake_done(sx_sleep_manager_t *mgr);

uint8_t sx_sleep_manager_wake_tick(sx_sleep_manager_t *mgr, uint32_t delta_ms);

void sx_sleep_manager_reset_wake(sx_sleep_manager_t *mgr);

static inline wake_reason_t sx_sleep_manager_get_wake_reason(sx_sleep_manager_t *mgr) {
    return sx_sleep_get_wake_reason(mgr->sleep);
}

#ifdef __cplusplus
}
#endif

#endif