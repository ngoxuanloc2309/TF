#ifndef SX_GPS_H
#define SX_GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sx_uart.h"
#include "sx_gpio.h"
#include "minmea.h"
#include <time.h>


#define GPS_BUFF_MAX_SIZE 256

typedef struct sx_gps sx_gps_t;

typedef void (*gps_callback)(sx_gps_t *gps,char *message,void *arg);

typedef enum{
    GPS_IDLE = 0,
    GPS_READ_MESSAGE,
    GPS_NEW_MESSAGE,
}sx_gps_state_t;

struct sx_gps
{
    /* data */
    sx_uart_t comm;
    sx_gpio_t pwr;
    // sx_gpio_t rst;
    uint32_t buff_id;
    float latitude;
    float longtitude;
    float altitude;
    float speed;
    int satellites;
    struct tm tim;
    enum minmea_sentence_id _sentence_id;
    uint8_t buff[GPS_BUFF_MAX_SIZE];
    sx_gps_state_t state;
    gps_callback callback;
    uint32_t time;
    void *arg;
    
};

void gps_init(sx_gps_t *_gps,sx_uart_config_t *_uart_cfg,sx_gpio_ops_t *_gpio_ops,void *_pwr_arg,void *_rst_arg);
void gps_set_callback_handle(sx_gps_t *_gps, gps_callback _cb,void *_arg);
void gps_reset(sx_gps_t *_gps);
void gps_power_on(sx_gps_t *_gps);
void gps_power_off(sx_gps_t *_gps);
void gps_process(sx_gps_t *_gps,uint32_t _timestamp);

#ifdef __cplusplus
}
#endif

#endif