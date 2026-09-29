#ifndef SX_USER_CDC_H
#define SX_USER_CDC_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sx_usb_tiny_cdc.h"
 
/*  Handle  */
typedef struct {
    sx_usb_tiny_t *usb;
} sx_user_cdc_t;
 
/*  API  */
void sx_user_cdc_init     (sx_user_cdc_t *h, sx_usb_tiny_t *usb);
void sx_user_cdc_process  (sx_user_cdc_t *h);
 
bool sx_user_cdc_connected(sx_user_cdc_t *h);
int  sx_user_cdc_available(sx_user_cdc_t *h);
 
int  sx_user_cdc_read     (sx_user_cdc_t *h, uint8_t *buf, uint32_t len, uint32_t timeout_ms);
void sx_user_cdc_write    (sx_user_cdc_t *h, const uint8_t *buf, uint32_t len);
void sx_user_cdc_printf   (sx_user_cdc_t *h, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif