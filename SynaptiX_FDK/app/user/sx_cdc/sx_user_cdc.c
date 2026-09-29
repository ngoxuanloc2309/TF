#include "sx_user_cdc.h"
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void sx_user_cdc_init(sx_user_cdc_t *h, sx_usb_tiny_t *usb)
{
    h->usb = usb;
}

void sx_user_cdc_process(sx_user_cdc_t *h)
{
    sx_usb_tiny_process(h->usb);
}

bool sx_user_cdc_connected(sx_user_cdc_t *h)
{
    return sx_usb_tiny_connected(h->usb);
}

int sx_user_cdc_available(sx_user_cdc_t *h)
{
    return sx_usb_tiny_available(h->usb);
}

int sx_user_cdc_read(sx_user_cdc_t *h, uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    return sx_usb_tiny_read(h->usb, buf, len, timeout_ms);
}

void sx_user_cdc_write(sx_user_cdc_t *h, const uint8_t *buf, uint32_t len)
{
    sx_usb_tiny_write(h->usb, buf, len);
}

void sx_user_cdc_printf(sx_user_cdc_t *h, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sx_user_cdc_write(h, (uint8_t *)buf, strlen(buf));
}