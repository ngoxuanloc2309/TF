#ifndef __SX_USB_TINY_H
#define __SX_USB_TINY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sx_config.h"
#include "cqueue.h"
#include "tusb_types.h"

#if (SX_USE_OS == 1)
    #include "sx_os.h"

#endif
    
/* ── Config ── */
typedef struct sx_usb_tiny_config {
    uint32_t rx_buf_size;
    uint32_t tx_buf_size;
} sx_usb_tiny_config_t;

/* ── Handle ── */
typedef struct sx_usb_tiny sx_usb_tiny_t;

struct sx_usb_tiny {
    sx_usb_tiny_config_t *config;

    uint8_t  *rxBuffer;
    CQueue_t  rxQueue;

    uint8_t  *txBuffer;
    CQueue_t  txQueue;

    bool      connected;

    #if (SX_USE_OS == 1)
    sx_mutex_t rxMutex;
    #endif
};

void sx_usb_tiny_init(sx_usb_tiny_t *_usb, sx_usb_tiny_config_t *_config);
void sx_usb_tiny_process(sx_usb_tiny_t *_usb);
void sx_usb_tiny_write(sx_usb_tiny_t *_usb, const uint8_t *_data, uint32_t _len);
bool sx_usb_tiny_connected(sx_usb_tiny_t *_usb);
int sx_usb_tiny_available(sx_usb_tiny_t *_usb);
int sx_usb_tiny_read(sx_usb_tiny_t *_usb, uint8_t *_data, uint32_t _len, uint32_t _timeoutMS);
void sx_usb_tiny_printf(sx_usb_tiny_t *_usb, const char *fmt, ...);

void tud_mount_cb(void);
void tud_umount_cb(void);
void tud_suspend_cb(bool remote_wakeup_en);
void tud_resume_cb(void);

#if (SX_USE_OS == 1)
#include "sx_os.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SX_USB_TINY_H__ */
