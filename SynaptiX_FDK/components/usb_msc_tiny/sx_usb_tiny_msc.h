#ifndef __SX_USB_MSC_TINY_H
#define __SX_USB_MSC_TINY_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stdbool.h"
#include "sx_config.h"

#define SX_MSC_BLOCK_SIZE   512
#define SX_MSC_BLOCK_COUNT  128
#define SX_MSC_DISK_LABEL   "SynaptiX   "

typedef enum {
    SX_MSC_STATE_IDLE = 0,
    SX_MSC_STATE_BUILDING,
    SX_MSC_STATE_ACTIVE,
    SX_MSC_STATE_EJECTED,
} sx_msc_state_t;

typedef struct {
    uint32_t        block_count;
    sx_msc_state_t  state;
    bool            media_changed;
} sx_usb_tiny_msc_t;

void sx_usb_tiny_msc_init(sx_usb_tiny_msc_t *msc);
void sx_usb_tiny_msc_enter(sx_usb_tiny_msc_t *msc);
void sx_usb_tiny_msc_exit(sx_usb_tiny_msc_t *msc);

sx_msc_state_t sx_usb_tiny_msc_get_state(sx_usb_tiny_msc_t *msc);
bool           sx_usb_tiny_msc_is_active(sx_usb_tiny_msc_t *msc);
void           sx_usb_tiny_msc_set_media_changed(sx_usb_tiny_msc_t *msc);

void sx_usb_tiny_msc_load(sx_usb_tiny_msc_t *msc, uint32_t block_count);

void sx_usb_tiny_msc_connect(void);
void sx_usb_tiny_msc_disconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* EOF */