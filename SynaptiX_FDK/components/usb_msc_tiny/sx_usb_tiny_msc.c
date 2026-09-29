#include "sx_usb_tiny_msc.h"
#include "sx_config.h"
#include "logger.h"
#include "ff.h"
#include "diskio.h"
#include <string.h>
#include "stdio.h"

#if (SX_PLATFORM == SX_PLATFORM_STM32H5)
    #include "stm32h5xx_hal.h"
    #include "tusb.h"
#endif

#if CFG_TUD_MSC

static const char* TAG = "SX_MSC";
static sx_usb_tiny_msc_t *s_msc = NULL;

static uint8_t  s_write_tmp[SX_MSC_BLOCK_SIZE];
static uint32_t s_write_tmp_lba = 0xFFFFFFFF;

void sx_usb_tiny_msc_init(sx_usb_tiny_msc_t *msc)
{
    memset(msc, 0x00, sizeof(sx_usb_tiny_msc_t));
    msc->state         = SX_MSC_STATE_IDLE;
    msc->media_changed = false;
    msc->block_count   = 0;
    s_msc = msc;
    log_info(TAG, "MSC Initialized");
}

void sx_usb_tiny_msc_enter(sx_usb_tiny_msc_t *msc)
{
    if (msc->state == SX_MSC_STATE_ACTIVE || msc->state == SX_MSC_STATE_BUILDING) {
        log_warn(TAG, "Already in MSC mode");
        return;
    }
    if (msc->block_count == 0) {
        log_error(TAG, "No block count set — call sx_usb_tiny_msc_load() first");
        return;
    }
    msc->state         = SX_MSC_STATE_ACTIVE;
    msc->media_changed = true;
    log_info(TAG, "MSC active: %lu blocks", msc->block_count);
}

void sx_usb_tiny_msc_exit(sx_usb_tiny_msc_t *msc)
{
    if (msc->state == SX_MSC_STATE_IDLE) {
        log_warn(TAG, "MSC not active");
        return;
    }
    msc->state         = SX_MSC_STATE_IDLE;
    msc->media_changed = false;
    log_info(TAG, "MSC exited");
}

sx_msc_state_t sx_usb_tiny_msc_get_state(sx_usb_tiny_msc_t *msc)  { return msc->state; }
bool           sx_usb_tiny_msc_is_active(sx_usb_tiny_msc_t *msc)
{
    return (msc->state == SX_MSC_STATE_ACTIVE || msc->state == SX_MSC_STATE_BUILDING);
}

void sx_usb_tiny_msc_set_media_changed(sx_usb_tiny_msc_t *msc)
{
    msc->media_changed = true;
    log_info(TAG, "Media changed flagged");
}

void sx_usb_tiny_msc_load(sx_usb_tiny_msc_t *msc, uint32_t block_count)
{
    if (block_count == 0) {
        log_error(TAG, "Invalid block count");
        return;
    }
    msc->block_count = block_count;
    log_info(TAG, "Disk loaded: %lu blocks (%lu bytes)",
             block_count, block_count * SX_MSC_BLOCK_SIZE);
}

void sx_usb_tiny_msc_disconnect(void) { tud_disconnect(); }
void sx_usb_tiny_msc_connect(void)    { tud_connect();    }


uint32_t tud_msc_inquiry2_cb(uint8_t lun, scsi_inquiry_resp_t *resp, uint32_t bufsize)
{
    (void)lun; (void)bufsize;
    memcpy(resp->vendor_id,   "SynaptiX",         8);
    memcpy(resp->product_id,  "Tracking Storage", 16);
    memcpy(resp->product_rev, "1.0 ",              4);
    return sizeof(scsi_inquiry_resp_t);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    if (s_msc == NULL || !sx_usb_tiny_msc_is_active(s_msc))
        return tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);

    if (s_msc->state == SX_MSC_STATE_BUILDING)
        return tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x01);

    if (s_msc->media_changed) {
        s_msc->media_changed = false;
        log_info(TAG, "UNIT_ATTENTION: media changed");
        return tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
    }
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = (s_msc != NULL) ? s_msc->block_count : 0;
    *block_size  = SX_MSC_BLOCK_SIZE;
}

bool tud_msc_is_writable_cb(uint8_t lun)  { (void)lun; return true; }

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun; (void)power_condition;
    if (load_eject && !start && s_msc != NULL) {
        /* Host ejected — flush any pending write cache */
        disk_ioctl(0, CTRL_SYNC, NULL);
        s_msc->state = SX_MSC_STATE_EJECTED;
        log_info(TAG, "Host ejected disk — cache flushed");
    }
    return true;
}

/* READ: flash is memory-mapped, read directly via diskio */
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           void *buffer, uint32_t bufsize)
{
    (void)lun;
    if (s_msc == NULL || s_msc->state != SX_MSC_STATE_ACTIVE) return -1;
    if (lba >= s_msc->block_count)                             return -1;

    /* offset is always 0 in practice but handle it correctly */
    uint8_t  tmp[SX_MSC_BLOCK_SIZE];
    DRESULT  res = disk_read(0, tmp, (LBA_t)lba, 1);
    if (res != RES_OK) return -1;

    memcpy(buffer, tmp + offset, bufsize);
    //log_debug(TAG, "READ  lba=%lu off=%lu size=%lu", lba, offset, bufsize);
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                            uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    if (s_msc == NULL || s_msc->state != SX_MSC_STATE_ACTIVE) return -1;
    if (lba >= s_msc->block_count)                             return -1;

    if (lba != s_write_tmp_lba) {
        s_write_tmp_lba = lba;
        disk_read(0, s_write_tmp, (LBA_t)lba, 1);
    }

    memcpy(s_write_tmp + offset, buffer, bufsize);

    if (offset + bufsize >= SX_MSC_BLOCK_SIZE) {
        DRESULT res = disk_write(0, s_write_tmp, (LBA_t)lba, 1);
        s_write_tmp_lba = 0xFFFFFFFF; // reset
        if (res != RES_OK) return -1;
    }

    //log_debug(TAG, "WRITE lba=%lu off=%lu size=%lu", lba, offset, bufsize);
    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                         void *buffer, uint16_t bufsize)
{
    (void)lun; (void)scsi_cmd; (void)buffer; (void)bufsize;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

#endif /* CFG_TUD_MSC */