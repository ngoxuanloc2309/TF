#include "sx_user_msc.h"
#include "sx_usb_tiny_msc.h"
#include "sx_fatfs.h"
#include "sx_delay.h"
#include "logger.h"
#include <string.h>

static const char *TAG = "SX_USER_MSC";

static sx_usb_tiny_msc_t s_msc;
static bool s_initialized = false;

/* ── Internal ───────────────────────────────────── */

static sx_user_msc_err_t _remount(void)
{
    sx_usb_tiny_msc_disconnect();
    sx_delay_ms(500);
    sx_usb_tiny_msc_connect();
    s_msc.media_changed = true;
    log_info(TAG, "Remounted");
    return SX_USER_MSC_OK;
}

/* ── API ────────────────────────────────────────── */

sx_user_msc_err_t sx_user_msc_init(void)
{
    sx_usb_tiny_msc_init(&s_msc);
    s_initialized = true;
    log_info(TAG, "User MSC initialized");
    return SX_USER_MSC_OK;
}

sx_user_msc_err_t sx_user_msc_create_disk(const char *label)
{
    if (!s_initialized) return SX_USER_MSC_ERR_INIT;
    if (label == NULL)  return SX_USER_MSC_ERR_PARAM;

    /* Format flash FAT — mỗi lần cắm USB tạo mới hoàn toàn */
    int ret = sx_fatfs_mount();
    if (ret != 0) {
        log_error(TAG, "fatfs mount failed: %d", ret);
        return SX_USER_MSC_ERR_DISK;
    }

    FRESULT res = f_setlabel(label);
    if (res != FR_OK) {
        log_warn(TAG, "f_setlabel failed: %d (non-critical)", res);
    }

    /* Flash backend: chỉ cần block_count, không cần pointer buffer */
    sx_usb_tiny_msc_load(&s_msc, sx_fatfs_get_block_count());
    sx_usb_tiny_msc_enter(&s_msc);

    log_info(TAG, "Disk created: label=\"%s\" blocks=%lu",
             label, sx_fatfs_get_block_count());
    return SX_USER_MSC_OK;
}

sx_user_msc_err_t sx_user_msc_create_file(const char *filename)
{
    if (!s_initialized)   return SX_USER_MSC_ERR_INIT;
    if (filename == NULL) return SX_USER_MSC_ERR_PARAM;

    FIL     f;
    FRESULT res = f_open(&f, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        log_error(TAG, "create_file(%s) failed: %d", filename, res);
        return SX_USER_MSC_ERR_FILE;
    }
    f_close(&f);
    log_info(TAG, "File created: %s", filename);
    return SX_USER_MSC_OK;
}

sx_user_msc_err_t sx_user_msc_write(const char *filename, const uint8_t *data, uint32_t len)
{
    if (!s_initialized)                   return SX_USER_MSC_ERR_INIT;
    if (filename == NULL || data == NULL) return SX_USER_MSC_ERR_PARAM;
    if (len == 0)                         return SX_USER_MSC_ERR_PARAM;

    int ret = sx_fatfs_write_file(filename, data, len);
    if (ret != 0) {
        log_error(TAG, "write(%s) failed: %d", filename, ret);
        return SX_USER_MSC_ERR_IO;
    }
    log_info(TAG, "write(%s): %lu bytes", filename, len);
    return SX_USER_MSC_OK;
}

sx_user_msc_err_t sx_user_msc_append(const char *filename, const uint8_t *data, uint32_t len)
{
    if (!s_initialized)                   return SX_USER_MSC_ERR_INIT;
    if (filename == NULL || data == NULL) return SX_USER_MSC_ERR_PARAM;
    if (len == 0)                         return SX_USER_MSC_ERR_PARAM;

    FIL     f;
    UINT    bw;
    FRESULT res;

    res = f_open(&f, filename, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) {
        log_error(TAG, "append open(%s) failed: %d", filename, res);
        return SX_USER_MSC_ERR_FILE;
    }

    res = f_write(&f, data, len, &bw);
    f_sync(&f);
    f_close(&f);

    if (res != FR_OK || bw != len) {
        log_error(TAG, "append write(%s) failed: %d", filename, res);
        return SX_USER_MSC_ERR_IO;
    }
    log_info(TAG, "append(%s): %lu bytes", filename, len);
    return SX_USER_MSC_OK;
}

sx_user_msc_err_t sx_user_msc_read(const char *filename, uint8_t *buf,
                                    uint32_t buf_len, uint32_t *out_len)
{
    if (!s_initialized)                  return SX_USER_MSC_ERR_INIT;
    if (filename == NULL || buf == NULL) return SX_USER_MSC_ERR_PARAM;
    if (buf_len == 0)                    return SX_USER_MSC_ERR_PARAM;

    int ret = sx_fatfs_read_file(filename, buf, buf_len, out_len);
    if (ret != 0) {
        log_error(TAG, "read(%s) failed: %d", filename, ret);
        return SX_USER_MSC_ERR_IO;
    }
    log_info(TAG, "read(%s): %lu bytes", filename, out_len ? *out_len : 0);
    return SX_USER_MSC_OK;
}

sx_user_msc_err_t sx_user_msc_update_file(const char *filename,
                                           const uint8_t *data, uint32_t len)
{
    if (!s_initialized)                   return SX_USER_MSC_ERR_INIT;
    if (filename == NULL || data == NULL) return SX_USER_MSC_ERR_PARAM;
    if (len == 0)                         return SX_USER_MSC_ERR_PARAM;

    sx_user_msc_err_t ret = sx_user_msc_append(filename, data, len);
    if (ret != SX_USER_MSC_OK) return ret;

    return _remount();
}

sx_user_msc_err_t sx_user_msc_remount_disk(void)
{
    if (!s_initialized) return SX_USER_MSC_ERR_INIT;

    int ret = sx_fatfs_remount();
    if (ret != 0) return SX_USER_MSC_ERR_DISK;

    sx_usb_tiny_msc_load(&s_msc, sx_fatfs_get_block_count());
    sx_usb_tiny_msc_enter(&s_msc);
    log_info(TAG, "MSC disk remounted");
    return SX_USER_MSC_OK;
}