#ifndef SX_USER_MSC_H
#define SX_USER_MSC_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SX_USER_MSC_OK          =  0,
    SX_USER_MSC_ERR_INIT    = -1,
    SX_USER_MSC_ERR_DISK    = -2,
    SX_USER_MSC_ERR_FILE    = -3,
    SX_USER_MSC_ERR_IO      = -4,
    SX_USER_MSC_ERR_PARAM   = -5,
} sx_user_msc_err_t;

/* ── API ── */
sx_user_msc_err_t sx_user_msc_init(void);

sx_user_msc_err_t sx_user_msc_create_disk(const char *label);

sx_user_msc_err_t sx_user_msc_create_file(const char *filename);

sx_user_msc_err_t sx_user_msc_write(const char *filename, const uint8_t *data, uint32_t len);

sx_user_msc_err_t sx_user_msc_append(const char *filename, const uint8_t *data, uint32_t len);

sx_user_msc_err_t sx_user_msc_read(const char *filename, uint8_t *buf, uint32_t buf_len, uint32_t *out_len);

sx_user_msc_err_t sx_user_msc_update_file(const char *filename, const uint8_t *data, uint32_t len);

sx_user_msc_err_t sx_user_msc_remount_disk(void);

#ifdef __cplusplus
}
#endif

#endif