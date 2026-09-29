#ifndef __SX_FATFS_H
#define __SX_FATFS_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stdbool.h"
#include "ff.h"

/*  API */
int sx_fatfs_mount(void);
void sx_fatfs_unmount(void);

int sx_fatfs_write_file(const char *path, const uint8_t *data, uint32_t len);
int sx_fatfs_read_file(const char *path, uint8_t *buf, uint32_t len, uint32_t *out_len);

int sx_fatfs_remount(void);

uint8_t  *sx_fatfs_get_disk_buffer(void);
uint32_t sx_fatfs_get_block_count(void);

#ifdef __cplusplus
}
#endif

#endif