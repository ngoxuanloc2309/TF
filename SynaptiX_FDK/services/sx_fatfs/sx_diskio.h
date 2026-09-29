#ifndef __SX_DISKIO_H
#define __SX_DISKIO_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stdbool.h"
#include "ff.h"
#include "diskio.h"

#define SX_FLASHDISK_BLOCK_SIZE    512U
#define SX_FLASHDISK_BLOCK_COUNT   512U

#define SX_RAMDISK_BLOCK_SIZE   SX_FLASHDISK_BLOCK_SIZE
#define SX_RAMDISK_BLOCK_COUNT  SX_FLASHDISK_BLOCK_COUNT

/* API */
uint8_t  *sx_diskio_get_buffer(void);      
uint32_t  sx_diskio_get_block_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __SX_DISKIO_H */