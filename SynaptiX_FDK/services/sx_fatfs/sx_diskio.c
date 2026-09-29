#include "sx_diskio.h"
#include "sx_flash.h"
#include <string.h>
#include "logger.h"
#include "sx_delay.h"

#define FLASH_STORAGE_BASE      0x081C0000UL
#define FLASH_SECTOR_SIZE_BYTES 0x2000UL        /* 8 KB  */
#define FAT_BLOCKS_PER_SECTOR   (FLASH_SECTOR_SIZE_BYTES / SX_FLASHDISK_BLOCK_SIZE)  /* 16 */

static uint8_t  s_cache_buf[FLASH_SECTOR_SIZE_BYTES];
static int32_t  s_cache_sector = -1;    
static bool     s_cache_dirty  = false;
static bool     s_initialized  = false;

static uint32_t s_last_write_tick = 0;
static bool     s_pending_flush   = false;

static inline uint32_t _lba_to_addr(LBA_t lba)
{
    return FLASH_STORAGE_BASE + (uint32_t)lba * SX_FLASHDISK_BLOCK_SIZE;
}

static inline int32_t _lba_to_flash_sector(LBA_t lba)
{
    return (int32_t)(lba / FAT_BLOCKS_PER_SECTOR);
}

/** Load a flash sector into the RAM cache */
static void _cache_load(int32_t flash_sector)
{
    uint32_t addr = FLASH_STORAGE_BASE + (uint32_t)flash_sector * FLASH_SECTOR_SIZE_BYTES;
    sx_flash_read(addr, s_cache_buf, FLASH_SECTOR_SIZE_BYTES);
    s_cache_sector = flash_sector;
    s_cache_dirty  = false;
}

static void _cache_flush(void)
{
    if (!s_cache_dirty || s_cache_sector < 0) return;

    uint32_t addr = FLASH_STORAGE_BASE + (uint32_t)s_cache_sector * FLASH_SECTOR_SIZE_BYTES;

    log_info("DISKIO", "flush sector=%ld addr=0x%08lX", s_cache_sector, addr);
    log_info("DISKIO", ">>> FLASH WRITE sector=%ld addr=0x%08lX", 
             s_cache_sector, addr);

    sx_flash_unlock();
    log_info("DISKIO", "erase start");
    sx_flash_erase(addr, FLASH_SECTOR_SIZE_BYTES);
    log_info("DISKIO", "erase done, write start");
    sx_flash_write(addr, s_cache_buf, FLASH_SECTOR_SIZE_BYTES);
    log_info("DISKIO", "write done");
    sx_flash_lock();

    s_cache_dirty = false;
}

uint32_t sx_diskio_get_block_count(void)
{
    return SX_FLASHDISK_BLOCK_COUNT;
}

uint8_t *sx_diskio_get_buffer(void)
{
    return NULL;
}


DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    s_cache_sector = -1;
    s_cache_dirty  = false;
    s_initialized  = true;
    return 0;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return s_initialized ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0)                                   return RES_PARERR;
    if (!s_initialized)                              return RES_NOTRDY;
    if (sector + count > SX_FLASHDISK_BLOCK_COUNT)   return RES_PARERR;

    for (UINT i = 0; i < count; i++) {
        LBA_t    lba        = sector + i;
        int32_t  fsect      = _lba_to_flash_sector(lba);
        uint32_t buf_offset = (uint32_t)(lba % FAT_BLOCKS_PER_SECTOR) * SX_FLASHDISK_BLOCK_SIZE;

        if (s_cache_sector == fsect) {
            /* Cache hit */
            memcpy(buff + i * SX_FLASHDISK_BLOCK_SIZE,
                   s_cache_buf + buf_offset,
                   SX_FLASHDISK_BLOCK_SIZE);
        } else {
            sx_flash_read(_lba_to_addr(lba),
                          buff + i * SX_FLASHDISK_BLOCK_SIZE,
                          SX_FLASHDISK_BLOCK_SIZE);
        }
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0)                                   return RES_PARERR;
    if (!s_initialized)                              return RES_NOTRDY;
    if (sector + count > SX_FLASHDISK_BLOCK_COUNT)   return RES_PARERR;

    for (UINT i = 0; i < count; i++) {
        LBA_t    lba        = sector + i;
        int32_t  fsect      = _lba_to_flash_sector(lba);
        uint32_t buf_offset = (uint32_t)(lba % FAT_BLOCKS_PER_SECTOR) * SX_FLASHDISK_BLOCK_SIZE;

        if (s_cache_sector != fsect) {
            _cache_flush();
            _cache_load(fsect);
        }

        memcpy(s_cache_buf + buf_offset,
               buff + i * SX_FLASHDISK_BLOCK_SIZE,
               SX_FLASHDISK_BLOCK_SIZE);
        s_cache_dirty = true;
    }
    s_pending_flush   = true;        
    s_last_write_tick = HAL_GetTick();
    return RES_OK;
}

void sx_diskio_process(void)
{
    if (!s_pending_flush) return;
    if ((HAL_GetTick() - s_last_write_tick) < 500) return; 

    _cache_flush();
    s_pending_flush = false;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0)      return RES_PARERR;
    if (!s_initialized) return RES_NOTRDY;

    switch (cmd) {
        case CTRL_SYNC:
            _cache_flush();
            return RES_OK;

        case GET_SECTOR_COUNT:
            *(LBA_t *)buff = SX_FLASHDISK_BLOCK_COUNT;
            return RES_OK;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = SX_FLASHDISK_BLOCK_SIZE;
            return RES_OK;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = FAT_BLOCKS_PER_SECTOR;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}