#include "sx_fs.h"
#include "logger.h"

static const char *TAG = "SX_FS";
static sx_ext_flash_t *s_flash = NULL;
static file_system_config_t s_config;

static int lfs_flash_read(const struct lfs_config *c,
                           lfs_block_t block, lfs_off_t off,
                           void *buf, lfs_size_t size)
{
    // uint32_t addr = block * c->block_size + off;
    // log_info("SX_FS", "lfs read block=%lu off=%lu size=%lu addr=0x%06lX", (unsigned long)block, (unsigned long)off, (unsigned long)size, (unsigned long)addr);
    // int ret = sx_ext_flash_read(s_flash, addr, buf, size);
    // if (ret != 0) log_error("SX_FS", "read FAILED ret=%d", ret);
    // return ret;
    uint32_t addr = PART_GPS_LOG_OFFSET + block * c->block_size + off;
    return sx_ext_flash_read(s_flash, addr, buf, size);
}

static int lfs_flash_write(const struct lfs_config *c,
                           lfs_block_t block, lfs_off_t off,
                           const void *buf, lfs_size_t size)
{
    // uint32_t addr = block *c->block_size + off;
    // int ret = sx_ext_flash_write(s_flash, addr, buf, size);
    uint32_t addr = PART_GPS_LOG_OFFSET + block * c->block_size + off;
    int ret = sx_ext_flash_write(s_flash, addr, buf, size);
    if (ret != 0) {
        log_error("SX_FS", "write FAILED ret=%d", ret);
        return ret;
    }

    static uint8_t verify_buf[16];
    sx_ext_flash_read(s_flash, addr, verify_buf, size);
    if (memcmp(buf, verify_buf, size) != 0) {
        log_error("SX_FS", "VERIFY FAILED at addr=0x%06lX", (unsigned long)addr);
        log_error("SX_FS", "wrote: %02X %02X %02X %02X ...", ((uint8_t*)buf)[0], ((uint8_t*)buf)[1], ((uint8_t*)buf)[2], ((uint8_t*)buf)[3]);
        log_error("SX_FS", "read : %02X %02X %02X %02X ...", verify_buf[0], verify_buf[1], verify_buf[2], verify_buf[3]);
    } else {
        log_info("SX_FS", "verify OK at addr=0x%06lX", (unsigned long)addr);
    }
    return ret;
}

static int lfs_flash_erase(const struct lfs_config *c, lfs_block_t block){
    // uint32_t addr = block * c->block_size;
    uint32_t addr = PART_GPS_LOG_OFFSET + block * c->block_size;
    return sx_ext_flash_erase_sector(s_flash, addr);
}


int sx_fs_init(sx_ext_flash_t *flash){
    s_flash = flash;
    s_config.block_size  = flash->info.sector_size;
    
    s_config.block_count = PART_GPS_LOG_SIZE / flash->info.sector_size;
    
    s_config.file_function.read  = lfs_flash_read;
    s_config.file_function.write = lfs_flash_write;
    s_config.file_function.erase = lfs_flash_erase;
    return file_system_init(&s_config);
}