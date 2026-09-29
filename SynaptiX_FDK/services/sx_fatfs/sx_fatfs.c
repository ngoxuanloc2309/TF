#include "sx_fatfs.h"
#include "sx_diskio.h"
#include "logger.h"
#include <string.h>

static const char *TAG = "SX_FATFS";

static FATFS s_fatfs;
static bool s_mounted = false;

#define WORK_SIZE   2048

/*  API */
int sx_fatfs_mount(void)
{
    FRESULT res;
    static BYTE work[WORK_SIZE];

    DSTATUS st = disk_initialize(0);
    log_info(TAG, "disk_initialize status: %d", st);

    res = f_mount(&s_fatfs, "0:", 1);
    if (res == FR_OK) {
        s_mounted = true;
        log_info(TAG, "Flash FAT valid — skipping format");
        return 0;
    }

    log_info(TAG, "No valid FAT — formatting...");

    MKFS_PARM opt = {
        .fmt     = FM_FAT | FM_SFD,
        .n_fat   = 1,
        .align   = 1,
        .n_root  = 256,
        .au_size = 512
    };

    res = f_mkfs("0:", &opt, work, sizeof(work));
    if (res != FR_OK) {
        log_error(TAG, "f_mkfs failed: %d", res);
        return -1;
    }

    disk_ioctl(0, CTRL_SYNC, NULL);

    res = f_mount(&s_fatfs, "0:", 1);
    if (res != FR_OK) {
        log_error(TAG, "f_mount failed: %d", res);
        return -2;
    }

    s_mounted = true;
    log_info(TAG, "Flash FAT formatted and mounted");
    return 0;
}
void sx_fatfs_unmount(void)
{
    if (!s_mounted) return;
    f_mount(NULL, "0:", 0);
    s_mounted = false;
    log_info(TAG, "RAM disk unmounted");
}

int sx_fatfs_write_file(const char *path, const uint8_t *data, uint32_t len){
    if (!s_mounted || path == NULL || data == NULL || len == 0) 
        return -1;
    FIL f;
    UINT bw;
    FRESULT res;

    res = f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS);
    if(res != FR_OK){
        log_error(TAG, "f_open(%s) failed: %d", path, res);
        return -1;
    }

    res = f_write(&f, data, len, &bw);
    f_close(&f);

    if(res != FR_OK || bw != len){
        log_error(TAG, "f_write(%s) failed: %d", path, res);
        return -2;
    }

    log_info(TAG, "write %s: %lu bytes", path, len);
    return 0;
}

int sx_fatfs_read_file(const char *path, uint8_t *buf, uint32_t len, uint32_t *out_len){
    if (!s_mounted || path == NULL || buf == NULL || len == 0) return -1;

    FIL     f;
    UINT    br;
    FRESULT res;

    res = f_open(&f, path, FA_READ);
    if (res != FR_OK) {
        log_error(TAG, "f_open(%s) failed: %d", path, res);
        return -1;
    }

    res = f_read(&f, buf, len, &br);
    f_close(&f);

    if (res != FR_OK) {
        log_error(TAG, "f_read(%s) failed: %d", path, res);
        return -2;
    }

    if (out_len != NULL) *out_len = (uint32_t)br;
    log_info(TAG, "read %s: %lu bytes", path, (uint32_t)br);
    return 0;
}

uint8_t *sx_fatfs_get_disk_buffer(void){
    return sx_diskio_get_buffer();
}

uint32_t sx_fatfs_get_block_count(void){
    return sx_diskio_get_block_count();
}

int sx_fatfs_remount(void)
{
    f_mount(NULL, "0:", 0);
    s_mounted = false;

    disk_ioctl(0, CTRL_SYNC, NULL);
    log_info(TAG, "SYNC done — remounting...");

    FRESULT res = f_mount(&s_fatfs, "0:", 1);
    log_info(TAG, "f_mount result: %d", res);
    s_mounted = (res == FR_OK);
    return (res == FR_OK) ? 0 : -1;
}

void sx_fatfs_debug_list(void)
{
    DIR     dir;
    FILINFO fno;

    FRESULT res = f_opendir(&dir, "/");
    if (res != FR_OK) {
        log_error("FATFS_DBG", "f_opendir failed: %d", res);
        return;
    }

    log_info("FATFS_DBG", "=== Root directory ===");
    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;
        log_info("FATFS_DBG", "  [%s] %lu bytes", fno.fname, fno.fsize);
    }
    f_closedir(&dir);
    log_info("FATFS_DBG", "=== End ===");
}