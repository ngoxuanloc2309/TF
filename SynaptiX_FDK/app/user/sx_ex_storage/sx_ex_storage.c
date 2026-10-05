#include "sx_ex_storage.h"
#include "sx_spi.h"
#include "sx_gpio.h"
#include "sx_W25Q128.h"
#include "sx_external_flash.h"
#include "sx_fs.h"
#include "file_io.h"
#include "logger.h"

static const char *TAG = "SX_STORAGE";

static sx_W25Q128_t     s_w25q128;
static sx_ext_flash_t   s_flash;
static bool             s_initialized = false;
static bool             s_powered     = false;   /* flash rail + SPI1 are up */
static sx_storage_cfg_t *s_cfg        = NULL;

sx_storage_err_t sx_storage_init(sx_storage_cfg_t *cfg)
{
    if (cfg == NULL) return SX_STORAGE_ERR_INVALID;

    sx_gpio_init(&cfg->s_cs, &sx_gpio_ops, &cfg->cs_pin);
    sx_gpio_write(&cfg->s_cs, SX_GPIO_HIGH);
    sx_spi_init(&cfg->s_spi, &sx_spi_ops, cfg->hspi, &cfg->s_cs); 

    s_cfg = cfg;
    if (!sx_W25Q128_init(&s_w25q128, &cfg->s_spi, &sx_gpio_ops, &cfg->pwr_pin)) {
        log_error(TAG, "W25Q128 init failed");
        return SX_STORAGE_ERR_IO;
    }
    s_powered = true;

    /* External flash interface */
    sx_ext_flash_init(&s_flash, &sx_W25Q128_ops, &sx_W25Q128_info);

    /* Filesystem */
    if (sx_fs_init(&s_flash) != 0) {
        log_error(TAG, "Filesystem init failed");
        return SX_STORAGE_ERR_IO;
    }

    s_initialized = true;
    log_info(TAG, "Storage init OK");
    return SX_STORAGE_OK;
}

/* Every file operation goes through here. If the flash was powered down for sleep, bring it back
 * first, so callers (write_gps_log, read_last_gps, ...) never have to know about the rail. */
static sx_storage_err_t _ensure_power(void)
{
    if (s_powered) return SX_STORAGE_OK;
    return sx_storage_wake();
}

static sx_storage_err_t _check_init(void)
{
    if (!s_initialized) {
        log_error(TAG, "Not initialized");
        return SX_STORAGE_ERR_NOT_INIT;
    }
    if (_ensure_power() != SX_STORAGE_OK) return SX_STORAGE_ERR_IO;
    return SX_STORAGE_OK;
}

sx_storage_err_t sx_storage_write(const char *path, const void *data, uint32_t len)
{
    if (_check_init() != SX_STORAGE_OK)     return SX_STORAGE_ERR_NOT_INIT;
    if (path == NULL || data == NULL || len == 0) return SX_STORAGE_ERR_INVALID;

    FILE *f = fopen(path, "w");
    if (f == NULL) return SX_STORAGE_ERR_IO;

    int ret = fwrite(data, 1, len, f);
    fclose(f);

    if (ret < 0) return SX_STORAGE_ERR_IO;
    return SX_STORAGE_OK;
}

sx_storage_err_t sx_storage_append(const char *path, const void *data, uint32_t len)
{
    if (_check_init() != SX_STORAGE_OK)           return SX_STORAGE_ERR_NOT_INIT;
    if (path == NULL || data == NULL || len == 0) return SX_STORAGE_ERR_INVALID;

    FILE *f = fopen(path, "a");
    if (f == NULL) return SX_STORAGE_ERR_IO;

    int ret = fwrite(data, 1, len, f);
    fclose(f);

    if (ret < 0) return SX_STORAGE_ERR_IO;
    return SX_STORAGE_OK;
}

sx_storage_err_t sx_storage_read(const char *path, void *buf, uint32_t len)
{
    if (_check_init() != SX_STORAGE_OK)          return SX_STORAGE_ERR_NOT_INIT;
    if (path == NULL || buf == NULL || len == 0) return SX_STORAGE_ERR_INVALID;

    FILE *f = fopen(path, "r");
    if (f == NULL) return SX_STORAGE_ERR_NOT_FOUND;

    int ret = fread(buf, 1, len, f);
    fclose(f);

    if (ret < 0) return SX_STORAGE_ERR_IO;
    return SX_STORAGE_OK;
}

sx_storage_err_t sx_storage_read_partial(const char *path, void *buf, uint32_t offset, uint32_t len)
{
    if (_check_init() != SX_STORAGE_OK)          return SX_STORAGE_ERR_NOT_INIT;
    if (path == NULL || buf == NULL || len == 0)  return SX_STORAGE_ERR_INVALID;

    FILE *f = fopen(path, "r");
    if (f == NULL) return SX_STORAGE_ERR_NOT_FOUND;

    if (fseek(f, offset, LFS_SEEK_SET) < 0) {
        fclose(f);
        return SX_STORAGE_ERR_IO;
    }

    int ret = fread(buf, 1, len, f);
    fclose(f);

    if (ret < 0) return SX_STORAGE_ERR_IO;
    return SX_STORAGE_OK;
}

sx_storage_err_t sx_storage_delete(const char *path)
{
    if (_check_init() != SX_STORAGE_OK) return SX_STORAGE_ERR_NOT_INIT;
    if (path == NULL)                    return SX_STORAGE_ERR_INVALID;

    if (remove(path) < 0) return SX_STORAGE_ERR_NOT_FOUND;
    return SX_STORAGE_OK;
}

bool sx_storage_exists(const char *path){
    if (!s_initialized || path == NULL) return false;
    if (_ensure_power() != SX_STORAGE_OK) return false;

    FILE *f = fopen(path, "r");
    if (f == NULL) return false;
    fclose(f);
    return true;
}

int32_t sx_storage_size(const char *path)
{
    if (!s_initialized || path == NULL) return -1;
    if (_ensure_power() != SX_STORAGE_OK) return -1;

    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;

    int32_t sz = (int32_t)file_size(f);
    fclose(f);
    return sz;
}

sx_storage_err_t sx_storage_mkdir(const char *path)
{
    if (_check_init() != SX_STORAGE_OK) return SX_STORAGE_ERR_NOT_INIT;
    if (path == NULL)                    return SX_STORAGE_ERR_INVALID;

    if (mkdir(path) < 0) return SX_STORAGE_ERR_IO;
    return SX_STORAGE_OK;
}

void sx_storage_list_dir(const char *path)
{
    if (!s_initialized || path == NULL) return;
    if (_ensure_power() != SX_STORAGE_OK) return;
    fs_list_dir(path);
}

int32_t sx_storage_free_space(void)
{
    if (!s_initialized) return -1;
    if (_ensure_power() != SX_STORAGE_OK) return -1;
    int32_t used = (int32_t)fs_size();
    if (used < 0) return -1;
    return (int32_t)sx_W25Q128_info.total_bytes - used;
}

int32_t sx_storage_total_space(void)
{
    return (int32_t)sx_W25Q128_info.total_bytes;
}

sx_storage_err_t sx_storage_format(void)
{
    if (_check_init() != SX_STORAGE_OK) return SX_STORAGE_ERR_NOT_INIT;

    s_initialized = false;
    if (sx_fs_init(&s_flash) != 0) {
        log_error(TAG, "Format failed");
        return SX_STORAGE_ERR_IO;
    }

    s_initialized = true;
    log_info(TAG, "Format OK");
    return SX_STORAGE_OK;
}

sx_storage_err_t sx_storage_factory_reset(void)
{
    if (_check_init() != SX_STORAGE_OK) return SX_STORAGE_ERR_NOT_INIT;

    s_initialized = false;
    log_info(TAG, "Factory reset start...");

    sx_W25Q128_chip_erase(&s_w25q128);

    if (sx_fs_init(&s_flash) != 0) {
        log_error(TAG, "Factory reset: fs init failed");
        return SX_STORAGE_ERR_IO;
    }

    s_initialized = true;
    log_info(TAG, "Factory reset done");
    return SX_STORAGE_OK;
}

/* Power the flash off: wait for WIP, cut Flash_PWR, DeInit SPI1 (SCK/MISO/MOSI -> analog) and drive CS
 * LOW so no signal is pushed into a chip that has no supply. All file handles are closed after each
 * operation, so LittleFS has nothing pending in the flash. Idempotent. */
void sx_storage_sleep(void)
{
    if (!s_initialized || !s_powered || s_cfg == NULL) return;

    sx_W25Q128_power_down(&s_w25q128);          /* waits for WIP, then rail off */
    HAL_SPI_DeInit(s_cfg->hspi);
    sx_gpio_write(&s_cfg->s_cs, SX_GPIO_LOW);
    s_powered = false;
    log_info(TAG, "Flash powered off");
}

/* Power the flash back on: rail, settle, SPI1 init, CS idle HIGH, release power-down, check JEDEC.
 * On failure the rail is cut again and the error is returned. Idempotent. */
sx_storage_err_t sx_storage_wake(void)
{
    if (!s_initialized || s_cfg == NULL) return SX_STORAGE_ERR_NOT_INIT;
    if (s_powered) return SX_STORAGE_OK;

    sx_W25Q128_power_up(&s_w25q128);            /* rail on + 10 ms */

    if (HAL_SPI_Init(s_cfg->hspi) != HAL_OK) {
        log_error(TAG, "SPI1 re-init failed");
        sx_gpio_write(&s_w25q128.power, SX_GPIO_HIGH);
        return SX_STORAGE_ERR_IO;
    }
    sx_gpio_write(&s_cfg->s_cs, SX_GPIO_HIGH);

    if (!sx_W25Q128_probe(&s_w25q128)) {
        log_error(TAG, "Flash did not answer after power-up");
        HAL_SPI_DeInit(s_cfg->hspi);
        sx_gpio_write(&s_cfg->s_cs, SX_GPIO_LOW);
        sx_gpio_write(&s_w25q128.power, SX_GPIO_HIGH);
        return SX_STORAGE_ERR_IO;
    }

    s_powered = true;
    log_info(TAG, "Flash powered on");
    return SX_STORAGE_OK;
}

bool sx_storage_is_powered(void)
{
    return s_powered;
}