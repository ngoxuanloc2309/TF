#ifndef __SX_W25Q128_H
#define __SX_W25Q128_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sx_spi.h"
#include "cqueue.h"
#include "sx_external_flash.h"

/*User config*/
#ifndef SX_W25Q_MAX_ENTRIES
#  define SX_W25Q_MAX_ENTRIES       3600U
#endif

#define W25Q128_PAGE_SIZE           256U    /*From datasheet*/
#define W25Q128_SECTOR_SIZE         4096U   //4kb
#define W25Q128_BLOCK_SIZE          (64U * 1024U)
#define W25Q128_TOTAL_BYTES         (16U * 1024U * 1024U)

/*  REGISTER    */
#define W25Q128_CMD_WRITE_ENABLE       0x06
#define W25Q128_CMD_WRITE_DISABLE      0x04
#define W25Q128_CMD_READ_STATUS1       0x05
#define W25Q128_CMD_PAGE_PROGRAM       0x02
#define W25Q128_CMD_READ_DATA          0x03
#define W25Q128_CMD_SECTOR_ERASE_4K    0x20
#define W25Q128_CMD_CHIP_ERASE         0xC7
#define W25Q128_CMD_POWER_DOWN         0xB9
#define W25Q128_CMD_RELEASE_POWER_DOWN 0xAB
#define W25Q128_CMD_JEDEC_ID           0x9F

#define W25Q128_STATUS1_BUSY           0x01

/* Driver handle  */
typedef struct {
    sx_spi_t *spi;
    bool      initialized;
    sx_gpio_t power;
} sx_W25Q128_t;

/*API*/

bool sx_W25Q128_init            (sx_W25Q128_t *dev, sx_spi_t *spi, sx_gpio_ops_t *pwr_ops, sx_gpio_pin_t *pwr_pin);
int  sx_W25Q128_read            (uint32_t addr, uint8_t *buf, uint32_t len);
int  sx_W25Q128_write           (uint32_t addr, const uint8_t *buf, uint32_t len);
int  sx_W25Q128_erase_sector    (uint32_t addr);
void sx_W25Q128_chip_erase(sx_W25Q128_t *dev);
bool sx_W25Q128_is_busy         (void);

void sx_W25Q128_sleep_on(sx_W25Q128_t *dev);
void sx_W25Q128_sleep_off(sx_W25Q128_t *dev);
void sx_W25Q128_power_down(sx_W25Q128_t *dev);
void sx_W25Q128_power_up(sx_W25Q128_t *dev);

extern sx_ext_flash_ops_t sx_W25Q128_ops;
extern sx_ext_flash_info_t sx_W25Q128_info;

#ifdef __cplusplus
}
#endif

#endif