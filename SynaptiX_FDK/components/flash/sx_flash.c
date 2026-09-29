#include "sx_flash.h"
#include "sx_config.h"
#include <string.h>

#if (SX_PLATFORM == SX_PLATFORM_STM32H5)
#include "stm32h5xx_hal.h"
#elif (SX_PLATFORM == SX_PLATFORM_STM32F4)
#include "stm32f4xx_hal.h"
#elif (SX_PLATFORM == SX_PLATFORM_STM32F7)
#include "stm32f7xx_hal.h"
#elif (SX_PLATFORM == SX_PLATFORM_STM32F1)
#include "stm32f1xx_hal.h"
#elif (SX_PLATFORM == SX_PLATFORM_STM32H7)
#include "stm32h7xx_hal.h"
#else
#error "Unsupported platform"
#endif

#if (SX_PLATFORM == SX_PLATFORM_STM32H5)

#define FLASH_BANK1_BASE    0x08000000UL
#define FLASH_BANK2_BASE    0x08100000UL
#define FLASH_SECTOR_BYTES  0x2000UL        /* 8 KB */

static void _addr_to_bank_sector(uint32_t addr, uint32_t *bank, uint32_t *sector)
{
    if (addr >= FLASH_BANK2_BASE) {
        *bank   = FLASH_BANK_2;
        *sector = (addr - FLASH_BANK2_BASE) / FLASH_SECTOR_BYTES;
    } else {
        *bank   = FLASH_BANK_1;
        *sector = (addr - FLASH_BANK1_BASE) / FLASH_SECTOR_BYTES;
    }
}

#endif /* STM32H5 */

/* ── Public API ─────────────────────────────────── */

void sx_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    memcpy(buf, (const void *)addr, len);
}

void sx_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
#if (SX_PLATFORM == SX_PLATFORM_STM32H5)
    uint32_t buf[4];
    uint32_t written = 0;

    while (written < len) {
        uint32_t chunk = len - written;
        if (chunk >= 16) {
            memcpy(buf, data + written, 16);
        } else {
            memset(buf, 0xFF, 16);          /* pad with 0xFF (erased state) */
            memcpy(buf, data + written, chunk);
        }
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                          addr + written,
                          (uint32_t)buf);
        written += 16;
    }

#elif (SX_PLATFORM == SX_PLATFORM_STM32F1)
    for (uint32_t i = 0; i < len; i += 2) {
        uint16_t data16 = 0xFFFF;
        uint32_t chunk  = (len - i >= 2) ? 2 : 1;
        memcpy(&data16, data + i, chunk);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, data16);
    }

#elif (SX_PLATFORM == SX_PLATFORM_STM32H7) || \
      (SX_PLATFORM == SX_PLATFORM_STM32F7) || \
      (SX_PLATFORM == SX_PLATFORM_STM32F4)
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t data32 = 0xFFFFFFFF;
        uint32_t chunk  = (len - i >= 4) ? 4 : (len - i);
        memcpy(&data32, data + i, chunk);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, data32);
    }
#endif
}

void sx_flash_erase(uint32_t addr, uint32_t len)
{
#if (SX_PLATFORM == SX_PLATFORM_STM32H5)
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error          = 0;

    uint32_t bank, sector;
    _addr_to_bank_sector(addr, &bank, &sector);

    uint32_t nb_sectors = (len + FLASH_SECTOR_BYTES - 1) / FLASH_SECTOR_BYTES;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks     = bank;
    erase.Sector    = sector;
    erase.NbSectors = nb_sectors;

    HAL_FLASHEx_Erase(&erase, &page_error);

#elif (SX_PLATFORM == SX_PLATFORM_STM32F1)
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error          = 0;

    erase.TypeErase  = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = addr;
    erase.NbPages    = (len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

    HAL_FLASHEx_Erase(&erase, &page_error);

#elif (SX_PLATFORM == SX_PLATFORM_STM32H7) || \
      (SX_PLATFORM == SX_PLATFORM_STM32F7) || \
      (SX_PLATFORM == SX_PLATFORM_STM32F4)
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error          = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector    = addr / FLASH_SECTOR_SIZE;
    erase.NbSectors = (len + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

    HAL_FLASHEx_Erase(&erase, &page_error);
#endif
}

void sx_flash_unlock(void)
{
    HAL_FLASH_Unlock();
}

void sx_flash_lock(void)
{
    HAL_FLASH_Lock();
}