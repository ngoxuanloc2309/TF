#include "sx_external_flash.h"
#include "sx_W25Q128.h"
#include "sx_spi.h"
#include "logger.h"
#include "string.h"
#include "sx_delay.h"

#define W25Q128_MAX_READ_LEN  256U

static const char *TAG = "W25Q128";
static sx_W25Q128_t *s_dev = NULL;

sx_ext_flash_ops_t sx_W25Q128_ops = {
    .read         = sx_W25Q128_read,
    .write        = sx_W25Q128_write,
    .erase_sector = sx_W25Q128_erase_sector,
    .is_busy      = sx_W25Q128_is_busy,
};

sx_ext_flash_info_t sx_W25Q128_info = {
    .page_size   = W25Q128_PAGE_SIZE,
    .sector_size = W25Q128_SECTOR_SIZE,
    .block_size  = W25Q128_BLOCK_SIZE,
    .total_bytes = W25Q128_TOTAL_BYTES,
};

static bool w25q_write_enable(void)
{
    uint8_t cmd = W25Q128_CMD_WRITE_ENABLE;
    return (sx_spi_write(s_dev->spi, &cmd, 1) == 0);
}

static void w25q_wait_busy(void)
{
    uint8_t tx[2] = { W25Q128_CMD_READ_STATUS1, 0xFF };
    uint8_t rx[2];
    uint32_t timeout = 500;
    do {
        sx_delay_ms(5);
        sx_spi_write_read(s_dev->spi, tx, rx, 2);
        if (--timeout == 0) {
            log_error("W25Q128", "wait_busy TIMEOUT STATUS=0x%02X", rx[1]);
            return;
        }
    } while (rx[1] & W25Q128_STATUS1_BUSY);

    // Read it twice :)) - DONT EDIT IT!!!!!!
    sx_delay_ms(1);
    sx_spi_write_read(s_dev->spi, tx, rx, 2);
    if (rx[1] & W25Q128_STATUS1_BUSY) {
        log_error("W25Q128", "still BUSY after confirm, STATUS=0x%02X", rx[1]);
        timeout = 500;
        do {
            sx_delay_ms(5);
            sx_spi_write_read(s_dev->spi, tx, rx, 2);
            if (--timeout == 0) {
                log_error("W25Q128", "wait_busy TIMEOUT2 STATUS=0x%02X", rx[1]);
                return;
            }
        } while (rx[1] & W25Q128_STATUS1_BUSY);
    }
}

static void w25q_wait_busy_long(uint32_t timeout_ms)
{
    uint8_t tx[2] = { W25Q128_CMD_READ_STATUS1, 0xFF };
    uint8_t rx[2];
    uint32_t elapsed = 0;

    do {
        sx_delay_ms(100);           
        elapsed += 100;
        sx_spi_write_read(s_dev->spi, tx, rx, 2);
        if (elapsed >= timeout_ms) {
            log_error("W25Q128", "wait_busy_long TIMEOUT STATUS=0x%02X", rx[1]);
            return;
        }
    } while (rx[1] & W25Q128_STATUS1_BUSY);

    log_info("W25Q128", "wait_busy_long done in %lu ms", (unsigned long)elapsed);
}

/*API*/

bool sx_W25Q128_init(sx_W25Q128_t *dev, sx_spi_t *spi, sx_gpio_ops_t *pwr_ops, sx_gpio_pin_t *pwr_pin)
{
    dev->spi         = spi;
    dev->initialized = false;
    s_dev            = dev;

    sx_gpio_init(&dev->power, pwr_ops, pwr_pin);
    sx_gpio_write(&dev->power, SX_GPIO_LOW);  
    sx_delay_ms(10);   

    /* Wake from power-down — 1 byte write, 1 lan CS */
    uint8_t wake = W25Q128_CMD_RELEASE_POWER_DOWN;
    sx_spi_write(dev->spi, &wake, 1);
    sx_delay_ms(1);
    w25q_wait_busy();

    uint8_t tx[4] = { W25Q128_CMD_JEDEC_ID, 0x00, 0x00, 0x00 };
    uint8_t rx[4] = { 0 };
    sx_spi_write_read(dev->spi, tx, rx, 4);

    if (rx[1] != 0xEF || rx[2] != 0x40 || rx[3] != 0x18) {
        log_error(TAG, "JEDEC ID mismatch: %02X %02X %02X", rx[1], rx[2], rx[3]);
        return false;
    }

    dev->initialized = true;
    log_info(TAG, "W25Q128 OK (16MB)");
    return true;
}

bool sx_W25Q128_is_busy(void)
{
    if (!s_dev || !s_dev->initialized) return false;
    uint8_t tx[2] = { W25Q128_CMD_READ_STATUS1, 0x00 };
    uint8_t rx[2] = { 0 };
    sx_spi_write_read(s_dev->spi, tx, rx, 2);
    return (rx[1] & W25Q128_STATUS1_BUSY) != 0;
}

int sx_W25Q128_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!s_dev || !s_dev->initialized) return -1;
    if (buf == NULL || len == 0)        return -1;

    uint32_t offset = 0;
    while (offset < len) {
        uint32_t chunk = len - offset;
        if (chunk > W25Q128_MAX_READ_LEN) chunk = W25Q128_MAX_READ_LEN;

        uint8_t tx[4 + W25Q128_MAX_READ_LEN];
        uint8_t rx[4 + W25Q128_MAX_READ_LEN];

        uint32_t cur_addr = addr + offset;
        tx[0] = W25Q128_CMD_READ_DATA;
        tx[1] = (uint8_t)(cur_addr >> 16);
        tx[2] = (uint8_t)(cur_addr >>  8);
        tx[3] = (uint8_t)(cur_addr >>  0);
        memset(tx + 4, 0x00, chunk);

        if (sx_spi_write_read(s_dev->spi, tx, rx, 4 + chunk) != 0) {
            log_error(TAG, "spi_write_read failed at addr=0x%06lX chunk=%lu",
              (unsigned long)cur_addr, (unsigned long)chunk);
            return -1;
        }
        memcpy(buf + offset, rx + 4, chunk);

        offset += chunk;
    }
    return 0;
}

int sx_W25Q128_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!s_dev || !s_dev->initialized)   return -1;
    if (buf == NULL || len == 0)          return -1;
    uint32_t offset = 0;
    while (offset < len) {
        uint32_t page_offset = (addr + offset) % W25Q128_PAGE_SIZE;
        uint32_t chunk = W25Q128_PAGE_SIZE - page_offset;
        if (chunk > (len - offset)) chunk = len - offset;

        if (!w25q_write_enable()) {
            log_error(TAG, "write_enable FAILED");
            return -1;
        }

        uint8_t stx[2] = { W25Q128_CMD_READ_STATUS1, 0x00 };
        uint8_t srx[2] = { 0 };
        sx_spi_write_read(s_dev->spi, stx, srx, 2);
        log_info(TAG, "STATUS1 before write addr=0x%06lX: 0x%02X (WEL=%d)", (unsigned long)(addr + offset), srx[1], (srx[1] >> 1) & 1);

        uint8_t tx[4 + W25Q128_PAGE_SIZE];
        uint32_t cur_addr = addr + offset;
        tx[0] = W25Q128_CMD_PAGE_PROGRAM;
        tx[1] = (uint8_t)(cur_addr >> 16);
        tx[2] = (uint8_t)(cur_addr >>  8);
        tx[3] = (uint8_t)(cur_addr >>  0);
        memcpy(tx + 4, buf + offset, chunk);

        uint8_t stx2[2] = { W25Q128_CMD_READ_STATUS1, 0x00 };
        uint8_t srx2[2] = { 0 };
        sx_spi_write_read(s_dev->spi, stx2, srx2, 2);
        log_info(TAG, "STATUS1 JUST BEFORE page_program: 0x%02X", srx2[1]);

        if (sx_spi_write(s_dev->spi, tx, 4 + chunk) != 0) {
            log_error(TAG, "spi_write FAILED");
            return -1;
        }

        w25q_wait_busy();
        offset += chunk;
    }
    return 0;
}

int sx_W25Q128_erase_sector(uint32_t addr)
{
    if (!s_dev || !s_dev->initialized) return -1;

    addr &= ~(W25Q128_SECTOR_SIZE - 1U);

    if (!w25q_write_enable()) return -1;

    uint8_t cmd[4] = {
        W25Q128_CMD_SECTOR_ERASE_4K,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >>  8),
        (uint8_t)(addr >>  0),
    };

    if (sx_spi_write(s_dev->spi, cmd, 4) != 0) return -1;

    w25q_wait_busy();
    sx_delay_ms(5);
    log_info(TAG, "Sector erased at 0x%06lX", (unsigned long)addr);
    return 0;
}

void sx_W25Q128_chip_erase(sx_W25Q128_t *dev)
{
    s_dev = dev;
    w25q_write_enable();
    uint8_t cmd = W25Q128_CMD_CHIP_ERASE;
    sx_spi_write(dev->spi, &cmd, 1);
    w25q_wait_busy_long(200000);    
    log_info("W25Q128", "Chip erase done");
}

void sx_W25Q128_power_down(sx_W25Q128_t *dev){
    sx_gpio_write(&dev->power, SX_GPIO_HIGH);
}

void sx_W25Q128_power_up(sx_W25Q128_t *dev){
    sx_gpio_write(&dev->power, SX_GPIO_LOW);
}

void sx_W25Q128_sleep_on(sx_W25Q128_t *dev){
    uint8_t cmd = W25Q128_CMD_POWER_DOWN;
    sx_spi_write(dev->spi, &cmd, 1);
    dev->initialized = false;
}

void sx_W25Q128_sleep_off(sx_W25Q128_t *dev){
    uint8_t cmd = W25Q128_CMD_RELEASE_POWER_DOWN;
    sx_spi_write(dev->spi, &cmd, 1);
    sx_delay_ms(5);
    dev->initialized = true;
}