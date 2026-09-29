#ifndef SX_SPI_H
#define SX_SPI_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sx_gpio.h"

typedef struct sx_spi sx_spi_t;

typedef struct sx_spi_ops{
    int (*write)(sx_spi_t *spi, const uint8_t *data, uint32_t len);
    int (*read)(sx_spi_t *spi, uint8_t *data, uint32_t len);
    int (*write_read)(sx_spi_t *spi, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);
} sx_spi_ops_t;

struct sx_spi
{
    /* data */
    sx_spi_ops_t *ops;
    void *pDriver;
    sx_gpio_t *cs;
};

static inline void sx_spi_init(sx_spi_t *spi, sx_spi_ops_t *ops, void *pDriver, sx_gpio_t *cs){
    spi->ops = ops;
    spi->pDriver = pDriver;
    spi->cs = cs;
}

static inline int sx_spi_write(sx_spi_t *spi, const uint8_t *data, uint32_t len){
    if(spi->ops && spi->ops->write){
        return spi->ops->write(spi, data, len);
    }
    return -1;
}

static inline int sx_spi_read(sx_spi_t *spi, uint8_t *data, uint32_t len){
    if(spi->ops && spi->ops->read){
        return spi->ops->read(spi, data, len);
    }
    return -1;
}

static inline int sx_spi_write_read(sx_spi_t *spi, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len){
    if(spi->ops && spi->ops->write_read){
        return spi->ops->write_read(spi, tx_buf, rx_buf, len);
    }
    return -1;
}

static inline void sx_spi_cs_low(sx_spi_t *spi) {
    if (spi->cs) sx_gpio_write(spi->cs, SX_GPIO_LOW);
}
static inline void sx_spi_cs_high(sx_spi_t *spi) {
    if (spi->cs) sx_gpio_write(spi->cs, SX_GPIO_HIGH);
}

extern sx_spi_ops_t sx_spi_ops;


#ifdef __cplusplus
}
#endif
#endif // SX_SPI_H