#include "sx_config.h"
#include "sx_delay.h"

static const char *TAG = "SX_SPI";

#if SX_PLATFORM == SX_PLATFORM_STM32H5 
    #include "stm32h5xx_hal.h"
    #include "stm32h5xx_hal_spi.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32H7
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_hal_spi.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F7
    #include "stm32f7xx_hal.h"
    #include "stm32f7xx_hal_spi.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F4
    #include "stm32f4xx_hal.h"
    #include "stm32f4xx_hal_spi.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F1
    #include "stm32f1xx_hal.h"
    #include "stm32f1xx_hal_spi.h"
#endif

#include "sx_spi.h"

#define SX_SPI_TIMEOUT_MS   100U

typedef HAL_StatusTypeDef (*spi_write_fn_t) (SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
typedef HAL_StatusTypeDef (*spi_read_fn_t) (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
typedef HAL_StatusTypeDef (*spi_write_read_fn_t) (SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
                                                  uint16_t Size, uint32_t Timeout);

static spi_write_fn_t   spi_write = HAL_SPI_Transmit;
static spi_read_fn_t    spi_read = HAL_SPI_Receive;
static spi_write_read_fn_t spi_write_read = HAL_SPI_TransmitReceive;

static int _sx_spi_write(sx_spi_t *spi, const uint8_t *data, uint32_t len);
static int _sx_spi_read(sx_spi_t *spi, uint8_t *data, uint32_t len);
static int _sx_spi_write_read(sx_spi_t *spi, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);

sx_spi_ops_t sx_spi_ops = {
    .write      = _sx_spi_write,
    .read       = _sx_spi_read,
    .write_read = _sx_spi_write_read,
};

static int _sx_spi_write(sx_spi_t *spi, const uint8_t *data, uint32_t len){
    sx_spi_cs_low(spi);
    int ret = spi_write((SPI_HandleTypeDef *)spi->pDriver, (const uint8_t *)data, (uint16_t)len, SX_SPI_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    sx_spi_cs_high(spi);
    return ret;
}

static int _sx_spi_read(sx_spi_t *spi, uint8_t *data, uint32_t len){
    sx_spi_cs_low(spi);
    int ret = spi_read((SPI_HandleTypeDef *)spi->pDriver, data, (uint16_t) len, SX_SPI_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    sx_spi_cs_high(spi);
    return ret;
}

static int _sx_spi_write_read(sx_spi_t *spi, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len){
    sx_spi_cs_low(spi);
    int ret = spi_write_read((SPI_HandleTypeDef *)spi->pDriver, (const uint8_t *)tx_buf, rx_buf, (uint16_t)len, SX_SPI_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    sx_spi_cs_high(spi);
    return ret;
}
