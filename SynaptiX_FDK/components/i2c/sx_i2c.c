#include "sx_i2c.h"
#include "sx_config.h"

#if SX_PLATFORM == SX_PLATFORM_STM32H5
    #include "stm32h5xx_hal.h"
    #include "stm32h5xx_hal_i2c.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32H7
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_hal_i2c.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F7
    #include "stm32f7xx_hal.h"
    #include "stm32f7xx_hal_i2c.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F4
    #include "stm32f4xx_hal.h"
    #include "stm32f4xx_hal_i2c.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F1
    #include "stm32f1xx_hal.h"
    #include "stm32f1xx_hal_i2c.h"
#endif

#include "i2c.h"

#define SX_I2C_TIMEOUT_MS   100U

typedef HAL_StatusTypeDef (*i2c_transmit_fn_t)    (I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t, uint32_t);
typedef HAL_StatusTypeDef (*i2c_receive_fn_t)     (I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t, uint32_t);
typedef HAL_StatusTypeDef (*i2c_mem_write_fn_t)   (I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t, uint32_t);
typedef HAL_StatusTypeDef (*i2c_mem_read_fn_t)    (I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t, uint32_t);
typedef HAL_StatusTypeDef (*i2c_is_device_ready_fn_t)(I2C_HandleTypeDef*, uint16_t, uint32_t, uint32_t);

static i2c_transmit_fn_t        i2c_transmit        = HAL_I2C_Master_Transmit;
static i2c_receive_fn_t         i2c_receive         = HAL_I2C_Master_Receive;
static i2c_mem_write_fn_t       i2c_mem_write       = HAL_I2C_Mem_Write;
static i2c_mem_read_fn_t        i2c_mem_read        = HAL_I2C_Mem_Read;
static i2c_is_device_ready_fn_t i2c_is_device_ready = HAL_I2C_IsDeviceReady;

static int _sx_i2c_write(sx_i2c_t *i2c, uint16_t dev_addr, const uint8_t *data, uint32_t len){
    return i2c_transmit((I2C_HandleTypeDef *)i2c->pDriver,
                        dev_addr, (uint8_t *)data, (uint16_t)len,
                        SX_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int _sx_i2c_read(sx_i2c_t *i2c, uint16_t dev_addr, uint8_t *data, uint32_t len){
    return i2c_receive((I2C_HandleTypeDef *)i2c->pDriver,
                       dev_addr, data, (uint16_t)len,
                       SX_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int _sx_i2c_mem_write(sx_i2c_t *i2c, uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, const uint8_t *data, uint32_t len){
    return i2c_mem_write((I2C_HandleTypeDef *)i2c->pDriver,
                         dev_addr, mem_addr, mem_addr_size,
                         (uint8_t *)data, (uint16_t)len,
                         SX_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int _sx_i2c_mem_read(sx_i2c_t *i2c, uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *data, uint32_t len){
    return i2c_mem_read((I2C_HandleTypeDef *)i2c->pDriver,
                        dev_addr, mem_addr, mem_addr_size,
                        data, (uint16_t)len,
                        SX_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int _sx_i2c_is_device_ready(sx_i2c_t *i2c, uint16_t dev_addr, uint8_t num_trial, uint32_t timeout){
    return i2c_is_device_ready((I2C_HandleTypeDef *)i2c->pDriver,
                                dev_addr, num_trial, timeout) == HAL_OK ? 0 : -1;
}

sx_i2c_ops_t sx_i2c_ops = {
    .write           = _sx_i2c_write,
    .read            = _sx_i2c_read,
    .mem_write       = _sx_i2c_mem_write,
    .mem_read        = _sx_i2c_mem_read,
    .is_device_ready = _sx_i2c_is_device_ready,
};