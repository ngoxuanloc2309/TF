#include "sx_config.h"
#include "sx_gpio.h"

#if SX_PLATFORM == SX_PLATFORM_STM32H5 
    #include "stm32h5xx_hal.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32H7
    #include "stm32h7xx_hal.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F4
    #include "stm32f4xx_hal.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F1
    #include "stm32f1xx_hal.h"
#endif

static int _sx_gpio_write(sx_gpio_t *gpio, SX_GPIO_VALUE value) {
    sx_gpio_pin_t *pin = (sx_gpio_pin_t *)gpio->pDriver;
    HAL_GPIO_WritePin((GPIO_TypeDef *)pin->port, pin->pin,
                      value == SX_GPIO_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return 0;
}

static int _sx_gpio_read(sx_gpio_t *gpio, SX_GPIO_VALUE *value) {
    sx_gpio_pin_t *pin = (sx_gpio_pin_t *)gpio->pDriver;
    *value = (HAL_GPIO_ReadPin((GPIO_TypeDef *)pin->port, pin->pin) == GPIO_PIN_SET)
             ? SX_GPIO_HIGH : SX_GPIO_LOW;
    return 0;
}

static int _sx_gpio_toggle(sx_gpio_t *gpio) {
    sx_gpio_pin_t *pin = (sx_gpio_pin_t *)gpio->pDriver;
    HAL_GPIO_TogglePin((GPIO_TypeDef *)pin->port, pin->pin);
    return 0;
}

sx_gpio_ops_t sx_gpio_ops = {
    .write  = _sx_gpio_write,
    .read   = _sx_gpio_read,
    .toggle = _sx_gpio_toggle,
};