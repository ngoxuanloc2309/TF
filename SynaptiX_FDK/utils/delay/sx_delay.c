#include "sx_delay.h"

#if SX_PLATFORM == SX_PLATFORM_STM32H5
#include "stm32h5xx_hal.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32H7
#include "stm32h7xx_hal.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F7
#include "stm32f7xx_hal.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F4
#include "stm32f4xx_hal.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F1
#include "stm32f1xx_hal.h"
#endif

void sx_delay_ms(uint32_t ms)
{
#if (SX_USE_OS == 1)
    sx_os_msleep(ms);
#else
#if SX_PLATFORM == SX_PLATFORM_STM32H5 || SX_PLATFORM == SX_PLATFORM_STM32H7 || SX_PLATFORM == SX_PLATFORM_STM32F7 || SX_PLATFORM == SX_PLATFORM_STM32F4 || SX_PLATFORM == SX_PLATFORM_STM32F1
    HAL_Delay(ms);
#endif
#endif
}
uint32_t sx_gettick(void){
    #if SX_PLATFORM == SX_PLATFORM_STM32H5 || SX_PLATFORM == SX_PLATFORM_STM32H7 || SX_PLATFORM == SX_PLATFORM_STM32F7 || SX_PLATFORM == SX_PLATFORM_STM32F4 || SX_PLATFORM == SX_PLATFORM_STM32F1
        return HAL_GetTick();
    #endif
}