#include "sx_can.h"
#include "sx_config.h"
//#include "fdcan.h"

#if SX_PLATFORM == SX_PLATFORM_STM32H5 
    #include "stm32h5xx_hal.h"
    //#include "stm32h5xx_hal_fdcan.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32H7
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_hal_fdcan.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F7
    #include "stm32f7xx_hal.h"
    #include "stm32f7xx_hal_fdcan.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F4
    #include "stm32f4xx_hal.h"
    #include "stm32f4xx_hal_fdcan.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F1
    #include "stm32f1xx_hal.h"
    #include "stm32f1xx_hal_fdcan.h"
#endif

