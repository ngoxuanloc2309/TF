#include "sx_uart.h"
#include "sx_mem.h"
#include "sx_config.h"
#include "sx_delay.h"
#include "logger.h"

static const char *TAG = "SX_UART";

#if SX_PLATFORM == SX_PLATFORM_STM32H5 
    #include "stm32h5xx_hal.h"
    #include "stm32h5xx_hal_uart.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32H7
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_hal_uart.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F7
    #include "stm32f7xx_hal.h"
    #include "stm32f7xx_hal_uart.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F4
    #include "stm32f4xx_hal.h"
    #include "stm32f4xx_hal_uart.h"
#elif SX_PLATFORM == SX_PLATFORM_STM32F1
    #include "stm32f1xx_hal.h"
    #include "stm32f1xx_hal_uart.h"
#endif

void sx_uart_init(sx_uart_t *_uart, sx_uart_config_t *_config, int _rxBufferSize, int _txBufferSize){
    _uart->config = _config;
    _uart->rxBufferSize = _rxBufferSize;
    if(!_uart->rxBuffer) {
        _uart->rxBuffer = sx_malloc(_rxBufferSize);
    }
    cqueue_init_static(&_uart->rxQueue, _uart->rxBuffer, _rxBufferSize, sizeof(uint8_t));  // len=512, size=1
    _uart->txBufferSize = _txBufferSize;
    if(!_uart->txBuffer) {
        _uart->txBuffer = sx_malloc(_txBufferSize);
    }
    cqueue_init_static(&_uart->txQueue, _uart->txBuffer, _txBufferSize, sizeof(uint8_t));  // len=512, size=1
    #if (SX_USE_OS == 1)
    sx_os_mutex_new(&_uart->rxMutex);
    #endif
    // log_debug(TAG, "UART initialized with baudrate: %d, bits: %d, parity: %d, stopbits: %d", _config->baudrate, _config->bits, _config->parity, _config->stopbits);
}
void sx_uart_write(sx_uart_t *_uart, const uint8_t *_data, int _len){
    // log_debug(TAG, "UART write: %d bytes", _len);
#if SX_PLATFORM == SX_PLATFORM_STM32H5 || SX_PLATFORM == SX_PLATFORM_STM32H7 || SX_PLATFORM == SX_PLATFORM_STM32F7 || SX_PLATFORM == SX_PLATFORM_STM32F4 || SX_PLATFORM == SX_PLATFORM_STM32F1
    HAL_UART_Transmit((UART_HandleTypeDef *)_uart->config->pDriver, (uint8_t *)_data, _len, 1000);
#endif
}
int sx_uart_read(sx_uart_t *_uart, uint8_t *_data, int _len, uint32_t _timeoutMS){
    #if (SX_USE_OS == 1)
    sx_os_mutex_lock(_uart->rxMutex);
    #endif
    int len = 0;
    uint32_t time = 0;
    while(len < _len && time < _timeoutMS){
        if(cqueue_receive(&_uart->rxQueue, _data + len) == false){
            // sx_delay_ms(1);
            // time += 1;
            break;
        }
        len++;
    }
    #if (SX_USE_OS == 1)
    sx_os_mutex_unlock(_uart->rxMutex);
    #endif
    return len;
}
int sx_uart_available(sx_uart_t *_uart){
    //log_debug(TAG, "UART available: %d bytes", _uart->rxQueue.count);
    return _uart->rxQueue.count;
}

int sx_uart_rx_callback(sx_uart_t *_uart, const uint8_t *_data, int _len){
    int len = 0;
    while(len < _len){
        if(cqueue_send(&_uart->rxQueue, _data + len) == true){
            len++;
        } else {
            break;
        }
    }
    return len;
}
int sx_uart_tx_callback(sx_uart_t *_uart){
    return 0;
}

int sx_uart_flush(sx_uart_t *_uart)
{
    cqueue_init_static(&_uart->rxQueue, _uart->rxBuffer, _uart->rxBufferSize, sizeof(uint8_t)); 
    return 0;
}
