#ifndef SX__UART_H
#define SX__UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sx_config.h"
#include "cqueue.h"

#if (SX_USE_OS == 1)
#include "sx_os.h"
#endif

typedef struct sx_uart_config {
    void *pDriver;
    uint32_t baudrate;
    uint8_t bits;
    uint8_t parity;
    uint8_t stopbits;
} sx_uart_config_t;

typedef struct sx_uart sx_uart_t;

struct sx_uart {
    sx_uart_config_t *config;

    uint8_t *rxBuffer;
    int rxBufferSize;
    CQueue_t rxQueue;

    uint8_t *txBuffer;
    int txBufferSize;
    CQueue_t txQueue;
    #if (SX_USE_OS == 1)
    sx_mutex_t rxMutex;
    #endif
};

void sx_uart_init(sx_uart_t *_uart, sx_uart_config_t *_config, int _rxBufferSize, int _txBufferSize);
void sx_uart_write(sx_uart_t *_uart, const uint8_t *_data, int _len);
int sx_uart_read(sx_uart_t *_uart, uint8_t *_data, int _len,uint32_t _timeoutMS);
int sx_uart_available(sx_uart_t *_uart);
int sx_uart_rx_callback(sx_uart_t *_uart, const uint8_t *_data, int _len);
int sx_uart_tx_callback(sx_uart_t *_uart);
int sx_uart_flush(sx_uart_t *_uart);
#ifdef __cplusplus
}
#endif

#endif