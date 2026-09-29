#ifndef SX_DELAY_H
#define SX_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sx_config.h"
#if (SX_USE_OS == 1)
#include "sx_os.h"
#endif

#include <stdint.h>

void sx_delay_ms(uint32_t ms);
uint32_t sx_gettick(void);
#ifdef __cplusplus
}
#endif
#endif // SX_DELAY_H