#ifndef SX_READ_BAT_H
#define SX_READ_BAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sx_filter.h"

typedef struct {
    kalman1d_t  kalman;
    lpf_t       lpf;
    float       v_bat_filtered;
    uint32_t    sample_elapsed_ms;
    uint32_t    log_elapsed_ms;
} sx_adc_reader_t;

void sx_adc_reader_init(sx_adc_reader_t *reader);
void sx_adc_reader_process(sx_adc_reader_t *reader, void *hal_adc, uint32_t timestamp);

#ifdef __cplusplus
}
#endif

#endif /* SX_READ_BAT_H */