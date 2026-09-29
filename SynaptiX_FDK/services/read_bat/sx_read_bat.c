#include "sx_read_bat.h"
#include "logger.h"
#include "sx_board.h"
#include "app_config.h"

static const char *TAG = "READ_BAT";

// void sx_adc_reader_init(sx_adc_reader_t *reader)
// {
//     if (!reader) return;
//     reader->v_bat_filtered    = 0.0f;
//     reader->sample_elapsed_ms = 0;
//     reader->log_elapsed_ms    = 0;
//     kalman1d_init(&reader->kalman, VBAT_KALMAN_Q, VBAT_KALMAN_R, 0.0f);
// }

void sx_adc_reader_init(sx_adc_reader_t *reader)
{
    if (!reader) return;
    reader->v_bat_filtered    = 0.0f;
    reader->sample_elapsed_ms = 0;
    reader->log_elapsed_ms    = 0;
    lpf_init(&reader->lpf, VBAT_LPF_ALPHA);
    kalman1d_init(&reader->kalman, VBAT_KALMAN_Q, VBAT_KALMAN_R, 0.0f);
}

void sx_adc_reader_process(sx_adc_reader_t *reader, void *hal_adc, uint32_t timestamp)
{
    if (!reader || !hal_adc) return;

    ADC_HandleTypeDef *adc = (ADC_HandleTypeDef *)hal_adc;

    reader->sample_elapsed_ms += timestamp;
    if (reader->sample_elapsed_ms >= SAMPLE_PERIOD_MS) {
        reader->sample_elapsed_ms = 0;

        HAL_ADC_Start(adc);
        if (HAL_ADC_PollForConversion(adc, 10) == HAL_OK) {
            uint32_t raw   = HAL_ADC_GetValue(adc);
            float    v_adc = ((float)raw * ADC_VREF) / ADC_RESOLUTION;
            float    v_raw = v_adc * VBAT_DIVIDER_RATIO;

            float v_lpf = lpf_update(&reader->lpf, v_raw);

            kalman1d_predict(&reader->kalman, 0.0f);
            kalman1d_update(&reader->kalman, v_lpf);
            reader->v_bat_filtered = kalman1d_get(&reader->kalman);
        }
        HAL_ADC_Stop(adc);
    }

    // if (reader->sample_elapsed_ms >= SAMPLE_PERIOD_MS) {
    //     reader->sample_elapsed_ms = 0;

    //     uint32_t sum = 0;
    //     HAL_ADC_Start(adc);
    //     for (uint8_t i = 0; i < OVERSAMPLE_COUNT; i++) {
    //         if (HAL_ADC_PollForConversion(adc, 10) == HAL_OK)
    //             sum += HAL_ADC_GetValue(adc);
    //     }
    //     HAL_ADC_Stop(adc);

    //     float v_adc = ((float)(sum / OVERSAMPLE_COUNT) * ADC_VREF) / ADC_RESOLUTION;
    //     float v_raw = v_adc * VBAT_DIVIDER_RATIO;

    //     float v_lpf = lpf_update(&reader->lpf, v_raw);

    //     kalman1d_predict(&reader->kalman, 0.0f);
    //     kalman1d_update(&reader->kalman, v_lpf);
    //     reader->v_bat_filtered = kalman1d_get(&reader->kalman);
    // }

    reader->log_elapsed_ms += timestamp;
    if (reader->log_elapsed_ms >= LOG_PERIOD_MS) {
        reader->log_elapsed_ms = 0;
        log_info(TAG, "Vbat = %.4f V", reader->v_bat_filtered);
    }
}