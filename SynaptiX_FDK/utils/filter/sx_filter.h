#ifndef SX_FILTER_H
#define SX_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* IIR first-order low-pass: y[n] = a*x[n] + (1-a)*y[n-1] */
typedef struct {
    float alpha;
    float value;
    bool  initialized;
} lpf_t;

void  lpf_init(lpf_t *f, float alpha);
float lpf_update(lpf_t *f, float input);
void  lpf_reset(lpf_t *f);

/* Scalar Kalman filter for 1-D state (e.g. velocity)
 * Predict: x += u,  P += Q
 * Update:  K = P/(P+R),  x += K*(z-x),  P = (1-K)*P  */
typedef struct {
    float x;   /* state estimate    */
    float P;   /* estimate variance */
    float Q;   /* process noise     */
    float R;   /* measurement noise */
} kalman1d_t;

void  kalman1d_init(kalman1d_t *k, float Q, float R, float x0);
void  kalman1d_predict(kalman1d_t *k, float u);
void  kalman1d_update(kalman1d_t *k, float z);
float kalman1d_get(const kalman1d_t *k);
void  kalman1d_reset(kalman1d_t *k);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */