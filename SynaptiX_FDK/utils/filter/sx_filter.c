#include "sx_filter.h"
/* ---------- LPF ---------- */

void lpf_init(lpf_t *f, float alpha)
{
    if (!f) return;
    f->alpha       = (alpha > 0.0f && alpha <= 1.0f) ? alpha : 0.1f;
    f->value       = 0.0f;
    f->initialized = false;
}

float lpf_update(lpf_t *f, float input)
{
    if (!f) return input;
    if (!f->initialized) {
        f->value       = input;
        f->initialized = true;
        return f->value;
    }
    f->value = f->alpha * input + (1.0f - f->alpha) * f->value;
    return f->value;
}

void lpf_reset(lpf_t *f)
{
    if (!f) return;
    f->value       = 0.0f;
    f->initialized = false;
}

/* ---------- Kalman 1D ---------- */

void kalman1d_init(kalman1d_t *k, float Q, float R, float x0)
{
    if (!k) return;
    k->x = x0;
    k->P = 1.0f;
    k->Q = Q;
    k->R = R;
}

void kalman1d_predict(kalman1d_t *k, float u)
{
    if (!k) return;
    k->x += u;
    k->P += k->Q;
}

void kalman1d_update(kalman1d_t *k, float z)
{
    if (!k) return;
    float K = k->P / (k->P + k->R);
    k->x   += K * (z - k->x);
    k->P    = (1.0f - K) * k->P;
}

float kalman1d_get(const kalman1d_t *k)
{
    if (!k) return 0.0f;
    return k->x;
}

void kalman1d_reset(kalman1d_t *k)
{
    if (!k) return;
    k->x = 0.0f;
    k->P = 1.0f;
}