/**
 * @file    foc_math.h
 * @brief   CORDIC sine/cosine and space-vector PWM (plan: Firmware
 *          conventions, Transforms and Space-vector PWM).
 *
 * Phase A axis is electrical angle 0, B at +120 deg, C at +240 deg.
 * Register access only; safe in the control interrupt. The CORDIC is used
 * from one context only (the control interrupt from Stage 8).
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdbool.h>

/** Configure the CORDIC for cosine/sine in zero-overhead mode. Needs its
 *  clock (CubeMX: CORDIC activated). Returns false if the clock is off. */
bool foc_init(void);

/** sin and cos of theta (rad, any value; wrapped to [-pi, pi)). */
void foc_sincos(float theta, float *s, float *c);

/** Square root on the FPU (VSQRT.F32), without libm. */
static inline float foc_sqrtf(float x)
{
  float r;
  __asm volatile("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
  return r;
}

/** Inverse Park (plan: Transforms): V_alpha = Vd cos - Vq sin,
 *  V_beta = Vd sin + Vq cos, with s, c = sin, cos of theta_e. */
static inline void foc_inv_park(float vd, float vq, float s, float c, float *v_alpha, float *v_beta)
{
  *v_alpha = (vd * c) - (vq * s);
  *v_beta = (vd * s) + (vq * c);
}

/** Space-vector PWM: limit |V| to 0.497 x vbus, inverse Clarke, midpoint
 *  injection, duty = 0.5 + v / vbus clamped to [0, 0.93]. vph receives the
 *  phase voltages before injection (V), duty the three duties. */
void foc_svpwm(float v_alpha, float v_beta, float vbus, float vph[3], float duty[3]);

#endif /* FOC_MATH_H */
