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

/** Space-vector PWM: limit |V| to 0.497 x vbus, inverse Clarke, midpoint
 *  injection, duty = 0.5 + v / vbus clamped to [0, 0.93]. vph receives the
 *  phase voltages before injection (V), duty the three duties. */
void foc_svpwm(float v_alpha, float v_beta, float vbus, float vph[3], float duty[3]);

#endif /* FOC_MATH_H */
