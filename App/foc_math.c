/**
 * @file    foc_math.c
 * @brief   CORDIC sine/cosine and space-vector PWM.
 */
#include "foc_math.h"
#include "pwm.h"
#include "main.h"
#include <stdint.h>

/* ---- Constants ------------------------------------------------------------ */

#define FOC_INV_PI        0.318309886184f
#define FOC_SQRT3_2       0.866025403784f
#define FOC_Q31           2147483648.0f       /* 2^31: q1.31 full scale */
#define FOC_INV_Q31       4.656612873e-10f    /* 2^-31 */

/* Largest voltage vector whose duties stay inside the 93 % cap (plan: SVPWM
 * step 1: 0.497 x Vbus). */
#define FOC_VLIM_FRAC     0.497f

/* Below this bus voltage the modulation isn't meaningful: hold 50 %. */
#define FOC_VBUS_MIN_V    1.0f

/* CORDIC (RM0440 17.3.6, Tables 106 and 118): FUNC 0 = cosine (RES1 = cos,
 * RES2 = sin, with modulus ARG2 = +1 from reset), PRECISION 6 = 24
 * iterations in 6 cycles, max error 2^-19; NRES = 1: two results; NARGS = 0:
 * one argument (ARG2 keeps +1); 32-bit q1.31 in and out. Zero-overhead
 * mode: writing WDATA starts it, reading RDATA waits for the result. */
#define FOC_CORDIC_PRECISION  6UL
#define FOC_CORDIC_CSR    ((0UL << CORDIC_CSR_FUNC_Pos) | \
                           (FOC_CORDIC_PRECISION << CORDIC_CSR_PRECISION_Pos) | \
                           CORDIC_CSR_NRES)

/* ---- Helpers -------------------------------------------------------------- */

/* Square root on the FPU (VSQRT.F32), without pulling in libm. */
static inline float foc_sqrtf(float x)
{
  float r;
  __asm volatile("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
  return r;
}

/* ---- API ------------------------------------------------------------------ */

bool foc_init(void)
{
  if ((RCC->AHB1ENR & RCC_AHB1ENR_CORDICEN) == 0U)
  {
    return false;
  }
  CORDIC->CSR = FOC_CORDIC_CSR;
  return true;
}

void foc_sincos(float theta, float *s, float *c)
{
  /* Angle / pi, wrapped to [-1, 1) for ARG1. floor() by hand (no libm). */
  float x = theta * FOC_INV_PI;
  float y = (x + 1.0f) * 0.5f;
  int32_t n = (int32_t)y;
  if ((float)n > y)
  {
    n--;
  }
  x -= 2.0f * (float)n;
  if (x >= 1.0f)
  {
    x -= 2.0f;
  }

  CORDIC->WDATA = (uint32_t)(int32_t)(x * FOC_Q31);
  *c = (float)(int32_t)CORDIC->RDATA * FOC_INV_Q31;
  *s = (float)(int32_t)CORDIC->RDATA * FOC_INV_Q31;
}

void foc_svpwm(float v_alpha, float v_beta, float vbus, float vph[3], float duty[3])
{
  if (!(vbus > FOC_VBUS_MIN_V))
  {
    vph[0] = vph[1] = vph[2] = 0.0f;
    duty[0] = duty[1] = duty[2] = PWM_DUTY_ZERO_CMD;
    return;
  }

  /* 1. Limit the vector. */
  float vmax = FOC_VLIM_FRAC * vbus;
  float m2 = (v_alpha * v_alpha) + (v_beta * v_beta);
  if (m2 > (vmax * vmax))
  {
    float k = vmax / foc_sqrtf(m2);   /* m2 > vmax^2 > 0 */
    v_alpha *= k;
    v_beta *= k;
  }

  /* 2. Inverse Clarke. */
  vph[0] = v_alpha;
  vph[1] = (-0.5f * v_alpha) + (FOC_SQRT3_2 * v_beta);
  vph[2] = (-0.5f * v_alpha) - (FOC_SQRT3_2 * v_beta);

  /* 3. Midpoint (min-max) injection. */
  float vhi = vph[0];
  float vlo = vph[0];
  for (uint32_t i = 1U; i < 3U; i++)
  {
    if (vph[i] > vhi) { vhi = vph[i]; }
    if (vph[i] < vlo) { vlo = vph[i]; }
  }
  float mid = 0.5f * (vhi + vlo);

  /* 4. Duty, clamped (a backstop: step 1 keeps it inside). */
  float inv_vbus = 1.0f / vbus;
  for (uint32_t i = 0U; i < 3U; i++)
  {
    float d = PWM_DUTY_ZERO_CMD + ((vph[i] - mid) * inv_vbus);
    duty[i] = (d < 0.0f) ? 0.0f : ((d > PWM_DUTY_MAX) ? PWM_DUTY_MAX : d);
  }
}
