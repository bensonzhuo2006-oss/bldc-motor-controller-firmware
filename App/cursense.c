/**
 * @file    cursense.c
 * @brief   Injected ADC sampling of SOA/SOB/SOC, offsets, conversion to
 *          amps, and the 20 kHz control interrupt.
 *
 * ADC (CubeMX): ADC1/2/3 synchronous clock HCLK / 4 = 42.5 MHz, injected
 * rank 1 = channel 12, 2.5-cycle sampling, trigger TIM1 TRGO2 rising.
 * OC4REF (PWM mode 1, CCR4 = 4200) rises when the down-counting counter
 * passes 4200, 50 counts (294 ns) after the counter peak, while every low
 * side is on (plan: Sampling).
 */
#include "cursense.h"
#include "ctrl.h"
#include "debug.h"
#include "pwm.h"
#include "adc.h"
#include "main.h"

/* ---- Constants ------------------------------------------------------------ */

/* Offsets: VREF / 2 = mid-scale (VREF_DIV = 1). Tolerance: DRV input offset
 * VOFF +/-4 mV x 40 = +/-160 mV = +/-199 counts (datasheet 7.5), rounded up
 * for VREF / 2 accuracy. Outside it the amplifier or its supply is wrong. */
#define CURSENSE_OFFSET_NOMINAL    2048.0f
#define CURSENSE_OFFSET_TOL        250.0f
#define CURSENSE_OFFSET_SAMPLES    1024U     /* plan Stage 7 */
/* Skip 10 ms after a configure: CSA calibration 100 us (datasheet 8.3.4.3),
 * settling 1.2 us at 40 V/V (datasheet 7.5, tSET). */
#define CURSENSE_OFFSET_SETTLE     200U

/* Heartbeat timeout (plan: Control-loop heartbeat, "a few ms"). */
#define CURSENSE_HEARTBEAT_TIMEOUT_MS  3U

/* ADC2 and ADC3 share ADC1's clock and trigger, so they finish together;
 * wait at most about 1 us for them before counting the sample late. */
#define CURSENSE_LATE_WAIT_LOOPS   50U

/* ADC enable: ADRDY within a few us; calibration-to-ADEN gap >= 4 ADC
 * clocks (RM0440 ADC calibration), 100 CPU cycles used. */
#define CURSENSE_ADRDY_TIMEOUT_MS  2U
#define CURSENSE_CAL_TO_EN_CYCLES  100U

#define CURSENSE_JFLAGS            (ADC_ISR_JEOC | ADC_ISR_JEOS)

typedef enum
{
  OFF_IDLE = 0,
  OFF_SETTLE,
  OFF_ACCUM,
  OFF_DONE
} off_state_t;

/* ---- State (shared with the control interrupt) ----------------------------- */

static volatile uint32_t s_count;
static volatile uint16_t s_raw[PHASE_COUNT];
static volatile float    s_amps[PHASE_COUNT];
static volatile float    s_offset[PHASE_COUNT] =
{
  CURSENSE_OFFSET_NOMINAL, CURSENSE_OFFSET_NOMINAL, CURSENSE_OFFSET_NOMINAL
};
static volatile bool     s_offset_valid;

static volatile off_state_t s_off_state;
static volatile uint32_t    s_off_k;
static volatile uint32_t    s_off_sum[PHASE_COUNT];

static volatile uint16_t s_min[PHASE_COUNT];
static volatile uint16_t s_max[PHASE_COUNT];
static volatile uint32_t s_sum[PHASE_COUNT];
static volatile uint64_t s_sumsq[PHASE_COUNT];   /* of (count - 2048), keeps the variance precise */
static volatile uint32_t s_n;

static volatile uint32_t s_isr_max;        /* since boot */
static volatile uint32_t s_isr_win_max;    /* since the last stats_take */
static volatile uint32_t s_isr_win_sum;
static volatile uint32_t s_late;

/* Main loop only. */
static uint32_t s_hb_count;
static uint32_t s_hb_ms;

/* ---- Helpers -------------------------------------------------------------- */

static void cursense_stats_reset(void)
{
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    s_min[ph] = 0xFFFFU;
    s_max[ph] = 0U;
    s_sum[ph] = 0U;
    s_sumsq[ph] = 0U;
  }
  s_n = 0U;
  s_isr_win_max = 0U;
  s_isr_win_sum = 0U;
}

/* Square root on the FPU (VSQRT.F32), without pulling in libm. */
static inline float cursense_sqrtf(float x)
{
  float r;
  __asm volatile("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
  return r;
}

static bool cursense_adc_enable(ADC_HandleTypeDef *h)
{
  if (HAL_ADCEx_Calibration_Start(h, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return false;
  }
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < CURSENSE_CAL_TO_EN_CYCLES)
  {
  }
  LL_ADC_ClearFlag_ADRDY(h->Instance);
  LL_ADC_Enable(h->Instance);
  uint32_t t0 = HAL_GetTick();
  while (!LL_ADC_IsActiveFlag_ADRDY(h->Instance))
  {
    if ((HAL_GetTick() - t0) > CURSENSE_ADRDY_TIMEOUT_MS)
    {
      return false;
    }
  }
  return true;
}

/* ---- API ------------------------------------------------------------------ */

bool cursense_init(const char **what)
{
  static const char ok[] = "OK";
  *what = ok;

  if (!LL_ADC_IsEnabled(ADC1))
  {
    *what = "ADC1 not enabled (power_init first)";
    return false;
  }
  if (!cursense_adc_enable(&hadc2))
  {
    *what = "ADC2 cal/enable";
    return false;
  }
  if (!cursense_adc_enable(&hadc3))
  {
    *what = "ADC3 cal/enable";
    return false;
  }

  cursense_stats_reset();
  s_hb_count = s_count;
  s_hb_ms = HAL_GetTick();

  /* Arm the three injected triggers; each conversion then waits for TRGO2. */
  ADC1->ISR = CURSENSE_JFLAGS;
  ADC2->ISR = CURSENSE_JFLAGS;
  ADC3->ISR = CURSENSE_JFLAGS;
  LL_ADC_INJ_StartConversion(ADC2);
  LL_ADC_INJ_StartConversion(ADC3);
  LL_ADC_INJ_StartConversion(ADC1);

  /* ADC1 end of injected sequence = the control interrupt (ADC1_2_IRQn,
   * priority 0 from CubeMX). ADC2 raises no interrupts. */
  LL_ADC_EnableIT_JEOS(ADC1);
  return true;
}

void cursense_offset_start(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_offset_valid = false;
  s_off_k = 0U;
  s_off_state = OFF_SETTLE;
  __set_PRIMASK(primask);
}

bool cursense_offset_poll(float means[PHASE_COUNT], bool *ok)
{
  if (s_off_state != OFF_DONE)
  {
    return false;
  }
  *ok = true;
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    means[ph] = (float)s_off_sum[ph] / (float)CURSENSE_OFFSET_SAMPLES;
    float dev = means[ph] - CURSENSE_OFFSET_NOMINAL;
    if ((dev > CURSENSE_OFFSET_TOL) || (dev < -CURSENSE_OFFSET_TOL))
    {
      *ok = false;
    }
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (*ok)
  {
    for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
    {
      s_offset[ph] = means[ph];
    }
  }
  s_offset_valid = *ok;
  s_off_state = OFF_IDLE;
  __set_PRIMASK(primask);
  return true;
}

bool cursense_offsets_valid(void)
{
  return s_offset_valid;
}

void cursense_offsets(float counts[PHASE_COUNT])
{
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    counts[ph] = s_offset[ph];
  }
}

bool cursense_heartbeat_ok(uint32_t now_ms)
{
  uint32_t c = s_count;
  if (c != s_hb_count)
  {
    s_hb_count = c;
    s_hb_ms = now_ms;
    return true;
  }
  return (now_ms - s_hb_ms) <= CURSENSE_HEARTBEAT_TIMEOUT_MS;
}

void cursense_snapshot(cursense_snapshot_t *out)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    out->raw[ph] = s_raw[ph];
    out->amps[ph] = s_amps[ph];
  }
  out->count = s_count;
  __set_PRIMASK(primask);
}

void cursense_stats_take(cursense_stats_t *out)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  out->samples = s_n;
  out->isr_mean_cycles = (s_n > 0U) ? (s_isr_win_sum / s_n) : 0U;
  out->isr_max_cycles = s_isr_win_max;
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    out->min[ph] = s_min[ph];
    out->max[ph] = s_max[ph];
    out->mean[ph] = (s_n > 0U) ? ((float)s_sum[ph] / (float)s_n) : 0.0f;
    out->rms[ph] = 0.0f;
    if (s_n > 0U)
    {
      /* Variance about the mean from sums of (count - 2048): both terms stay
       * small, so there's no cancellation. Double, once per call. */
      double m = (double)out->mean[ph] - (double)CURSENSE_OFFSET_NOMINAL;
      double var = ((double)s_sumsq[ph] / (double)s_n) - (m * m);
      out->rms[ph] = (var > 0.0) ? cursense_sqrtf((float)var) : 0.0f;
    }
  }
  cursense_stats_reset();
  __set_PRIMASK(primask);
}

uint32_t cursense_isr_max_cycles(void)
{
  return s_isr_max;
}

uint32_t cursense_late_count(void)
{
  return s_late;
}

/* ---- Control interrupt (20 kHz) -------------------------------------------- */

/* Replaces the CubeMX handler ("Generate IRQ handler" unticked for ADC1 and
 * ADC2 global interrupt). Register access only: no HAL, no logging, no
 * blocking, no SPI3 (plan: Timing, interrupts and data sharing). */
void ADC1_2_IRQHandler(void)
{
  if ((ADC1->ISR & ADC_ISR_JEOS) == 0U)
  {
    return;
  }
  uint32_t t0 = DWT->CYCCNT;
  debug_timing_high();   /* PA3 high for the whole interrupt */

  uint32_t wait = 0U;
  while (((ADC2->ISR & ADC3->ISR & ADC_ISR_JEOS) == 0U) && (wait < CURSENSE_LATE_WAIT_LOOPS))
  {
    wait++;
  }
  if (wait >= CURSENSE_LATE_WAIT_LOOPS)
  {
    s_late++;
  }

  uint16_t r[PHASE_COUNT];
  r[PHASE_A] = (uint16_t)ADC2->JDR1;   /* SOA */
  r[PHASE_B] = (uint16_t)ADC1->JDR1;   /* SOB */
  r[PHASE_C] = (uint16_t)ADC3->JDR1;   /* SOC */
  ADC1->ISR = CURSENSE_JFLAGS;         /* write-1-to-clear; ADC1 EOC (VM) untouched */
  ADC2->ISR = CURSENSE_JFLAGS;
  ADC3->ISR = CURSENSE_JFLAGS;

  float amps[PHASE_COUNT];
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    uint16_t v = r[ph];
    s_raw[ph] = v;
    amps[ph] = ((float)v - s_offset[ph]) * CURSENSE_AMPS_PER_COUNT;
    s_amps[ph] = amps[ph];
    if (v < s_min[ph]) { s_min[ph] = v; }
    if (v > s_max[ph]) { s_max[ph] = v; }
    s_sum[ph] += v;
    int32_t d = (int32_t)v - (int32_t)CURSENSE_OFFSET_NOMINAL;
    s_sumsq[ph] += (uint64_t)(uint32_t)(d * d);
  }
  s_n++;

  if (s_off_state == OFF_SETTLE)
  {
    if (++s_off_k >= CURSENSE_OFFSET_SETTLE)
    {
      s_off_k = 0U;
      s_off_sum[PHASE_A] = 0U;
      s_off_sum[PHASE_B] = 0U;
      s_off_sum[PHASE_C] = 0U;
      s_off_state = OFF_ACCUM;
    }
  }
  else if (s_off_state == OFF_ACCUM)
  {
    s_off_sum[PHASE_A] += r[PHASE_A];
    s_off_sum[PHASE_B] += r[PHASE_B];
    s_off_sum[PHASE_C] += r[PHASE_C];
    if (++s_off_k >= CURSENSE_OFFSET_SAMPLES)
    {
      s_off_state = OFF_DONE;
    }
  }

  s_count++;
  debug_scope_write(r[PHASE_A]);      /* PA2: SOA as the ADC sees it */
  debug_capture_push(r[PHASE_A], r[PHASE_B], r[PHASE_C]);
  ctrl_isr(amps);                     /* overcurrent trip, then the PWM output */

  debug_timing_low();
  uint32_t dt = DWT->CYCCNT - t0;
  s_isr_win_sum += dt;
  if (dt > s_isr_win_max)
  {
    s_isr_win_max = dt;
  }
  if (dt > s_isr_max)
  {
    s_isr_max = dt;
  }
}
