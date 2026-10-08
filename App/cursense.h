/**
 * @file    cursense.h
 * @brief   Phase current sensing and the 20 kHz control interrupt
 *          (plan: Stage 7; Current sensing; Firmware conventions).
 *
 * SOA -> ADC2_IN12 (PB2), SOB -> ADC1_IN12 (PB1), SOC -> ADC3_IN12 (PB0),
 * one injected conversion each, all three started together by TIM1 TRGO2
 * (OC4REF, CCR4 = 4200). ADC1's end-of-injected-sequence interrupt is the
 * control interrupt: highest priority, register access only.
 *
 * Current is positive into the motor (plan: Current sign):
 * I = (count - offset) x (3.3 V / 4096) / (40 V/V x 5 mOhm).
 */
#ifndef CURSENSE_H
#define CURSENSE_H

#include "board.h"
#include <stdbool.h>
#include <stdint.h>

/* Conversion (plan: Current sensing; DRV 0x06 CSA_GAIN = 11b). */
#define CURSENSE_VREF_V            3.3f      /* VREF+ = VDDA = DRV VREF */
#define CURSENSE_ADC_COUNTS        4096.0f   /* 12-bit */
#define CURSENSE_CSA_GAIN          40.0f
#define CURSENSE_SHUNT_OHM         0.005f
#define CURSENSE_AMPS_PER_COUNT    (CURSENSE_VREF_V / CURSENSE_ADC_COUNTS / (CURSENSE_CSA_GAIN * CURSENSE_SHUNT_OHM))

/** Latest sample, read from the main loop with cursense_snapshot(). */
typedef struct
{
  uint16_t raw[PHASE_COUNT];     /* ADC counts, phase order A, B, C */
  float    amps[PHASE_COUNT];    /* A, positive into the motor */
  uint32_t count;                /* control interrupts since start (heartbeat) */
} cursense_snapshot_t;

/** Noise and level statistics per phase since the last call. */
typedef struct
{
  uint16_t min[PHASE_COUNT];
  uint16_t max[PHASE_COUNT];
  float    mean[PHASE_COUNT];    /* counts */
  float    rms[PHASE_COUNT];     /* standard deviation about the mean, counts */
  uint32_t samples;
  uint32_t isr_mean_cycles;      /* control interrupt duration over the window */
  uint32_t isr_max_cycles;
} cursense_stats_t;

/** Calibrate and enable ADC2 and ADC3 (ADC1 is done by power_init(), which
 *  must run first), arm the three injected triggers and enable the control
 *  interrupt. Returns false if a calibration or enable step fails; *what
 *  names it. */
bool cursense_init(const char **what);

/** Start an offset measurement: wait for the amplifiers to settle, then
 *  average CURSENSE_OFFSET_SAMPLES per phase. Offsets are invalid until it
 *  completes. Call after every DRV configure, with no current flowing. */
void cursense_offset_start(void);

/** Main loop: finish a measurement that the interrupt has completed.
 *  Returns true once per measurement, with the measured means (counts) and
 *  *ok = all three within tolerance of mid-scale. Only in-range offsets
 *  are applied; otherwise they stay invalid. */
bool cursense_offset_poll(float means[PHASE_COUNT], bool *ok);

bool cursense_offsets_valid(void);
void cursense_offsets(float counts[PHASE_COUNT]);

/** Heartbeat: false if the control interrupt count hasn't moved for
 *  CURSENSE_HEARTBEAT_TIMEOUT_MS (now_ms from HAL_GetTick()). */
bool cursense_heartbeat_ok(uint32_t now_ms);

void cursense_snapshot(cursense_snapshot_t *out);
void cursense_stats_take(cursense_stats_t *out);

/** Longest control interrupt so far, in CPU cycles; ADC2/ADC3 results that
 *  weren't ready when ADC1's arrived. */
uint32_t cursense_isr_max_cycles(void);
uint32_t cursense_late_count(void);

#endif /* CURSENSE_H */
