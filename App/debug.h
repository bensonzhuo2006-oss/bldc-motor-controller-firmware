/**
 * @file    debug.h
 * @brief   SWO logging, CPU cycle counter, scope DAC on PA2, timing pin on PA3,
 *          HardFault dump.
 *
 * Logging is main-loop only (plan: Timing, interrupts and data sharing).
 * The scope and timing-pin helpers are register writes, safe in interrupts.
 */
#ifndef DEBUG_H
#define DEBUG_H

#include "main.h"
#include "board.h"
#include <stdbool.h>
#include <stdint.h>

/* HCLK = 170 MHz (plan: TIM1 and control timing, "Clock"). */
#define DEBUG_CPU_HZ             170000000UL
#define DEBUG_CYCLES_PER_US      (DEBUG_CPU_HZ / 1000000UL)

/* DAC3 is 12-bit; 4095 is full scale, about 3.3 V at PA2 via OPAMP1. */
#define DEBUG_SCOPE_FULL_SCALE   4095UL

/** Start the cycle counter and the scope output (DAC3_CH1 -> OPAMP1 -> PA2).
 *  Returns false if the DAC or op-amp failed to start. */
bool debug_init(void);

/** printf-style log line over SWO (ITM port 0); a newline is appended.
 *  Main loop only. Integers only: newlib-nano printf has no float support. */
void debug_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** Capture buffer (Stage 7 onward). push: control interrupt, every sample.
 *  trigger: keeps 512 samples before it, records 512 after, then freezes.
 *  dump_start / dump_tick: main loop, one SWO line per call; the capture
 *  re-arms after the last line. */
void debug_capture_push(uint16_t a, uint16_t b, uint16_t c);
void debug_capture_trigger(void);
bool debug_capture_done(void);
bool debug_capture_dump_start(void);
bool debug_capture_dump_tick(void);

/** Log the reset cause from RCC_CSR, then clear the flags (plan Stage 2). */
void debug_log_reset_cause(void);

/** NMI response, called from NMI_Handler: outputs off, then ENABLE low,
 *  then FLASH_ECCR and SYSCFG_CFGR2 over SWO; halts (debugger) or blinks. */
void debug_nmi_report(void) __attribute__((noreturn));

/** Raw SWO output without printf, usable from fault handlers. */
void debug_put_str(const char *s);
void debug_put_hex32(uint32_t v);

/** Free-running CPU cycle count (DWT CYCCNT), wraps every ~25 s at 170 MHz. */
static inline uint32_t debug_cycles(void)
{
  return DWT->CYCCNT;
}

/** Timing pin PA3. */
static inline void debug_timing_high(void)
{
  BOARD_TIMING_PORT->BSRR = BOARD_TIMING_PIN;
}

static inline void debug_timing_low(void)
{
  BOARD_TIMING_PORT->BRR = BOARD_TIMING_PIN;
}

static inline void debug_timing_toggle(void)
{
  uint32_t odr = BOARD_TIMING_PORT->ODR;
  BOARD_TIMING_PORT->BSRR = ((odr & BOARD_TIMING_PIN) << 16U) | (~odr & BOARD_TIMING_PIN);
}

/** Scope output: 12-bit code (0..4095) to DAC3 CH1, buffered by OPAMP1 on PA2. */
static inline void debug_scope_write(uint32_t code)
{
  DAC3->DHR12R1 = (code > DEBUG_SCOPE_FULL_SCALE) ? DEBUG_SCOPE_FULL_SCALE : code;
}

#endif /* DEBUG_H */
