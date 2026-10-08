/**
 * @file    fault.h
 * @brief   Latched fault codes (plan: Firmware conventions, "Faults are an
 *          enum of codes, latched until an explicit clear, with the first
 *          fault's code and context logged over SWO"; Fault latch and
 *          logging, Stage 4 onward).
 *
 * A fault never clears itself. fault_clear() is the only way out, called
 * when the developer sets g_fault_clear_request from the debugger.
 * Main loop only.
 */
#ifndef FAULT_H
#define FAULT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  FAULT_NONE = 0,
  FAULT_DRV_NOT_PRESENT,     /* 0x03 didn't read 0x3FF after wake */
  FAULT_DRV_CONFIG_WRITE,    /* a configuration write didn't verify */
  FAULT_DRV_LOCK_TEST,       /* a write got through after locking */
  FAULT_DRV_CONFIG_MISMATCH, /* periodic readback differs from the shadow copy */
  FAULT_DRV_NFAULT,          /* nFAULT went low outside the wake/sleep window */
  FAULT_DRV_FAULT_BITS,      /* fault status registers not clear */
  FAULT_DRV_SPI,             /* SPI3 transfer timed out */
  FAULT_DRV_WAKE,            /* nFAULT still low after the wake time */
  FAULT_PWM_BREAK,           /* TIM1 break (nFAULT on BKIN, or software break) */
  FAULT_CTRL_HEARTBEAT,      /* control interrupt count stopped (Stage 7) */
  FAULT_CUR_OFFSET,          /* current-sense offset out of range (Stage 7) */
  FAULT_COUNT
} fault_code_t;

/** Latch a fault. The first one is kept with its context; later ones are
 *  counted. Returns true if this call latched the first fault. */
bool fault_raise(fault_code_t code, uint32_t ctx1, uint32_t ctx2);

bool fault_active(void);
fault_code_t fault_first(void);
void fault_first_context(uint32_t *ctx1, uint32_t *ctx2, uint32_t *tick_ms);
uint32_t fault_count(void);

/** Explicit clear: the only way a latched fault goes away. */
void fault_clear(void);

const char *fault_name(fault_code_t code);

/** Set to 1 from the debugger (Watch or Debug Console) to request a clear. */
extern volatile uint32_t g_fault_clear_request;

#endif /* FAULT_H */
