/**
 * @file    debug.c
 * @brief   SWO logging, cycle counter, scope output and HardFault dump.
 */
#include "debug.h"
#include "dac.h"
#include "opamp.h"
#include <stdarg.h>
#include <stdio.h>

/* One log line, including the appended newline; longer lines are truncated. */
#define DEBUG_LOG_LINE_MAX  128U

/* Fault-loop LED half period in busy-wait iterations (a few Hz at -O0;
 * DWT may not be running if the fault came before debug_init()). */
#define DEBUG_FAULT_BLINK_LOOPS  2000000UL

/* ---- SWO output ----------------------------------------------------------- */

/* ITM_SendChar() returns at once when the debugger hasn't enabled ITM port 0,
 * so logging costs almost nothing with no probe attached. */
void debug_put_str(const char *s)
{
  while (*s != '\0')
  {
    (void)ITM_SendChar((uint32_t)(uint8_t)*s);
    s++;
  }
}

void debug_put_hex32(uint32_t v)
{
  static const char hex[] = "0123456789ABCDEF";
  debug_put_str("0x");
  for (int shift = 28; shift >= 0; shift -= 4)
  {
    (void)ITM_SendChar((uint32_t)hex[(v >> (uint32_t)shift) & 0xFU]);
  }
}

void debug_log(const char *fmt, ...)
{
  static char line[DEBUG_LOG_LINE_MAX];
  va_list args;

  va_start(args, fmt);
  (void)vsnprintf(line, sizeof(line) - 1U, fmt, args);
  va_end(args);

  debug_put_str(line);
  (void)ITM_SendChar('\n');
}

/* ---- Init ----------------------------------------------------------------- */

bool debug_init(void)
{
  bool ok = true;

  /* DWT cycle counter (ARMv7-M: DEMCR.TRCENA gates the DWT). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Scope output: DAC3 CH1 (internal) -> OPAMP1 follower -> PA2. */
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
  {
    ok = false;
  }
  if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_1) != HAL_OK)
  {
    ok = false;
  }
  debug_scope_write(0U);

  return ok;
}

/* ---- HardFault dump ------------------------------------------------------- */

/* Called from HardFault_Handler with the stacked exception frame
 * (r0, r1, r2, r3, r12, lr, pc, xpsr) and the EXC_RETURN value. */
void debug_fault_report(const uint32_t *frame, uint32_t exc_return) __attribute__((used, noreturn));

void debug_fault_report(const uint32_t *frame, uint32_t exc_return)
{
  /* Make the power stage safe first, in disarm order: outputs off, then
   * DRV ENABLE low (plan: Hard rules, disarm order). */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  BOARD_DRV_ENABLE_PORT->BRR = BOARD_DRV_ENABLE_PIN;

  debug_put_str("\nFAULT HardFault\nFAULT pc=");
  debug_put_hex32(frame[6]);
  debug_put_str(" lr=");
  debug_put_hex32(frame[5]);
  debug_put_str(" psr=");
  debug_put_hex32(frame[7]);
  debug_put_str("\nFAULT r0=");
  debug_put_hex32(frame[0]);
  debug_put_str(" r1=");
  debug_put_hex32(frame[1]);
  debug_put_str(" r2=");
  debug_put_hex32(frame[2]);
  debug_put_str(" r3=");
  debug_put_hex32(frame[3]);
  debug_put_str(" r12=");
  debug_put_hex32(frame[4]);
  debug_put_str("\nFAULT cfsr=");
  debug_put_hex32(SCB->CFSR);
  debug_put_str(" hfsr=");
  debug_put_hex32(SCB->HFSR);
  debug_put_str(" mmfar=");
  debug_put_hex32(SCB->MMFAR);
  debug_put_str(" bfar=");
  debug_put_hex32(SCB->BFAR);
  debug_put_str(" exc_ret=");
  debug_put_hex32(exc_return);
  debug_put_str("\n");

  /* With a debugger attached, stop here so the state can be inspected.
   * Without one, BKPT would escalate to lockup, so it's skipped. */
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    __BKPT(0);
  }

  /* Fast LED blink marks a fault without a debugger. */
  for (;;)
  {
    uint32_t odr = BOARD_LED_PORT->ODR;
    BOARD_LED_PORT->BSRR = ((odr & BOARD_LED_PIN) << 16U) | (~odr & BOARD_LED_PIN);
    for (volatile uint32_t i = 0U; i < DEBUG_FAULT_BLINK_LOOPS; i++)
    {
    }
  }
}

/* Replaces the CubeMX-generated handler (requires "Generate IRQ handler"
 * unticked for Hard fault in CubeMX NVIC > Code generation). Naked, so no
 * prologue moves the stack before the exception frame is located. */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile(
    "tst   lr, #4              \n"
    "ite   eq                  \n"
    "mrseq r0, msp             \n"
    "mrsne r0, psp             \n"
    "mov   r1, lr              \n"
    "b     debug_fault_report  \n");
}
