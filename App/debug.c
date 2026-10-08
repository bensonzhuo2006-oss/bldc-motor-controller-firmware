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

/* ---- Capture buffer ------------------------------------------------------- */

/* Every control-interrupt sample (three ADC counts) in a ring; a trigger
 * keeps the half before it and fills the half after it, then freezes
 * (plan Stage 7: RAM capture buffer around a trigger). 1024 x 3 x 2 bytes
 * = 6 KB, 51.2 ms at 20 kHz. Power of two, so the index wraps with a mask. */
#define DEBUG_CAP_SAMPLES   1024U
#define DEBUG_CAP_MASK      (DEBUG_CAP_SAMPLES - 1U)
#define DEBUG_CAP_POST      (DEBUG_CAP_SAMPLES / 2U)

typedef enum
{
  CAP_RUN = 0,
  CAP_TRIGGERED,
  CAP_DONE
} cap_state_t;

static volatile uint16_t s_cap[DEBUG_CAP_SAMPLES][3];
static volatile uint32_t s_cap_idx;
static volatile uint32_t s_cap_post;
static volatile cap_state_t s_cap_state;
static uint32_t s_cap_dump_left;   /* main loop only */
static uint32_t s_cap_dump_i;

void debug_capture_push(uint16_t a, uint16_t b, uint16_t c)
{
  if (s_cap_state == CAP_DONE)
  {
    return;
  }
  uint32_t i = s_cap_idx;
  s_cap[i][0] = a;
  s_cap[i][1] = b;
  s_cap[i][2] = c;
  s_cap_idx = (i + 1U) & DEBUG_CAP_MASK;
  if ((s_cap_state == CAP_TRIGGERED) && (--s_cap_post == 0U))
  {
    s_cap_state = CAP_DONE;
  }
}

void debug_capture_trigger(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_cap_state == CAP_RUN)
  {
    s_cap_post = DEBUG_CAP_POST;
    s_cap_state = CAP_TRIGGERED;
  }
  __set_PRIMASK(primask);
}

bool debug_capture_done(void)
{
  return s_cap_state == CAP_DONE;
}

bool debug_capture_dump_start(void)
{
  if ((s_cap_state != CAP_DONE) || (s_cap_dump_left != 0U))
  {
    return false;
  }
  s_cap_dump_i = s_cap_idx;   /* oldest sample */
  s_cap_dump_left = DEBUG_CAP_SAMPLES;
  debug_log("CAP begin n=%u trigger_at=%u (t = n x 50 us; counts A B C)",
            DEBUG_CAP_SAMPLES, DEBUG_CAP_SAMPLES - DEBUG_CAP_POST);
  return true;
}

bool debug_capture_dump_tick(void)
{
  if (s_cap_dump_left == 0U)
  {
    return false;
  }
  uint32_t n = DEBUG_CAP_SAMPLES - s_cap_dump_left;
  uint32_t i = s_cap_dump_i;
  debug_log("CAP %lu %u %u %u", n, (unsigned)s_cap[i][0], (unsigned)s_cap[i][1], (unsigned)s_cap[i][2]);
  s_cap_dump_i = (i + 1U) & DEBUG_CAP_MASK;
  if (--s_cap_dump_left == 0U)
  {
    debug_log("CAP end; capture re-armed");
    s_cap_state = CAP_RUN;
  }
  return true;
}

/* ---- Reset cause ---------------------------------------------------------- */

/* RCC_CSR reset flags (RM0440 RCC_CSR). Every reset also pulses NRST, so
 * PINRSTF is set alongside the real cause; it's reported as the cause only
 * when nothing else is set. A power-on sets BORRSTF (no separate POR flag). */
void debug_log_reset_cause(void)
{
  uint32_t csr = RCC->CSR;
  const char *cause;

  if ((csr & RCC_CSR_BORRSTF) != 0U)       { cause = "POWER_ON_OR_BROWNOUT"; }
  else if ((csr & RCC_CSR_IWDGRSTF) != 0U) { cause = "IWDG"; }
  else if ((csr & RCC_CSR_WWDGRSTF) != 0U) { cause = "WWDG"; }
  else if ((csr & RCC_CSR_SFTRSTF) != 0U)  { cause = "SOFTWARE"; }
  else if ((csr & RCC_CSR_LPWRRSTF) != 0U) { cause = "LOW_POWER"; }
  else if ((csr & RCC_CSR_OBLRSTF) != 0U)  { cause = "OPTION_BYTE"; }
  else if ((csr & RCC_CSR_PINRSTF) != 0U)  { cause = "PIN"; }
  else                                     { cause = "NONE"; }

  debug_log("RESET cause=%s csr=0x%08lX", cause, csr);
  RCC->CSR |= RCC_CSR_RMVF;
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

/* ---- HardFault and NMI dumps ---------------------------------------------- */

/* Make the power stage safe first, in disarm order: outputs off, then
 * DRV ENABLE low (plan: Hard rules, disarm order). Register writes only,
 * so it works whatever state the crash left behind. */
static void debug_fault_safe(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  BOARD_DRV_ENABLE_PORT->BRR = BOARD_DRV_ENABLE_PIN;
}

static void debug_fault_halt(void) __attribute__((noreturn));

static void debug_fault_halt(void)
{
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

/* Called from HardFault_Handler with the stacked exception frame
 * (r0, r1, r2, r3, r12, lr, pc, xpsr) and the EXC_RETURN value. */
void debug_fault_report(const uint32_t *frame, uint32_t exc_return) __attribute__((used, noreturn));

void debug_fault_report(const uint32_t *frame, uint32_t exc_return)
{
  debug_fault_safe();

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

  debug_fault_halt();
}

/* NMI sources on this board: flash double-ECC error (FLASH_ECCR ECCD) and
 * SRAM parity error (SYSCFG_CFGR2 SPF); the third source, HSE CSS, isn't
 * enabled (RM0440 vector table, NMI). Both are also TIM1 system break
 * sources (pwm_init), so the outputs are already off in hardware; this
 * covers ENABLE and the report. */
void debug_nmi_report(void)
{
  debug_fault_safe();

  debug_put_str("\nFAULT NMI flash_eccr=");
  debug_put_hex32(FLASH->ECCR);
  debug_put_str(" syscfg_cfgr2=");
  debug_put_hex32(SYSCFG->CFGR2);
  debug_put_str("\n");

  debug_fault_halt();
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
