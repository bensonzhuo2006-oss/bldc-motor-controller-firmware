/**
 * @file    pwm.c
 * @brief   TIM1 PWM, arming and disarming, break handling.
 */
#include "pwm.h"
#include "app_config.h"
#include "board.h"
#include "fault.h"
#include "tim.h"
#include "main.h"
#if BRINGUP_STAGE >= 6
#include "power.h"
#include "drv8323.h"
#endif
#if BRINGUP_STAGE >= 7
#include "cursense.h"
#endif

/* ---- Constants ------------------------------------------------------------ */

/* Dead time: DTG = 17 with tDTS = 1/170 MHz (CKD = 00) = 100 ns
 * (RM0440 TIMx_BDTR DTG[7:0], DTG[7:5] = 0xx: DT = DTG x tDTS). */
#define PWM_DTG_EXPECTED     17U
#define PWM_CCR4_EXPECTED    4200U    /* ADC trigger slightly below ARR (plan Stage 5) */

/* CMS = 01: center-aligned mode 1. */
#define PWM_CMS_CENTER1      TIM_CR1_CMS_0

/* OCxM = 0110: PWM mode 1 (CCMR1/2 OC1M..OC4M low three bits). */
#define PWM_OCM_PWM1         6UL

/* System break sources connected to TIM1 (RM0440 Table 271, SYSCFG_CFGR2):
 * core lockup, SRAM parity error, flash double-ECC error. CSS is always
 * connected; PVD isn't configured. */
#define PWM_SYS_BREAK_SOURCES  (SYSCFG_CFGR2_CLL | SYSCFG_CFGR2_SPL | SYSCFG_CFGR2_ECCL)

/* MOE readback after setting it: a few reads cover the resynchronisation
 * delay (a few APB cycles, RM0440 TIM1 break). */
#define PWM_MOE_READBACK_TRIES 16U

/* Break flags: external/software break and system break. */
#define PWM_BREAK_FLAGS        (TIM_SR_BIF | TIM_SR_SBIF)

/* GPIO MODER field values. */
#define PWM_MODER_OUTPUT     1UL
#define PWM_MODER_AF         2UL

/* Gate-input pins per phase (plan pin map) and their TIM1 CCER enables. */
typedef struct
{
  GPIO_TypeDef *inh_port;
  uint32_t      inh_pin;    /* pin number */
  GPIO_TypeDef *inl_port;
  uint32_t      inl_pin;
  uint32_t      ccer;       /* CCxE | CCxNE */
} pwm_phase_io_t;

static const pwm_phase_io_t s_phase_io[PHASE_COUNT] =
{
  [PHASE_A] = { GPIOA, 10U, GPIOB, 15U, TIM_CCER_CC3E | TIM_CCER_CC3NE },
  [PHASE_B] = { GPIOA,  9U, GPIOB, 14U, TIM_CCER_CC2E | TIM_CCER_CC2NE },
  [PHASE_C] = { GPIOA,  8U, GPIOB, 13U, TIM_CCER_CC1E | TIM_CCER_CC1NE },
};

/* ---- State ---------------------------------------------------------------- */

static volatile bool s_break_event;
static volatile uint32_t s_break_events;

/* Duty command from the main loop, applied by the control interrupt. */
static volatile float s_cmd[PHASE_COUNT] = { PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD };

/* ---- Helpers -------------------------------------------------------------- */

static inline uint32_t pwm_duty_to_ccr(float d)
{
  if (!(d >= 0.0f))   /* also catches NaN */
  {
    d = 0.0f;
  }
  else if (d > PWM_DUTY_MAX)
  {
    d = PWM_DUTY_MAX;
  }
  return (uint32_t)(d * (float)PWM_ARR + 0.5f);
}

static void pwm_pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
  port->MODER = (port->MODER & ~(3UL << (2U * pin))) | (mode << (2U * pin));
}

static uint32_t pwm_oc_mode(uint32_t ccmr, uint32_t shift_lo, uint32_t bit3)
{
  return ((ccmr >> shift_lo) & 7UL) | (((ccmr & bit3) != 0U) ? 8UL : 0UL);
}

/* ---- API ------------------------------------------------------------------ */

bool pwm_init(const char **what)
{
  /* Pass/fail is *what == ok, by pointer: a mismatch message can start with
   * the same letters ("OCxM ..."). */
  static const char ok[] = "OK";
  *what = ok;

  if (TIM1->ARR != PWM_ARR)                                   { *what = "ARR"; }
  else if ((TIM1->CR1 & TIM_CR1_CMS) != PWM_CMS_CENTER1)      { *what = "CMS"; }
  else if ((TIM1->CR1 & TIM_CR1_CKD) != 0U)                   { *what = "CKD"; }
  else if (TIM1->PSC != 0U)                                   { *what = "PSC"; }
  else if ((TIM1->BDTR & TIM_BDTR_DTG) != PWM_DTG_EXPECTED)   { *what = "DTG"; }
  else if ((TIM1->BDTR & TIM_BDTR_BKE) == 0U)                 { *what = "BKE"; }
  else if ((TIM1->BDTR & TIM_BDTR_BKP) != 0U)                 { *what = "BKP (want active low)"; }
  else if ((TIM1->BDTR & TIM_BDTR_AOE) != 0U)                 { *what = "AOE"; }
  else if ((TIM1->CR2 & TIM_CR2_MMS2) != TIM_TRGO2_OC4REF)    { *what = "MMS2 (want OC4REF)"; }
  else if (TIM1->CCR4 != PWM_CCR4_EXPECTED)                   { *what = "CCR4"; }
  else if ((pwm_oc_mode(TIM1->CCMR1, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1M_3) != PWM_OCM_PWM1) ||
           (pwm_oc_mode(TIM1->CCMR1, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2M_3) != PWM_OCM_PWM1) ||
           (pwm_oc_mode(TIM1->CCMR2, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3M_3) != PWM_OCM_PWM1) ||
           (pwm_oc_mode(TIM1->CCMR2, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4M_3) != PWM_OCM_PWM1))
  {
    *what = "OCxM (want PWM mode 1)";
  }

  /* Debugger freeze: a halted core stops TIM1 and disables its outputs as
   * if MOE were cleared; with OSSI = 1 they go to their low idle level
   * (RM0440 DBGMCU_APB2FZR DBG_TIM1_STOP; TIM1 "Debug mode"). */
  DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;

  /* Connect the system break sources, so a core lockup (fault inside the
   * HardFault handler), SRAM parity or flash double-ECC error turns the
   * outputs off in hardware. Set once; cleared only by a system reset.
   * SPF is write-1-to-clear, so it's kept out of the write. */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  SYSCFG->CFGR2 = (SYSCFG->CFGR2 & ~SYSCFG_CFGR2_SPF) | PWM_SYS_BREAK_SOURCES;
  if (((SYSCFG->CFGR2 & PWM_SYS_BREAK_SOURCES) != PWM_SYS_BREAK_SOURCES) && (*what == ok))
  {
    *what = "SYSCFG_CFGR2 system break";
  }

  /* Outputs safe and off (MOE = 0, OSSI = OSSR = 1, idle low,
   * CCxE = CCxNE = 1 for channels 1-3), zero command. */
  board_tim1_outputs_safe();
  pwm_zero_command();

  /* Break flag from any earlier nFAULT pulse (wake/sleep in Stages 3-4)
   * cleared. The break interrupt stays off while disarmed: motor_can_arm()
   * enables it, motor_disarm() and the break callback turn it off. BIF can't
   * be cleared while the break input is active (RM0440 TIMx_SR BIF), so a
   * held-low nFAULT with BIE = 1 would re-enter the interrupt forever, and
   * the DRV's wake/sleep nFAULT pulses would latch faults while disarmed. */
  TIM1->SR = ~PWM_BREAK_FLAGS;
  TIM1->DIER &= ~TIM_DIER_BIE;

  TIM1->CR1 |= TIM_CR1_CEN;         /* counter runs; outputs stay off (MOE = 0) */

  return *what == ok;
}

bool motor_can_arm(const char **reason)
{
  uint32_t enable_high = BOARD_DRV_ENABLE_PORT->ODR & BOARD_DRV_ENABLE_PIN;
  bool nfault_low = (BOARD_DRV_NFAULT_PORT->IDR & BOARD_DRV_NFAULT_PIN) == 0U;

  if (pwm_is_armed())                                           { *reason = "already armed"; return false; }
  if (fault_active())                                           { *reason = "fault latched"; return false; }
  if ((DBGMCU->APB2FZ & DBGMCU_APB2FZ_DBG_TIM1_STOP) == 0U)     { *reason = "debug freeze off"; return false; }
  if ((TIM1->BDTR & (TIM_BDTR_OSSI | TIM_BDTR_OSSR)) != (TIM_BDTR_OSSI | TIM_BDTR_OSSR))
  {
    *reason = "OSSI/OSSR not set";
    return false;
  }
  if ((TIM1->BDTR & TIM_BDTR_AOE) != 0U)                        { *reason = "AOE set"; return false; }
  if ((TIM1->CR2 & BOARD_TIM1_OIS_MASK) != 0U)                  { *reason = "idle levels not low"; return false; }
  if ((TIM1->CR1 & TIM_CR1_CEN) == 0U)                          { *reason = "timer not running"; return false; }
  if ((TIM1->CCR1 != TIM1->CCR2) || (TIM1->CCR2 != TIM1->CCR3)) { *reason = "command not zero"; return false; }
  if (nfault_low)                                               { *reason = "nFAULT low (break input active)"; return false; }

#if BRINGUP_STAGE == 5
  /* Stage 5 switches only into a sleeping DRV. */
  if (enable_high != 0U)                                        { *reason = "DRV ENABLE high (Stage 5 needs it low)"; return false; }
#else
  (void)enable_high;
#endif
#if BRINGUP_STAGE >= 6
  /* Plan Stage 6 arming rules. */
  if (!power_vm_ok())                                           { *reason = "no VM_OK"; return false; }
  if (!drv8323_is_configured())                                 { *reason = "DRV not configured"; return false; }
#endif
#if BRINGUP_STAGE >= 7
  /* Plan Stage 7 arming rules: ADC offsets valid, control heartbeat alive. */
  if (!cursense_offsets_valid())                                { *reason = "current offsets not valid"; return false; }
  if (!cursense_heartbeat_ok(HAL_GetTick()))                    { *reason = "control interrupt not running"; return false; }
#endif

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  TIM1->SR = ~PWM_BREAK_FLAGS;      /* break flags cleared before arming */
  TIM1->DIER |= TIM_DIER_BIE;       /* break events latched while armed */
  TIM1->BDTR |= TIM_BDTR_MOE;
  __set_PRIMASK(primask);

  /* MOE is written on the asynchronous path: a read straight after the
   * write can still show 0 (RM0440 TIM1 break: "a delay must be inserted
   * (dummy instruction) before reading it correctly"). Poll briefly; if it
   * really didn't set, force the outputs off so the state is never "refused
   * but armed". */
  bool on = false;
  for (uint32_t i = 0U; (i < PWM_MOE_READBACK_TRIES) && !on; i++)
  {
    on = pwm_is_armed();
  }
  if (!on)
  {
    motor_disarm();
    *reason = "MOE did not set";
    return false;
  }
  *reason = "OK";
  return true;
}

void motor_disarm(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  BOARD_DRV_ENABLE_PORT->BRR = BOARD_DRV_ENABLE_PIN;
  TIM1->DIER &= ~TIM_DIER_BIE;      /* no break interrupts while disarmed */
}

bool pwm_is_armed(void)
{
  return (TIM1->BDTR & TIM_BDTR_MOE) != 0U;
}

void pwm_set_duty(float a, float b, float c)
{
  BOARD_CCR_PHASE_A = pwm_duty_to_ccr(a);
  BOARD_CCR_PHASE_B = pwm_duty_to_ccr(b);
  BOARD_CCR_PHASE_C = pwm_duty_to_ccr(c);
}

bool pwm_phase_output(board_phase_t ph, bool on)
{
  const pwm_phase_io_t *io;
  if (pwm_is_armed() || (ph >= PHASE_COUNT))
  {
    return false;
  }
  io = &s_phase_io[ph];
  if (on)
  {
    /* Channel enabled first (driven low by OSSI while disarmed), then the
     * pins handed back to TIM1. */
    TIM1->CCER |= io->ccer;
    pwm_pin_mode(io->inh_port, io->inh_pin, PWM_MODER_AF);
    pwm_pin_mode(io->inl_port, io->inl_pin, PWM_MODER_AF);
  }
  else
  {
    /* Pins already low (disarmed); latch low in ODR, switch to GPIO output,
     * then disable the channel. INH = INL = 0: both FETs off (DRV 6x mode,
     * Table 8-2), driven, not left to the DRV pull-downs. */
    io->inh_port->BRR = 1UL << io->inh_pin;
    io->inl_port->BRR = 1UL << io->inl_pin;
    pwm_pin_mode(io->inh_port, io->inh_pin, PWM_MODER_OUTPUT);
    pwm_pin_mode(io->inl_port, io->inl_pin, PWM_MODER_OUTPUT);
    TIM1->CCER &= ~io->ccer;
  }
  return true;
}

void pwm_zero_command(void)
{
  if (pwm_is_armed())
  {
    return;   /* UG would restart the PWM period mid-cycle */
  }
  pwm_set_duty(PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD);
  TIM1->EGR = TIM_EGR_UG;           /* load the preloaded CCRs now */
}

void pwm_command_duty(float a, float b, float c)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_cmd[PHASE_A] = a;
  s_cmd[PHASE_B] = b;
  s_cmd[PHASE_C] = c;
  __set_PRIMASK(primask);
}

void pwm_isr_update(void)
{
  if (pwm_is_armed())
  {
    pwm_set_duty(s_cmd[PHASE_A], s_cmd[PHASE_B], s_cmd[PHASE_C]);
  }
}

void pwm_get_duty_permille(uint32_t *a, uint32_t *b, uint32_t *c)
{
  *a = (BOARD_CCR_PHASE_A * 1000UL + PWM_ARR / 2U) / PWM_ARR;
  *b = (BOARD_CCR_PHASE_B * 1000UL + PWM_ARR / 2U) / PWM_ARR;
  *c = (BOARD_CCR_PHASE_C * 1000UL + PWM_ARR / 2U) / PWM_ARR;
}

void pwm_software_break(void)
{
  TIM1->EGR = TIM_EGR_BG;
}

bool pwm_take_break_event(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool ev = s_break_event;
  s_break_event = false;
  __set_PRIMASK(primask);
  return ev;
}

uint32_t pwm_break_events(void)
{
  return s_break_events;
}

/* TIM1 break interrupt (HAL_TIM_IRQHandler from TIM1_BRK_TIM15_IRQHandler
 * clears BIF and calls this). Outputs are already off in hardware; latch
 * only (plan: TIM1 break interrupt, "Must not re-arm anything"). BIE off,
 * so a break input that stays active can't re-enter this interrupt. */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    TIM1->DIER &= ~TIM_DIER_BIE;
    s_break_events++;
    s_break_event = true;
  }
}
