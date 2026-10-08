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

/* ---- Constants ------------------------------------------------------------ */

/* Dead time: DTG = 17 with tDTS = 1/170 MHz (CKD = 00) = 100 ns
 * (RM0440 TIMx_BDTR DTG[7:0], DTG[7:5] = 0xx: DT = DTG x tDTS). */
#define PWM_DTG_EXPECTED     17U
#define PWM_CCR4_EXPECTED    4200U    /* ADC trigger slightly below ARR (plan Stage 5) */

/* CMS = 01: center-aligned mode 1. */
#define PWM_CMS_CENTER1      TIM_CR1_CMS_0

/* OCxM = 0110: PWM mode 1 (CCMR1/2 OC1M..OC4M low three bits). */
#define PWM_OCM_PWM1         6UL

/* ---- State ---------------------------------------------------------------- */

static volatile bool s_break_event;
static volatile uint32_t s_break_events;

/* ---- Helpers -------------------------------------------------------------- */

static inline uint32_t pwm_duty_to_ccr(float d)
{
  if (d < 0.0f)
  {
    d = 0.0f;
  }
  else if (d > PWM_DUTY_MAX)
  {
    d = PWM_DUTY_MAX;
  }
  return (uint32_t)(d * (float)PWM_ARR + 0.5f);
}

static uint32_t pwm_oc_mode(uint32_t ccmr, uint32_t shift_lo, uint32_t bit3)
{
  return ((ccmr >> shift_lo) & 7UL) | (((ccmr & bit3) != 0U) ? 8UL : 0UL);
}

/* ---- API ------------------------------------------------------------------ */

bool pwm_init(const char **what)
{
  *what = "OK";

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

  /* Outputs safe and off (MOE = 0, OSSI = OSSR = 1, idle low,
   * CCxE = CCxNE = 1 for channels 1-3), zero command. */
  board_tim1_outputs_safe();
  pwm_set_duty(PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD);
  TIM1->EGR = TIM_EGR_UG;           /* load the preloaded CCRs */

  /* Break flag from any earlier nFAULT pulse (wake/sleep in Stages 3-4)
   * cleared. The break interrupt stays off while disarmed: motor_can_arm()
   * enables it, motor_disarm() and the break callback turn it off. BIF can't
   * be cleared while the break input is active (RM0440 TIMx_SR BIF), so a
   * held-low nFAULT with BIE = 1 would re-enter the interrupt forever, and
   * the DRV's wake/sleep nFAULT pulses would latch faults while disarmed. */
  TIM1->SR = ~TIM_SR_BIF;
  TIM1->DIER &= ~TIM_DIER_BIE;

  TIM1->CR1 |= TIM_CR1_CEN;         /* counter runs; outputs stay off (MOE = 0) */

  return (*what)[0] == 'O';
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

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  TIM1->SR = ~TIM_SR_BIF;           /* break flag cleared before arming */
  TIM1->DIER |= TIM_DIER_BIE;       /* break events latched while armed */
  TIM1->BDTR |= TIM_BDTR_MOE;
  __set_PRIMASK(primask);
  if (!pwm_is_armed())
  {
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
