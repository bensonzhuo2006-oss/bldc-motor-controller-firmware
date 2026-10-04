/**
 * @file    board.h
 * @brief   Pin definitions, PWM phase mapping and the safe-pin boot routine.
 *
 * Pin assignments: plan "Pin mappings", checked against the STM32 schematic
 * sheet and DS13122 Table 12 (Bring-up log, 2026-10-04).
 */
#ifndef BOARD_H
#define BOARD_H

#include "main.h"
#include <stdint.h>

/* ---- GPIO ---------------------------------------------------------------- */
#define BOARD_LED_PORT          LED_STATUS_GPIO_Port      /* PC3 -> Q7 -> LED4 */
#define BOARD_LED_PIN           LED_STATUS_Pin
#define BOARD_TIMING_PORT       Timing_GPIO_Port          /* PA3 on J15 */
#define BOARD_TIMING_PIN        Timing_Pin
#define BOARD_DRV_ENABLE_PORT   ENABLE_GPIO_Port          /* PC8, low at boot */
#define BOARD_DRV_ENABLE_PIN    ENABLE_Pin
#define BOARD_DRV_CAL_PORT      CAL_GPIO_Port             /* PC9, held low */
#define BOARD_DRV_CAL_PIN       CAL_Pin
#define BOARD_DRV_NSCS_PORT     GPIOD                     /* PD2, R38 10k pull-up */
#define BOARD_DRV_NSCS_PIN      GPIO_PIN_2
#define BOARD_ENC_CS_PORT       SPI1_CS_ENCODER_GPIO_Port /* PA4, R39 10k pull-up */
#define BOARD_ENC_CS_PIN        SPI1_CS_ENCODER_Pin
#define BOARD_MOTOR_EN_PORT     MOTOR_EN_GPIO_Port        /* PC1, R16/R17 divider */
#define BOARD_MOTOR_EN_PIN      MOTOR_EN_Pin

/* PD2 pin number, for the MODER readback in the safe-pin check. */
#define BOARD_DRV_NSCS_PIN_NUM  2U

/* ---- PWM phase mapping (plan: phase A on TIM1 CH3, C on CH1) -------------- */
typedef enum
{
  PHASE_A = 0,
  PHASE_B = 1,
  PHASE_C = 2,
  PHASE_COUNT = 3
} board_phase_t;

#define BOARD_CCR_PHASE_A       (TIM1->CCR3)
#define BOARD_CCR_PHASE_B       (TIM1->CCR2)
#define BOARD_CCR_PHASE_C       (TIM1->CCR1)

/* Gate inputs: INHC/INHB/INHA on PA8/PA9/PA10, INLC/INLB/INLA on PB13/14/15. */
#define BOARD_INH_PINS_MASK     (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)
#define BOARD_INL_PINS_MASK     (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

/* TIM1 CCER/CR2 bits for channels 1-3 and their complements. */
#define BOARD_TIM1_CCER_EN_MASK   (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                                   TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                                   TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define BOARD_TIM1_CCER_POL_MASK  (TIM_CCER_CC1P | TIM_CCER_CC1NP | \
                                   TIM_CCER_CC2P | TIM_CCER_CC2NP | \
                                   TIM_CCER_CC3P | TIM_CCER_CC3NP)
#define BOARD_TIM1_OIS_MASK       (TIM_CR2_OIS1 | TIM_CR2_OIS1N | \
                                   TIM_CR2_OIS2 | TIM_CR2_OIS2N | \
                                   TIM_CR2_OIS3 | TIM_CR2_OIS3N)

/* Violation bits returned by board_safe_pins_check(). */
#define BOARD_SAFE_ERR_ENABLE_HIGH   (1UL << 0)
#define BOARD_SAFE_ERR_CAL_HIGH      (1UL << 1)
#define BOARD_SAFE_ERR_MOE_SET       (1UL << 2)
#define BOARD_SAFE_ERR_OSSI_CLEAR    (1UL << 3)
#define BOARD_SAFE_ERR_CCER          (1UL << 4)
#define BOARD_SAFE_ERR_SPI3_ENABLED  (1UL << 5)
#define BOARD_SAFE_ERR_NSCS_DRIVEN   (1UL << 6)
#define BOARD_SAFE_ERR_PWM_PIN_HIGH  (1UL << 7)

/**
 * Put every pin that reaches the DRV8323 into its safe state.
 *
 * - TIM1: MOE and AOE cleared, OSSI = OSSR = 1, idle levels (OISx) low,
 *   polarities active-high, CCxE = CCxNE = 1. With MOE = 0 and OSSI = 1 this
 *   drives all six gate inputs to their inactive level, low (RM0440 Table 282).
 *   They are driven rather than left Hi-Z because the DRV's 100 kOhm input
 *   pull-downs can't be relied on while the DRV is unpowered. MOE stays 0;
 *   only motor_can_arm() may set it (from Stage 6).
 * - DRV ENABLE and CAL low, after the outputs (plan: disarm order).
 * - SPI3 disabled, so SCLK/SDI aren't driven into the unpowered DRV
 *   (RM0440 SPI: SCK isn't driven while SPE = 0). PD2 stays analog (undriven).
 */
static inline void board_safe_pins(void)
{
  TIM1->BDTR &= ~(TIM_BDTR_MOE | TIM_BDTR_AOE);
  TIM1->BDTR |= (TIM_BDTR_OSSI | TIM_BDTR_OSSR);
  TIM1->CR2 &= ~BOARD_TIM1_OIS_MASK;
  TIM1->CCER &= ~BOARD_TIM1_CCER_POL_MASK;
  TIM1->CCER |= BOARD_TIM1_CCER_EN_MASK;

  BOARD_DRV_ENABLE_PORT->BRR = BOARD_DRV_ENABLE_PIN;
  BOARD_DRV_CAL_PORT->BRR = BOARD_DRV_CAL_PIN;

  SPI3->CR1 &= ~SPI_CR1_SPE;
}

/** Six gate-input pin levels, bit 0..5 = INHA, INHB, INHC, INLA, INLB, INLC. */
static inline uint32_t board_pwm_pins_read(void)
{
  uint32_t a = GPIOA->IDR;
  uint32_t b = GPIOB->IDR;
  return (((a >> 10) & 1UL) << 0) |   /* INHA PA10 */
         (((a >> 9)  & 1UL) << 1) |   /* INHB PA9  */
         (((a >> 8)  & 1UL) << 2) |   /* INHC PA8  */
         (((b >> 15) & 1UL) << 3) |   /* INLA PB15 */
         (((b >> 14) & 1UL) << 4) |   /* INLB PB14 */
         (((b >> 13) & 1UL) << 5);    /* INLC PB13 */
}

/** Read back the safe state; returns 0 if safe, else BOARD_SAFE_ERR_* bits. */
static inline uint32_t board_safe_pins_check(void)
{
  uint32_t err = 0U;
  uint32_t nscs_mode = (BOARD_DRV_NSCS_PORT->MODER >> (2U * BOARD_DRV_NSCS_PIN_NUM)) & 3UL;

  if ((BOARD_DRV_ENABLE_PORT->ODR & BOARD_DRV_ENABLE_PIN) != 0U) { err |= BOARD_SAFE_ERR_ENABLE_HIGH; }
  if ((BOARD_DRV_CAL_PORT->ODR & BOARD_DRV_CAL_PIN) != 0U)       { err |= BOARD_SAFE_ERR_CAL_HIGH; }
  if ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)                         { err |= BOARD_SAFE_ERR_MOE_SET; }
  if ((TIM1->BDTR & TIM_BDTR_OSSI) == 0U)                        { err |= BOARD_SAFE_ERR_OSSI_CLEAR; }
  if ((TIM1->CCER & (BOARD_TIM1_CCER_EN_MASK | BOARD_TIM1_CCER_POL_MASK)) != BOARD_TIM1_CCER_EN_MASK)
  {
    err |= BOARD_SAFE_ERR_CCER;
  }
  if ((SPI3->CR1 & SPI_CR1_SPE) != 0U)                           { err |= BOARD_SAFE_ERR_SPI3_ENABLED; }
  if (nscs_mode != 3UL)                                          { err |= BOARD_SAFE_ERR_NSCS_DRIVEN; }
  if (board_pwm_pins_read() != 0U)                               { err |= BOARD_SAFE_ERR_PWM_PIN_HIGH; }
  return err;
}

#endif /* BOARD_H */
