/**
 * @file    pwm.h
 * @brief   TIM1 three-phase PWM: setup checks, arming (the only path that
 *          sets MOE), disarm, duty writes, break handling (plan: Stage 5;
 *          Hard rules).
 *
 * TIM1 (CubeMX): center-aligned mode 1, ARR = 4250 -> 20 kHz at 170 MHz,
 * PWM mode 1 (high side on while CNT < CCR), complementary outputs, dead
 * time DTG = 17 -> 100 ns, OSSI = OSSR = 1, idle levels low, AOE = 0,
 * break input PB10 (nFAULT) active low, TRGO2 = OC4REF with CCR4 = 4200.
 * Phase A on CH3, B on CH2, C on CH1 (board.h).
 */
#ifndef PWM_H
#define PWM_H

#include "board.h"
#include <stdbool.h>
#include <stdint.h>

#define PWM_ARR             4250U    /* plan: TIM1 and control timing */
#define PWM_DUTY_MAX        0.93f    /* plan: duty cap for the current-sample window */
#define PWM_DUTY_ZERO_CMD   0.5f     /* equal duties = zero voltage vector */

/** Check TIM1 against the plan, set the debugger freeze, put the outputs
 *  in their safe off state (MOE = 0, low), set 50 % duties and start the
 *  counter. Returns false if TIM1 isn't configured as the plan requires;
 *  *what names the first mismatch. */
bool pwm_init(const char **what);

/** The only function that sets MOE. Checks the arming rules for the
 *  current stage; arms only from a zero command. Returns false with
 *  *reason if any rule fails. */
bool motor_can_arm(const char **reason);

/** Disarm: outputs off (MOE = 0) first, then DRV ENABLE low, within the
 *  DRV's 40 us window. With the hardware break, the only normal path that
 *  clears MOE. Safe to call from an interrupt. */
void motor_disarm(void);

bool pwm_is_armed(void);

/** Duty per phase, 0..1, clamped to [0, PWM_DUTY_MAX]. Register writes
 *  only; safe in the control interrupt. Takes effect at the next update. */
void pwm_set_duty(float a, float b, float c);

/** Stage 6 diagnostic, while disarmed only (returns false if armed):
 *  on = false takes a phase's INH and INL pins from TIM1 and drives them
 *  low as GPIO, so the DRV holds both its FETs off and the phase floats;
 *  on = true gives them back to TIM1. */
bool pwm_phase_output(board_phase_t ph, bool on);

/** While disarmed: 50 % on all phases, loaded at once (UG), so
 *  motor_can_arm() sees a zero command. Does nothing while armed. */
void pwm_zero_command(void);

/** Stage 7 onward: the main loop sets the duty command (0..1 per phase),
 *  the control interrupt writes it to TIM1 with pwm_isr_update() while
 *  armed (plan: the main loop doesn't write CCR registers). */
void pwm_command_duty(float a, float b, float c);
void pwm_isr_update(void);

/** Duties as written, in permille, for logging. */
void pwm_get_duty_permille(uint32_t *a, uint32_t *b, uint32_t *c);

/** Software break (TIM1 EGR.BG): MOE cleared in hardware, BIF set. */
void pwm_software_break(void);

/** True once if a break (hardware or software) happened while armed since
 *  the last call. Breaks while disarmed (outputs already off, e.g. the DRV's
 *  wake/sleep nFAULT pulses) aren't reported; the nFAULT EXTI covers those. */
bool pwm_take_break_event(void);
uint32_t pwm_break_events(void);

#endif /* PWM_H */
