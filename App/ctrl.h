/**
 * @file    ctrl.h
 * @brief   Control step run inside the 20 kHz control interrupt, after the
 *          currents are read: software overcurrent trip, then the output
 *          (Stage 7: duty command; Stage 8: fixed voltage vector).
 *
 * Not in the plan's original module list: added in Stage 8 to keep the
 * control logic out of the ADC driver (cursense) and the PWM driver.
 */
#ifndef CTRL_H
#define CTRL_H

#include "board.h"
#include <stdbool.h>
#include <stdint.h>

/** Called by the control interrupt with the phase currents (A, positive
 *  into the motor). Register access only. */
void ctrl_isr(const float amps[PHASE_COUNT]);

/** Overcurrent trip level, A. Clamped to [MOTOR_OC_TRIP_MIN_A,
 *  MOTOR_OC_TRIP_STAGE_MAX_A]; NaN gives the default. Returns the level
 *  applied. Main loop. */
float ctrl_set_trip(float amps);

/** True once after the interrupt tripped on overcurrent (it has already
 *  disarmed and triggered the capture); *phase and *amps say where. */
bool ctrl_take_oc_event(uint32_t *phase, float *amps);

/** Stage 8: voltage vector command, phase-voltage amplitude v (V) at
 *  electrical angle theta (rad), plus a common-mode duty shift added to
 *  all three duties. Applied by the interrupt while armed. Main loop. */
void ctrl_set_vector(float v, float theta, float shift);

/** Stage 8: apply v_step at the next interrupt and trigger the capture in
 *  that same interrupt, so the capture's trigger marks the step. */
void ctrl_vector_step(float v_step);

/** Stage 9 forced angle. reset: angle, speed and cycle count to 0 (call
 *  while disarmed). set_spin: target electrical frequency (Hz, sign =
 *  direction) and the slew toward it (Hz/s). spin_state: present speed and
 *  electrical cycles since the reset (fractional). Main loop. */
void ctrl_spin_reset(void);
void ctrl_set_spin(float f_target_hz, float accel_hz_s);
void ctrl_spin_state(float *f_now_hz, float *cycles);

/** Phase voltages last commanded (V, before injection) and the highest
 *  duty, for logging. */
void ctrl_last_output(float vph[PHASE_COUNT], float *duty_max);

#endif /* CTRL_H */
