/**
 * @file    motor_params.h
 * @brief   4015 gimbal motor parameters and stage current limits (plan:
 *          Motor (4015 gimbal); Firmware conventions, Units).
 *
 * Wye-equivalent per-phase values: line-to-line = 2 x phase, whatever the
 * internal winding. Values not yet measured are TODO_MEASURED with the
 * best estimate.
 */
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#define MOTOR_POLE_PAIRS       11U

/* Stage 8, 2026-10-08: 4.4 Ohm line-to-line measured with a meter
 * (developer). The plan expected 3.2 Ohm from 4.8 Ohm per delta winding;
 * 4.4 Ohm fits the 4.8 Ohm spec being line-to-line instead. */
#define MOTOR_R_PHASE_OHM      2.2f

/* Stage 8, 2026-10-08: three L steps 0.5 -> 1.0 V at 0 deg (tau from the
 * 63.2 % crossing, 50 us samples, L = tau x R): 525, 611, 483 uH, mean
 * 540 uH, spread +/-12 % (tau spans about 5 samples). V/I 2.24-2.27 Ohm,
 * agreeing with R. Lower than the spec's 2.6 mH suggested (0.87-1.3 mH). */
#define MOTOR_L_PHASE_H        0.54e-3f

/* Back-EMF constant, datasheet (plan: Motor table). */
#define MOTOR_KE_V_PER_RAD_S   0.1562f

/* Bus voltage used for modulation until VM_SENSE feeds it (plan: SVPWM). */
#define MOTOR_VBUS_V           12.0f

/* Software overcurrent trip, Stage 8 (plan: Firmware protection): 1 A until
 * the current readings match a multimeter, then 1.5 A. The firmware never
 * accepts more than the stage maximum. */
#define MOTOR_OC_TRIP_DEFAULT_A   1.0f
#define MOTOR_OC_TRIP_STAGE_MAX_A 1.5f
#define MOTOR_OC_TRIP_MIN_A       0.1f    /* lowest settable, for the trip test (step 7) */

#endif /* MOTOR_PARAMS_H */
