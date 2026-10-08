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

/* TODO_MEASURED (Stage 8 step 7): if the 2.6 mH spec is line-to-line like
 * the resistance, the wye-equivalent is 1.3 mH (0.87 mH if it was per delta
 * winding). */
#define MOTOR_L_PHASE_H        1.3e-3f

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
