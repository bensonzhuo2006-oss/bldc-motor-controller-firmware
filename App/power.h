/**
 * @file    power.h
 * @brief   MOTOR_EN debounce, VM_SENSE measurement and the VM power state
 *          machine (plan: Stage 2, "Power monitor" in Firmware protection).
 *
 * NO_VM -> VM_SETTLING -> VM_OK. VM_OK needs MOTOR_EN high and VM above
 * 10 V, held for 20 ms. VM below 9 V or MOTOR_EN low drops to NO_VM.
 * Main loop only (uses ADC1 regular conversions, polled).
 */
#ifndef POWER_H
#define POWER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  POWER_NO_VM = 0,
  POWER_VM_SETTLING,
  POWER_VM_OK
} power_state_t;

/** Calibrate and enable ADC1 for VM_SENSE. Returns false on a timeout. */
bool power_init(void);

/** Call every 1 ms tick. Returns true if the state changed on this call. */
bool power_update(void);

power_state_t power_state(void);
const char *power_state_name(power_state_t state);

/** True only in VM_OK. */
bool power_vm_ok(void);

/** Debounced MOTOR_EN (SW1 on = true). */
bool power_motor_en(void);

/** Raw MOTOR_EN pin level, not debounced. */
bool power_motor_en_raw(void);

/** Latest VM in millivolts, and the averaged ADC code behind it. */
uint32_t power_vm_mv(void);
uint16_t power_vm_code(void);

/** Number of ADC conversions that timed out since init. */
uint32_t power_adc_timeouts(void);

#endif /* POWER_H */
