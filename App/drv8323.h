/**
 * @file    drv8323.h
 * @brief   DRV8323S gate driver: SPI3 register access, wake/sleep, fault
 *          decoding (plan: DRV8323 gate driver; Stage 3).
 *
 * SPI frame (datasheet 8.5.1.1): 16 bits, MSB first. Bit 15 = read (1) /
 * write (0), bits 14-11 = address, bits 10-0 = data. SDO returns 5
 * don't-care bits then 11 data bits; a write returns the old register value.
 * Main loop only: never call from the control interrupt (plan: no SPI3
 * traffic in the control interrupt).
 */
#ifndef DRV8323_H
#define DRV8323_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Register addresses (datasheet Table 8-10). */
#define DRV_REG_FAULT1        0x00U
#define DRV_REG_FAULT2        0x01U
#define DRV_REG_DRIVER_CTRL   0x02U
#define DRV_REG_GATE_HS       0x03U
#define DRV_REG_GATE_LS       0x04U
#define DRV_REG_OCP_CTRL      0x05U
#define DRV_REG_CSA_CTRL      0x06U
#define DRV_REG_COUNT         7U

#define DRV_DATA_MASK         0x7FFU

/* Reset values of the control registers 0x02-0x06 (datasheet 8.6.2). */
#define DRV_DEFAULT_DRIVER_CTRL  0x000U
#define DRV_DEFAULT_GATE_HS      0x3FFU
#define DRV_DEFAULT_GATE_LS      0x7FFU
#define DRV_DEFAULT_OCP_CTRL     0x159U
#define DRV_DEFAULT_CSA_CTRL     0x283U

/* Driver Control CLR_FLT (bit 0), Gate Drive HS LOCK field (bits 10-8). */
#define DRV_DRIVER_CTRL_CLR_FLT  0x001U
#define DRV_GATE_HS_LOCK_MASK    0x700U
#define DRV_GATE_HS_LOCK_ON      0x600U   /* 110b: lock */
#define DRV_GATE_HS_LOCK_OFF     0x300U   /* 011b: unlock */

/* Bring-up configuration (plan: DRV8323 register table, "Bring-up value").
 * If IDRIVE is retuned in Stage 6, change GATE_HS and GATE_LS here; the
 * shadow copy for the readback check follows. */
#define DRV_CFG_DRIVER_CTRL       0x000U   /* 6x PWM, fault reporting on */
#define DRV_CFG_GATE_HS_UNLOCKED  0x322U   /* LOCK 011b, IDRIVEP 60 mA, IDRIVEN 120 mA */
#define DRV_CFG_GATE_HS_LOCKED    0x622U   /* LOCK 110b after setup */
#define DRV_CFG_GATE_LS           0x722U   /* CBC 1, TDRIVE 4 us, IDRIVE 60/120 mA */
#define DRV_CFG_OCP_CTRL          0x110U   /* 100 ns dead time, latched OCP, 4 us deglitch, VDS 0.06 V */
#define DRV_CFG_CSA_CTRL          0x2C3U   /* VREF/2, 40 V/V, CAL off, SEN_LVL 1 V */
#define DRV_CSA_CAL_ALL           0x01CU   /* CSA_CAL_A/B/C, bits 4..2 */
#define DRV_LOCK_TEST_VALUE       DRV_DEFAULT_OCP_CTRL   /* written to 0x05 after locking */

typedef enum
{
  DRV_CFG_OK = 0,
  DRV_CFG_NOT_PRESENT,
  DRV_CFG_WRITE_FAIL,
  DRV_CFG_CAL_FAIL,
  DRV_CFG_LOCK_FAIL,
  DRV_CFG_LOCK_TEST_FAIL
} drv_cfg_status_t;

/** Where configure stopped, for the log and the fault context. */
typedef struct
{
  drv_cfg_status_t status;
  uint8_t  addr;    /* register involved */
  uint16_t wrote;   /* value written */
  uint16_t read;    /* value read back */
} drv_cfg_result_t;

/** Check the SPI3 configuration, set up the nFAULT interrupt (EXTI10 on
 *  PB10) and leave the DRV asleep with its SPI pins undriven.
 *  Returns false if SPI3 isn't configured as the plan says. */
bool drv8323_init(void);

/** After a wake: write and verify the bring-up configuration, run the CSA
 *  offset calibration, lock, and check that the lock holds (plan: Stage 4,
 *  "DRV8323: register readback" checks 1 and 2). */
bool drv8323_configure(drv_cfg_result_t *result);

/** True after a successful drv8323_configure(), until the next sleep. */
bool drv8323_is_configured(void);

/** Read 0x02-0x06 and compare with the configured shadow copy (readback
 *  check 3). On a mismatch, addr/value say where. */
bool drv8323_check_config(uint8_t *addr, uint16_t *value);

/** nFAULT falling edges seen by the interrupt: outside the wake/sleep
 *  window (real faults) and inside it (ignored wake/sleep pulses). */
uint32_t drv8323_nfault_events(void);
uint32_t drv8323_nfault_blanked_events(void);

/** True once if a real nFAULT edge arrived since the last call. */
bool drv8323_take_nfault_event(void);

/** Wake: PD2 driven high, SPI3 enabled, ENABLE high, wait tWAKE.
 *  Caller makes sure VM_OK is set first. Returns true if nFAULT is high
 *  (released) after the wake time. */
bool drv8323_wake(void);

/** Sleep: disarm (outputs off, then ENABLE low), SPI3 disabled,
 *  PD2 released to analog (R38 keeps nSCS high). */
void drv8323_sleep(void);

bool drv8323_is_awake(void);

/** nFAULT pin level: true when low (fault, or during wake/sleep). */
bool drv8323_nfault_low(void);

/** Register access. Return false if asleep or on an SPI timeout. */
bool drv8323_read(uint8_t addr, uint16_t *value);
bool drv8323_write(uint8_t addr, uint16_t value, uint16_t *previous);

/** Write, then read back; returns true only if the readback matches. */
bool drv8323_write_verify(uint8_t addr, uint16_t value, uint16_t *readback);

/** Presence check: register 0x03 reads 0x3FF after wake. An unpowered DRV
 *  reads 0x7FF (SDO pull-up), so 0x04 can't be used (plan: Presence check). */
bool drv8323_present(uint16_t *reg03);

/** Text list of the set fault bits, e.g. "UVLO VDS_HA"; "none" if clear. */
void drv8323_fault_text(uint16_t fault1, uint16_t fault2, char *buf, size_t len);

/** SPI transfers that timed out since init. */
uint32_t drv8323_spi_errors(void);

#endif /* DRV8323_H */
