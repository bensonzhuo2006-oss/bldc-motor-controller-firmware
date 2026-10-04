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

/** Check the SPI3 configuration and leave the DRV asleep with its SPI
 *  pins undriven. Returns false if SPI3 isn't configured as the plan says. */
bool drv8323_init(void);

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
