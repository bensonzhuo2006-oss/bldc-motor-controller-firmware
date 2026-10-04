/**
 * @file    mt6701.h
 * @brief   MT6701 magnetic encoder over SSI on SPI1: read, CRC and status
 *          checks, multi-turn unwrapping, PLL speed estimate.
 *
 * Frame (MT6701 datasheet 7.8.2, Figure-25): 24 bits, MSB first.
 *   bits 23..10  angle D[13:0], 16384 counts per turn
 *   bits  9..6   status Mg[3:0]: Mg[1:0] field, Mg[2] push, Mg[3] loss of track
 *   bits  5..0   CRC-6 over angle + status, polynomial x^6 + x + 1
 *
 * Angles here are mechanical; the FOC code converts to electrical
 * (plan: Firmware conventions, "Phase order, angles and direction").
 */
#ifndef MT6701_H
#define MT6701_H

#include <stdbool.h>
#include <stdint.h>

#define MT6701_COUNTS_PER_TURN  16384U   /* 14-bit angle, datasheet 7.8.2 */

typedef enum
{
  MT6701_FIELD_NORMAL     = 0,   /* Mg[1:0] = 0 */
  MT6701_FIELD_TOO_STRONG = 1,   /* Mg[1:0] = 1 */
  MT6701_FIELD_TOO_WEAK   = 2,   /* Mg[1:0] = 2 */
  MT6701_FIELD_RESERVED   = 3    /* Mg[1:0] = 3, "-" in the datasheet */
} mt6701_field_t;

/** Fault and event counters, all since mt6701_init(). */
typedef struct
{
  uint32_t reads;           /* read attempts */
  uint32_t spi_timeouts;    /* SPI transfer didn't complete */
  uint32_t crc_errors;      /* CRC-6 mismatch */
  uint32_t field_strong;    /* samples with field too strong */
  uint32_t field_weak;      /* samples with field too weak or reserved code */
  uint32_t loss_of_track;   /* samples with Mg[3] set */
  uint32_t angle_jumps;     /* angle change too large to be real */
} mt6701_counters_t;

/** Consistent copy of the encoder state for the main loop. */
typedef struct
{
  uint32_t raw_word;        /* last raw 24-bit frame */
  uint16_t angle_counts;    /* last good angle, 0..16383 */
  uint8_t  status;          /* last good Mg[3:0] */
  bool     sample_ok;       /* last read passed SPI, CRC and jump checks */
  bool     healthy;         /* sample_ok, field normal, no loss of track */
  int32_t  turns;           /* whole mechanical turns since init */
  float    angle_rad;       /* mechanical angle, [0, 2*pi) */
  float    position_rad;    /* multi-turn mechanical position */
  float    speed_rad_s;     /* PLL mechanical speed estimate */
  mt6701_counters_t counters;
} mt6701_snapshot_t;

/** Check the SPI1 configuration, enable SPI1 and reset the state.
 *  dt_s is the interval between mt6701_update() calls.
 *  Returns false if SPI1 isn't configured as the plan requires. */
bool mt6701_init(float dt_s);

/** Read one frame and update angle, position and speed. Register access
 *  only, no HAL, bounded wait: usable from the control interrupt.
 *  Returns true if the sample was good. */
bool mt6701_update(void);

/** Copy the state with interrupts briefly disabled. */
void mt6701_snapshot(mt6701_snapshot_t *out);

/** Field status from a Mg[3:0] value. */
static inline mt6701_field_t mt6701_field(uint8_t status)
{
  return (mt6701_field_t)(status & 0x3U);
}

static inline bool mt6701_push(uint8_t status)
{
  return (status & 0x4U) != 0U;
}

static inline bool mt6701_loss_of_track(uint8_t status)
{
  return (status & 0x8U) != 0U;
}

const char *mt6701_field_name(mt6701_field_t field);

#endif /* MT6701_H */
