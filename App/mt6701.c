/**
 * @file    mt6701.c
 * @brief   MT6701 SSI read on SPI1 (register access), decode, checks,
 *          unwrapping and PLL speed estimate.
 *
 * SPI1 (CubeMX): mode 1 (CPOL = 0, CPHA = 1), 12-bit frames, MSB first,
 * software NSS, 170 MHz / 32 = 5.3 MHz. One read = two 12-bit frames with
 * dummy transmit data (plan: MT6701 encoder, "Clock" and "Speed").
 */
#include "mt6701.h"
#include "board.h"
#include "main.h"

/* ---- Constants ------------------------------------------------------------ */

#define MT6701_TWO_PI            6.28318530718f
#define MT6701_PI                3.14159265359f
#define MT6701_RAD_PER_COUNT     (MT6701_TWO_PI / (float)MT6701_COUNTS_PER_TURN)
#define MT6701_HALF_TURN_COUNTS  ((int32_t)(MT6701_COUNTS_PER_TURN / 2U))

/* Frame layout, datasheet 7.8.2 Figure-25. */
#define MT6701_FRAME_BITS        12U
#define MT6701_FRAME_MASK        0xFFFU
#define MT6701_ANGLE_SHIFT       10U
#define MT6701_ANGLE_MASK        0x3FFFU
#define MT6701_STATUS_SHIFT      6U
#define MT6701_STATUS_MASK       0xFU
#define MT6701_CRC_MASK          0x3FU
#define MT6701_CRC_DATA_BITS     18U        /* D[13:0] + Mg[3:0] */
#define MT6701_CRC_POLY_LOW      0x03U      /* x^6 + x + 1, x^6 implicit */

/* SSI timing, datasheet 7.8.1: T_H >= 100 ns (CSN fall to first CLK rise),
 * T_L >= 0.5 * T_CLK (last CLK fall to CSN rise), T_CLK = 188 ns here.
 * 34 cycles at 170 MHz = 200 ns, twice the larger requirement. */
#define MT6701_CS_SETUP_CYCLES   34U
#define MT6701_CS_HOLD_CYCLES    34U

/* Transfer timeout: a read takes about 24 x 188 ns = 4.5 us; 3,400 cycles is
 * 20 us at 170 MHz. */
#define MT6701_SPI_TIMEOUT_CYCLES 3400U

/* RX FIFO is 32 bits: at most two 12-bit entries to flush (RM0440 SPI). */
#define MT6701_RX_FIFO_MAX_ENTRIES 4U

/* Angle-jump check: fastest believable mechanical speed. 1,500 rpm is about
 * twice the 4015 motor's 730 rpm no-load speed (plan: Motor table, Kv). */
#define MT6701_MAX_PLAUSIBLE_RPM 1500.0f
/* Floor on the jump threshold so normal jitter never trips it. */
#define MT6701_MIN_JUMP_COUNTS   64

/* PLL speed tracker: critically damped, natural frequency 2*pi*50 rad/s.
 * Kp = 2*zeta*wn, Ki = wn^2. 50 Hz is below the 1 kHz update rate of
 * Stage 1 (wn*dt = 0.31); revisit when reads move to the 20 kHz interrupt. */
#define MT6701_PLL_BW_HZ         50.0f
#define MT6701_PLL_ZETA          1.0f

/* Expected SPI1 register fields (CubeMX settings, plan Stage 1). */
#define MT6701_CR1_CHECK_MASK    (SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_BR | \
                                  SPI_CR1_LSBFIRST | SPI_CR1_SSI | SPI_CR1_SSM | \
                                  SPI_CR1_RXONLY | SPI_CR1_BIDIMODE)
#define MT6701_CR1_EXPECTED      (SPI_CR1_CPHA | SPI_CR1_MSTR | SPI_CR1_BR_2 | \
                                  SPI_CR1_SSI | SPI_CR1_SSM)      /* mode 1, /32 */
#define MT6701_CR2_CHECK_MASK    (SPI_CR2_DS | SPI_CR2_FRXTH)
#define MT6701_CR2_EXPECTED      (0xBUL << SPI_CR2_DS_Pos)        /* 12-bit, FRXTH = 0 */

/* ---- State ---------------------------------------------------------------- */

static volatile mt6701_snapshot_t s_state;
static float s_dt_s;
static float s_pll_kp;
static float s_pll_ki;
static float s_pll_angle_rad;
static int32_t s_jump_limit_counts;
static bool s_have_reference;

/* ---- Helpers -------------------------------------------------------------- */

static inline void mt6701_wait_cycles(uint32_t cycles)
{
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

static inline float mt6701_wrap_pm_pi(float x)
{
  if (x > MT6701_PI)
  {
    x -= MT6701_TWO_PI;
  }
  else if (x < -MT6701_PI)
  {
    x += MT6701_TWO_PI;
  }
  return x;
}

static inline float mt6701_wrap_2pi(float x)
{
  if (x >= MT6701_TWO_PI)
  {
    x -= MT6701_TWO_PI;
  }
  else if (x < 0.0f)
  {
    x += MT6701_TWO_PI;
  }
  return x;
}

/* CRC-6, polynomial x^6 + x + 1, MSB first, initial value 0
 * (datasheet 7.8.2: over D[13:0] then Mg[3:0], D13 first). */
static uint8_t mt6701_crc6(uint32_t data)
{
  uint8_t crc = 0U;
  for (int32_t i = (int32_t)MT6701_CRC_DATA_BITS - 1; i >= 0; i--)
  {
    uint8_t in = (uint8_t)((data >> (uint32_t)i) & 1U);
    uint8_t feedback = (uint8_t)(((crc >> 5) & 1U) ^ in);
    crc = (uint8_t)((crc << 1) & MT6701_CRC_MASK);
    if (feedback != 0U)
    {
      crc ^= MT6701_CRC_POLY_LOW;
    }
  }
  return crc;
}

/* Wait for a received frame, bounded. */
static bool mt6701_wait_rx(uint32_t start)
{
  while ((SPI1->SR & SPI_SR_RXNE) == 0U)
  {
    if ((DWT->CYCCNT - start) > MT6701_SPI_TIMEOUT_CYCLES)
    {
      return false;
    }
  }
  return true;
}

/* One 24-clock SSI read as two 12-bit frames. */
static bool mt6701_spi_read24(uint32_t *word)
{
  volatile uint16_t *dr = (volatile uint16_t *)&SPI1->DR;  /* 16-bit access for 12-bit frames */
  uint16_t hi = 0U;
  uint16_t lo = 0U;
  bool ok;

  for (uint32_t i = 0U; (i < MT6701_RX_FIFO_MAX_ENTRIES) && ((SPI1->SR & SPI_SR_RXNE) != 0U); i++)
  {
    (void)*dr;
  }

  BOARD_ENC_CS_PORT->BRR = BOARD_ENC_CS_PIN;
  mt6701_wait_cycles(MT6701_CS_SETUP_CYCLES);

  uint32_t start = DWT->CYCCNT;
  *dr = 0U;   /* dummy frame 1; TX FIFO holds both frames */
  *dr = 0U;   /* dummy frame 2 */

  ok = mt6701_wait_rx(start);
  if (ok)
  {
    hi = *dr;
    ok = mt6701_wait_rx(start);
  }
  if (ok)
  {
    lo = *dr;
    while ((SPI1->SR & SPI_SR_BSY) != 0U)
    {
      if ((DWT->CYCCNT - start) > MT6701_SPI_TIMEOUT_CYCLES)
      {
        ok = false;
        break;
      }
    }
  }

  mt6701_wait_cycles(MT6701_CS_HOLD_CYCLES);
  BOARD_ENC_CS_PORT->BSRR = BOARD_ENC_CS_PIN;

  *word = (((uint32_t)hi & MT6701_FRAME_MASK) << MT6701_FRAME_BITS) |
          ((uint32_t)lo & MT6701_FRAME_MASK);
  return ok;
}

/* ---- API ------------------------------------------------------------------ */

bool mt6701_init(float dt_s)
{
  bool cfg_ok = ((SPI1->CR1 & MT6701_CR1_CHECK_MASK) == MT6701_CR1_EXPECTED) &&
                ((SPI1->CR2 & MT6701_CR2_CHECK_MASK) == MT6701_CR2_EXPECTED);

  /* The CS timing waits and transfer timeouts use the DWT cycle counter;
   * make sure it runs (debug_init() normally has already done this). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Enable SPI1 with CS high so SCK is driven at its idle level (low) before
   * the first CS falling edge; it isn't driven while SPE = 0 (RM0440 SPI). */
  BOARD_ENC_CS_PORT->BSRR = BOARD_ENC_CS_PIN;
  SPI1->CR1 |= SPI_CR1_SPE;

  float wn = MT6701_TWO_PI * MT6701_PLL_BW_HZ;
  s_dt_s = dt_s;
  s_pll_kp = 2.0f * MT6701_PLL_ZETA * wn;
  s_pll_ki = wn * wn;
  s_pll_angle_rad = 0.0f;

  float counts_per_sample = (MT6701_MAX_PLAUSIBLE_RPM / 60.0f) * (float)MT6701_COUNTS_PER_TURN * dt_s;
  s_jump_limit_counts = (int32_t)counts_per_sample;
  if (s_jump_limit_counts < MT6701_MIN_JUMP_COUNTS)
  {
    s_jump_limit_counts = MT6701_MIN_JUMP_COUNTS;
  }

  s_have_reference = false;
  s_state.raw_word = 0U;
  s_state.angle_counts = 0U;
  s_state.status = 0U;
  s_state.sample_ok = false;
  s_state.healthy = false;
  s_state.turns = 0;
  s_state.angle_rad = 0.0f;
  s_state.position_rad = 0.0f;
  s_state.speed_rad_s = 0.0f;
  s_state.counters.reads = 0U;
  s_state.counters.spi_timeouts = 0U;
  s_state.counters.crc_errors = 0U;
  s_state.counters.field_strong = 0U;
  s_state.counters.field_weak = 0U;
  s_state.counters.loss_of_track = 0U;
  s_state.counters.angle_jumps = 0U;

  return cfg_ok;
}

bool mt6701_update(void)
{
  uint32_t word;

  s_state.counters.reads++;

  if (!mt6701_spi_read24(&word))
  {
    s_state.counters.spi_timeouts++;
    s_state.sample_ok = false;
    s_state.healthy = false;
    return false;
  }
  s_state.raw_word = word;

  uint32_t data18 = word >> MT6701_STATUS_SHIFT;
  if (mt6701_crc6(data18) != (uint8_t)(word & MT6701_CRC_MASK))
  {
    s_state.counters.crc_errors++;
    s_state.sample_ok = false;
    s_state.healthy = false;
    return false;
  }

  uint16_t angle = (uint16_t)((word >> MT6701_ANGLE_SHIFT) & MT6701_ANGLE_MASK);
  uint8_t status = (uint8_t)((word >> MT6701_STATUS_SHIFT) & MT6701_STATUS_MASK);
  mt6701_field_t field = mt6701_field(status);

  if (field == MT6701_FIELD_TOO_STRONG)
  {
    s_state.counters.field_strong++;
  }
  else if (field != MT6701_FIELD_NORMAL)
  {
    s_state.counters.field_weak++;
  }
  if (mt6701_loss_of_track(status))
  {
    s_state.counters.loss_of_track++;
  }

  float angle_rad = (float)angle * MT6701_RAD_PER_COUNT;

  if (!s_have_reference)
  {
    s_have_reference = true;
    s_pll_angle_rad = angle_rad;
  }
  else
  {
    /* Unwrap: shortest signed step, half a turn either way. */
    int32_t step = (int32_t)angle - (int32_t)s_state.angle_counts;
    if (step > MT6701_HALF_TURN_COUNTS)
    {
      step -= (int32_t)MT6701_COUNTS_PER_TURN;
    }
    else if (step < -MT6701_HALF_TURN_COUNTS)
    {
      step += (int32_t)MT6701_COUNTS_PER_TURN;
    }

    if ((step > s_jump_limit_counts) || (step < -s_jump_limit_counts))
    {
      /* Too large to be real: count it, take the new angle as the reference
       * without changing the turn count, and skip the PLL for this sample. */
      s_state.counters.angle_jumps++;
      s_state.angle_counts = angle;
      s_state.status = status;
      s_state.angle_rad = angle_rad;
      s_state.sample_ok = false;
      s_state.healthy = false;
      return false;
    }

    if (((int32_t)s_state.angle_counts + step) >= (int32_t)MT6701_COUNTS_PER_TURN)
    {
      s_state.turns++;
    }
    else if (((int32_t)s_state.angle_counts + step) < 0)
    {
      s_state.turns--;
    }

    /* PLL: track the measured angle; the integrator is the speed. */
    float err = mt6701_wrap_pm_pi(angle_rad - s_pll_angle_rad);
    s_state.speed_rad_s += s_pll_ki * err * s_dt_s;
    s_pll_angle_rad = mt6701_wrap_2pi(s_pll_angle_rad + (s_state.speed_rad_s + s_pll_kp * err) * s_dt_s);
  }

  s_state.angle_counts = angle;
  s_state.status = status;
  s_state.angle_rad = angle_rad;
  s_state.position_rad = ((float)s_state.turns * MT6701_TWO_PI) + angle_rad;
  s_state.sample_ok = true;
  s_state.healthy = (field == MT6701_FIELD_NORMAL) && !mt6701_loss_of_track(status);
  return true;
}

void mt6701_snapshot(mt6701_snapshot_t *out)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  out->raw_word = s_state.raw_word;
  out->angle_counts = s_state.angle_counts;
  out->status = s_state.status;
  out->sample_ok = s_state.sample_ok;
  out->healthy = s_state.healthy;
  out->turns = s_state.turns;
  out->angle_rad = s_state.angle_rad;
  out->position_rad = s_state.position_rad;
  out->speed_rad_s = s_state.speed_rad_s;
  out->counters.reads = s_state.counters.reads;
  out->counters.spi_timeouts = s_state.counters.spi_timeouts;
  out->counters.crc_errors = s_state.counters.crc_errors;
  out->counters.field_strong = s_state.counters.field_strong;
  out->counters.field_weak = s_state.counters.field_weak;
  out->counters.loss_of_track = s_state.counters.loss_of_track;
  out->counters.angle_jumps = s_state.counters.angle_jumps;
  __set_PRIMASK(primask);
}

const char *mt6701_field_name(mt6701_field_t field)
{
  switch (field)
  {
    case MT6701_FIELD_NORMAL:     return "NORMAL";
    case MT6701_FIELD_TOO_STRONG: return "TOO_STRONG";
    case MT6701_FIELD_TOO_WEAK:   return "TOO_WEAK";
    default:                      return "RESERVED";
  }
}
