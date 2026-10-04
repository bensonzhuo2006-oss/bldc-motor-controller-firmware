/**
 * @file    drv8323.c
 * @brief   DRV8323S SPI3 register access, wake/sleep and fault decoding.
 *
 * SPI3 (CubeMX): mode 1 (CPOL = 0, CPHA = 1), 16-bit, MSB first, software
 * NSS on PD2, 170 MHz / 256 = 664 kHz (SDO is open-drain with only the R23
 * 10k pull-up; plan: DRV8323, "SPI speed").
 */
#include "drv8323.h"
#include "board.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ---- Constants ------------------------------------------------------------ */

#define DRV_CMD_READ             0x8000U
#define DRV_ADDR_SHIFT           11U

/* Datasheet 7.6 SPI timing: nSCS setup >= 50 ns, hold >= 50 ns, high >= 400 ns
 * between words. At 170 MHz: 34 cycles = 200 ns, 170 cycles = 1 us. */
#define DRV_CS_SETUP_CYCLES      34U
#define DRV_CS_HOLD_CYCLES       34U
#define DRV_CS_HIGH_CYCLES       170U

/* One 16-bit word at 664 kHz takes 24 us; time out after 100 us. */
#define DRV_SPI_TIMEOUT_CYCLES   17000U

/* RX FIFO is 32 bits: at most two 16-bit entries (RM0440 SPI). */
#define DRV_RX_FIFO_MAX_ENTRIES  4U

/* Datasheet 7.5: tWAKE (ENABLE high to outputs ready) and tSLEEP
 * (ENABLE low to sleep) are each 1 ms max. Wait 10% more on wake; a re-wake
 * waits until tSLEEP has passed since the last sleep. */
#define DRV_WAKE_WAIT_US         1100U
#define DRV_SLEEP_MIN_MS         2U

#define DRV_CYCLES_PER_US        170U   /* HCLK 170 MHz */

/* Expected SPI3 register fields (CubeMX settings, plan Stage 3). */
#define DRV_CR1_CHECK_MASK       (SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_BR | \
                                  SPI_CR1_LSBFIRST | SPI_CR1_SSI | SPI_CR1_SSM | \
                                  SPI_CR1_RXONLY | SPI_CR1_BIDIMODE)
#define DRV_CR1_EXPECTED         (SPI_CR1_CPHA | SPI_CR1_MSTR | SPI_CR1_BR | \
                                  SPI_CR1_SSI | SPI_CR1_SSM)       /* mode 1, /256 */
#define DRV_CR2_CHECK_MASK       (SPI_CR2_DS | SPI_CR2_FRXTH)
#define DRV_CR2_EXPECTED         (0xFUL << SPI_CR2_DS_Pos)         /* 16-bit, FRXTH = 0 */

/* PD2 MODER field values. */
#define DRV_MODER_OUTPUT         1UL
#define DRV_MODER_ANALOG         3UL

/* ---- State ---------------------------------------------------------------- */

static bool s_awake;
static uint32_t s_sleep_tick;
static uint32_t s_spi_errors;

/* ---- Helpers -------------------------------------------------------------- */

static inline void drv_wait_cycles(uint32_t cycles)
{
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

static void drv_nscs_mode(uint32_t mode)
{
  uint32_t shift = 2U * BOARD_DRV_NSCS_PIN_NUM;
  uint32_t moder = BOARD_DRV_NSCS_PORT->MODER;
  moder &= ~(3UL << shift);
  moder |= (mode << shift);
  BOARD_DRV_NSCS_PORT->MODER = moder;
}

static bool drv_xfer(uint16_t tx, uint16_t *rx)
{
  volatile uint16_t *dr = (volatile uint16_t *)&SPI3->DR;   /* 16-bit access */
  uint16_t value = 0U;
  bool ok = true;

  for (uint32_t i = 0U; (i < DRV_RX_FIFO_MAX_ENTRIES) && ((SPI3->SR & SPI_SR_RXNE) != 0U); i++)
  {
    (void)*dr;
  }

  BOARD_DRV_NSCS_PORT->BRR = BOARD_DRV_NSCS_PIN;
  drv_wait_cycles(DRV_CS_SETUP_CYCLES);

  uint32_t start = DWT->CYCCNT;
  *dr = tx;
  while ((SPI3->SR & SPI_SR_RXNE) == 0U)
  {
    if ((DWT->CYCCNT - start) > DRV_SPI_TIMEOUT_CYCLES)
    {
      ok = false;
      break;
    }
  }
  if (ok)
  {
    value = *dr;
    while ((SPI3->SR & SPI_SR_BSY) != 0U)
    {
      if ((DWT->CYCCNT - start) > DRV_SPI_TIMEOUT_CYCLES)
      {
        ok = false;
        break;
      }
    }
  }

  drv_wait_cycles(DRV_CS_HOLD_CYCLES);
  BOARD_DRV_NSCS_PORT->BSRR = BOARD_DRV_NSCS_PIN;
  drv_wait_cycles(DRV_CS_HIGH_CYCLES);

  if (!ok)
  {
    s_spi_errors++;
  }
  *rx = (uint16_t)(value & DRV_DATA_MASK);
  return ok;
}

static void drv_spi3_disable(void)
{
  /* RM0440 SPI disable procedure: wait until TX FIFO empty and not busy,
   * clear SPE, then drain the RX FIFO. Bounded waits. */
  uint32_t start = DWT->CYCCNT;
  while (((SPI3->SR & (SPI_SR_FTLVL | SPI_SR_BSY)) != 0U) &&
         ((DWT->CYCCNT - start) < DRV_SPI_TIMEOUT_CYCLES))
  {
  }
  SPI3->CR1 &= ~SPI_CR1_SPE;
  for (uint32_t i = 0U; (i < DRV_RX_FIFO_MAX_ENTRIES) && ((SPI3->SR & SPI_SR_FRLVL) != 0U); i++)
  {
    (void)*(volatile uint16_t *)&SPI3->DR;
  }
}

/* ---- API ------------------------------------------------------------------ */

bool drv8323_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* DWT for timing */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  bool cfg_ok = ((SPI3->CR1 & DRV_CR1_CHECK_MASK) == DRV_CR1_EXPECTED) &&
                ((SPI3->CR2 & DRV_CR2_CHECK_MASK) == DRV_CR2_EXPECTED);

  drv8323_sleep();
  s_spi_errors = 0U;
  return cfg_ok;
}

bool drv8323_wake(void)
{
  if ((HAL_GetTick() - s_sleep_tick) < DRV_SLEEP_MIN_MS)
  {
    HAL_Delay(DRV_SLEEP_MIN_MS);
  }

  /* nSCS driven high (deselected) before anything else on the bus. */
  BOARD_DRV_NSCS_PORT->BSRR = BOARD_DRV_NSCS_PIN;
  drv_nscs_mode(DRV_MODER_OUTPUT);

  /* SPI3 on before the first nSCS edge, so SCLK is driven low (CPOL = 0). */
  SPI3->CR1 |= SPI_CR1_SPE;

  BOARD_DRV_ENABLE_PORT->BSRR = BOARD_DRV_ENABLE_PIN;
  drv_wait_cycles(DRV_WAKE_WAIT_US * DRV_CYCLES_PER_US);

  s_awake = true;
  return !drv8323_nfault_low();
}

void drv8323_sleep(void)
{
  motor_disarm();             /* outputs off, then ENABLE low */
  drv_spi3_disable();         /* SCLK/SDI no longer driven */
  drv_nscs_mode(DRV_MODER_ANALOG);   /* PD2 released; R38 holds nSCS high */
  s_awake = false;
  s_sleep_tick = HAL_GetTick();
}

bool drv8323_is_awake(void)
{
  return s_awake;
}

bool drv8323_nfault_low(void)
{
  return (BOARD_DRV_NFAULT_PORT->IDR & BOARD_DRV_NFAULT_PIN) == 0U;
}

bool drv8323_read(uint8_t addr, uint16_t *value)
{
  if (!s_awake)
  {
    return false;
  }
  uint16_t tx = (uint16_t)(DRV_CMD_READ | ((uint16_t)(addr & 0xFU) << DRV_ADDR_SHIFT));
  return drv_xfer(tx, value);
}

bool drv8323_write(uint8_t addr, uint16_t value, uint16_t *previous)
{
  if (!s_awake)
  {
    return false;
  }
  uint16_t old;
  uint16_t tx = (uint16_t)(((uint16_t)(addr & 0xFU) << DRV_ADDR_SHIFT) | (value & DRV_DATA_MASK));
  bool ok = drv_xfer(tx, &old);
  if (previous != NULL)
  {
    *previous = old;
  }
  return ok;
}

bool drv8323_write_verify(uint8_t addr, uint16_t value, uint16_t *readback)
{
  uint16_t rb = 0U;
  bool ok = drv8323_write(addr, value, NULL) && drv8323_read(addr, &rb);
  if (readback != NULL)
  {
    *readback = rb;
  }
  return ok && (rb == (value & DRV_DATA_MASK));
}

bool drv8323_present(uint16_t *reg03)
{
  uint16_t v = 0U;
  bool ok = drv8323_read(DRV_REG_GATE_HS, &v);
  if (reg03 != NULL)
  {
    *reg03 = v;
  }
  return ok && (v == DRV_DEFAULT_GATE_HS);
}

void drv8323_fault_text(uint16_t fault1, uint16_t fault2, char *buf, size_t len)
{
  /* Datasheet Tables 8-12 and 8-13, bit 10 first. */
  static const char *const f1_names[11] =
  {
    "VDS_LC", "VDS_HC", "VDS_LB", "VDS_HB", "VDS_LA", "VDS_HA",
    "OTSD", "UVLO", "GDF", "VDS_OCP", "FAULT"
  };
  static const char *const f2_names[11] =
  {
    "VGS_LC", "VGS_HC", "VGS_LB", "VGS_HB", "VGS_LA", "VGS_HA",
    "CPUV", "OTW", "SC_OC", "SB_OC", "SA_OC"
  };
  size_t used = 0U;

  if (len == 0U)
  {
    return;
  }
  buf[0] = '\0';

  for (int32_t bit = 10; bit >= 0; bit--)
  {
    if (((fault1 >> (uint32_t)bit) & 1U) != 0U)
    {
      int n = snprintf(&buf[used], len - used, "%s%s", (used > 0U) ? " " : "", f1_names[bit]);
      if ((n < 0) || ((size_t)n >= (len - used))) { return; }
      used += (size_t)n;
    }
  }
  for (int32_t bit = 10; bit >= 0; bit--)
  {
    if (((fault2 >> (uint32_t)bit) & 1U) != 0U)
    {
      int n = snprintf(&buf[used], len - used, "%s%s", (used > 0U) ? " " : "", f2_names[bit]);
      if ((n < 0) || ((size_t)n >= (len - used))) { return; }
      used += (size_t)n;
    }
  }
  if (used == 0U)
  {
    (void)snprintf(buf, len, "none");
  }
}

uint32_t drv8323_spi_errors(void)
{
  return s_spi_errors;
}
