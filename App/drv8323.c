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
#include "pwm.h"
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

/* CSA auto offset calibration takes 100 us after CSA_CAL_X is set
 * (datasheet 8.3.4.3); hold it twice that. */
#define DRV_CSA_CAL_US           200U

/* nFAULT interrupt: EXTI line 10 on port B (RM0440 SYSCFG_EXTICR3,
 * EXTI10 field: 0001 = PB). The pin stays in its TIM1_BKIN alternate
 * function; the EXTI takes the GPIO input path, which is active in AF mode.
 * Priority below the control interrupt and TIM1 break (0) and above SysTick
 * (plan: Timing, interrupts). */
#define DRV_EXTI_PORT_B          1UL
#define DRV_NFAULT_IRQ_PRIORITY  4U

/* ---- State ---------------------------------------------------------------- */

static bool s_awake;
static bool s_configured;
static uint32_t s_sleep_tick;
static uint32_t s_spi_errors;

/* Shared with the nFAULT interrupt. s_nfault_blank is true while the DRV is
 * asleep or in its wake/sleep window, when nFAULT pulses low by design
 * (datasheet 8.4.1.2). */
static volatile bool s_nfault_blank = true;
static volatile bool s_nfault_event;
static volatile uint32_t s_nfault_events;
static volatile uint32_t s_nfault_blanked_events;

/* IDRIVE code for 0x03 and 0x04; changed only while asleep. */
static uint8_t s_idrive = DRV_CFG_IDRIVE;

/* Peak gate currents per 4-bit code, mA (datasheet Tables 8-16, 8-17). */
static const uint16_t s_idrivep_ma[16] =
{
  10U, 30U, 60U, 80U, 120U, 140U, 170U, 190U, 260U, 330U, 370U, 440U, 570U, 680U, 820U, 1000U
};
static const uint16_t s_idriven_ma[16] =
{
  20U, 60U, 120U, 160U, 240U, 280U, 340U, 380U, 520U, 660U, 740U, 880U, 1140U, 1360U, 1640U, 2000U
};

/* ---- Helpers -------------------------------------------------------------- */

/* ENABLE as driven. motor_disarm() can drop it without drv8323_sleep()
 * (disarm command, fault path, later the control interrupt); sleep resets
 * the registers and disables SPI (datasheet 8.4.1.1), so awake/configured
 * hold only while ENABLE is high. */
static inline bool drv_enable_high(void)
{
  return (BOARD_DRV_ENABLE_PORT->ODR & BOARD_DRV_ENABLE_PIN) != 0U;
}

/* Configured value of control register 0x02-0x06, 0x03 as locked: the
 * shadow copy for the readback check. */
static uint16_t drv_cfg_value(uint8_t addr)
{
  switch (addr)
  {
    case DRV_REG_DRIVER_CTRL: return DRV_CFG_DRIVER_CTRL;
    case DRV_REG_GATE_HS:     return (uint16_t)(DRV_GATE_HS_LOCK_ON | s_idrive);
    case DRV_REG_GATE_LS:     return (uint16_t)(DRV_CFG_GATE_LS_CTRL | s_idrive);
    case DRV_REG_OCP_CTRL:    return DRV_CFG_OCP_CTRL;
    case DRV_REG_CSA_CTRL:    return DRV_CFG_CSA_CTRL;
    default:                  return 0U;
  }
}

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

  /* nFAULT falling-edge interrupt (plan: "Before the PWM is configured
   * (Stage 4), nFAULT is watched with an EXTI interrupt on PB10"). */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~SYSCFG_EXTICR3_EXTI10) |
                      (DRV_EXTI_PORT_B << SYSCFG_EXTICR3_EXTI10_Pos);
  EXTI->RTSR1 &= ~EXTI_RTSR1_RT10;
  EXTI->FTSR1 |= EXTI_FTSR1_FT10;
  EXTI->PR1 = EXTI_PR1_PIF10;
  EXTI->IMR1 |= EXTI_IMR1_IM10;
  NVIC_SetPriority(EXTI15_10_IRQn, DRV_NFAULT_IRQ_PRIORITY);
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  return cfg_ok;
}

/* nFAULT falling edge. Latch only; the main loop reads the fault registers
 * before anything puts the DRV to sleep (sleep resets them). */
void EXTI15_10_IRQHandler(void)
{
  if ((EXTI->PR1 & EXTI_PR1_PIF10) != 0U)
  {
    EXTI->PR1 = EXTI_PR1_PIF10;
    if (s_nfault_blank)
    {
      s_nfault_blanked_events++;
    }
    else
    {
      s_nfault_events++;
      s_nfault_event = true;
    }
  }
}

bool drv8323_configure(drv_cfg_result_t *r)
{
  /* Unlocked register writes, in order; 0x03 last, with LOCK 011b. */
  static const uint8_t order[] =
  {
    DRV_REG_DRIVER_CTRL, DRV_REG_GATE_LS, DRV_REG_OCP_CTRL, DRV_REG_CSA_CTRL, DRV_REG_GATE_HS
  };
  const uint16_t hs_unlocked = (uint16_t)(DRV_GATE_HS_LOCK_OFF | s_idrive);
  uint16_t rb = 0U;

  s_configured = false;
  r->status = DRV_CFG_OK;
  r->addr = DRV_REG_GATE_HS;
  r->wrote = 0U;
  r->read = 0U;

  if (!drv8323_present(&rb))
  {
    r->status = DRV_CFG_NOT_PRESENT;
    r->read = rb;
    return false;
  }

  for (uint32_t i = 0U; i < (sizeof(order) / sizeof(order[0])); i++)
  {
    uint16_t v = (order[i] == DRV_REG_GATE_HS) ? hs_unlocked : drv_cfg_value(order[i]);
    if (!drv8323_write_verify(order[i], v, &rb))
    {
      r->status = DRV_CFG_WRITE_FAIL;
      r->addr = order[i];
      r->wrote = v;
      r->read = rb;
      return false;
    }
  }

  /* CSA offset calibration: inputs shorted and auto-trimmed while
   * CSA_CAL_X = 1, then back to normal (datasheet 8.3.4.3). Before the
   * lock, because 0x06 can't be written once locked. */
  r->addr = DRV_REG_CSA_CTRL;
  r->wrote = DRV_CFG_CSA_CTRL | DRV_CSA_CAL_ALL;
  if (!drv8323_write_verify(DRV_REG_CSA_CTRL, DRV_CFG_CSA_CTRL | DRV_CSA_CAL_ALL, &rb))
  {
    r->status = DRV_CFG_CAL_FAIL;
    r->read = rb;
    return false;
  }
  drv_wait_cycles(DRV_CSA_CAL_US * DRV_CYCLES_PER_US);
  r->wrote = DRV_CFG_CSA_CTRL;
  if (!drv8323_write_verify(DRV_REG_CSA_CTRL, DRV_CFG_CSA_CTRL, &rb))
  {
    r->status = DRV_CFG_CAL_FAIL;
    r->read = rb;
    return false;
  }

  /* Lock. */
  r->addr = DRV_REG_GATE_HS;
  r->wrote = drv_cfg_value(DRV_REG_GATE_HS);
  if (!drv8323_write_verify(DRV_REG_GATE_HS, r->wrote, &rb))
  {
    r->status = DRV_CFG_LOCK_FAIL;
    r->read = rb;
    return false;
  }

  /* Lock test: a different value written to 0x05 must be ignored. */
  r->addr = DRV_REG_OCP_CTRL;
  r->wrote = DRV_LOCK_TEST_VALUE;
  if (!drv8323_write(DRV_REG_OCP_CTRL, DRV_LOCK_TEST_VALUE, NULL) ||
      !drv8323_read(DRV_REG_OCP_CTRL, &rb) || (rb != DRV_CFG_OCP_CTRL))
  {
    r->status = DRV_CFG_LOCK_TEST_FAIL;
    r->read = rb;
    return false;
  }
  r->read = rb;

  s_configured = true;
  return true;
}

bool drv8323_is_configured(void)
{
  return s_configured && drv_enable_high();
}

bool drv8323_sync(void)
{
  if (s_awake && !drv_enable_high())
  {
    drv8323_sleep();   /* finish the sleep: SPI3 off, PD2 released, tSLEEP timer */
    return true;
  }
  return false;
}

bool drv8323_check_config(uint8_t *addr, uint16_t *value)
{
  for (uint8_t a = DRV_REG_DRIVER_CTRL; a < DRV_REG_COUNT; a++)
  {
    uint16_t v = 0U;
    bool ok = drv8323_read(a, &v);
    if (!ok || (v != drv_cfg_value(a)))
    {
      *addr = a;
      *value = v;
      return false;
    }
  }
  return true;
}

bool drv8323_set_idrive(uint8_t code)
{
  if (s_awake)
  {
    return false;
  }
  s_idrive = code;
  return true;
}

uint8_t drv8323_idrive(void)
{
  return s_idrive;
}

void drv8323_idrive_ma(uint8_t code, uint32_t *source_ma, uint32_t *sink_ma)
{
  *source_ma = s_idrivep_ma[(code >> 4) & 0xFU];
  *sink_ma = s_idriven_ma[code & 0xFU];
}

uint32_t drv8323_nfault_events(void)
{
  return s_nfault_events;
}

uint32_t drv8323_nfault_blanked_events(void)
{
  return s_nfault_blanked_events;
}

bool drv8323_take_nfault_event(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool ev = s_nfault_event;
  s_nfault_event = false;
  __set_PRIMASK(primask);
  return ev;
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

  s_nfault_blank = true;      /* nFAULT is held low during wake */
  s_nfault_event = false;     /* drop any event left from before the last sleep */
  BOARD_DRV_ENABLE_PORT->BSRR = BOARD_DRV_ENABLE_PIN;
  drv_wait_cycles(DRV_WAKE_WAIT_US * DRV_CYCLES_PER_US);

  s_awake = true;
  s_configured = false;       /* registers are at their defaults after wake */
  bool released = !drv8323_nfault_low();
  if (released)
  {
    s_nfault_blank = false;   /* from here a falling edge is a real fault */
  }
  return released;
}

void drv8323_sleep(void)
{
  s_nfault_blank = true;      /* nFAULT pulses low while powering down */
  s_configured = false;
  motor_disarm();             /* outputs off, then ENABLE low */
  drv_spi3_disable();         /* SCLK/SDI no longer driven */
  drv_nscs_mode(DRV_MODER_ANALOG);   /* PD2 released; R38 holds nSCS high */
  s_awake = false;
  s_sleep_tick = HAL_GetTick();
}

bool drv8323_is_awake(void)
{
  return s_awake && drv_enable_high();
}

bool drv8323_nfault_low(void)
{
  return (BOARD_DRV_NFAULT_PORT->IDR & BOARD_DRV_NFAULT_PIN) == 0U;
}

bool drv8323_read(uint8_t addr, uint16_t *value)
{
  if (!drv8323_is_awake())
  {
    return false;
  }
  uint16_t tx = (uint16_t)(DRV_CMD_READ | ((uint16_t)(addr & 0xFU) << DRV_ADDR_SHIFT));
  return drv_xfer(tx, value);
}

bool drv8323_write(uint8_t addr, uint16_t value, uint16_t *previous)
{
  if (!drv8323_is_awake())
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
