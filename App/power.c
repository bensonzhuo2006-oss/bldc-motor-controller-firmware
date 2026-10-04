/**
 * @file    power.c
 * @brief   MOTOR_EN debounce, VM_SENSE on ADC1 and the VM power state machine.
 */
#include "power.h"
#include "board.h"
#include "adc.h"
#include "main.h"

/* ---- Constants ------------------------------------------------------------ */

/* VM_SENSE: PB11 = ADC1_IN14 (CubeMX regular rank 1). Divider 180k over 10k
 * (plan pin map; schematic peripherals sheet), so VM = V_pin x 19. C26 100 nF
 * across the 10k gives tau = 9.47k x 100 nF = 0.95 ms and supplies the ADC's
 * sampling charge, so the 2.5-cycle sampling time set in CubeMX is enough. */
#define POWER_VM_DIVIDER_RATIO    19.0f
#define POWER_VDDA_V              3.3f      /* VREF+ = VDDA = 3.3 V LDO (plan: Current sensing) */
#define POWER_ADC_COUNTS          4096.0f   /* 12-bit */
#define POWER_SAMPLES_PER_TICK    4U        /* averaged per 1 ms update */

/* Thresholds (plan Stage 2): VM_OK above 10 V held 20 ms; below 9 V drops. */
#define POWER_VM_ON_MV            10000UL
#define POWER_VM_OFF_MV           9000UL
#define POWER_SETTLE_MS           20U

/* MOTOR_EN debounce: SW1 contact bounce; must be stable this long. */
#define POWER_EN_DEBOUNCE_MS      5U

/* ADC timeouts: one conversion is (2.5 + 12.5) ADC clocks at 42.5 MHz
 * (HCLK / 4) = 0.35 us; 1,700 CPU cycles = 10 us. ADRDY after ADEN takes a
 * few us; allow 2 ms. Calibration-to-enable gap: >= 4 ADC clocks (RM0440 ADC
 * calibration), 16 CPU cycles; 100 used. */
#define POWER_ADC_CONV_TIMEOUT_CYCLES  1700U
#define POWER_ADC_READY_TIMEOUT_MS     2U
#define POWER_ADC_CAL_TO_EN_CYCLES     100U

/* ---- State ---------------------------------------------------------------- */

static power_state_t s_state = POWER_NO_VM;
static uint32_t s_settle_ms;
static bool s_en_debounced;
static uint32_t s_en_change_ms;
static uint32_t s_vm_mv;
static uint16_t s_vm_code;
static uint32_t s_adc_timeouts;

/* ---- ADC ------------------------------------------------------------------ */

static bool power_adc_read(uint16_t *code)
{
  LL_ADC_REG_StartConversion(ADC1);
  uint32_t start = DWT->CYCCNT;
  while (!LL_ADC_IsActiveFlag_EOC(ADC1))
  {
    if ((DWT->CYCCNT - start) > POWER_ADC_CONV_TIMEOUT_CYCLES)
    {
      return false;
    }
  }
  *code = LL_ADC_REG_ReadConversionData12(ADC1);  /* reading DR clears EOC */
  return true;
}

/* ---- API ------------------------------------------------------------------ */

bool power_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* DWT for timeouts */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return false;
  }
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < POWER_ADC_CAL_TO_EN_CYCLES)
  {
  }

  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  uint32_t t0 = HAL_GetTick();
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1))
  {
    if ((HAL_GetTick() - t0) > POWER_ADC_READY_TIMEOUT_MS)
    {
      return false;
    }
  }

  s_state = POWER_NO_VM;
  s_settle_ms = 0U;
  s_en_debounced = power_motor_en_raw();
  s_en_change_ms = 0U;
  s_adc_timeouts = 0U;
  (void)power_update();
  s_state = POWER_NO_VM;   /* always start from NO_VM; settling begins next tick */
  s_settle_ms = 0U;
  return true;
}

bool power_update(void)
{
  /* VM: average a few conversions. */
  uint32_t sum = 0U;
  uint32_t n = 0U;
  for (uint32_t i = 0U; i < POWER_SAMPLES_PER_TICK; i++)
  {
    uint16_t code;
    if (power_adc_read(&code))
    {
      sum += code;
      n++;
    }
    else
    {
      s_adc_timeouts++;
    }
  }
  if (n > 0U)
  {
    s_vm_code = (uint16_t)(sum / n);
    float vm_v = ((float)s_vm_code * POWER_VDDA_V / POWER_ADC_COUNTS) * POWER_VM_DIVIDER_RATIO;
    s_vm_mv = (uint32_t)(vm_v * 1000.0f);
  }
  else
  {
    s_vm_mv = 0U;   /* no valid reading: treat as no VM */
  }

  /* MOTOR_EN debounce. */
  bool en_raw = power_motor_en_raw();
  if (en_raw != s_en_debounced)
  {
    s_en_change_ms++;
    if (s_en_change_ms >= POWER_EN_DEBOUNCE_MS)
    {
      s_en_debounced = en_raw;
      s_en_change_ms = 0U;
    }
  }
  else
  {
    s_en_change_ms = 0U;
  }

  /* State machine. */
  power_state_t prev = s_state;
  bool on_cond = s_en_debounced && (s_vm_mv >= POWER_VM_ON_MV);
  bool off_cond = !s_en_debounced || (s_vm_mv < POWER_VM_OFF_MV);

  switch (s_state)
  {
    case POWER_NO_VM:
      if (on_cond)
      {
        s_state = POWER_VM_SETTLING;
        s_settle_ms = 0U;
      }
      break;

    case POWER_VM_SETTLING:
      if (!on_cond)
      {
        s_state = POWER_NO_VM;
      }
      else if (++s_settle_ms >= POWER_SETTLE_MS)
      {
        s_state = POWER_VM_OK;
      }
      break;

    case POWER_VM_OK:
    default:
      if (off_cond)
      {
        s_state = POWER_NO_VM;
      }
      break;
  }

  return s_state != prev;
}

power_state_t power_state(void)
{
  return s_state;
}

const char *power_state_name(power_state_t state)
{
  switch (state)
  {
    case POWER_NO_VM:       return "NO_VM";
    case POWER_VM_SETTLING: return "VM_SETTLING";
    case POWER_VM_OK:       return "VM_OK";
    default:                return "?";
  }
}

bool power_vm_ok(void)
{
  return s_state == POWER_VM_OK;
}

bool power_motor_en(void)
{
  return s_en_debounced;
}

bool power_motor_en_raw(void)
{
  return (BOARD_MOTOR_EN_PORT->IDR & BOARD_MOTOR_EN_PIN) != 0U;
}

uint32_t power_vm_mv(void)
{
  return s_vm_mv;
}

uint16_t power_vm_code(void)
{
  return s_vm_code;
}

uint32_t power_adc_timeouts(void)
{
  return s_adc_timeouts;
}
