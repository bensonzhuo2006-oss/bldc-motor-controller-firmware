/**
 * @file    app.c
 * @brief   BRINGUP_STAGE switch and the 1 kHz main-loop tasks.
 */
#include "app.h"
#include "app_config.h"
#include "board.h"
#include "debug.h"
#if BRINGUP_STAGE >= 1
#include "mt6701.h"
#endif
#if BRINGUP_STAGE >= 2
#include "power.h"
#endif
#include <stdbool.h>
#include <stdint.h>

#ifndef BRINGUP_STAGE
#error "BRINGUP_STAGE must be defined in app_config.h"
#endif

/* Main-loop tick: 1 kHz (plan: Timing, interrupts and data sharing). */
#define APP_TICK_S             0.001f

/* Expected clock tree (plan: TIM1 and control timing, "Clock"):
 * 24 MHz HSE, PLL M = 6, N = 85, R = 2 -> 170 MHz SYSCLK = HCLK. */
#define CLK_EXPECTED_HZ        170000000UL
#define CLK_EXPECTED_PLLM      6UL
#define CLK_EXPECTED_PLLN      85UL
#define CLK_EXPECTED_PLLR      2UL

/* Cycle-counter self-check: cycles over a HAL_Delay of this many ms. */
#define CYC_CHECK_MS           10UL

/* Main-loop status line period. */
#define ALIVE_PERIOD_MS        1000UL

#define RAD_S_TO_RPM           9.54929658551f   /* 60 / (2*pi) */

#if BRINGUP_STAGE == 0
/* Stage 0 test patterns (plan: Stage 0 procedure).
 * LED4: 500 ms on / 500 ms off, a 1.000 Hz square wave to time on the scope.
 * PA3:  toggled every 1 ms tick, a 500.0 Hz square wave.
 * PA2:  triangle, 100 ms up and 100 ms down (5 Hz), 0 to full scale. */
#define STAGE0_LED_HALF_PERIOD_MS   500UL
#define STAGE0_TRI_HALF_PERIOD_MS   100UL
#define STAGE0_FAULT_TEST_AT_MS     5000UL
#endif

#if BRINGUP_STAGE == 1
/* Stage 1 test mode (plan: Stage 1 procedure).
 * LED4: 1 Hz heartbeat. PA2: mechanical angle as a sawtooth, 0-3.3 V per turn.
 * ENC status line every 200 ms. */
#define STAGE1_LED_HALF_PERIOD_MS   500UL
#define STAGE1_LOG_PERIOD_MS        200UL
#define STAGE1_ANGLE_TO_DAC_SHIFT   2U      /* 14-bit angle -> 12-bit DAC */
#endif

#if BRINGUP_STAGE >= 2
/* LED4 shows the power state (plan Stage 2): NO_VM slow blink (1 Hz),
 * VM_SETTLING fast blink (5 Hz), VM_OK steady on. */
#define LED_NO_VM_HALF_PERIOD_MS     500UL
#define LED_SETTLING_HALF_PERIOD_MS  100UL
#endif

#if BRINGUP_STAGE == 2
/* Stage 2 test mode (plan: Stage 2 procedure).
 * PWR status line every 500 ms. On each MOTOR_EN rising edge (SW1 on), VM is
 * recorded every 1 ms for 500 ms, then printed every 10 ms with the times
 * from SW1 on to VM >= 10 V and to VM_OK (procedure step 4). */
#define STAGE2_LOG_PERIOD_MS         500UL
#define STAGE2_RAMP_SAMPLES          500U
#define STAGE2_RAMP_PRINT_STEP       10U
#define STAGE2_RAMP_10V_MV           10000UL
#endif

static uint32_t s_last_tick_ms;
static uint32_t s_task_max_cycles;

#if BRINGUP_STAGE >= 1
static uint32_t s_enc_read_max_cycles;
static uint8_t s_enc_logged_status;
static bool s_enc_logged_valid;
#endif

#if BRINGUP_STAGE == 2
static uint16_t s_ramp_mv[STAGE2_RAMP_SAMPLES];
static bool s_ramp_capturing;
static bool s_ramp_dumping;
static uint32_t s_ramp_idx;
static int32_t s_ramp_t10v_ms;
static int32_t s_ramp_tok_ms;
static bool s_en_raw_prev;
#endif

/* ---- Boot checks ---------------------------------------------------------- */

static bool app_check_clock(void)
{
  uint32_t pllcfgr = RCC->PLLCFGR;
  uint32_t pllm = ((pllcfgr & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1UL;
  uint32_t plln = (pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
  uint32_t pllr = (((pllcfgr & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1UL) * 2UL;
  bool hse_ready = (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0U);
  bool pll_from_hse = (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE);
  bool sys_from_pll = (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK);
  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  uint32_t hclk = HAL_RCC_GetHCLKFreq();

  bool ok = hse_ready && pll_from_hse && sys_from_pll &&
            (pllm == CLK_EXPECTED_PLLM) && (plln == CLK_EXPECTED_PLLN) &&
            (pllr == CLK_EXPECTED_PLLR) &&
            (sysclk == CLK_EXPECTED_HZ) && (hclk == CLK_EXPECTED_HZ) &&
            (SystemCoreClock == CLK_EXPECTED_HZ);

  debug_log("CLK hse_rdy=%u pll_src=%s sys_src=%s m=%lu n=%lu r=%lu",
            hse_ready ? 1U : 0U, pll_from_hse ? "HSE" : "OTHER",
            sys_from_pll ? "PLL" : "OTHER", pllm, plln, pllr);
  debug_log("CLK sysclk=%lu hclk=%lu pclk1=%lu pclk2=%lu %s",
            sysclk, hclk, HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq(),
            ok ? "OK" : "FAIL");
  return ok;
}

static void app_check_cycle_counter(void)
{
  uint32_t start = debug_cycles();
  HAL_Delay(CYC_CHECK_MS);
  uint32_t elapsed = debug_cycles() - start;

  /* HAL_Delay waits at least CYC_CHECK_MS and up to one tick more. */
  debug_log("CYC %lums=%lu expect>=%lu %s", CYC_CHECK_MS, elapsed,
            CYC_CHECK_MS * (DEBUG_CPU_HZ / 1000UL),
            (elapsed >= CYC_CHECK_MS * (DEBUG_CPU_HZ / 1000UL)) ? "OK" : "FAIL");
}

static void app_check_safe_pins(void)
{
  uint32_t err = board_safe_pins_check();
  debug_log("SAFE pwm_pins=0x%02lX err=0x%02lX %s",
            board_pwm_pins_read(), err, (err == 0U) ? "OK" : "FAIL");
}

/* ---- Stage 0 tick task ---------------------------------------------------- */

#if BRINGUP_STAGE == 0
static void stage0_tick(uint32_t now_ms)
{
  /* PA3: 500 Hz square wave. */
  debug_timing_toggle();

  /* LED4: 1 Hz. */
  if ((now_ms % STAGE0_LED_HALF_PERIOD_MS) == 0U)
  {
    HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
  }

  /* PA2: triangle wave, full DAC range. */
  uint32_t phase = now_ms % (2UL * STAGE0_TRI_HALF_PERIOD_MS);
  uint32_t ramp = (phase < STAGE0_TRI_HALF_PERIOD_MS) ? phase : (2UL * STAGE0_TRI_HALF_PERIOD_MS - phase);
  debug_scope_write((ramp * DEBUG_SCOPE_FULL_SCALE) / STAGE0_TRI_HALF_PERIOD_MS);

#if STAGE0_FAULT_TEST
  if (now_ms == STAGE0_FAULT_TEST_AT_MS)
  {
    __asm volatile("udf #0");  /* undefined instruction -> HardFault */
  }
#endif
}
#endif

/* ---- Encoder (Stage 1 onward) --------------------------------------------- */

#if BRINGUP_STAGE >= 1
static void enc_log_status_change(const mt6701_snapshot_t *s)
{
  if (!s_enc_logged_valid || (mt6701_field(s->status) != mt6701_field(s_enc_logged_status)))
  {
    debug_log("ENC field=%s", mt6701_field_name(mt6701_field(s->status)));
  }
  if (!s_enc_logged_valid || (mt6701_loss_of_track(s->status) != mt6701_loss_of_track(s_enc_logged_status)))
  {
    debug_log("ENC loss_of_track=%u", mt6701_loss_of_track(s->status) ? 1U : 0U);
  }
  if (!s_enc_logged_valid || (mt6701_push(s->status) != mt6701_push(s_enc_logged_status)))
  {
    debug_log("ENC push=%u", mt6701_push(s->status) ? 1U : 0U);
  }
  s_enc_logged_status = s->status;
  s_enc_logged_valid = true;
}

/* Read the encoder every tick. PA3 is high during the read (scope trigger)
 * until the control interrupt takes over PA3 in Stage 7. */
static void enc_tick(mt6701_snapshot_t *s)
{
  debug_timing_high();
  uint32_t start = debug_cycles();
  (void)mt6701_update();
  uint32_t elapsed = debug_cycles() - start;
  debug_timing_low();
  if (elapsed > s_enc_read_max_cycles)
  {
    s_enc_read_max_cycles = elapsed;
  }

  mt6701_snapshot(s);
  if (s->counters.reads > (s->counters.spi_timeouts + s->counters.crc_errors))
  {
    enc_log_status_change(s);
  }
}
#endif

/* ---- Stage 1 tick task ---------------------------------------------------- */

#if BRINGUP_STAGE == 1
static void stage1_tick(uint32_t now_ms, const mt6701_snapshot_t *s)
{
  debug_scope_write((uint32_t)s->angle_counts >> STAGE1_ANGLE_TO_DAC_SHIFT);

  if ((now_ms % STAGE1_LED_HALF_PERIOD_MS) == 0U)
  {
    HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
  }

  if ((now_ms % STAGE1_LOG_PERIOD_MS) == 0U)
  {
    debug_log("ENC a=%u turns=%ld rpm=%ld ok=%u crc=%lu jump=%lu to=%lu strong=%lu weak=%lu lot=%lu rd_us=%lu",
              (unsigned)s->angle_counts, (long)s->turns,
              (long)(s->speed_rad_s * RAD_S_TO_RPM),
              s->healthy ? 1U : 0U,
              s->counters.crc_errors, s->counters.angle_jumps, s->counters.spi_timeouts,
              s->counters.field_strong, s->counters.field_weak, s->counters.loss_of_track,
              s_enc_read_max_cycles / DEBUG_CYCLES_PER_US);
  }
}
#endif

/* ---- Power (Stage 2 onward) ----------------------------------------------- */

#if BRINGUP_STAGE >= 2
static void power_tick(uint32_t now_ms)
{
  power_state_t prev = power_state();
  if (power_update())
  {
    debug_log("PWR %s -> %s en=%u vm_mv=%lu",
              power_state_name(prev), power_state_name(power_state()),
              power_motor_en() ? 1U : 0U, power_vm_mv());
  }

  /* LED4 shows the power state. */
  switch (power_state())
  {
    case POWER_VM_OK:
      BOARD_LED_PORT->BSRR = BOARD_LED_PIN;
      break;
    case POWER_VM_SETTLING:
      if ((now_ms % LED_SETTLING_HALF_PERIOD_MS) == 0U)
      {
        HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
      }
      break;
    case POWER_NO_VM:
    default:
      if ((now_ms % LED_NO_VM_HALF_PERIOD_MS) == 0U)
      {
        HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
      }
      break;
  }
}
#endif

/* ---- Stage 2 tick task ---------------------------------------------------- */

#if BRINGUP_STAGE == 2
static void stage2_ramp(void)
{
  bool en_raw = power_motor_en_raw();

  if (en_raw && !s_en_raw_prev && !s_ramp_capturing && !s_ramp_dumping)
  {
    s_ramp_capturing = true;
    s_ramp_idx = 0U;
    s_ramp_t10v_ms = -1;
    s_ramp_tok_ms = -1;
  }
  s_en_raw_prev = en_raw;

  if (s_ramp_capturing)
  {
    uint32_t mv = power_vm_mv();
    s_ramp_mv[s_ramp_idx] = (uint16_t)((mv > 0xFFFFUL) ? 0xFFFFUL : mv);
    if ((s_ramp_t10v_ms < 0) && (mv >= STAGE2_RAMP_10V_MV))
    {
      s_ramp_t10v_ms = (int32_t)s_ramp_idx;
    }
    if ((s_ramp_tok_ms < 0) && power_vm_ok())
    {
      s_ramp_tok_ms = (int32_t)s_ramp_idx;
    }
    s_ramp_idx++;
    if (s_ramp_idx >= STAGE2_RAMP_SAMPLES)
    {
      s_ramp_capturing = false;
      s_ramp_dumping = true;
      s_ramp_idx = 0U;
    }
  }
  else if (s_ramp_dumping)
  {
    /* One line per tick, so printing never holds up the loop. */
    if (s_ramp_idx < STAGE2_RAMP_SAMPLES)
    {
      debug_log("RAMP t=%lums vm_mv=%u", s_ramp_idx, (unsigned)s_ramp_mv[s_ramp_idx]);
      s_ramp_idx += STAGE2_RAMP_PRINT_STEP;
    }
    else
    {
      debug_log("RAMP sw1_on->10V=%ldms sw1_on->VM_OK=%ldms (-1 = not within %ums)",
                (long)s_ramp_t10v_ms, (long)s_ramp_tok_ms, STAGE2_RAMP_SAMPLES);
      s_ramp_dumping = false;
    }
  }
}

static void stage2_tick(uint32_t now_ms)
{
  stage2_ramp();

  if (((now_ms % STAGE2_LOG_PERIOD_MS) == 0U) && !s_ramp_capturing && !s_ramp_dumping)
  {
    debug_log("PWR st=%s en=%u en_raw=%u vm_mv=%lu code=%u adc_to=%lu",
              power_state_name(power_state()), power_motor_en() ? 1U : 0U,
              power_motor_en_raw() ? 1U : 0U, power_vm_mv(),
              (unsigned)power_vm_code(), power_adc_timeouts());
  }
}
#endif

/* ---- Entry points --------------------------------------------------------- */

void app_init(void)
{
  /* Safe pins first, before anything is logged. */
  board_safe_pins();

  bool debug_ok = debug_init();

  debug_log("BOOT stage=%d", BRINGUP_STAGE);
#if BRINGUP_STAGE >= 2
  debug_log_reset_cause();
#endif
  debug_log("SCOPE dac3+opamp1 %s", debug_ok ? "OK" : "FAIL");
  (void)app_check_clock();
  app_check_cycle_counter();
  app_check_safe_pins();

#if BRINGUP_STAGE >= 1
  bool enc_cfg_ok = mt6701_init(APP_TICK_S);
  debug_log("ENC spi1 mode1 12bit div32 %s", enc_cfg_ok ? "OK" : "FAIL");
  (void)mt6701_update();
  mt6701_snapshot_t s;
  mt6701_snapshot(&s);
  debug_log("ENC first raw=0x%06lX a=%u st=0x%X crc=%s field=%s lot=%u",
            s.raw_word, (unsigned)s.angle_counts, (unsigned)s.status,
            (s.counters.crc_errors == 0U) ? "OK" : "FAIL",
            mt6701_field_name(mt6701_field(s.status)),
            mt6701_loss_of_track(s.status) ? 1U : 0U);
#endif

#if BRINGUP_STAGE >= 2
  bool pwr_ok = power_init();
  debug_log("PWR adc1 cal+enable %s en=%u vm_mv=%lu", pwr_ok ? "OK" : "FAIL",
            power_motor_en() ? 1U : 0U, power_vm_mv());
#endif

  s_last_tick_ms = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  /* Run every 1 ms tick once, catching up if the loop was late. */
  while (s_last_tick_ms != now)
  {
    s_last_tick_ms++;

    uint32_t start = debug_cycles();
#if BRINGUP_STAGE >= 1
    mt6701_snapshot_t enc;
    enc_tick(&enc);
#endif
#if BRINGUP_STAGE >= 2
    power_tick(s_last_tick_ms);
#endif

#if BRINGUP_STAGE == 0
    stage0_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 1
    stage1_tick(s_last_tick_ms, &enc);
#elif BRINGUP_STAGE == 2
    stage2_tick(s_last_tick_ms);
#endif
    uint32_t elapsed = debug_cycles() - start;
    if (elapsed > s_task_max_cycles)
    {
      s_task_max_cycles = elapsed;
    }

    if ((s_last_tick_ms % ALIVE_PERIOD_MS) == 0U)
    {
#if BRINGUP_STAGE >= 1
      debug_log("ALIVE t=%lus task_max_us=%lu enc_ok=%u enc_crc=%lu enc_jump=%lu",
                s_last_tick_ms / 1000UL, s_task_max_cycles / DEBUG_CYCLES_PER_US,
                enc.healthy ? 1U : 0U, enc.counters.crc_errors, enc.counters.angle_jumps);
#else
      debug_log("ALIVE t=%lus task_max_us=%lu", s_last_tick_ms / 1000UL,
                s_task_max_cycles / DEBUG_CYCLES_PER_US);
#endif
    }
  }
}
