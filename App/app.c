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
 * LED4: 1 Hz heartbeat. PA3: high during each encoder read (scope trigger
 * and read-time check). PA2: mechanical angle as a sawtooth, 0-3.3 V per turn.
 * ENC status line every 200 ms; field, push and loss-of-track logged on change. */
#define STAGE1_LED_HALF_PERIOD_MS   500UL
#define STAGE1_LOG_PERIOD_MS        200UL
#define STAGE1_ANGLE_TO_DAC_SHIFT   2U      /* 14-bit angle -> 12-bit DAC */
#define RAD_S_TO_RPM                9.54929658551f   /* 60 / (2*pi) */
#endif

static uint32_t s_last_tick_ms;
static uint32_t s_task_max_cycles;

#if BRINGUP_STAGE == 1
static uint32_t s_enc_read_max_cycles;
static uint8_t s_enc_logged_status;
static bool s_enc_logged_valid;
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

/* ---- Stage 1 tick task ---------------------------------------------------- */

#if BRINGUP_STAGE == 1
static void stage1_log_status_change(const mt6701_snapshot_t *s)
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

static void stage1_tick(uint32_t now_ms)
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

  mt6701_snapshot_t s;
  mt6701_snapshot(&s);

  debug_scope_write((uint32_t)s.angle_counts >> STAGE1_ANGLE_TO_DAC_SHIFT);

  if (s.counters.reads > (s.counters.spi_timeouts + s.counters.crc_errors))
  {
    stage1_log_status_change(&s);
  }

  if ((now_ms % STAGE1_LED_HALF_PERIOD_MS) == 0U)
  {
    HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
  }

  if ((now_ms % STAGE1_LOG_PERIOD_MS) == 0U)
  {
    debug_log("ENC a=%u turns=%ld rpm=%ld ok=%u crc=%lu jump=%lu to=%lu strong=%lu weak=%lu lot=%lu rd_us=%lu",
              (unsigned)s.angle_counts, (long)s.turns,
              (long)(s.speed_rad_s * RAD_S_TO_RPM),
              s.healthy ? 1U : 0U,
              s.counters.crc_errors, s.counters.angle_jumps, s.counters.spi_timeouts,
              s.counters.field_strong, s.counters.field_weak, s.counters.loss_of_track,
              s_enc_read_max_cycles / DEBUG_CYCLES_PER_US);
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
  debug_log("SCOPE dac3+opamp1 %s", debug_ok ? "OK" : "FAIL");
  (void)app_check_clock();
  app_check_cycle_counter();
  app_check_safe_pins();

#if BRINGUP_STAGE >= 1
  bool enc_cfg_ok = mt6701_init(APP_TICK_S);
  debug_log("ENC spi1 mode1 12bit div32 %s", enc_cfg_ok ? "OK" : "FAIL");
  bool enc_ok = mt6701_update();
  mt6701_snapshot_t s;
  mt6701_snapshot(&s);
  debug_log("ENC first raw=0x%06lX a=%u st=0x%X crc=%s field=%s lot=%u",
            s.raw_word, (unsigned)s.angle_counts, (unsigned)s.status,
            (s.counters.crc_errors == 0U) ? "OK" : "FAIL",
            mt6701_field_name(mt6701_field(s.status)),
            mt6701_loss_of_track(s.status) ? 1U : 0U);
  (void)enc_ok;
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
#if BRINGUP_STAGE == 0
    stage0_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 1
    stage1_tick(s_last_tick_ms);
#endif
    uint32_t elapsed = debug_cycles() - start;
    if (elapsed > s_task_max_cycles)
    {
      s_task_max_cycles = elapsed;
    }

    if ((s_last_tick_ms % ALIVE_PERIOD_MS) == 0U)
    {
      debug_log("ALIVE t=%lus task_max_us=%lu", s_last_tick_ms / 1000UL,
                s_task_max_cycles / DEBUG_CYCLES_PER_US);
    }
  }
}
