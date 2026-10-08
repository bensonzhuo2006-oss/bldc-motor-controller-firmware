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
#if BRINGUP_STAGE >= 3
#include "drv8323.h"
#endif
#if BRINGUP_STAGE >= 4
#include "fault.h"
#endif
#if BRINGUP_STAGE >= 5
#include "pwm.h"
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

#if BRINGUP_STAGE == 3
/* Stage 3 test mode (plan: Stage 3 procedure). After every wake: presence
 * check, all registers read and compared with their defaults, write and
 * verify 0x05, lock test, unlock, CLR_FLT, then sleep/wake and check the
 * defaults came back. Status line every second while running. */
#define STAGE3_LOG_PERIOD_MS        1000UL
#define STAGE3_TEST_OCP_VALUE       0x110U   /* plan register table: bring-up 0x05 */
#define STAGE3_FAULT_TEXT_LEN       96U
#endif

#if BRINGUP_STAGE >= 4
/* DRV configuration readback period (plan: readback check 3, every 100 ms). */
#define DRV_CHECK_PERIOD_MS         100UL
#define DRV_FAULT_TEXT_LEN          96U
#endif

#if BRINGUP_STAGE == 4
/* Stage 4 test mode: status line every second with the configuration
 * counters, for the SW1 cycling test (plan: Stage 4 procedure). */
#define STAGE4_LOG_PERIOD_MS        1000UL
#endif

#if BRINGUP_STAGE >= 5
/* Debugger commands: pause, `set var g_app_cmd = N` in the Debug Console,
 * continue (Stage 5 onward). Arming is always an explicit action (plan:
 * Rules for the bench, 4). */
#define APP_CMD_NONE       0U
#define APP_CMD_ARM        1U   /* arm from the zero command, then slew to the test duties */
#define APP_CMD_DISARM     2U
#define APP_CMD_SW_BREAK   3U   /* TIM1 software break */
#define APP_CMD_SWEEP      4U   /* Stage 6: duty sweep on all phases */
#define APP_CMD_IDRIVE     5U   /* Stage 6: apply g_drv_idrive (sleep, re-wake, reconfigure) */
volatile uint32_t g_app_cmd;
#endif

#if (BRINGUP_STAGE == 5) || (BRINGUP_STAGE == 6)
/* PWM test mode. Target duty per phase, 0..1, set from the debugger
 * (`set var g_duty_a = 0.5`). The applied duties slew toward the targets,
 * so every arm starts from the zero command and every change ramps
 * (plan: Hard rules). Targets are clamped to [0, PWM_DUTY_MAX]. */
#define TEST_SLEW_PER_MS     0.001f   /* full scale in 1 s */
#define TEST_LOG_PERIOD_MS   1000UL
#if BRINGUP_STAGE == 5
/* Plan Stage 5 step 1: a different duty per phase to identify each. */
#define TEST_DUTY_A_INIT     0.20f
#define TEST_DUTY_B_INIT     0.50f
#define TEST_DUTY_C_INIT     0.80f
#else
/* Plan Stage 6 step 1: one phase switching at 50 %; the others at 0 %
 * (low side held on, not switching). */
#define TEST_DUTY_A_INIT     0.50f
#define TEST_DUTY_B_INIT     0.00f
#define TEST_DUTY_C_INIT     0.00f
#endif
volatile float g_duty_a = TEST_DUTY_A_INIT;
volatile float g_duty_b = TEST_DUTY_B_INIT;
volatile float g_duty_c = TEST_DUTY_C_INIT;
#endif

#if BRINGUP_STAGE == 6
/* Plan Stage 6 step 4: duty sweep, all phases together, each step held
 * long enough to read the switch-node average on a meter. The plan says
 * 5-95 %; the top step is the PWM_DUTY_MAX cap (93 %). */
#define SWEEP_STEP_MS        4000UL
static const float s_sweep_duty[] = { 0.05f, 0.10f, 0.25f, 0.50f, 0.75f, 0.90f, PWM_DUTY_MAX };
#define SWEEP_STEPS          (sizeof(s_sweep_duty) / sizeof(s_sweep_duty[0]))

/* Plan Stage 6 step 2: IDRIVE code to try, IDRIVEP << 4 | IDRIVEN
 * (datasheet Tables 8-16, 8-17), same on both sides. Applied by command 5. */
volatile uint32_t g_drv_idrive = DRV_CFG_IDRIVE;

/* Phases held off at the next arm, bit 0 = A, 1 = B, 2 = C: their INH and
 * INL are driven low, so both FETs stay off and the phase floats. Isolates
 * one half-bridge (2026-10-08, VDS_HA on phase A). */
volatile uint32_t g_phase_off;
#endif

static uint32_t s_last_tick_ms;
static uint32_t s_task_max_cycles;

#if (BRINGUP_STAGE == 5) || (BRINGUP_STAGE == 6)
static float s_duty[PHASE_COUNT];   /* applied duties */
#endif
#if BRINGUP_STAGE == 6
static uint32_t s_sweep_step;
static uint32_t s_sweep_ms;
static bool s_sweeping;
#endif

#if BRINGUP_STAGE >= 4
static uint32_t s_drv_wakes;
static uint32_t s_drv_cfg_ok;
static uint32_t s_drv_cfg_fail;
static uint32_t s_drv_checks;
static uint32_t s_drv_spi_err_seen;
static bool s_fault_logged;
#endif

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

/* ---- Stage 3 test sequence ------------------------------------------------ */

#if BRINGUP_STAGE == 3
static bool stage3_log_faults(void)
{
  uint16_t f1 = 0U;
  uint16_t f2 = 0U;
  char text[STAGE3_FAULT_TEXT_LEN];
  bool ok = drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2);
  drv8323_fault_text(f1, f2, text, sizeof(text));
  debug_log("DRV faults f1=0x%03X f2=0x%03X %s nfault=%s", (unsigned)f1, (unsigned)f2,
            ok ? text : "READ_FAIL", drv8323_nfault_low() ? "LOW" : "high");
  return ok && (f1 == 0U) && (f2 == 0U);
}

static bool stage3_dump_regs(void)
{
  static const uint16_t defaults[DRV_REG_COUNT] =
  {
    0U, 0U, DRV_DEFAULT_DRIVER_CTRL, DRV_DEFAULT_GATE_HS, DRV_DEFAULT_GATE_LS,
    DRV_DEFAULT_OCP_CTRL, DRV_DEFAULT_CSA_CTRL
  };
  bool all_ok = true;

  for (uint8_t addr = 0U; addr < DRV_REG_COUNT; addr++)
  {
    uint16_t v = 0U;
    bool ok = drv8323_read(addr, &v);
    if (addr <= DRV_REG_FAULT2)
    {
      debug_log("DRV reg 0x%02X = 0x%03X %s", addr, (unsigned)v, ok ? "" : "READ_FAIL");
    }
    else
    {
      bool match = ok && (v == defaults[addr]);
      debug_log("DRV reg 0x%02X = 0x%03X default=0x%03X %s", addr, (unsigned)v,
                (unsigned)defaults[addr], match ? "OK" : "FAIL");
      all_ok = all_ok && match;
    }
  }
  return all_ok;
}

static void stage3_test(void)
{
  bool pass = true;
  bool ok;
  uint16_t v = 0U;

  /* 1. Presence: 0x03 must read 0x3FF (0x7FF = no reply, SDO pull-up). */
  ok = drv8323_present(&v);
  debug_log("DRV present reg03=0x%03X %s", (unsigned)v,
            ok ? "OK" : ((v == DRV_DATA_MASK) ? "FAIL (0x7FF = no reply)" : "FAIL"));
  if (!ok)
  {
    debug_log("DRV test FAIL (not present)");
    return;
  }

  /* 2. All registers at their defaults; no faults after wake. */
  pass &= stage3_dump_regs();
  pass &= stage3_log_faults();

  /* 3. Write and verify 0x05. */
  ok = drv8323_write_verify(DRV_REG_OCP_CTRL, STAGE3_TEST_OCP_VALUE, &v);
  debug_log("DRV write 0x05=0x%03X readback=0x%03X %s", STAGE3_TEST_OCP_VALUE, (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;

  /* 4. Lock, then try to restore 0x05: the write must be ignored. */
  ok = drv8323_write_verify(DRV_REG_GATE_HS, DRV_GATE_HS_LOCK_ON | (DRV_DEFAULT_GATE_HS & ~DRV_GATE_HS_LOCK_MASK), &v);
  debug_log("DRV lock reg03=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;
  ok = drv8323_write(DRV_REG_OCP_CTRL, DRV_DEFAULT_OCP_CTRL, NULL) && drv8323_read(DRV_REG_OCP_CTRL, &v) &&
       (v == STAGE3_TEST_OCP_VALUE);
  debug_log("DRV locked write to 0x05 ignored: reg05=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;

  /* 5. Unlock, then the restore must work. */
  ok = drv8323_write_verify(DRV_REG_GATE_HS, DRV_DEFAULT_GATE_HS, &v);
  debug_log("DRV unlock reg03=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;
  ok = drv8323_write_verify(DRV_REG_OCP_CTRL, DRV_DEFAULT_OCP_CTRL, &v);
  debug_log("DRV unlocked write 0x05=0x%03X readback=0x%03X %s", DRV_DEFAULT_OCP_CTRL, (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;

  /* 6. CLR_FLT: bit self-clears, so 0x02 reads back 0x000. */
  ok = drv8323_write(DRV_REG_DRIVER_CTRL, DRV_DRIVER_CTRL_CLR_FLT, NULL) &&
       drv8323_read(DRV_REG_DRIVER_CTRL, &v) && (v == DRV_DEFAULT_DRIVER_CTRL);
  debug_log("DRV clr_flt reg02=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;
  pass &= stage3_log_faults();

  /* 7. Sleep and wake: registers must return to defaults. */
  ok = drv8323_write_verify(DRV_REG_OCP_CTRL, STAGE3_TEST_OCP_VALUE, &v);
  pass &= ok;
  drv8323_sleep();
  bool nfault_ok = drv8323_wake();
  ok = drv8323_read(DRV_REG_OCP_CTRL, &v) && (v == DRV_DEFAULT_OCP_CTRL);
  debug_log("DRV sleep/wake reset: reg05=0x%03X (was 0x%03X) nfault=%s %s", (unsigned)v,
            STAGE3_TEST_OCP_VALUE, nfault_ok ? "high" : "LOW", ok ? "OK" : "FAIL");
  pass &= ok && nfault_ok;
  pass &= stage3_dump_regs();

  debug_log("DRV test %s spi_err=%lu", pass ? "PASS" : "FAIL", drv8323_spi_errors());
}
#endif

/* ---- Gate driver (Stage 3 onward) ----------------------------------------- */

#if BRINGUP_STAGE >= 4
static fault_code_t drv_cfg_fault_code(drv_cfg_status_t status)
{
  switch (status)
  {
    case DRV_CFG_NOT_PRESENT:    return FAULT_DRV_NOT_PRESENT;
    case DRV_CFG_LOCK_TEST_FAIL: return FAULT_DRV_LOCK_TEST;
    default:                     return FAULT_DRV_CONFIG_WRITE;
  }
}

static void drv_log_registers(const char *prefix)
{
  uint16_t v[DRV_REG_COUNT] = {0U};
  for (uint8_t a = 0U; a < DRV_REG_COUNT; a++)
  {
    (void)drv8323_read(a, &v[a]);
  }
  debug_log("%s f1=0x%03X f2=0x%03X 02=0x%03X 03=0x%03X 04=0x%03X 05=0x%03X 06=0x%03X",
            prefix, (unsigned)v[0], (unsigned)v[1], (unsigned)v[2], (unsigned)v[3],
            (unsigned)v[4], (unsigned)v[5], (unsigned)v[6]);
}

/* After a wake: configure, verify, calibrate, lock (plan Stage 4). */
static void drv_configure_after_wake(bool nfault_released, uint32_t blanked_before)
{
  uint32_t pulses = drv8323_nfault_blanked_events() - blanked_before;

  s_drv_wakes++;
  if (!nfault_released)
  {
    debug_log("DRV wake #%lu nfault still LOW after %s", s_drv_wakes, "1.1 ms");
    (void)fault_raise(FAULT_DRV_WAKE, 0U, 0U);
    return;
  }

  drv_cfg_result_t r;
  if (drv8323_configure(&r))
  {
    s_drv_cfg_ok++;
    debug_log("DRV wake #%lu nfault=high wake_pulses_ignored=%lu cfg OK (cal done, locked, lock test reg05=0x%03X)",
              s_drv_wakes, pulses, (unsigned)r.read);
    drv_log_registers("DRV cfg");
  }
  else
  {
    s_drv_cfg_fail++;
    debug_log("DRV wake #%lu cfg FAIL status=%u reg=0x%02X wrote=0x%03X read=0x%03X",
              s_drv_wakes, (unsigned)r.status, r.addr, (unsigned)r.wrote, (unsigned)r.read);
    (void)fault_raise(drv_cfg_fault_code(r.status),
                      ((uint32_t)r.addr << 16) | r.wrote, r.read);
  }
}

/* Every tick while awake: nFAULT events, SPI errors, 100 ms readback.
 * Then the fault response: log, read the fault registers, put the DRV to
 * sleep (disarm). Faults stay latched until g_fault_clear_request. */
static void drv_monitor(uint32_t now_ms)
{
  if (g_fault_clear_request != 0U)
  {
    g_fault_clear_request = 0U;
    debug_log("FAULT cleared by request (was %s, count=%lu)",
              fault_name(fault_first()), fault_count());
    fault_clear();
    s_fault_logged = false;
  }

  if (drv8323_is_awake() && !fault_active())
  {
    if (drv8323_take_nfault_event())
    {
      uint16_t f1 = 0U;
      uint16_t f2 = 0U;
      (void)drv8323_read(DRV_REG_FAULT1, &f1);
      (void)drv8323_read(DRV_REG_FAULT2, &f2);
      (void)fault_raise(FAULT_DRV_NFAULT, f1, f2);
    }
    else if (drv8323_is_configured() && ((now_ms % DRV_CHECK_PERIOD_MS) == 0U))
    {
      uint8_t addr = 0U;
      uint16_t val = 0U;
      uint16_t f1 = 0U;
      uint16_t f2 = 0U;
      s_drv_checks++;
      if (!drv8323_check_config(&addr, &val))
      {
        (void)fault_raise(FAULT_DRV_CONFIG_MISMATCH, addr, val);
      }
      else if (drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2) &&
               ((f1 != 0U) || (f2 != 0U)))
      {
        (void)fault_raise(FAULT_DRV_FAULT_BITS, f1, f2);
      }
    }

    if (drv8323_spi_errors() != s_drv_spi_err_seen)
    {
      s_drv_spi_err_seen = drv8323_spi_errors();
      (void)fault_raise(FAULT_DRV_SPI, s_drv_spi_err_seen, 0U);
    }
  }

#if BRINGUP_STAGE >= 5
  if (pwm_take_break_event())
  {
    (void)fault_raise(FAULT_PWM_BREAK, pwm_break_events(), board_pwm_pins_read());
  }
#endif

  if (fault_active() && !s_fault_logged)
  {
    uint32_t c1;
    uint32_t c2;
    uint32_t t;
    fault_first_context(&c1, &c2, &t);
    debug_log("FAULT latched %s ctx1=0x%lX ctx2=0x%lX t=%lums", fault_name(fault_first()), c1, c2, t);
    if (drv8323_is_awake())
    {
      uint16_t f1 = 0U;
      uint16_t f2 = 0U;
      char text[DRV_FAULT_TEXT_LEN];
      bool ok = drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2);
      drv8323_fault_text(f1, f2, text, sizeof(text));
      debug_log("FAULT drv f1=0x%03X f2=0x%03X %s nfault=%s", (unsigned)f1, (unsigned)f2,
                ok ? text : "READ_FAIL", drv8323_nfault_low() ? "LOW" : "high");
      drv8323_sleep();   /* disarms first: outputs off, then ENABLE low */
    }
#if BRINGUP_STAGE >= 5
    motor_disarm();
#endif
    debug_log("FAULT disarmed, DRV asleep. Set g_fault_clear_request=1 in the debugger to clear.");
    s_fault_logged = true;
  }
}
#endif

#if BRINGUP_STAGE >= 3
/* The DRV is awake exactly while VM_OK holds and no fault is latched
 * (plan Stage 3: wake on VM_OK; on VM_OK loss disarm, ENABLE low, release
 * PD2; Stage 4: configure after every wake). Runs after power_tick(). */
static void drv_tick(void)
{
  bool vm_ok = power_vm_ok();

  if (drv8323_sync())
  {
    debug_log("DRV sleep (ENABLE low after disarm); re-wakes if VM_OK and no fault");
  }
#if BRINGUP_STAGE == 5
  /* Stage 5: PWM into a sleeping DRV, so ENABLE is held low throughout. */
  bool may_wake = false;
#elif BRINGUP_STAGE >= 4
  bool may_wake = vm_ok && !fault_active();
#else
  bool may_wake = vm_ok;
#endif

  if (may_wake && !drv8323_is_awake())
  {
#if BRINGUP_STAGE >= 4
    uint32_t blanked_before = drv8323_nfault_blanked_events();
    bool nfault_released = drv8323_wake();
    drv_configure_after_wake(nfault_released, blanked_before);
#else
    bool nfault_released = drv8323_wake();
    debug_log("DRV wake nfault=%s", nfault_released ? "high" : "LOW");
#endif
#if BRINGUP_STAGE == 3
    stage3_test();
#endif
  }
  else if (!vm_ok && drv8323_is_awake())
  {
    drv8323_sleep();
    debug_log("DRV sleep (VM_OK lost) vm_mv=%lu", power_vm_mv());
  }
}
#endif

#if BRINGUP_STAGE == 4
static void stage4_tick(uint32_t now_ms)
{
  if ((now_ms % STAGE4_LOG_PERIOD_MS) != 0U)
  {
    return;
  }
  const char *st = drv8323_is_configured() ? "CONFIGURED" : (drv8323_is_awake() ? "AWAKE" : "ASLEEP");
  uint16_t f1 = 0U;
  uint16_t f2 = 0U;
  if (drv8323_is_awake())
  {
    (void)drv8323_read(DRV_REG_FAULT1, &f1);
    (void)drv8323_read(DRV_REG_FAULT2, &f2);
  }
  debug_log("DRV %s pwr=%s nfault=%s f1=0x%03X f2=0x%03X wakes=%lu cfg_ok=%lu cfg_fail=%lu checks=%lu nf_ev=%lu nf_ignored=%lu fault=%s",
            st, power_state_name(power_state()), drv8323_nfault_low() ? "LOW" : "high",
            (unsigned)f1, (unsigned)f2, s_drv_wakes, s_drv_cfg_ok, s_drv_cfg_fail, s_drv_checks,
            drv8323_nfault_events(), drv8323_nfault_blanked_events(), fault_name(fault_first()));
}
#endif

#if BRINGUP_STAGE == 3
static void stage3_tick(uint32_t now_ms)
{
  if ((now_ms % STAGE3_LOG_PERIOD_MS) != 0U)
  {
    return;
  }
  if (drv8323_is_awake())
  {
    uint16_t f1 = 0U;
    uint16_t f2 = 0U;
    char text[STAGE3_FAULT_TEXT_LEN];
    bool ok = drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2);
    drv8323_fault_text(f1, f2, text, sizeof(text));
    debug_log("DRV awake pwr=%s vm_mv=%lu nfault=%s f1=0x%03X f2=0x%03X %s spi_err=%lu",
              power_state_name(power_state()), power_vm_mv(),
              drv8323_nfault_low() ? "LOW" : "high", (unsigned)f1, (unsigned)f2,
              ok ? text : "READ_FAIL", drv8323_spi_errors());
  }
  else
  {
    debug_log("DRV asleep pwr=%s vm_mv=%lu en=%u", power_state_name(power_state()),
              power_vm_mv(), power_motor_en() ? 1U : 0U);
  }
}
#endif

/* ---- PWM test mode (Stages 5 and 6) --------------------------------------- */

#if (BRINGUP_STAGE == 5) || (BRINGUP_STAGE == 6)
static float test_target(float t)
{
  if (!(t >= 0.0f))   /* also catches NaN from a mistyped debugger value */
  {
    return 0.0f;
  }
  return (t > PWM_DUTY_MAX) ? PWM_DUTY_MAX : t;
}

static float test_slew(float now, float target)
{
  float step = target - now;
  if (step > TEST_SLEW_PER_MS)
  {
    step = TEST_SLEW_PER_MS;
  }
  else if (step < -TEST_SLEW_PER_MS)
  {
    step = -TEST_SLEW_PER_MS;
  }
  return now + step;
}

static void test_arm(void)
{
  const char *reason;
#if BRINGUP_STAGE == 6
  uint32_t off = g_phase_off & 0x7UL;
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    (void)pwm_phase_output((board_phase_t)ph, (off & (1UL << ph)) == 0U);
  }
  if (off != 0U)
  {
    debug_log("PWM phases held off (INH = INL = low): A=%u B=%u C=%u",
              (unsigned)(off & 1UL), (unsigned)((off >> 1) & 1UL), (unsigned)((off >> 2) & 1UL));
  }
#endif
  pwm_zero_command();
  s_duty[PHASE_A] = PWM_DUTY_ZERO_CMD;
  s_duty[PHASE_B] = PWM_DUTY_ZERO_CMD;
  s_duty[PHASE_C] = PWM_DUTY_ZERO_CMD;
  if (motor_can_arm(&reason))
  {
    debug_log("PWM armed from zero command; slewing to A=%u B=%u C=%u permille",
              (unsigned)(test_target(g_duty_a) * 1000.0f), (unsigned)(test_target(g_duty_b) * 1000.0f),
              (unsigned)(test_target(g_duty_c) * 1000.0f));
  }
  else
  {
    debug_log("PWM arm refused: %s", reason);
  }
}

#if BRINGUP_STAGE == 6
static void sweep_apply_step(void)
{
  float d = s_sweep_duty[s_sweep_step];
  g_duty_a = d;
  g_duty_b = d;
  g_duty_c = d;
  s_sweep_ms = 0U;
  debug_log("SWEEP %lu/%u duty=%u permille: expect MOTA/B/C avg = %lu mV (duty x VM)",
            s_sweep_step + 1UL, (unsigned)SWEEP_STEPS, (unsigned)(d * 1000.0f),
            (uint32_t)(d * (float)power_vm_mv()));
}

static void sweep_tick(void)
{
  if (!s_sweeping || (++s_sweep_ms < SWEEP_STEP_MS))
  {
    return;
  }
  if (++s_sweep_step < SWEEP_STEPS)
  {
    sweep_apply_step();
    return;
  }
  s_sweeping = false;
  g_duty_a = PWM_DUTY_ZERO_CMD;
  g_duty_b = PWM_DUTY_ZERO_CMD;
  g_duty_c = PWM_DUTY_ZERO_CMD;
  debug_log("SWEEP done; back to 50 %% on all phases");
}

/* Disarm and sleep, set the new IDRIVE, then drv_tick() wakes the DRV and
 * configure() writes, verifies and locks it. Re-arm by command to test. */
static void test_apply_idrive(void)
{
  uint32_t code = g_drv_idrive;
  uint32_t src_ma;
  uint32_t snk_ma;
  if (code > 0xFFUL)
  {
    debug_log("IDRIVE 0x%lX invalid (0x00-0xFF: IDRIVEP << 4 | IDRIVEN)", code);
    return;
  }
  s_sweeping = false;
  drv8323_sleep();                       /* disarms first */
  (void)drv8323_set_idrive((uint8_t)code);
  drv8323_idrive_ma((uint8_t)code, &src_ma, &snk_ma);
  debug_log("IDRIVE 0x%02lX source=%lumA sink=%lumA; DRV re-waking, re-arm to test", code, src_ma, snk_ma);
}
#endif

static void test_command(uint32_t cmd)
{
  switch (cmd)
  {
    case APP_CMD_ARM:
      test_arm();
      break;
    case APP_CMD_DISARM:
      motor_disarm();
      debug_log("PWM disarmed by command: moe=%u pins=0x%02lX", pwm_is_armed() ? 1U : 0U, board_pwm_pins_read());
      break;
    case APP_CMD_SW_BREAK:
      pwm_software_break();
      debug_log("PWM software break: moe=%u pins=0x%02lX (expect moe=0 pins=0x00)",
                pwm_is_armed() ? 1U : 0U, board_pwm_pins_read());
      break;
#if BRINGUP_STAGE == 6
    case APP_CMD_SWEEP:
      if (!pwm_is_armed())
      {
        debug_log("SWEEP refused: arm first (g_app_cmd = 1)");
        break;
      }
      s_sweeping = true;
      s_sweep_step = 0U;
      sweep_apply_step();
      break;
    case APP_CMD_IDRIVE:
      test_apply_idrive();
      break;
#endif
    default:
      debug_log("CMD %lu unknown", cmd);
      break;
  }
}

static void test_tick(uint32_t now_ms)
{
  uint32_t cmd = g_app_cmd;
  if (cmd != APP_CMD_NONE)
  {
    g_app_cmd = APP_CMD_NONE;
    test_command(cmd);
  }

  if (pwm_is_armed())
  {
#if BRINGUP_STAGE == 6
    sweep_tick();
#endif
    s_duty[PHASE_A] = test_slew(s_duty[PHASE_A], test_target(g_duty_a));
    s_duty[PHASE_B] = test_slew(s_duty[PHASE_B], test_target(g_duty_b));
    s_duty[PHASE_C] = test_slew(s_duty[PHASE_C], test_target(g_duty_c));
    pwm_set_duty(s_duty[PHASE_A], s_duty[PHASE_B], s_duty[PHASE_C]);
  }
#if BRINGUP_STAGE == 6
  else
  {
    s_sweeping = false;   /* a break or fault ends the sweep */
  }
#endif

  if ((now_ms % TEST_LOG_PERIOD_MS) == 0U)
  {
    uint32_t a;
    uint32_t b;
    uint32_t c;
    const char *drv = drv8323_is_configured() ? "CFG" : (drv8323_is_awake() ? "AWAKE" : "ASLEEP");
    pwm_get_duty_permille(&a, &b, &c);
    debug_log("PWM armed=%u A=%lu B=%lu C=%lu vm=%lumV drv=%s nf=%s idrive=0x%02X brk=%lu fault=%s",
              pwm_is_armed() ? 1U : 0U, a, b, c, power_vm_mv(), drv,
              drv8323_nfault_low() ? "LOW" : "high", (unsigned)drv8323_idrive(),
              pwm_break_events(), fault_name(fault_first()));
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
#if BRINGUP_STAGE == 2
  /* Start from the current SW1 level, so booting with SW1 already on
   * doesn't count as a switch-on edge. */
  s_en_raw_prev = power_motor_en_raw();
#endif

#if BRINGUP_STAGE >= 3
  bool drv_cfg_ok = drv8323_init();
  debug_log("DRV spi3 mode1 16bit div256 %s, asleep (ENABLE low, PD2 released)",
            drv_cfg_ok ? "OK" : "FAIL");
#endif

#if BRINGUP_STAGE >= 5
  const char *what;
  bool pwm_ok = pwm_init(&what);
  debug_log("PWM tim1 center 20kHz dt=100ns brk=PB10 low trgo2=OC4REF ccr4=4200 %s (%s); dbg freeze on, counter running, moe=%u",
            pwm_ok ? "OK" : "FAIL", what, pwm_is_armed() ? 1U : 0U);
  debug_log("PWM sysbrk lockup+sram_parity+flash_ecc cfgr2=0x%03lX", SYSCFG->CFGR2);
#endif
#if BRINGUP_STAGE == 5
  debug_log("STAGE5 DRV held asleep. Commands: set g_app_cmd = 1 arm, 2 disarm, 3 software break");
#elif BRINGUP_STAGE == 6
  debug_log("STAGE6 g_app_cmd: 1 arm, 2 disarm, 3 sw break, 4 sweep, 5 apply g_drv_idrive");
  debug_log("STAGE6 targets g_duty_a/b/c (0..0.93), now A=%u B=%u C=%u permille",
            (unsigned)(g_duty_a * 1000.0f), (unsigned)(g_duty_b * 1000.0f), (unsigned)(g_duty_c * 1000.0f));
  debug_log("STAGE6 g_phase_off: 1 = A, 2 = B, 4 = C held off (both FETs) from the next arm");
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
#if BRINGUP_STAGE >= 3
    drv_tick();
#endif
#if BRINGUP_STAGE >= 4
    drv_monitor(s_last_tick_ms);
#endif

#if BRINGUP_STAGE == 0
    stage0_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 1
    stage1_tick(s_last_tick_ms, &enc);
#elif BRINGUP_STAGE == 2
    stage2_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 3
    stage3_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 4
    stage4_tick(s_last_tick_ms);
#elif (BRINGUP_STAGE == 5) || (BRINGUP_STAGE == 6)
    test_tick(s_last_tick_ms);
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
