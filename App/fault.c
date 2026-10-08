/**
 * @file    fault.c
 * @brief   Latched fault codes.
 */
#include "fault.h"
#include "main.h"

volatile uint32_t g_fault_clear_request;

static fault_code_t s_first = FAULT_NONE;
static uint32_t s_ctx1;
static uint32_t s_ctx2;
static uint32_t s_tick_ms;
static uint32_t s_count;

bool fault_raise(fault_code_t code, uint32_t ctx1, uint32_t ctx2)
{
  s_count++;
  if (s_first != FAULT_NONE)
  {
    return false;
  }
  s_first = code;
  s_ctx1 = ctx1;
  s_ctx2 = ctx2;
  s_tick_ms = HAL_GetTick();
  return true;
}

bool fault_active(void)
{
  return s_first != FAULT_NONE;
}

fault_code_t fault_first(void)
{
  return s_first;
}

void fault_first_context(uint32_t *ctx1, uint32_t *ctx2, uint32_t *tick_ms)
{
  *ctx1 = s_ctx1;
  *ctx2 = s_ctx2;
  *tick_ms = s_tick_ms;
}

uint32_t fault_count(void)
{
  return s_count;
}

void fault_clear(void)
{
  s_first = FAULT_NONE;
  s_ctx1 = 0U;
  s_ctx2 = 0U;
  s_tick_ms = 0U;
  s_count = 0U;
}

const char *fault_name(fault_code_t code)
{
  switch (code)
  {
    case FAULT_NONE:                return "NONE";
    case FAULT_DRV_NOT_PRESENT:     return "DRV_NOT_PRESENT";
    case FAULT_DRV_CONFIG_WRITE:    return "DRV_CONFIG_WRITE";
    case FAULT_DRV_LOCK_TEST:       return "DRV_LOCK_TEST";
    case FAULT_DRV_CONFIG_MISMATCH: return "DRV_CONFIG_MISMATCH";
    case FAULT_DRV_NFAULT:          return "DRV_NFAULT";
    case FAULT_DRV_FAULT_BITS:      return "DRV_FAULT_BITS";
    case FAULT_DRV_SPI:             return "DRV_SPI";
    case FAULT_DRV_WAKE:            return "DRV_WAKE";
    case FAULT_PWM_BREAK:           return "PWM_BREAK";
    case FAULT_CTRL_HEARTBEAT:      return "CTRL_HEARTBEAT";
    case FAULT_CUR_OFFSET:          return "CUR_OFFSET";
    case FAULT_OVERCURRENT:         return "OVERCURRENT";
    case FAULT_ENCODER:             return "ENCODER";
    default:                        return "?";
  }
}
