/**
 * @file    app_config.h
 * @brief   Bring-up stage selection and build-time test switches.
 *
 * Plan section "Stage selection": each stage's test mode is compiled in only
 * when selected; modules from earlier stages stay active underneath it.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Current bring-up stage (plan: "Stages 0-4" onward). */
#define BRINGUP_STAGE 9

/* Stage 0 one-off: execute an undefined instruction 5 s after boot to check
 * that the HardFault dump appears over SWO. Set to 1 only for that test
 * (needs BRINGUP_STAGE 0). */
#define STAGE0_FAULT_TEST 0

/* Code on the 20 kHz control-interrupt path runs from SRAM: the linker
 * script's .RamFunc input section goes into .data, which the startup code
 * copies from flash (STM32G491XX_FLASH.ld). From flash, with 4 wait states,
 * prefetch off and a small cache, the interrupt took 7.7 us armed (Stage 8,
 * 2026-10-08). */
#define APP_RAMFUNC   __attribute__((section(".RamFunc")))

#endif /* APP_CONFIG_H */
