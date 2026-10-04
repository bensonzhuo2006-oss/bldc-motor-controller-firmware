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
#define BRINGUP_STAGE 3

/* Stage 0 one-off: execute an undefined instruction 5 s after boot to check
 * that the HardFault dump appears over SWO. Set to 1 only for that test
 * (needs BRINGUP_STAGE 0). */
#define STAGE0_FAULT_TEST 0

#endif /* APP_CONFIG_H */
