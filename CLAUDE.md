# CLAUDE.md — BLDC controller firmware

Firmware for a 3-phase BLDC controller: STM32G491RE, DRV8323S gate driver (SPI),
MT6701 magnetic encoder (SSI on SPI1), 4015 gimbal motor (11 pole pairs, 12 V).
Goal: sensored FOC speed control, brought up stage by stage.

## Your role: write code only

- **Do not run any commands.** No builds, no flashing, no scripts, no git, no
  package installs, only commands that will allow you to read documentation.
- The developer builds, flashes and tests on real hardware with a scope and
  meter. You cannot see the hardware; never assume a stage worked.
- If you need information (a build error, a register dump, a scope result,
  a measured value), ask for it and stop.

## Read first

All reference material lives in the `docs/` folder: the test plan, pictures of
the schematic, and every datasheet and reference manual. Read what a stage
needs from there before writing code; check facts against these files, not
memory.

1. `docs/BLDC Controller Firmware Bring-Up Test Plan.md` — the test plan
   (referred to as "the plan" in this file). It is the specification: pin map,
   hardware facts, safety layers, configuration checks, firmware conventions,
   and the 16 bring-up stages. Follow it exactly; if something in it looks
   wrong or ambiguous, say so instead of guessing.
2. The **Firmware conventions** section of the plan — units, angle and current
   sign conventions, transforms, SVPWM, interrupt layout. Every module must
   follow it.
3. Schematic pictures in `docs/` — the board schematic as images. Use them to
   check pin assignments, pull-ups and dividers, test points and connector
   pinouts. The plan's pin map was read from the schematic; if the two
   disagree, stop and say so rather than picking one.
4. Datasheets and reference manuals in `docs/` — DRV8323, MT6701, STM32G491
   datasheet, RM0440 reference manual, the 4015 motor spec sheet, and any
   other documents in the folder. Use them to check register bits, timing and
   peripheral behaviour. If a document you need isn't in `docs/`, ask for it
   rather than relying on memory.

## Repository layout

- `*.ioc` and the CubeMX-generated code (`Core/`, `Drivers/`, CMake files) are
  owned by CubeMX. **Never edit the `.ioc`.** In generated files, write only
  inside `/* USER CODE BEGIN */ ... /* USER CODE END */` blocks, and keep those
  edits minimal (calls into our modules).
- If a peripheral setting must change (pin, clock, timer, ADC, DMA, NVIC),
  do not hand-patch generated init code. Tell the developer the exact CubeMX
  change to make, and wait for the regenerated code.
- Our code lives in `App/` (or the folder the developer names), one module per
  file pair as listed in the plan's "Firmware modules" table: `board.h`,
  `debug.c/h`, `mt6701.c/h`, `power.c/h`, `drv8323.c/h`, `pwm.c/h`,
  `cursense.c/h`, `foc_math.c/h`, `pi.c/h`, `motor_params.h`, `motor_sm.c/h`,
  `app.c`, `app_config.h`.

## Workflow: one stage at a time

1. Work only on the stage the developer asks for (`BRINGUP_STAGE` in
   `app_config.h`). Do not start the next stage.
2. Implement exactly the "Firmware added" items for that stage, plus anything
   the safety section says must exist by that stage.
3. When done, report:
   - the files changed and why;
   - any CubeMX changes the developer must make first;
   - what the developer should see: expected SWO output, scope signals,
     LED behaviour, and the stage's pass criteria from the plan;
   - which values are placeholders (`TODO_MEASURED`) to fill from the bench.
4. Wait for the developer's result. Fix problems from the evidence they give
   (logs, scope captures, register values) — do not guess at hardware faults.
5. When the developer gives measured values (R, L, current offsets, encoder
   offset and direction, IDRIVE), put them in `motor_params.h` / the DRV config
   and note the stage and date in a comment.

## Keep the plan current

The plan is the living record of the bring-up. Update it as things happen; don't
leave findings only in the conversation.

- Add a dated entry to the plan's **Bring-up log** section (at the end) for every
  finding, decision, schematic or datasheet discrepancy, measured value, stage
  pass or fail, and fix. Name the stage it affects and cite the source (datasheet
  section, schematic sheet, scope capture, the developer's report).
- When an entry changes a fact, also correct the section that holds that fact
  (pin map, hardware facts, register table, stage procedure, checklists), so the
  plan never contradicts itself.
- Tick checklist and prerequisite boxes when the developer confirms them.
- When a stage passes, log it with the date and the key results.

## Hard rules

- **Safety code is never removed, bypassed or weakened** to make a test pass.
  If a protection blocks progress, explain why and ask.
- Only `motor_can_arm()` may set TIM1 MOE. Only `motor_disarm()` and the
  hardware break clear it. Every arm starts from a zero command and ramps.
- TIM1 must have the debug-freeze bit set, OSSI = OSSR = 1, idle states low,
  and automatic output enable off, before any stage that switches the bridge.
- Disarm order: PWM outputs off first, then DRV ENABLE low (within 40 µs).
- Faults latch until explicitly cleared. Never auto-restart the motor.
- Current trips and limits follow the plan's values for the current stage.

## Code rules

- C (C11), no RTOS, no dynamic allocation.
- HAL for initialisation only. Inside the 20 kHz control interrupt use LL or
  direct register access: no HAL calls, no SWO/printf, no blocking, no SPI3
  traffic, no division by values that can be zero.
- `float` in SI units, constants with the `f` suffix; use the CORDIC for
  sin/cos in the control interrupt.
- Data shared between interrupt and main loop is `volatile`; multi-word reads
  in the main loop are protected (brief interrupt disable or double buffer).
- Logging (SWO) only from the main loop. Keep messages short and parseable
  (e.g. `DRV reg 0x05 = 0x110 OK`).
- Every magic number gets a named constant with a comment citing its source
  (datasheet table, plan section, or bench measurement).
- Keep each test mode behind `#if BRINGUP_STAGE >= N` (or `== N` for one-off
  tests) so earlier diagnostics stay available.
