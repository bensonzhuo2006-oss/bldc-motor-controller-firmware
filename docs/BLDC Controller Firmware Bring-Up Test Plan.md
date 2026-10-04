# BLDC Controller Firmware Bring-Up Test Plan

Oct 3, 2026 · @Benson

## Overview

The board is brought up in 16 stages, from a bare MCU on USB power to a speed-controlled FOC motor at 12 V. Each stage tests one component, adds only the firmware that component needs, and has a pass criterion before the next stage starts.

- **Board:** STM32G491RE MCU, DRV8323S gate driver (SPI, three current-sense amplifiers), six IAUC80N04S6N036 MOSFETs, 5 mΩ low-side shunts, MT6701 magnetic encoder on SPI1.
- **Motor:** 4015 gimbal motor, 11 pole pairs, 12 V, 0.65 A rated, 4 A peak.
- **Goal:** speed control with sensored FOC and a hardcoded speed profile.
- **Toolchain:** CubeMX-generated CMake project in VS Code, C, no RTOS. HAL for setup; LL and direct register access inside the control interrupt. ST-Link debugger, SWO trace output.
- **Build switch:** one codebase with a compile-time `BRINGUP_STAGE` setting. It selects which stage's test mode runs, so earlier tests stay available as diagnostics.

**Bench supply limit by stage.** The supply limit is the main protection while the code is unproven. It rises only when a stage passes.

| Stages | Power source | Supply current limit | Motor |
| --- | --- | --- | --- |
| 0–1 | USB only | — | Unconnected |
| 2–5 | 12 V bench supply | 150 mA | Unconnected |
| 6–7 | 12 V bench supply | 0.5 A | Unconnected |
| 8–10 | 12 V bench supply | 1 A | Connected |
| 11–12 | 12 V bench supply | 1.5 A | Connected |
| 13 | 12 V bench supply | 2 A | Connected |
| 14–15 | 12 V bench supply | up to 4 A | Connected |

## Pin mappings

All STM32G491RE pins the bring-up firmware uses, plus the pins it leaves alone, read from the schematic. PWM phase A is on TIM1 channel 3 and phase C on channel 1; a mapping table in `board.h` handles this.

### Used by the bring-up firmware

| Signal | Pin | Peripheral / mode | First stage | Notes |
| --- | --- | --- | --- | --- |
| Encoder CSN | PA4 | GPIO output (software chip select) | 1 | R39 10k pull-up to 3.3 V, idles high |
| Encoder CLK | PA5 | SPI1\_SCK | 1 | 49.9 Ω series (R41); SPI mode 1 (see MT6701 notes) |
| Encoder DO | PA6 | SPI1\_MISO | 1 | Data out from the MT6701; 49.9 Ω series (R40, at J14 pin 3) |
| (unused) | PA7 | SPI1\_MOSI | — | Assigned by the full-duplex SPI and sends dummy data; 49.9 Ω series (R42) to J14 pin 2, which is not connected on the encoder board |
| MOTOR\_EN | PC1 | GPIO input | 2 | 10k/20k divider from the 5 V relay-coil signal; high when SW1 is on |
| VM\_SENSE | PB11 | ADC12\_IN14 | 2 | 180k/10k divider (VM ÷ 19); used to confirm VM is above 10 V before waking the DRV |
| DRV nSCS | PD2 | GPIO output (software chip select) | 3 | Driven high only once VM is present. R38 10k pull-up to 3.3 V holds nSCS high whenever 3.3 V is up, even while PD2 is undriven |
| DRV SCLK | PC10 | SPI3\_SCK | 3 | SPI mode 1, 16-bit |
| DRV SDO | PC11 | SPI3\_MISO | 3 | Open-drain, R23 10k pull-up; limits SPI3 to about 664 kHz |
| DRV SDI | PC12 | SPI3\_MOSI | 3 |  |
| DRV ENABLE | PC8 | GPIO output | 3 | Low at boot; 1 ms wake time after going high |
| DRV CAL | PC9 | GPIO output | 3 | Held low; offset calibration done over SPI |
| DRV nFAULT | PB10 | TIM1\_BKIN (active low) + EXTI | 4 | R22 10k pull-up; LED6 lights during a fault |
| INHC | PA8 | TIM1\_CH1 | 5 | Phase C high side; do not use MCO on this pin |
| INHB | PA9 | TIM1\_CH2 | 5 | Phase B high side |
| INHA | PA10 | TIM1\_CH3 | 5 | Phase A high side |
| INLC | PB13 | TIM1\_CH1N | 5 | Phase C low side |
| INLB | PB14 | TIM1\_CH2N | 5 | Phase B low side |
| INLA | PB15 | TIM1\_CH3N | 5 | Phase A low side |
| SOA | PB2 | ADC2\_IN12 | 7 | Phase A current-sense output, 56 Ω / 2.2 nF filter |
| SOB | PB1 | ADC1\_IN12 | 7 | Phase B current-sense output, 56 Ω / 2.2 nF filter |
| SOC | PB0 | ADC3\_IN12 | 7 | Phase C current-sense output, 56 Ω / 2.2 nF filter |
| Scope DAC output | PA2 (J15) | DAC3\_CH1 → OPAMP1 follower → OPAMP1\_VOUT | 0 | One analog channel for the scope |
| Timing pin | PA3 (J15) | GPIO output | 0 | Toggled in the control interrupt |
| Status LED | PC3 | GPIO output (drives Q7) | 0 | LED4 |
| SWO | PB3 | Trace output | 0 | Logging and telemetry |
| SWDIO | PA13 | Debug | 0 | ST-Link on J2 |
| SWCLK | PA14 | Debug | 0 | ST-Link on J2 |
| HSE in / out | PF0 / PF1 | 24 MHz crystal | 0 | PLL to 170 MHz |
| NRST | PG10 | Reset | — | SW3 button |
| BOOT0 | PB8 | Boot select | — | SW2 button, R20 10k pull-down |

### Not used during bring-up

| Signal | Pin | Possible peripheral | Notes |
| --- | --- | --- | --- |
| THROTTLE1 | PA0 | ADC | On-board 10k pot through R13 100 Ω |
| THROTTLE2 | PA1 | ADC | On-board 10k pot through R15 100 Ω |
| HALL\_SENSE | PB12 | ADC1\_IN11 | ACS71240 total-current sensor |
| TEMP\_SENSE | PC0 | ADC | 100k thermistor with 4.87k pull-up |
| IMU SDA / SCL | PB9 / PA15 | I2C1 | LSM6DSOX and header J11 |
| IMU INT1 / INT2 | PB4 / PB7 | GPIO / EXTI |  |
| CAN RX / TX | PB5 / PB6 | FDCAN2 | SN65HVD232 transceiver, header J10 |
| USB D− / D+ | PA11 / PA12 | USB FS |  |
| Not connected | PC2, PC4, PC5, PC6, PC7, PC13, PC14 | — | No-connect on the schematic |

PA2 and PA3 are the UART pins on J15. They are borrowed for debugging because the UART isn't used.

## Key hardware facts and parameters

These numbers come from the DRV8323, MT6701 and STM32G491 datasheets and the motor spec sheet. The firmware is built around them.

### DRV8323 gate driver

- **SPI frame:** 16 bits, MSB first. Bit 15 = read (1) / write (0), bits 14–11 = address, bits 10–0 = data. Mode 1 (CPOL=0, CPHA=1).
- **SPI speed:** the driver allows 10 MHz, but SDO is open-drain with only a 10k pull-up, so its rising edges are slow. SPI3 runs at 170 MHz ÷ 256 ≈ 664 kHz (about 24 µs per word). Try 1.33 MHz only after checking edges on the scope.
- **Chip select:** high for at least 400 ns between words; SCLK must be low at both chip-select edges.
- **Wake:** ENABLE high, then wait 1 ms before SPI or PWM. nFAULT is held low during wake.
- **Sleep:** registers reset to defaults on every sleep or undervoltage, so the configuration is rewritten after every wake. INHx/INLx must be low within 40 µs of ENABLE going low.
- **Presence check:** an unpowered DRV reads back 0x7FF (the SDO pull-up), the same as register 0x04's default. Read register 0x03 and expect 0x3FF instead.

| Register | Address | Reset value | Bring-up value | Contents |
| --- | --- | --- | --- | --- |
| Fault Status 1 | 0x00 | — | — | Read only |
| VGS Status 2 | 0x01 | — | — | Read only |
| Driver Control | 0x02 | 0x000 | 0x000 | 6x PWM mode, all fault reporting on; the CLR\_FLT bit clears latched faults |
| Gate Drive HS | 0x03 | 0x3FF | 0x322 unlocked, 0x622 locked | IDRIVE 60 mA source (0010b) / 120 mA sink (0010b), tuned on the scope; LOCK 011b, then 110b after setup |
| Gate Drive LS | 0x04 | 0x7FF | 0x722 | CBC 1, TDRIVE 4 µs (11b), IDRIVE 60 / 120 mA |
| OCP Control | 0x05 | 0x159 | 0x110 | Dead time 100 ns, latched overcurrent, 4 µs deglitch, VDS trip 0.06 V (about 16–21 A: RDS(on) 2.8 mΩ typical, 3.6 mΩ maximum; short-circuit protection only) |
| CSA Control | 0x06 | 0x283 | 0x2C3 | Bidirectional reference (VREF ÷ 2), gain 40 V/V, calibration bits off in normal running, sense overcurrent 1 V |

### Current sensing

- **Shunt:** 5 mΩ low side per phase. VREF = VDDA = 3.3 V, so zero current sits at about 1.65 V.
- **Conversion:** the datasheet's equation, I = (1.65 V − VSO) / (40 × 0.005 Ω), gives current flowing down through the shunt to ground, which is current out of the motor. The firmware defines positive as into the motor, so it uses I = (VSO − Voffset) / (40 × 0.005 Ω), with Voffset the measured zero-current level (about 1.65 V). See Firmware conventions.
- **Range at 40 V/V:** ±8.25 A full scale, about 4 mA per ADC count (3.3 V ÷ 4096 ÷ 0.2 V/A). The motor's 4 A peak uses about half the range. Current convention: the shunts measure line current, and the Clarke transform is amplitude-invariant, so Iq equals peak line current. The motor's 4 A maximum is treated as peak line current, and every current limit and trip in this plan uses that convention. This is an assumption: the manufacturer doesn't say whether 4 A is peak or RMS, line or winding. Until that's confirmed, keep peak commands at or below 4 A and don't rely on margin above it.
- **Sampling:** all three ADCs start together from TIM1 TRGO2, generated from OC4REF, near the counter peak when the low-side switches are on. CCR4 sits slightly below ARR rather than exactly at it, because compare and OC4REF behave differently at the counter turnaround. The exact position is set by checking the PA3 timing pin against the real low-side gate waveform. Duty is capped at about 93% to leave enough low-side on-time: at gain 40 V/V the amplifier needs up to about 1.2 µs to settle (datasheet, 0.5 V step), and a 93% cap leaves about 1.5 µs between the low-side switch turning on and the mid-point sample.

### MT6701 encoder (SSI on SPI1)

- **Frame:** 24 bits. Bits 0–13 = angle (16,384 counts per turn), bits 14–17 = status (field too strong, field too weak, push button, loss of track), bits 18–23 = CRC-6 (polynomial x⁶ + x + 1).
- **Clock:** the datasheet shows CLK idling high, with data changing on the rising edge and sampled on the falling edge. Read with SPI mode 1 (CPOL=0, CPHA=1): the first edge is then a rising edge that puts out the first bit, and each bit is sampled on the next falling edge. In mode 2 (idle high) the first falling edge samples before any data is out, so a 24-clock read loses the last bit; people reading the MT6701 over SSI report the same and use mode 1. The CRC check decides: if mode 1 fails, try mode 2 with one extra clock.
- **Speed:** minimum clock period 64 ns. SPI1 starts at 170 MHz ÷ 32 ≈ 5.3 MHz (about 4.5 µs per read), read as two 12-bit frames in full-duplex master mode with dummy transmit data. Receive-only master mode is avoided: per RM0440 its clock runs continuously until the SPI is disabled, so it can't reliably give exactly 24 clocks.
- **Resolution:** 16,384 ÷ 11 ≈ 1,490 counts per electrical revolution, about 0.24° electrical.

### TIM1 and control timing

- **Clock:** 24 MHz crystal through the PLL (M = 6, N = 85, R = 2) gives 170 MHz.
- **PWM:** center-aligned, ARR = 4250 gives 20 kHz. That leaves 8,500 CPU cycles per control interrupt; target under 50%.
- **Dead time:** about 100 ns from the MCU and 100 ns from the DRV. They don't simply add: the DRV inserts its dead time after its gate-voltage handshake, so the two overlap, and the datasheet doesn't specify the combined result. The pass criterion is measured non-overlap of GHx and GLx on the scope, not a nominal number.
- **Break:** TIM1\_BKIN on PB10 (nFAULT), enabled only after the DRV has woken and the break flag is cleared.
- **Speed loop:** every 10th control interrupt, so 2 kHz.

### Motor (4015 gimbal)

| Parameter | Datasheet value | Value used for FOC |
| --- | --- | --- |
| Pole pairs | 11 (22 poles, 24 slots) | 11 |
| Winding | Delta | Converted to an equivalent wye model |
| Resistance | 4.8 Ω per winding | About 1.6 Ω per phase (wye), 3.2 Ω line-to-line |
| Inductance | 2.6 mH per winding | About 0.87 mH per phase (wye) |
| Kv | 61 rpm/V | About 730 rpm unloaded at 12 V. Kv refers to line voltage, so with Vq as the phase-voltage amplitude, no-load speed ≈ Kv × √3 × Vq. Theoretical maximum Vq = 12 V ÷ √3 ≈ 6.9 V; with the 93% duty cap the usable maximum is about 6.0 V, so top no-load speed is about 630 rpm |
| Back-EMF constant | 0.1562 V/(rad/s) | Matches Kv, a consistency check on the datasheet |
| Rated / max current | 0.65 A / 4 A | Iq clamp 4 A peak, 0.65 A continuous through a heating (I²t) model |

**Current-loop starting gains** at about 1 kHz bandwidth, from the wye values:

```latex
K_p = L \cdot \omega_{bw} \approx 0.87\,\text{mH} \times 2\pi \cdot 1000 \approx 5.4\ \text{V/A}
```

```latex
K_i = R \cdot \omega_{bw} \approx 1.6\,\Omega \times 2\pi \cdot 1000 \approx 10{,}000\ \text{V/(A·s)}
```

With 12 V DC across two leads, stall current is about 12 V ÷ 3.2 Ω ≈ 3.75 A. Full space-vector modulation without the duty cap would give 6.93 V ÷ 1.6 Ω ≈ 4.3 A at standstill; with the 93% cap the maximum is about 6.0 V, or 3.7 A. That is close enough to the 4 A maximum that the current clamp and software trip, not the motor's resistance, must limit current. The encoder-to-electrical offset depends on how the magnet is mounted, so Stage 10 measures it.

## Safety: protection layers

The board has three layers of protection: the bench supply and power-path hardware, the DRV8323's own protections wired into the timer's hardware break, and firmware checks in the control interrupt. Each layer must be proven before the one above it is trusted, and the faster a fault can do damage, the lower the layer that must catch it.

### Hardware protection

| Protection | What it catches | Response | Verified in |
| --- | --- | --- | --- |
| Bench supply current limit | Everything, including firmware bugs | Immediate, limits input current | Every stage (see the limit table in Overview) |
| Slide switch SW1 and relay K1 | Physical kill of motor power (VM). The MCU can't override it. | Immediate | Stage 2 |
| Fuse F1 and TVS diode ZD1 (SMAJ60CA) | Input overcurrent and voltage spikes | Fuse: ms to s; TVS: ns | Not tested. Check the F1 rating suits a 4 A motor. |
| DRV8323 6x PWM truth table | Shoot-through: INH and INL both high turns both gates off | Hardware, about 150 ns propagation | Stage 5 |
| DRV8323 dead time and gate handshake | Shoot-through during switching | Hardware, every edge | Stage 6 (scope) |
| DRV8323 VDS overcurrent | Short circuit in a FET, phase or motor lead. Trip at 0.06 V ≈ 16–21 A, latched. | 4 µs deglitch, then gates off | Stage 6 (fault path); trip level not tested |
| DRV8323 undervoltage, charge-pump, gate-drive and thermal faults | VM too low, gate voltage too low, gate not switching, overheating | Hardware, gates off, nFAULT low | Stage 4 (decode), Stage 6 (break path) |
| DRV8323 input pull-downs (100 kΩ) on ENABLE, INHx, INLx | Floating MCU pins during reset or boot | Inputs read low, gates stay off | Stage 0 (pins measure low) |
| nFAULT → TIM1\_BKIN (PB10) | Any DRV fault, without waiting for software | Hardware break, outputs off within a few clock cycles | Stage 5 (software break), Stage 6 (nFAULT shorted to GND) |

### Firmware protection

| Protection | What it catches | Response | Added in |
| --- | --- | --- | --- |
| Safe pin states at boot | Gates turning on before setup finishes | PWM pins low, ENABLE low, DRV SPI pins undriven | Stage 0 |
| Arming rules | Driving into an unpowered or faulted DRV | All arming goes through one function, motor\_can\_arm(), the only code allowed to set MOE. It requires VM\_OK, nFAULT high, DRV configuration verified, no latched fault, ADC offsets valid, control heartbeat alive, encoder healthy and a zero command. Each check applies from the stage that adds it: ADC offsets and heartbeat from Stage 7, encoder from Stage 9. Output then ramps up from zero. | Stage 6 |
| Power monitor (MOTOR\_EN + VM\_SENSE) | VM missing, ramping or collapsing | VM\_OK lost → disarm, ENABLE low | Stage 2 |
| TIM1 break configuration | Outputs left floating after a break | OSSI = 1, OSSR = 1, idle states low, automatic output re-enable off, so a break stays latched until re-armed | Stage 5 |
| Debugger freeze (DBG TIM1 stop) | A breakpoint leaving DC on the motor | TIM1 halts with the CPU, outputs disabled and driven low | Stage 5 |
| Disarm order | Gates floating if ENABLE drops while PWM runs | Outputs off first, then ENABLE low, within the DRV's 40 µs window | Stage 5 |
| MCU dead time (about 100 ns) | Backup to the DRV's own dead time | Every edge | Stage 5 |
| Duty and voltage clamp | Over-modulation and losing the current-sample window | Duty ≤ about 93%; voltage vector limited to about 6.0 V, the largest vector that fits under the cap | Stage 7 |
| Software overcurrent trip | Any phase current above the limit | Disarm within one 50 µs control cycle. 1 A until readings match a meter, then 1.5 A (Stage 8); 3 A (Stage 11); 4.5 A (Stage 14) | Stage 8 |
| DRV configuration readback | Registers lost to a reset or glitch | Mismatch → fault, disarm | Stage 4 |
| Encoder faults | CRC errors, magnet too weak or strong, loss of track, impossible angle jumps | Disarm, motor coasts | Stage 9 |
| Control-loop heartbeat | The control interrupt stops running (lost ADC trigger, stuck code) | Main loop sees no new count within a few ms → disarm | Stage 7 |
| I²t thermal limit | Motor overheating at more than 0.65 A for too long | Iq limit pulled back from 4 A toward 0.65 A | Stage 14 |
| Stall detection | Large Iq with no rotation | Disarm, fault logged | Stage 15 |
| VM overvoltage trip | Braking energy pumping VM up (the bench supply can't absorb it) | VM above about 16 V → coast | Stage 14 |
| Independent watchdog | Firmware hang or lock-up | MCU reset; boot leaves the gates off | Stage 15 |
| Fault latch and logging | Faults clearing themselves and restarting the motor | Every fault stays latched until explicitly cleared; the cause and reset reason are logged over SWO | Stage 4 onward |

### Rules for the bench

1. Don't connect the motor until Stage 5 has proven the debugger freeze and the break input, and Stage 6 has proven the nFAULT path.
2. Raise the supply current limit only after the stage that needs it passes, and after the software overcurrent trip for that stage is shown to work.
3. Never disable a protection to get past a problem. If you must lower a threshold for a test, put it back before moving on.
4. Faults never clear themselves. Re-arming is always an explicit action, and it always starts from a zero command that ramps up.
5. When something goes wrong, the first move is SW1 off. It removes motor power whatever the firmware is doing. The bulk capacitors on VM stay charged for minutes afterwards (τ ≈ 190 s through the VM\_SENSE divider, measured in Stage 2), so treat VM as live until it measures near 0 V. Firmware disarms on MOTOR\_EN going low, not on VM falling.
6. Remember the bench supply limits input current, not phase current. At low speed the phase current can be several times higher than the supply reading, so trust the software trip, not the supply display.

## Configuration checks: DRV8323 and MT6701

Each configuration is confirmed two ways: a register readback proves the value was written, and a physical measurement proves the setting does what it should. A correct value in the wrong field still reads back fine, so both are needed.

### DRV8323: register readback

Expected values are the "Bring-up value" column of the register table under Key hardware facts.

If IDRIVE is retuned in Stage 6, update the expected values for 0x03 and 0x04 in the shadow copy.

**Readback checks in firmware:**

1. Write then verify: after writing each register, read it back. A mismatch is a fault.
2. Lock test: after locking, try writing a different value to 0x05. It must still read 0x110, which proves the lock took.
3. Periodic check: read every register every 100 ms and compare with the shadow copy, to catch a reset from a VM dip or an ENABLE glitch.

### DRV8323: physical checks

| Setting | How to confirm | Stage |
| --- | --- | --- |
| 6x PWM mode | Each INH/INL combination gives the expected switch-node state; INH and INL both high gives both gates off | 5–6 |
| IDRIVE | Gate rise and fall times at J4–J9. Change IDRIVE one step and the edge speed should visibly change, proving writes reach the hardware | 6 |
| Dead time | Gap between GHx falling and GLx rising on two scope channels: a clear non-overlap with margin (the DRV's and MCU's dead times overlap rather than simply adding) | 6 |
| Bidirectional reference | SOA, SOB and SOC sit at about 1.65 V with no current (unidirectional mode would put them near 3.0 V, VREF − 0.3 V) | 4 |
| Offset calibration | SOx stays at about 1.65 V during calibration, and the ADC offsets land near 2048 counts afterwards | 4, 7 |
| Gain 40 V/V | A 0.5 V vector into 1.6 Ω gives about 0.31 A, so SOA should rise about 62 mV above its offset (0.31 A × 5 mΩ × 40; current into the motor raises SOx). | 8 |
| Fault reporting | Shorting nFAULT to GND trips the break; fault registers read clean after wake | 6 |
| VDS trip level | Register readback only. A real test needs about 16–21 A through a FET | — |

### MT6701: checks from the SSI data

In SSI mode the encoder's configuration registers can't be read (that needs I2C), so it is confirmed from the data it sends.

1. **Scope the frame.** CSN goes low, 24 clock pulses follow (CLK idles low in SPI mode 1), and DO changes on rising edges. If DO toggles as the shaft turns with no clock running, the chip is in ABZ mode: fix the MODE pin.
2. **CRC passes on every frame.** This is the definitive check. A wrong SPI mode, a one-bit offset or a bad connection makes nearly every frame fail. Zero failures over a few minutes means the link and bit alignment are right.
3. **Status bits are clean.** Field status reads normal and loss of track stays 0 through a full turn, confirming magnet strength and gap.
4. **The angle makes sense.** One mechanical turn gives one wrap, 0 → 16383. Mark the shaft and turn it 90°: the count changes by about 4,096. Held still, the count jitters by only a few counts.
5. **Direction.** Note which way the count rises. If it is opposite to the motor's forward direction, flip it in firmware; Stage 10 alignment detects it either way.

## Stages 0–4: MCU, encoder, power, gate driver

These stages prove everything up to the gate driver without switching any MOSFETs. Each stage lists the part under test, what is physically connected, the firmware added, the procedure and the pass criteria.

### Stage 0: MCU core

- **Under test:** STM32G491, its clock tree, the debug link, the status LED and the PA2 scope output.
- **Connected:** USB power and the ST-Link on J2. No 12 V, no motor, no encoder.
- **Firmware added:**
  - Clock: 24 MHz crystal through the PLL to 170 MHz.
  - Safe pins at boot: all six PWM pins low, ENABLE and CAL low, the SPI3 pins and PD2 left undriven so nothing feeds the unpowered DRV.
  - SWO logging on PB3, called from the main loop only.
  - CPU cycle counter for timing measurements.
  - Fault handler that prints the crash registers over SWO.
  - Status LED on PC3, timing pin on PA3, and the scope output: DAC3\_CH1 through OPAMP1 as a buffer, out on PA2.
- **Procedure:**
  1. Check SWO messages appear in VS Code.
  2. Time the LED blink on the scope. The clock can't be output on MCO because PA8 is INHC.
  3. Output a test triangle wave on PA2.
  4. Measure 3.3 V at TP1 and 5 V at TP3.
- **Pass:** the clock is exactly 170 MHz, SWO is stable, the triangle on PA2 spans 0–3.3 V, and all PWM pins measure low.

### Stage 1: Encoder (MT6701 on SPI1)

- **Under test:** encoder setup, link, frame decode and fault detection.
- **Connected:** USB power, ST-Link, encoder board on J14, magnet on the motor. Motor leads unconnected. No 12 V.
- **Encoder setup checklist (before any firmware test):**
  - [ ] The encoder board's MODE pin selects I2C/SSI output, not ABZ. In ABZ mode the SPI read returns garbage.
  - [ ] The magnet is diametrically magnetized (poles across the face, not top/bottom).
  - [ ] The magnet is centered on the rotor's rotation axis. On this hollow-shaft motor, check it isn't offset.
  - [ ] The gap between the magnet and the MT6701 chip is 0.5–2 mm, typically 1 mm (MT6701 datasheet air gap).
  - [ ] The encoder board is fixed to the stator side, so only the magnet turns with the rotor.
  - [ ] J14 wiring: pin 1 = 3.3 V, pin 2 = MOSI (PA7, dummy data), pin 3 = DO (PA6), pin 4 = CLK (PA5), pin 5 = CSN (PA4), pin 6 = GND.
  - [x] J14 pin 2 (MOSI) is not connected on the encoder board, so the dummy MOSI data goes nowhere (confirmed 2026-10-04).
- **Firmware added:** `mt6701` driver.
  - SPI1: mode 1, full-duplex master with dummy transmit, two 12-bit frames per read, software chip select on PA4.
  - Decode: 14-bit angle, 4 status bits, CRC-6 check.
  - Magnetic field status printed over SWO at startup and on change: normal, too strong or too weak. This is the firmware check that the magnet is set up right.
  - Fault counters: CRC errors, field too weak or too strong, loss of track, and angle jumps too large to be real.
  - Multi-turn position by unwrapping the angle, and speed from a PLL tracking loop.
  - The angle is sent to the PA2 scope output.
- **Procedure:**
  1. Work through the setup checklist above.
  2. Check CSN, CLK and DO timing on the scope.
  3. Turn the shaft slowly by hand through several full turns, watching the field status.
  4. Turn it quickly.
- **If the field status reads too weak:** the gap is too large or the magnet is off-center. Too strong: the gap is too small. If it changes around the turn, the magnet is off-axis.
- **Pass:** the angle covers 0–16383 smoothly with no jumps and PA2 shows a clean sawtooth. The count rises in one direction and falls in the other. No CRC errors over about a minute, and the field status stays normal all the way round.

### Stage 2: 12 V input, power path, MOTOR\_EN

- **Under test:** the buck converter and USB/buck power switch under 12 V, the relay and slide switch SW1, the MOTOR\_EN input on PC1, and the VM\_SENSE reading on PB11.
- **Connected:** 12 V bench supply on J12, plus ST-Link and encoder. USB unplugged. Motor unconnected. ENABLE held low.
- **Supply limit:** 150 mA, but see the switch-on note below.
- **Firmware added:**
  - Reset-cause logging at boot: brownout, pin reset, watchdog or power-on, read from the reset flags and printed over SWO, then cleared. This makes an unexpected reset obvious.
  - VM\_SENSE read on PB11 (ADC12\_IN14). VM = ADC voltage × 19 (180k/10k divider).
  - Power state machine: NO\_VM → VM\_SETTLING → VM\_OK. VM\_OK requires **both** MOTOR\_EN high **and** VM above 10 V, held for about 20 ms. VM below 9 V, or MOTOR\_EN low, drops back to NO\_VM.
  - State and VM logged over SWO and shown on the LED.
- **Switch-on note:** turning SW1 on connects 12 V straight onto about 1,000 µF of empty bulk capacitors (C27–C29, 3 × 330 µF, plus C31). With a 150 mA limit, the 12 V input can sag while they charge. The buck converter drops out, the MCU resets, and the relay coil (fed from the buck) can release and close again. If the MCU resets or the relay chatters when SW1 turns on, raise the supply limit to about 1 A for switch-on, then lower it again.
- **Procedure:**
  1. Power up with SW1 off and note the reset cause (should be power-on).
  2. Toggle SW1 several times. Watch for unexpected resets in the log.
  3. Check the buck output at TP2 and VM at J3. Compare the VM reading in the log with a meter.
  4. Log the VM ramp after SW1 turns on, to see how long VM takes to pass 10 V at this current limit.
- **Pass:** the MCU runs from 12 V alone and doesn't reset when SW1 is switched. The VM reading is within about 3% of the meter. VM\_OK appears only after VM passes 10 V, and drops when SW1 turns off.

### Stage 3: DRV8323 wake and SPI3 link

- **Under test:** DRV8323 power-up and SPI communication.
- **Connected:** as in Stage 2, with SW1 on. Motor unconnected.
- **Firmware added:** `drv8323` low-level driver.
  - Wake sequence: on VM\_OK (MOTOR\_EN high and VM above 10 V), set PD2 high, then ENABLE high, then wait 1 ms.
  - SPI3: mode 1, 16-bit, about 664 kHz.
  - Register functions: read, write, write-then-verify.
  - Presence check: read register 0x03 and expect 0x3FF.
  - Fault decode: both fault registers printed as text. When VM\_OK is lost, the PWM is disarmed, ENABLE goes low and PD2 is released.
- **Procedure:**
  1. Watch LED5 (the DRV's 3.3 V rail) turn on and nFAULT release.
  2. Read all registers.
  3. Write and read back register 0x05.
  4. Test lock/unlock and fault clearing (CLR\_FLT).
  5. Toggle ENABLE and confirm the registers return to defaults.
  6. Check SPI timing on the scope: chip select high at least 400 ns between words, SCLK low at each chip-select edge.
- **Pass:** the defaults read 0x000, 0x3FF, 0x7FF, 0x159, 0x283 for registers 0x02–0x06. Writes read back correctly. No faults.

### Stage 4: DRV8323 configuration and fault reporting

- **Under test:** gate driver configuration, current-sense amplifier setup and the fault path.
- **Connected:** as in Stage 3.
- **Firmware added:**
  - A config struct (values in Key hardware facts) written after every wake, verified by reading back, then locked.
  - nFAULT interrupt that latches a fault flag; the main loop decodes the fault registers.
  - Configuration read back every 100 ms and compared with a shadow copy; a mismatch counts as a fault.
  - The DRV's built-in current-sense offset calibration, run over SPI.
- **Procedure:**
  1. Cycle SW1 about 20 times and confirm the configuration is restored each time.
  2. Measure SOA, SOB and SOC with a meter.
  3. Check the nFAULT interrupt fires during the DRV's wake (nFAULT is held low briefly while it powers up) and that the firmware ignores that pulse. A powered-down DRV can't report an undervoltage fault over SPI, so real fault decoding is exercised in Stage 6 (nFAULT break) and Stage 8 (overcurrent).
- **Pass:** the configuration matches after every wake. SOx outputs sit at about 1.65 V. The wake pulse on nFAULT is seen and ignored, and the fault registers read clean after wake.

## Stages 5–8: PWM, power stage, current sense, motor at rest

These stages switch the MOSFETs for the first time, first into a sleeping driver, then with no load, and finally into the stationary motor.

### Stage 5: TIM1 PWM into a sleeping DRV

- **Under test:** timer setup, phase mapping, the break input and the debugger freeze. The gates can't switch yet.
- **Connected:** as in Stage 4, but ENABLE held low so the DRV is asleep and holds every gate low. Scope on the INH and INL test points.
- **Firmware added:** `pwm` module.
  - TIM1 center-aligned at 20 kHz (ARR = 4250), complementary outputs, about 100 ns of MCU dead time.
  - Outputs forced low when idle; arm and disarm functions. Disarm turns the outputs off before ENABLE goes low.
  - Duty written straight to the compare registers through the phase table (A on channel 3, B on 2, C on 1).
  - TIM1\_BKIN on PB10, active low.
  - TIM1 TRGO2 from OC4REF as the ADC trigger, with CCR4 slightly below ARR (near the counter peak). Final position set in Stage 7 against the low-side gate waveform.
  - **Debugger freeze:** set the TIM1 stop-on-halt bit (DBGMCU APB2 freeze register). When the debugger halts the CPU at a breakpoint, TIM1 stops and its outputs are disabled as if MOE were cleared. Set OSSI = 1 with all idle states low, so disabled outputs are driven low rather than left floating (this also applies to break events). Without this, a breakpoint leaves the last duty applied, which puts DC on the motor, up to about 3.7 A at the maximum duty. It must be set before any motor is connected.
- **Procedure:**
  1. Set a different duty on each of A, B and C and identify each on the scope.
  2. Measure the dead time.
  3. Trip the break in software and check that every output goes low.
  4. With PWM running, hit a breakpoint in the debugger and check that every INH and INL output goes low on the scope. Resume and note that the PWM carries on from where it stopped, so with a motor connected, disarm before resuming if the motor is stalled or loaded.
- **Pass:** 20.00 kHz. Each INH/INL pair is complementary with dead time. Phase mapping is correct. A break forces all outputs low and the PWM stays off until re-armed. A debugger halt forces all outputs low.

### Stage 6: Power stage switching, no motor

- **Under test:** the six MOSFETs, the gate drive and the real nFAULT-to-break path.
- **Connected:** 12 V limited to 0.5 A, ENABLE high. Motor unconnected. Scope on the gate test points J4–J9 and the switch nodes MOTA, MOTB and MOTC.
- **Firmware added:**
  - Arming goes only through motor\_can\_arm(): VM\_OK set, nFAULT high, DRV configuration verified, no latched fault. Every arm starts from a zero command.
  - The break input is enabled after the DRV wakes, with the break flag cleared first.
  - A test mode that sets a fixed duty per phase.
- **Procedure:**
  1. Run one phase at a time at 50% duty.
  2. Measure gate rise and fall times and switch-node ringing; adjust IDRIVE.
  3. Run all three phases together.
  4. Sweep the duty from 5% to 95%.
  5. Trip the real nFAULT path by briefly shorting nFAULT to GND while armed, for example at the R22/LED6 node. nFAULT is open-drain with a 10k pull-up, so this is safe.
- **Pass:** average switch-node voltage equals duty × 12 V. Edges are clean and gate voltages are correct. Supply current is low and stable: record the baseline with the DRV awake and PWM off (the DRV alone draws about 10 mA or more), then confirm PWM adds only the expected gate-drive current, with no unexplained DC load. No faults in normal running. The nFAULT break shuts the outputs off in hardware.

### Stage 7: Current-sense chain, no motor

- **Under test:** ADC timing, offsets and the noise floor.
- **Connected:** as in Stage 6, PWM running at 50% on all phases. Motor unconnected.
- **Firmware added:** `cursense` module.
  - ADC mapping: SOA → ADC2\_IN12, SOB → ADC1\_IN12, SOC → ADC3\_IN12, each self-calibrated at startup.
  - Injected conversions triggered by TIM1 TRGO2 (OC4REF). The ADC-done interrupt becomes the control interrupt, at the highest priority, and toggles the PA3 timing pin.
  - Offsets averaged over 1,024 samples per channel. Control-loop heartbeat: the interrupt increments a counter; if the main loop sees no change for a few ms, it disarms.
  - Conversion to amps, positive into the motor: I = (VSO − Voffset) / (40 × 5 mΩ).
  - SWO telemetry stream (sent at a reduced rate, non-blocking) and a RAM capture buffer that records every sample at 20 kHz around a trigger.
- **Procedure:**
  1. On the scope, compare PA3 with the PWM to confirm the samples land at the counter peak.
  2. Measure how long the interrupt takes.
  3. Log the zero-current noise. Then set the highest duty to the 93% cap and scope INLx against the PA3 timing pin: the sample must land after the low-side switch has turned on and the amplifier has had about 1.2 µs to settle. If not, lower the cap or move the trigger.
- **Pass:** offsets near 2048 counts, noise a few counts peak-to-peak, the sample point correct, and the interrupt under 10% of the cycle.

### Stage 8: Motor connected, standing still

- **Under test:** motor wiring, and that current readings match reality.
- **Connected:** motor on J13, 12 V limited to 1 A, encoder connected.
- **Firmware added:**
  - Software overcurrent trip in the control interrupt: about 1 A until the current readings are confirmed against a multimeter, then 1.5 A. Until that check passes, the readings that feed the trip are unproven.
  - Space-vector PWM producing a fixed voltage vector at a chosen angle.
- **Procedure:**
  1. First measure line-to-line resistance with a meter; expect about 3.2 Ω. If it reads 4.8 Ω, update R and L in `motor_params.h`.
  2. Apply a 0.5 V vector (phase-voltage amplitude) at 0°, 120° and 240°.
  3. Put a multimeter (DC amps) in series with the phase A lead and compare it with the measured Ia. The vector is DC, so the meter reads phase current directly; this is the independent check of the current readings. Then compare with Ohm's law. The supply current barely changes (about 20 mA more at 12 V, from power balance), so treat it only as a sanity check.
  4. Check each current channel matches its PWM phase and sign. At 0°: Ia ≈ +0.31 A, Ib and Ic ≈ −0.16 A each. At 120°: Ib is the large positive one. At 240°: Ic is. If the large current is on the wrong channel or has the wrong sign, fix the mapping before going on: this is the most common cause of a current loop that fights itself.
  5. Check that Ia + Ib + Ic ≈ 0.
  6. Watch the rotor snap to each angle.
  7. Lower the overcurrent threshold temporarily and confirm it trips, then set it to 1.5 A. Repeat the 0° vector with all three duties shifted up together so the highest phase reaches the 93% cap; the measured current must not change, which proves the sample window holds at maximum duty. Measure inductance: with the rotor held at 0° by the 0.5 V vector, step the 0° vector to 1.0 V and record Ia in the capture buffer (no torque, because the rotor is already aligned), fit the time constant τ, and compute L = τ × R (expect τ ≈ 0.54 ms and L ≈ 0.87 mH wye). An LCR meter across two motor leads also works; it reads twice the wye value (expect about 1.7 mH). Record the measured R and L in motor\_params.h.
- **Pass:** current matches V/R (about 0.3 A at 0.5 V with the 1.6 Ω wye resistance) within about 15% (dead time adds error at such low voltages). The three currents sum to about zero. The rotor holds firmly at each angle. The overcurrent trip works. Ia matches the multimeter within about 5%. Each angle puts the large current on the right channel with the right sign. Current doesn't change at maximum duty. Measured R and L are recorded.

## Stages 9–15: from first spin to speed-controlled FOC

The motor first spins open loop at Stage 9 and first runs under FOC at Stage 11. Speed control, the goal, is Stage 14.

### Stage 9: Open-loop spin (first rotation)

- **Under test:** the commutation math, and the pole-pair count measured by the encoder.
- **Connected:** as in Stage 8 (motor, encoder, 12 V limited to 1 A).
- **Firmware added:**
  - `foc_math`: CORDIC sine/cosine, inverse Park transform, space-vector PWM.
  - Forced-angle generator with a speed ramp.
  - Encoder fault checks now disarm the PWM.
- **Procedure:**
  1. Apply about 1 V amplitude and ramp from 0 to 5 Hz electrical.
  2. Run in both directions.
  3. Log the encoder against the forced angle.
- **Pass:** the motor turns smoothly. 11 electrical cycles equal one mechanical turn. Encoder speed matches forced speed. Current stays near 0.6 A.

### Stage 10: Encoder-to-electrical alignment

- **Under test:** encoder offset and direction.
- **Connected:** as in Stage 8.
- **Firmware added:** alignment routine.
  - Ramp the d-axis voltage vector to 0°, let it settle, and record the encoder.
  - Repeat at several electrical angles in both directions and average.
  - Detect the counting direction and store the offset.
- **Procedure:**
  1. Run alignment five times and compare the offsets.
  2. During open-loop spin, put the computed electrical angle on PA2 and phase A current on a second scope channel.
- **Pass:** the offset repeats within ±2° electrical, and the computed angle tracks the forced angle with only a constant error.
  - **Do not skip this stage.** A wrong offset, direction or phase order makes FOC run rough, vibrate or lock up, and those symptoms all look alike. Rerun it whenever the encoder or magnet is moved.

### Stage 11: Voltage-mode FOC (first spin under FOC)

- **Under test:** commutation driven by the encoder angle.
- **Connected:** as in Stage 8, with the limit raised to 1.5 A.
- **Firmware added:**
  - Control interrupt order: read the encoder → electrical angle with latency correction → inverse Park → space-vector PWM.
  - Commands: Vd = 0 and a hardcoded Vq, changed through a ramp rather than an instant step. Software overcurrent trip raised to about 3 A: at standstill the phase current is about Vq ÷ 1.6 Ω, so Vq = 4 V draws about 2.5 A until the motor speeds up.
- **Procedure:**
  1. Step Vq to 0.5, 1, 2 and 4 V, in both directions.
  2. Log speed and VM over SWO. Watch VM during each Vq step-down: it should stay below about 14 V. If it rises further, lengthen the Vq ramp. This characterizes braking energy before the speed loop can command hard deceleration.
- **Pass:** smooth rotation with speed roughly proportional to Vq. No-load speed approaches about Kv × √3 × Vq (about 420 rpm at Vq = 4 V). Current drops at speed as back-EMF rises.

### Stage 12: Rotating-frame current readings (shadow mode)

- **Under test:** Clarke and Park transforms and current signs. Current is measured but not yet controlled.
- **Connected:** as in Stage 11.
- **Firmware added:**
  - Clarke and Park transforms computing Id and Iq while voltage-mode FOC runs.
  - Id/Iq telemetry over SWO and into the capture buffer.
- **Procedure:**
  1. Run at a constant Vq.
  2. Slow the rotor by hand to add load.
- **Pass:** Iq is positive in the driving direction and rises with load. Id stays near 0. Id and Iq look like steady DC, not ripple at the electrical frequency.

### Stage 13: Current-loop FOC (torque mode)

- **Under test:** the Id and Iq PI controllers.
- **Connected:** as in Stage 11, with the limit raised to 2 A.
- **Firmware added:**
  - `pi` module with anti-windup and a clamp on the voltage vector.
  - Prerequisite: R and L measured in Stage 8. Gains computed from the measured values (datasheet values give Kp ≈ 5.4 V/A, Ki ≈ 10,000 V/(A·s)). First tests use 25–50% of the target bandwidth, raised only after the step response looks clean. Test-mode limits built into the firmware, not left to procedure: while the rotor is locked, Id and Iq commands are clamped to 0.3 A and the test disarms itself after 2 s.
  - Commands: Id = 0 and a hardcoded Iq.
- **Procedure:**
  1. Lock the rotor, or hold it on the d-axis.
  2. Capture an Id step response in the RAM buffer.
  3. Release it and command Iq = 0.2, 0.5 and 0.65 A.
- **Pass:** at the full 1 kHz target bandwidth the step response is roughly first-order, with a time constant of about 0.16 ms (10–90% rise about 0.35 ms); at the reduced starting gains it is proportionally slower, with little overshoot. Torque is smooth and Id ≈ 0 while rotating. The interrupt uses under 50% of the cycle.

### Stage 14: Speed control (goal)

- **Under test:** the full cascaded control loop.
- **Connected:** as in Stage 11, with the limit raised up to 4 A.
- **Firmware added:**
  - `motor_sm` state machine: IDLE → ALIGN → RUN → FAULT.
  - Speed PI at 2 kHz that outputs the Iq setpoint, clamped to 4 A peak and reduced toward 0.65 A by the I²t heating model.
  - Ramp limiter and a hardcoded speed profile.
  - Stop sequence: ramp down, then coast.
  - Software overcurrent trip raised to about 4.5 A (peak line current). VM overvoltage trip: VM\_SENSE above about 16 V makes the motor coast, because deceleration can pump energy back into VM and the bench supply can't absorb it.
- **Procedure:** run a 100 → 300 → 500 rpm step profile (top no-load speed is about 630 rpm with the duty cap) and tune the speed PI from the captured responses.
- **Pass:** speed holds within a few rpm, settles without oscillation, and recovers when the shaft is loaded by hand.

### Stage 15: Hardening

- **Under test:** behavior under faults.
- **Connected:** as in Stage 14.
- **Firmware added:**
  - Independent watchdog.
  - Stall detection: large Iq with no speed.
  - MOTOR\_EN dropping while running makes the motor coast and resets the state machine.
- **Procedure:** while running, pull the encoder connector, turn SW1 off, stall the shaft and force a watchdog reset.
- **Pass:** every case stops the motor safely and is logged, with no uncontrolled behavior.

## Firmware conventions

Every module follows these conventions. Most FOC bugs come from two pieces of code disagreeing on one of them.

### Units and types

- `float` (single precision, hardware FPU) in SI units everywhere: A, V, rad, rad/s, s. No fixed-point.
- Constants with the `f` suffix (`0.5f`), so nothing silently promotes to double.
- Measured values (R, L, encoder offset, direction, current offsets) live in `motor_params.h` or are measured at startup. Until the bench supplies them, they are marked `TODO_MEASURED` with the datasheet value.

### Phase order, angles and direction

- Phase A axis is electrical angle 0; phase B is at +120° (2π/3) and phase C at +240°. Positive rotation is the A → B → C sequence: the direction the forced angle increases in Stage 9.
- Mechanical angle from the encoder: θm = 2π × count / 16384.
- Electrical angle: θe = wrap(DIR × 11 × θm − θoffset), wrapped to \[0, 2π). DIR is +1 or −1 and θoffset is measured in Stage 10.
- Speed is electrical rad/s inside the control code. Conversion to rpm (mechanical) happens only at the setpoint and logging boundary: rpm = ωe × 60 / (2π × 11).

### Current sign

- Phase current is positive flowing **into** the motor terminal.
- The DRV8323 datasheet equation gives current flowing down through the shunt to ground, which is the opposite sign. The firmware therefore uses I = (VSO − Voffset) / (40 × 0.005 Ω), with Voffset the measured zero-current level.
- In ADC counts: I = (count − offset\_count) × (3.3 / 4096) / 0.2 A.

### Transforms

Amplitude-invariant Clarke, using all three measured currents:

```latex
I_\alpha = \tfrac{1}{3}(2I_a - I_b - I_c), \qquad I_\beta = \tfrac{1}{\sqrt{3}}(I_b - I_c)
```

Park and inverse Park:

```latex
I_d = I_\alpha\cos\theta_e + I_\beta\sin\theta_e, \qquad I_q = -I_\alpha\sin\theta_e + I_\beta\cos\theta_e
```

```latex
V_\alpha = V_d\cos\theta_e - V_q\sin\theta_e, \qquad V_\beta = V_d\sin\theta_e + V_q\cos\theta_e
```

With this Clarke form, Iq equals the peak phase current, matching the current convention under Key hardware facts.

### Space-vector PWM

1. Limit the voltage vector to |Vαβ| ≤ 0.497 × Vbus (about 6.0 V at 12 V), the largest vector whose duties stay within the 93% cap. Vd keeps priority; Vq gets what is left.
2. Inverse Clarke: va = Vα, vb = −Vα/2 + (√3/2)Vβ, vc = −Vα/2 − (√3/2)Vβ.
3. Midpoint (min-max) injection: subtract (max + min)/2 from all three.
4. Duty = 0.5 + v / Vbus, clamped to \[0, 0.93\]. The clamp is a backstop; step 1 already keeps duties inside it.
5. CCRx = duty × ARR, written through the phase table (A → CCR3, B → CCR2, C → CCR1). PWM mode 1: the high side is on while CNT < CCR, so every low side is on at the counter peak, where the ADCs sample.

Vbus is fixed at 12.0 V in software until VM\_SENSE is used for it.

### Timing, interrupts and data sharing

| Context | Rate | Runs | Must not |
| --- | --- | --- | --- |
| Control interrupt (ADC injected end of sequence) | 20 kHz | Read currents and encoder, overcurrent trip, FOC math, duty writes, heartbeat, capture buffer, scope DAC; speed loop every 10th call (2 kHz) | Call HAL, log over SWO, block, or touch SPI3 |
| TIM1 break interrupt | On fault | Latch the fault code; outputs are already off in hardware | Re-arm anything |
| Main loop | 1 kHz tick | Power state machine, DRV readback, fault decode, arming requests, I²t, logging, stage test sequences | Write CCR registers or MOE directly |

- Interrupt priority: control interrupt and TIM1 break at the highest priority; everything else lower. Before the PWM is configured (Stage 4), nFAULT is watched with an EXTI interrupt on PB10.
- Data shared between the interrupt and the main loop is `volatile`. Multi-word values the main loop reads are copied with interrupts briefly disabled, or double-buffered.
- Only `motor_can_arm()` sets MOE; only `motor_disarm()` and the hardware break clear it.
- Faults are an enum of codes, latched until an explicit clear, with the first fault's code and context logged over SWO.

### Stage selection

- `BRINGUP_STAGE` is defined in one header (`app_config.h`). Each stage's test mode is compiled in only when selected, and the modules from earlier stages stay active underneath it.

## Firmware modules and open items

All modules live outside the CubeMX "USER CODE" sections, so regenerating the project doesn't overwrite them.

| Module | Responsibility | First stage |
| --- | --- | --- |
| `board.h` | Pin definitions, phase-to-channel mapping, safe-pin boot routine | 0 |
| `debug.c/h` | SWO logging and telemetry, cycle counter, capture buffer, scope DAC on PA2, timing pin on PA3 | 0 |
| `mt6701.c/h` | SPI1 encoder read, CRC and status checks, unwrapping, PLL speed estimate | 1 |
| `power.c/h` | MOTOR\_EN debounce and the VM power state machine | 2 |
| `drv8323.c/h` | SPI3 register access, wake/sleep sequence, configuration, fault decoding | 3 |
| `fault.c/h` | Latched fault codes with first-fault context; explicit clear only (added in Stage 4; not in the original module list) | 4 |
| `pwm.c/h` | TIM1 setup, motor\_can\_arm() (the only path that sets MOE), disarm, duty writes, break handling | 5 |
| `cursense.c/h` | ADC injected sampling, offsets, conversion to amps | 7 |
| `foc_math.c/h` | Clarke/Park transforms, CORDIC sine/cosine, space-vector PWM | 8 |
| `pi.c/h` | PI controllers with anti-windup | 13 |
| `motor_params.h` | Pole pairs, R, L, back-EMF constant, encoder offset, current limits | 8 |
| `motor_sm.c/h` | Motor state machine, speed profile, protections | 14 |
| `app.c` | `BRINGUP_STAGE` switch, 1 kHz main-loop tasks | 0 |

**Stage prerequisites** (each blocks its stage until done)

- [ ] Stage 1: encoder MODE pin set for I2C/SSI, magnet checks done (Stage 1 setup checklist).
- [ ] Stage 8: motor line-to-line resistance measured (expect about 3.2 Ω) to confirm the delta conversion.
- [ ] Stage 8: current readings confirmed against a multimeter before the trip is raised above 1 A.
- [ ] Stage 13: inductance measured and current-loop gains recomputed from the measured R and L.

## Bring-up log

Dated record of findings, decisions and measured results, newest last. Each entry names the stage it affects. When an entry changes a fact, the section above that holds the fact is updated too.

### 2026-10-04: document and schematic review (before Stage 0)

- **Pin map checked** against the STM32 schematic sheet and the STM32G491 datasheet pin table (DS13122 Table 12). Every pin's function is available where the plan puts it: SPI1 on PA4–PA7, SPI3 on PC10–PC12, TIM1\_CH1–3 on PA8–PA10, CH1N–3N on PB13–PB15, TIM1\_BKIN on PB10, ADC2/1/3\_IN12 on PB2/PB1/PB0, ADC12\_IN14 on PB11, OPAMP1\_VOUT on PA2.
- **DRV8323 register values checked** against the datasheet register map (SLVSDJ3D section 8.6). All bring-up values and reset values in the register table decode correctly.
- **Encoder lines:** three 49.9 Ω series resistors, not one: R41 on CLK (PA5) and R42 on MOSI (PA7) at the MCU, and R40 on DO (PA6) at J14 pin 3. J14 pin order: 1 = 3.3 V, 2 = MOSI, 3 = DO, 4 = CLK, 5 = CSN, 6 = GND. Pin 2 is not connected on the encoder board. Affects Stage 1.
- **DRV nSCS pull-up:** R38 10k from 3.3 V holds nSCS high from power-up, so PD2 doesn't need to be driven to keep the DRV deselected. Affects Stages 0 and 3.
- **SPI clock idles undriven while the SPI is disabled.** RM0440 (SPI clock timing note) says the SCK idle level must be set by a pull resistor; with SPE = 0 the pin isn't driven. CubeMX init leaves SPE = 0 until the first transfer. That's what Stage 0 wants for SPI3. For Stages 1 and 3, firmware enables the SPI (SPE = 1) before the first chip-select falling edge, so the clock is already driven low when chip select goes low.
- **TIM1 outputs before arming:** with MOE = 0 and CCxE = CCxNE = 0, RM0440 Table 282 gives "output disabled". The PWM pins then read low through the DRV's 100 kΩ input pull-downs. Affects the Stage 0 "all PWM pins low" check.
- **CubeMX configuration for Stage 0 is already in place:** HSE 24 MHz, PLL M = 6, N = 85, R = 2 → 170 MHz, voltage scale 1 boost, flash latency 4; PA3 and PC3 outputs, low at boot; PC8 ENABLE and PC9 CAL low at boot; PD2 analog (undriven); PA4 high at boot; DAC3\_CH1 (internal) → OPAMP1 follower → PA2; SWO on PB3.
- **SWO viewing:** the STM32Cube VS Code extension doesn't support SWO yet. SWO is read with STM32CubeProgrammer's SWV viewer or the Cortex-Debug extension, with the core clock set to 170 MHz. Affects Stage 0.
- **Cortex-Debug setup:** with `servertype: stlink` (ST-LINK\_gdbserver), Cortex-Debug warns "SWO support is not available from the probe when using the ST-Link GDB server" and turns SWO off. `.vscode/launch.json` therefore uses ST's OpenOCD from STM32CubeIDE 2.1.1 (`interface/stlink.cfg`, `target/stm32g4x.cfg`, which creates the TPIU), with SWO at 2 MHz from a 170 MHz core clock, port 0 as a text console. With `interface/stlink.cfg` (the old `hla` driver), this OpenOCD build (0.12.0+dev, CubeIDE 2.4.400 plugin) recursed forever between `hla newtap` and the ST script `swj_newdap`, then died before reaching the probe. Fixed by using `interface/stlink-dap.cfg` (direct `st-link` DAP driver), which doesn't take that path. A temporary SWO test in `main.c` USER CODE 3 prints `SWO test N` every second and toggles LED4. With `stlink-dap.cfg`, the debug session connects, flashes, stops at `main()` and OpenOCD starts the SWO trace server ("Listening on port 50003 for tpiu\_swo\_trace connections"). **SWO confirmed working (2026-10-04):** the `SWO test N` lines appear in the Terminal panel, in a terminal named "SWO: SWO [type: console]" (Cortex-Debug shows console decoders as a terminal by default, not in the Output panel). Run: select "Debug (OpenOCD + SWO)", F5, F5 again past the stop at `main`, then open that terminal from the list on the right of the Terminal panel.

### 2026-10-04: Stage 0 firmware written (not yet built or tested)

- **Files:** `App/app_config.h` (`BRINGUP_STAGE 0`), `App/board.h`, `App/debug.c/h`, `App/app.c` plus `App/app.h` (prototypes for the two calls from `main.c`; not in the module table, added for the build). `CMakeLists.txt` adds the sources and the `App` include path. `main.c` calls `app_init()` in USER CODE 2 and `app_loop()` in USER CODE 3. The temporary SWO test is removed.
- **CubeMX change required:** NVIC → Code generation → Hard fault interrupt → untick "Generate IRQ handler", then regenerate. `debug.c` provides a naked `HardFault_Handler` so it can find the stacked registers reliably; until the box is unticked the link fails with a duplicate `HardFault_Handler`.
- **Decision, safe pins:** at boot `board_safe_pins()` sets TIM1 CCxE = CCxNE = 1 for channels 1–3 with MOE = 0, OSSI = 1 and OISx = 0, so TIM1 actively drives all six gate inputs low (RM0440 Table 282, MOE = 0 / OSSI = 1 row: off-state, inactive level). This is used instead of leaving them Hi-Z because the DRV8323's 100 kΩ pull-downs can't be relied on while the DRV is unpowered. MOE is never set here. ENABLE and CAL are forced low, SPI3 is disabled, and PD2 stays analog. `board_safe_pins_check()` reads all of this back.
- **Fault handler:** on a HardFault it clears MOE, then drives ENABLE low, then prints pc, lr, psr, r0–r3, r12, CFSR, HFSR, MMFAR, BFAR and EXC\_RETURN over SWO. It halts at a breakpoint if a debugger is attached, otherwise it blinks LED4 fast. `STAGE0_FAULT_TEST 1` in `app_config.h` triggers a fault (undefined instruction) 5 s after boot to test it.
- **Expected SWO at boot:** `BOOT stage=0`, `SCOPE dac3+opamp1 OK`, `CLK hse_rdy=1 pll_src=HSE sys_src=PLL m=6 n=85 r=2`, `CLK sysclk=170000000 hclk=170000000 pclk1=170000000 pclk2=170000000 OK`, `CYC 10ms=… expect>=1700000 OK`, `SAFE pwm_pins=0x00 err=0x00 OK`, then `ALIVE t=Ns task_max_us=…` every second.
- **Expected signals:** LED4 1.000 Hz (500 ms on, 500 ms off); PA3 500.0 Hz square (toggled every 1 ms tick); PA2 triangle at 5 Hz, 100 ms up and 100 ms down, near 0–3.3 V (OPAMP1 output may stop slightly short of the rails).

### 2026-10-04: Stage 0 passed

- CubeMX: "Generate IRQ handler" unticked for Hard fault; `HardFault_Handler` is now only in `App/debug.c`.
- The developer reported every Stage 0 check passed: boot log all OK, clock 170 MHz, SWO stable, PA2 triangle, PWM pins low, fault dump test. Measured values weren't recorded. `STAGE0_FAULT_TEST` set back to 0.

### 2026-10-04: Stage 1 firmware written (not yet built or tested)

- **Files:** `App/mt6701.c/h` (new), `App/app.c` (Stage 1 test mode), `App/app_config.h` (`BRINGUP_STAGE 1`), `CMakeLists.txt` (adds `mt6701.c`).
- **SPI1 read (register access, usable from the control interrupt later):**
  - At init the firmware checks SPI1 against CubeMX: mode 1, 12-bit, MSB first, software NSS, ÷32 = 5.31 MHz. It then sets SPE once with CS high, so CLK is driven low before the first CS edge.
  - Each read: CS low, wait 200 ns (datasheet T\_H ≥ 100 ns), two dummy 12-bit frames, wait for both received and BSY clear, wait 200 ns (T\_L ≥ 0.5 × 188 ns), CS high. The wait has a 20 µs timeout.
  - Word = frame1 << 12 | frame2.
- **Decode:** angle = bits 23–10, status Mg\[3:0\] = bits 9–6, CRC = bits 5–0. The CRC-6 (x⁶ + x + 1, MSB first, initial value 0) covers the 18 angle + status bits. The datasheet doesn't state the initial value; 0 is assumed. If every frame fails CRC while the angle looks right, check this first, before suspecting the SPI mode.
- **Checks and counters:** SPI timeouts, CRC errors, field too strong, field too weak (or the reserved code), loss of track, angle jumps. A jump is a step larger than 1,500 rpm allows per sample (409 counts at 1 kHz, minimum 64); 1,500 rpm is about twice the motor's 730 rpm no-load speed. A bad sample doesn't update position or speed. "Healthy" = good sample, field normal, no loss of track. Disarming on encoder faults comes in Stage 9.
- **Position and speed:** multi-turn by unwrapping (turn count ±1 on wrap). Speed from a second-order PLL in mechanical rad/s, critically damped, 50 Hz natural frequency (Kp = 2ωn, Ki = ωn²).
- **Stage 1 test mode:** read at 1 kHz in the main loop. PA3 is high during each read (scope trigger). PA2 shows the angle as a 0–3.3 V sawtooth per turn. LED4 1 Hz heartbeat. SWO: `ENC spi1 ... OK` and `ENC first raw=... crc=OK field=NORMAL` at boot; `ENC field=`, `ENC loss_of_track=` and `ENC push=` on change; status line every 200 ms: `ENC a= turns= rpm= ok= crc= jump= to= strong= weak= lot= rd_us=`.

### 2026-10-04: Stage 1 first run (SWO log, t = 30–55 s)

- **Link and decode good:** crc = 0, jump = 0, to = 0, strong = weak = lot = 0 throughout, while the shaft was turned by hand through several turns in both directions. CRC-6 with initial value 0 and SPI mode 1 confirmed by zero CRC errors (no mode 2 fallback needed).
- **Unwrapping good:** wraps counted correctly both ways (e.g. a 723 → 16166 gave turns 0 → −1; a 15725 → 23 gave turns −2 → −1). Hand-turning speeds read 20–75 rpm.
- **Noise at rest:** the angle holds within 1–2 counts when still (e.g. 6745–6746, 2765–2767).
- **Read time longer than estimated:** `rd_us` = 16 µs against about 5 µs expected (24 bits at 5.3 MHz = 4.5 µs plus 0.4 µs of CS waits). This is probably the -O0 Debug build (CRC loop, float PLL, volatile state) on top of the SPI time. To check: scope the CSN-low time against the PA3-high time. It doesn't block Stage 1, but it matters when the read moves into the 20 kHz control interrupt (50 µs budget). Revisit then.
- **Main-loop tick:** `task_max_us` = 803–813 µs, set by the ENC SWO line (about 110 characters at 2 MHz SWO, written inside the tick). It fits within the 1 ms tick; the catch-up loop keeps reads at 1 kHz on average.
- **Boot lines good:** all Stage 0 checks still OK at stage 1, plus `ENC spi1 mode1 12bit div32 OK` and `ENC first raw=0xF5802F a=15712 st=0x0 crc=OK field=NORMAL lot=0`. Hand decode of 0xF5802F agrees: angle = word >> 10 = 15712, status = (word >> 6) & 0xF = 0, CRC = 0x2F. `CYC 10ms=1852968` (10.9 ms) is inside the 10–11 ms that `HAL_Delay(10)` can take.
- **Still to confirm:** the PA2 sawtooth, the SSI frame on the scope, the 90° ≈ 4,096-count check, one full minute with crc = 0, and which physical direction makes the count rise.

### 2026-10-04: Stage 1 passed (developer's call)

- PA3 measured 1 kHz, as expected (one pulse per 1 ms encoder read). PA2 angle output works (the developer first saw nothing, then confirmed it after checking the DC level while turning).
- The developer moved on to Stage 2. **Not confirmed:** the SSI frame on the scope (CSN-low time against the 16 µs read time), the 90° ≈ 4,096-count check, a full minute of crc = 0 (about 25 s seen), and the physical direction of rising count. Stage 10 detects direction by itself. The read-time question comes back when the read moves into the control interrupt.
- [x] Stage 1 prerequisite (encoder MODE pin, magnet checks) treated as done, since the encoder works with clean field status.

### 2026-10-04: Stage 2 firmware written (not yet built or tested)

- **Files:** `App/power.c/h` (new), `App/debug.c/h` (`debug_log_reset_cause()`), `App/app.c` (restructured: the encoder now updates every tick from Stage 1 onward, the power module from Stage 2 onward, and each stage's test mode is separate), `App/app_config.h` (`BRINGUP_STAGE 2`), `CMakeLists.txt` (adds `power.c`).
- **VM\_SENSE:** ADC1 regular channel 14 (PB11), calibrated (single-ended) and enabled at boot, then four conversions averaged every 1 ms tick by direct register polling (no HAL). The polling avoids conflicting with the HAL handle once injected conversions start in Stage 7. VM = code × 3.3 / 4096 × 19, assuming VDDA = 3.3 V. Schematic: 180k / 10k divider (designators unreadable in the screenshot) with **C26 100 nF** across the 10k (τ ≈ 0.95 ms). C26 supplies the ADC's sampling charge, so CubeMX's 2.5-cycle sampling time is adequate. ADC clock = HCLK / 4 = 42.5 MHz (synchronous).
- **MOTOR\_EN:** PC1, debounced 5 ms.
- **State machine:** as in the plan, with these details. VM\_SETTLING needs MOTOR\_EN high and VM ≥ 10 V continuously for 20 ms, otherwise it goes back to NO\_VM. VM\_OK drops to NO\_VM on MOTOR\_EN low or VM < 9 V. It always starts in NO\_VM at boot.
- **Reset cause:** RCC\_CSR decoded and printed at boot, then cleared (RMVF). BORRSTF is reported as power-on/brownout. PINRSTF accompanies every reset, so "PIN" is reported only when nothing else is set. A reset from the debugger shows SOFTWARE or PIN, not power-on.
- **LED4:** NO\_VM blinks at 1 Hz, VM\_SETTLING at 5 Hz, VM\_OK is steady on.
- **Stage 2 test mode:** `PWR` status line every 500 ms; state changes logged immediately. On each raw MOTOR\_EN rising edge, VM is captured every 1 ms for 500 ms, then printed every 10 ms (`RAMP t= vm_mv=`) with a summary `RAMP sw1_on->10V=…ms sw1_on->VM_OK=…ms`.

### 2026-10-04: Stage 2 first runs (12 V, 150 mA limit, USB unplugged)

- **SW1 off:** boot log all OK from 12 V alone. en = 0, vm\_mv = 0, NO\_VM, LED4 slow blink. The first reset cause showed POWER\_ON\_OR\_BROWNOUT with csr = 0x1C000000 (BOR + PIN + SFT). The BOR flag was left over from applying 12 V, because earlier stages never cleared the flags.
- **SW1 switched on with the debug session running:** the meter read VM correctly and LED4 went steady (VM\_OK, firmware running), but **SWO output stopped**. A debugger pause then halted the CPU normally in `app_loop()`, so the debug link was still alive.
- **Restart with SW1 on:** `RESET cause=SOFTWARE csr=0x14000000` (PIN + SFT, **no BOR**), so the MCU **did not brown out** when SW1 was switched on. NO\_VM → VM\_SETTLING → VM\_OK in 20 ms, as designed. Why SWO stopped is still open. Candidates: the trace stream was disturbed by the relay switching transient, or a 3.3 V dip that stayed above the BOR threshold (BOR level 0 ≈ 1.7 V, so a dip to around 2.5 V wouldn't reset). The next test toggles SW1 with the session running and checks whether SWO continues.
- **VM reading:** 11,817 mV (code 772), steady to ±1 code (15 mV of VM per code). Not yet compared numerically with the meter at J3 (developer reports the reading is right).
- **Bug fixed:** booting with SW1 already on counted as a switch-on edge and captured a flat "ramp" at 11.8 V. `s_en_raw_prev` is now set to the current MOTOR\_EN level at init.

### 2026-10-04: Stage 2 VM accuracy and SW1 toggle

- **VM accuracy: pass.** Meter at J3 about 11.8 V against firmware 11.817 V (code 772), well within 3%. Noise ±1 code.
- **SW1 off: VM\_OK drops at once.** `VM_OK -> NO_VM en=0 vm_mv=11817`: the drop comes from MOTOR\_EN (debounced 5 ms), not from VM.
- **Finding: VM stays charged for minutes after SW1 off.** VM decayed 11.82 → 10.73 V in about 16 s. With the relay open, the bulk capacitors (about 1,000 µF) discharge only through the 190 kΩ VM\_SENSE divider (and the sleeping DRV's µA draw): τ ≈ 190 kΩ × 1,000 µF ≈ 190 s, which predicts 10.85 V after 16 s. That matches. VM takes about 15 minutes to fall below 1 V. SW1 removes the source, not the stored energy. Bench rule 5 updated.
- **SW1 on with VM still at 10.7 V:** VM\_OK after 24 ms (5 ms debounce + 20 ms settle), with VM back to 11.8 V within 10 ms. Little inrush, because the capacitors were nearly charged. SWO kept running and no reset occurred. This was **not** the cold-start ramp the procedure asks for (step 4 needs VM starting near 0 V).
- **Switch-on from VM = 7.6 V (150 mA limit), session running:** no reset. No new `BOOT` line, ALIVE uptime continuous (12 → 13 → 14 s); the only reset flags at boot were from the debugger (SOFTWARE). VM passed 10 V 6 ms after SW1 on and reached 11.77 V within 10 ms; VM\_OK at 26 ms. Rising 4.1 V in ≤ 10 ms into about 1,000 µF needs ≥ 0.4 A, above the 150 mA setting. The charge probably came from the bench supply's output capacitor before its current limit responded. Watch for this: the supply limit doesn't cap a fast surge.
- **Bulk capacitance estimate:** VM decay τ = 150–165 s across two runs (8.28 → 7.65 V in 12 s; 11.82 → 10.73 V in 16 s). Through 190 kΩ that gives C ≈ 0.8–0.87 mF, consistent with the nominal 1,000 µF (C27–C29 3 × 330 µF + C31 10 µF) given electrolytic tolerance and the DRV's sleep draw.
- **Still to do:** the cold-start ramp (VM discharged first), several SW1 toggles with no resets, TP2 buck voltage, and checking whether the earlier SWO dropout happens only on cold switch-on (large inrush).

### 2026-10-04: Stage 2 passed (with the switch-on note confirmed)

- **Switch-on from VM bled to about 2 V, 150 mA limit:** SWO stopped suddenly, the same as the first cold switch-on. This is consistent with the plan's switch-on note (inrush sags the 12 V input). Not proven whether the MCU reset or only the SWO trace dropped. A restart showing `RESET cause=POWER_ON_OR_BROWNOUT` would settle it.
- **Decision (developer):** raise the supply limit (about 1 A) for SW1 switch-on, as the switch-on note prescribes, then lower it to the stage's limit.
- **Pass:** runs from 12 V alone; VM within 3% of the meter (11.8 V against 11.817 V); VM\_OK only above 10 V, held 20 ms; VM\_OK drops on SW1 off. Not measured: TP2 buck output. No cold-start (0 V) ramp captured.

### 2026-10-04: Stage 3 firmware written (not yet built or tested)

- **Files:** `App/drv8323.c/h` (new), `App/board.h` (nFAULT pin, `motor_disarm()`), `App/app.c` (DRV wake/sleep tied to VM\_OK, Stage 3 test sequence), `App/app_config.h` (`BRINGUP_STAGE 3`), `CMakeLists.txt` (adds `drv8323.c`). No CubeMX change: PD2 stays analog in CubeMX and is switched to a push-pull output (driven high first) at wake, then back to analog at sleep. Runtime register writes, as the plan's "undriven until VM present" rule needs.
- **Datasheet facts used (SLVSDJ3D):** tWAKE and tSLEEP 1 ms max (7.5), so the firmware waits 1.1 ms after ENABLE high and keeps a re-wake at least 2 ms after a sleep. SPI timing (7.6): nSCS setup and hold ≥ 50 ns (firmware 200 ns), high ≥ 400 ns between words (firmware 1 µs). SPI format (8.5.1.1): SDO = 5 don't-care bits + 11 data bits; a write returns the old value. nFAULT is held low during wake and sleep for at most tWAKE/tSLEEP (8.4.1.2). Registers reset on sleep and UVLO.
- **Wake order:** PD2 high (output) → SPI3 SPE = 1 (SCLK driven low before any nSCS edge) → ENABLE high → wait 1.1 ms → check nFAULT. **Sleep order:** `motor_disarm()` (MOE = 0, then ENABLE low) → SPI3 disabled per the RM0440 procedure → PD2 back to analog (R38 holds nSCS high).
- **Register access:** register polling on SPI3 with a 100 µs timeout per word (a word takes 24 µs). Read and write are refused while the DRV is asleep. Write-then-verify, presence check (0x03 = 0x3FF; 0x7FF means no reply), and fault decode to text (both fault registers, bit names from datasheet Tables 8-12 and 8-13).
- **Hard-rule note:** `motor_disarm()` lives in `board.h` until `pwm.c` exists (Stage 5). Apart from the hardware break, it is the only normal-operation path that clears MOE. The boot safe-pin routine and the HardFault handler also clear MOE directly; these are deliberate exceptions for boot and fault handling.
- **Stage 3 test sequence (after each wake on VM\_OK):**
  1. Presence check.
  2. Read 0x00–0x06 and compare 0x02–0x06 with the defaults 0x000, 0x3FF, 0x7FF, 0x159, 0x283; fault registers should read clean.
  3. Write 0x05 = 0x110 and verify.
  4. Lock (0x03 = 0x6FF), then a write of 0x159 to 0x05 must be ignored (still 0x110).
  5. Unlock (0x03 = 0x3FF), then 0x05 = 0x159 must verify.
  6. CLR\_FLT: write 0x02 = 0x001, read back 0x000.
  7. Write 0x05 = 0x110, sleep, wake, then 0x05 must be back to 0x159 and all defaults restored.
  8. `DRV test PASS/FAIL`.

  After the sequence, a status line every second shows nFAULT, both fault registers decoded, and the SPI error count. On VM\_OK loss: `DRV sleep (VM_OK lost)`.

### 2026-10-04: Stage 3 first run: DRV test PASS

- Booted with SW1 already on (VM 11.8 V): VM\_OK, then `DRV wake nfault=high`, and the full test sequence passed (`DRV test PASS spi_err=0`).
  - Presence 0x03 = 0x3FF.
  - Defaults 0x000 / 0x3FF / 0x7FF / 0x159 / 0x283 for 0x02–0x06; fault registers 0x000 / 0x000.
  - Write-verify 0x05 = 0x110.
  - Lock (0x6FF) ignored a write to 0x05; unlock (0x3FF) then restore 0x159 worked.
  - CLR\_FLT read back 0x000.
  - After sleep/wake, 0x05 returned to 0x159 and all defaults were restored; nFAULT high after each wake.
- Steady state: nFAULT high, no faults, spi\_err = 0 for 20+ s.
- `task_max_us` = 14,241 µs: the one-time blocking test sequence (SPI words plus about 40 log lines) inside one tick. Expected for this test mode only. The catch-up loop recovers the missed ticks.
- **Still to check:** LED5 on/off with SW1; SW1 off → `DRV sleep (VM_OK lost)`; repeated SW1 cycles each passing; SPI timing on the scope (nSCS high ≥ 400 ns between words, SCLK low at both nSCS edges, SDO edges).

### 2026-10-04: Stage 3 passed

- Developer: the SPI scope captures look clean. **Decision: SPI3 stays at 664 kHz** (÷256); 1.33 MHz is not needed.
- Not explicitly reported: the LED5 on/off check and the SW1-off log lines (`DRV sleep (VM_OK lost)`).

### 2026-10-04: Stage 4 firmware written (not yet built or tested)

- **Files:** `App/fault.c/h` (new; added to the module table), `App/drv8323.c/h` (configuration, CSA calibration, lock and lock test, readback against the shadow copy, nFAULT EXTI), `App/app.c` (configure after every wake, 100 ms readback, fault latch and response, Stage 4 status line), `App/app_config.h` (`BRINGUP_STAGE 4`), `CMakeLists.txt` (adds `fault.c`). No CubeMX change.
- **Configuration after every wake:**
  1. Presence check.
  2. Write and verify 0x02 = 0x000, 0x04 = 0x722, 0x05 = 0x110, 0x06 = 0x2C3, 0x03 = 0x322.
  3. CSA offset calibration: 0x06 = 0x2DF (CSA\_CAL\_A/B/C set), hold 200 µs (datasheet 8.3.4.3: auto-trim takes 100 µs), back to 0x2C3 and verify. This has to happen before the lock, because a locked DRV ignores writes to 0x06.
  4. Lock: 0x03 = 0x622, verify.
  5. Lock test: write 0x159 to 0x05; it must still read 0x110.

  The shadow copy for the readback is 0x02–0x06 = 0x000 / 0x622 / 0x722 / 0x110 / 0x2C3.
- **Periodic check (every 100 ms while configured):** 0x02–0x06 against the shadow copy (a mismatch is FAULT\_DRV\_CONFIG\_MISMATCH), then both fault registers (any bit set is FAULT\_DRV\_FAULT\_BITS). Any SPI timeout is FAULT\_DRV\_SPI.
- **nFAULT interrupt:** EXTI line 10 on PB10, falling edge, NVIC priority 4. Set up at runtime: CubeMX can't put EXTI on a pin that's also TIM1\_BKIN. The pin stays in its BKIN alternate function; the EXTI uses the GPIO input path, which is active in AF mode (to be confirmed on the bench by the wake-pulse count). The interrupt only latches. A blanking flag covers sleep and the wake window (ENABLE high + 1.1 ms), so the wake pulse is counted as `nf_ignored`, not a fault. After the window, a falling edge raises FAULT\_DRV\_NFAULT (with both fault registers as context). nFAULT still low at the end of the wake window raises FAULT\_DRV\_WAKE.
- **Fault response:** the first fault is latched with its context and logged once. The DRV fault registers are read and decoded before the DRV is put to sleep, because sleep resets them. Then `drv8323_sleep()` disarms (outputs off, then ENABLE low). The DRV isn't woken again while a fault is latched. **Clearing is explicit:** set `g_fault_clear_request = 1` from the debugger (pause, edit in Watch or the Debug Console, continue). VM\_OK loss is normal operation, not a fault.
- **Note on 0x06 SEN\_LVL = 1 V:** the sense-overcurrent comparator looks at the SP pin voltage, which is the shunt voltage (datasheet 8.3.6.4). 1 V across 5 mΩ is 200 A, so SEN\_OCP never trips on this board. Short-circuit protection is VDS OCP (0.06 V, about 16–21 A), as the register table says. No change made.
- **First run (3 SW1 cycles in about 32 s):** every wake configured OK. Registers 02 = 0x000, 03 = 0x622, 04 = 0x722, 05 = 0x110, 06 = 0x2C3; lock test held 0x110; fault registers 0x000 / 0x000. Readback about 10 checks/s with no mismatch; nf\_ev = 0; fault = NONE.
  - **EXTI on PB10 in TIM1\_BKIN AF mode confirmed working:** `wake_pulses_ignored=1` on every wake. `nf_ignored` also rises by 1 at each sleep (the datasheet's power-down pulse), so it goes up by 2 per SW1 cycle.
  - `task_max_us` = 3,929 µs: the configure sequence (about 20 SPI words plus calibration plus logging) runs once per wake.
  - Still to do: complete about 20 SW1 cycles, SOA/SOB/SOC with a meter (expect about 1.65 V), and the optional nFAULT-short check of the fault latch.
- **Stage 4 test mode:** after each wake, `DRV wake #N ... wake_pulses_ignored=… cfg OK ...` and a register dump line. A status line every second: state, nFAULT, fault registers, wakes / cfg\_ok / cfg\_fail / checks, nf\_ev / nf\_ignored, latched fault.
