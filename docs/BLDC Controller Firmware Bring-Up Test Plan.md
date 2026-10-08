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
- **Range at 40 V/V:** ±8.25 A ideal full scale, about 4 mA per ADC count (3.3 V ÷ 4096 ÷ 0.2 V/A). The SOx linear range is 0.25 V to VREF − 0.25 V (datasheet 7.5, VLINEAR), so the usable range is about ±7.0 A ((1.65 − 0.25) V ÷ 0.2 V/A). Every limit and trip must stay inside ±7 A. The motor's 4 A peak uses about half the range. Current convention: the shunts measure line current, and the Clarke transform is amplitude-invariant, so Iq equals peak line current. The motor's 4 A maximum is treated as peak line current, and every current limit and trip in this plan uses that convention. This is an assumption: the manufacturer doesn't say whether 4 A is peak or RMS, line or winding. Until that's confirmed, keep peak commands at or below 4 A and don't rely on margin above it.
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
- **Break:** TIM1\_BKIN on PB10 (nFAULT). The hardware break (BKE) is always on, which is harmless while MOE = 0. The DRV's wake and sleep pulses set the break flag while disarmed, so `motor_can_arm()` clears BIF and SBIF and only then enables the break interrupt, together with MOE. Arming needs a configured DRV, so this always comes after the DRV has woken.
- **Speed loop:** every 10th control interrupt, so 2 kHz.

### Motor (4015 gimbal)

| Parameter | Datasheet value | Value used for FOC |
| --- | --- | --- |
| Pole pairs | 11 (22 poles, 24 slots) | 11 |
| Winding | Delta | Converted to an equivalent wye model |
| Resistance | 4.8 Ω (read as per winding) | **Measured 4.4 Ω line-to-line (Stage 8, 2026-10-08): 2.2 Ω per phase (wye).** That fits the 4.8 Ω spec being line-to-line, not per delta winding (which would give 3.2 Ω). |
| Inductance | 2.6 mH (read as per winding) | **Measured 0.54 mH per phase (wye)** (Stage 8 step 7, 2026-10-08: three current steps, 525 / 611 / 483 µH, spread ±12 %). Lower than both spec readings (1.3 mH if line-to-line, 0.87 mH if per delta winding) |
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

The gains above use the original 1.6 Ω / 0.87 mH estimate. With the measured 2.2 Ω and 0.54 mH (Stage 8) they become Kp ≈ 0.54 mH × 2π·1000 ≈ 3.4 V/A and Ki ≈ 2.2 Ω × 2π·1000 ≈ 13,800 V/(A·s); Stage 13 recomputes them from the final measured R and L.

With 12 V DC across two leads, stall current is about 12 V ÷ 4.4 Ω ≈ 2.7 A (measured R; 3.75 A with the original 3.2 Ω estimate). With the 93% cap the largest vector is about 6.0 V, or 6.0 ÷ 2.2 ≈ 2.7 A at standstill. That is below the 4 A maximum, but the current clamp and software trip, not the motor's resistance, still set the limit: winding resistance rises as the motor heats. The encoder-to-electrical offset depends on how the magnet is mounted, so Stage 10 measures it.

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
| TIM1 system break (SYSCFG\_CFGR2 CLL, SPL, ECCL) | Core lockup (a fault inside the fault handler), SRAM parity error, flash double-ECC error | Hardware break, outputs off; locked until reset | Stage 5 |
| Fault handlers (HardFault, NMI) | Crash or memory error while the bridge is switching | Outputs off, then ENABLE low, registers logged over SWO, then halt (debugger) or fast LED blink | Stage 0 (HardFault), Stage 5 (NMI) |
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

IDRIVE (bits 7–0 of 0x03 and 0x04) can be retuned at run time in Stage 6. The shadow copy follows it automatically. Once tuned, the chosen code becomes the boot value `DRV_CFG_IDRIVE` in `drv8323.h`, and the register table above is updated.

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
| Gain 40 V/V | A 0.5 V vector into the measured 2.2 Ω gives about 0.23 A, so SOA should rise about 45 mV (56 counts) above its offset (0.23 A × 5 mΩ × 40; current into the motor raises SOx). | 8 |
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
  - [x] The encoder board's MODE pin selects I2C/SSI output, not ABZ. In ABZ mode the SPI read returns garbage. (Confirmed by Stage 1: clean SSI frames, crc = 0.)
  - [x] The magnet is diametrically magnetized (poles across the face, not top/bottom). (Motor and encoder supplied assembled, 2026-10-08.)
  - [x] The magnet is centered on the rotor's rotation axis. On this hollow-shaft motor, check it isn't offset. (Supplied assembled.)
  - [x] The gap between the magnet and the MT6701 chip is 0.5–2 mm, typically 1 mm (MT6701 datasheet air gap). (Supplied assembled; field status NORMAL in Stage 1.)
  - [x] The encoder board is fixed to the stator side, so only the magnet turns with the rotor. (Supplied assembled.)
  - [x] J14 wiring: pin 1 = 3.3 V, pin 2 = MOSI (PA7, dummy data), pin 3 = DO (PA6), pin 4 = CLK (PA5), pin 5 = CSN (PA4), pin 6 = GND. (Confirmed by Stage 1 reads.)
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
  4. Sweep the duty from 5% to 93% (the duty cap; the firmware never exceeds it).
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
  1. First measure line-to-line resistance with a meter; expect about 3.2 Ω. If it reads 4.8 Ω, update R and L in `motor_params.h`. (Done 2026-10-08: 4.4 Ω, R = 2.2 Ω wye in `motor_params.h`. The expected values below use 2.2 Ω.)
  2. Apply a 0.5 V vector (phase-voltage amplitude) at 0°, 120° and 240°.
  3. Put a multimeter (DC amps) in series with the phase A lead and compare it with the measured Ia. The vector is DC, so the meter reads phase current directly; this is the independent check of the current readings. Then compare with Ohm's law. The supply current barely changes (about 20 mA more at 12 V, from power balance), so treat it only as a sanity check.
  4. Check each current channel matches its PWM phase and sign. At 0°: Ia ≈ +0.23 A, Ib and Ic ≈ −0.11 A each (2.2 Ω). At 120°: Ib is the large positive one. At 240°: Ic is. If the large current is on the wrong channel or has the wrong sign, fix the mapping before going on: this is the most common cause of a current loop that fights itself.
  5. Check that Ia + Ib + Ic ≈ 0.
  6. Watch the rotor snap to each angle.
  7. Lower the overcurrent threshold temporarily and confirm it trips, then set it to 1.5 A. Repeat the 0° vector with all three duties shifted up together so the highest phase reaches the 93% cap; the measured current must not change, which proves the sample window holds at maximum duty. Measure inductance: with the rotor held at 0° by the 0.5 V vector, step the 0° vector to 1.0 V and record Ia in the capture buffer (no torque, because the rotor is already aligned), fit the time constant τ, and compute L = τ × R (expect τ ≈ 0.59 ms and L ≈ 1.3 mH wye if the 2.6 mH spec is line-to-line; τ ≈ 0.40 ms and 0.87 mH if per delta winding). An LCR meter across two motor leads also works; it reads twice the wye value (expect about 2.6 mH or 1.7 mH). Record the measured R and L in motor\_params.h.
- **Pass:** current matches V/R (about 0.23 A at 0.5 V with the measured 2.2 Ω wye resistance) within about 15% (dead time adds error at such low voltages). The three currents sum to about zero. The rotor holds firmly at each angle. The overcurrent trip works. Ia matches the multimeter within about 5%. Each angle puts the large current on the right channel with the right sign. Current doesn't change at maximum duty. Measured R and L are recorded.

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
| `ctrl.c/h` | Control step inside the 20 kHz interrupt: software overcurrent trip, then the output (duty command, fixed vector, later FOC) (added in Stage 8; not in the original module list) | 8 |
| `pi.c/h` | PI controllers with anti-windup | 13 |
| `motor_params.h` | Pole pairs, R, L, back-EMF constant, encoder offset, current limits | 8 |
| `motor_sm.c/h` | Motor state machine, speed profile, protections | 14 |
| `app.c` | `BRINGUP_STAGE` switch, 1 kHz main-loop tasks | 0 |

**Stage prerequisites** (each blocks its stage until done)

- [x] Stage 1: encoder MODE pin set for I2C/SSI, magnet checks done (Stage 1 setup checklist).
- [x] Stage 8: motor line-to-line resistance measured (expect about 3.2 Ω) to confirm the delta conversion. (4.4 Ω, 2026-10-08: the delta conversion doesn't hold; the 4.8 Ω spec reads as line-to-line. R = 2.2 Ω wye.)
- [ ] Stage 8: current readings confirmed against a multimeter before the trip is raised above 1 A. (**Deferred by the developer, 2026-10-08**; trip stays at 1.0 A through Stage 9–10.)
- [ ] **Stage 11 (blocks it): the deferred Stage 8 current check** — DC ammeter in series with phase A, or the voltmeter method (V_AB across MOTA–MOTB at 0°, Ia = V_AB ÷ 3.3 Ω), within 5 % of the logged Ia. Needed before the trip goes to 3 A.
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

### 2026-10-04: Stage 4 passed (developer's call)

- The developer reports everything good. The SW1 cycle count, the SOA/SOB/SOC meter readings and the optional nFAULT-short check weren't sent, so the values aren't recorded.

### 2026-10-04: Stage 5 firmware written (not yet built or tested)

- **Files:** `App/pwm.c/h` (new), `App/board.h` (`motor_disarm()` moved to `pwm.c`; TIM1 safe-output part split out as `board_tim1_outputs_safe()`), `App/drv8323.c` (includes `pwm.h`), `App/fault.c/h` (FAULT\_PWM\_BREAK), `App/app.c` (PWM init, debugger commands, Stage 5 test mode; DRV held asleep in Stage 5), `App/app_config.h` (`BRINGUP_STAGE 5`), `CMakeLists.txt` (adds `pwm.c`). No CubeMX change.
- **TIM1 checked at boot against the plan:**
  - ARR = 4250, center-aligned mode 1, PSC = 0, CKD = 0.
  - DTG = 17 (17 × 5.88 ns = 100 ns, RM0440 BDTR DTG\[7:5\] = 0xx).
  - BKE = 1, BKP = 0 (active low), AOE = 0.
  - MMS2 = OC4REF, CCR4 = 4200.
  - Channels 1–4 in PWM mode 1.
- **Debugger freeze:** DBGMCU APB2FZ DBG\_TIM1\_STOP set at boot. RM0440 (TIM1 debug mode): with the counter stopped, "the outputs are disabled (as if the MOE bit was reset)", and with OSSI = 1 they go to their inactive (low) level. MOE itself stays set, so PWM resumes after the halt.
- **Idle:** MOE = 0, OSSI = OSSR = 1, OISx = 0, CCxE = CCxNE = 1 → all six outputs driven low; the counter runs. Duties start at 50 % (zero command) and CCRs are loaded with UG. BIF cleared; the break interrupt (BIE) is enabled only while armed (see the 2026-10-08 entry): `HAL_TIMEx_BreakCallback` turns BIE off, latches the event, and the main loop raises FAULT\_PWM\_BREAK. The fault response calls `motor_disarm()`, and the fault stays latched until `g_fault_clear_request`.
- **`motor_can_arm()`** is the only code that sets MOE. It refuses if:
  - already armed, or a fault is latched;
  - the debug freeze is off, OSSI/OSSR aren't set, AOE is set, idle levels aren't low, or the counter isn't running;
  - the duties aren't equal (not a zero command);
  - nFAULT/BKIN is low;
  - **Stage 5 only:** DRV ENABLE is high;
  - **Stage 6 onward:** VM\_OK is missing or the DRV isn't configured.

  It clears BIF, sets MOE, and confirms MOE actually set.
- **Duty writes:** `pwm_set_duty(a, b, c)` clamps to \[0, 0.93\] and writes CCR3/CCR2/CCR1 for A/B/C (register writes only, safe in the interrupt).
- **Stage 5 test mode:** the DRV is held asleep (ENABLE low). Debugger commands via `g_app_cmd`: 1 = arm from the zero command and ramp over 500 ms to A = 20 %, B = 50 %, C = 80 %; 2 = disarm; 3 = software break (EGR.BG). Status line every second: armed, duties in permille, ENABLE, nFAULT, break count, fault.

### 2026-10-08: encoder checklist closed; shoot-through review before the Stage 5 build

- **Stage 1 checklist ticked:** the developer reports the motor and encoder were supplied assembled (magnet type, centring, gap, mounting). The MODE pin and J14 wiring are confirmed by Stage 1's clean SSI reads. The Stage 1 prerequisite box is ticked to match the 2026-10-04 entry.
- **Shoot-through review (code, `tim.c`, RM0440, SLVSDJ3D, DS13122):** no path found that turns on both FETs of one half-bridge.
  - **Pairing:** each half-bridge is driven by one TIM1 channel and its complement: C = CH1/CH1N (PA8/PB13), B = CH2/CH2N (PA9/PB14), A = CH3/CH3N (PA10/PB15). The MCU dead-time generator therefore acts within each half-bridge. AFs match DS13122 (AF6, PB15 AF4).
  - **MCU:** PWM mode 1, CCxP = CCxNP = 0 (active high), DTG = 17 (100 ns), CCR preload on (OCxPE, set by `HAL_TIM_PWM_ConfigChannel`). Duty clamped to 0–0.93.
  - **Off states:** MOE = 0 with OSSI = 1 and OISx = OISxN = 0 drives both outputs of every phase low, for idle, break and debugger halt (RM0440 TIM1 debug mode: outputs disabled as if MOE were reset).
  - **DRV8323 (independent of the MCU):** 6x PWM mode (0x02 PWM\_MODE = 00b). Table 8-2: INHx = INLx = 1 gives GHx = GLx = L, so a firmware error that drives both inputs high turns both gates off. TDRIVE adds VGS-handshake dead time (8.3.1.4.2) plus DEAD\_TIME = 01b (100 ns, register 0x05).
  - **Stage 5:** the DRV is asleep, so the gates are held low whatever TIM1 does, and `motor_can_arm()` refuses if ENABLE is high.
- **Bug fixed in `pwm.c` (break interrupt):** BIF can't be cleared while the break input is active (RM0440 TIMx\_SR, BIF). With BIE always on, a latched DRV fault holding nFAULT low would re-enter the priority-0 break interrupt forever, starving the main loop, so nothing would be logged and the DRV wouldn't be put to sleep. The outputs are off in hardware, so this isn't a shoot-through risk, but the board would hang silently. Also, from Stage 6 the DRV's normal wake and sleep nFAULT pulses would have latched FAULT\_PWM\_BREAK on every SW1 cycle. Fix: BIE is enabled in `motor_can_arm()` (together with MOE, with interrupts briefly disabled) and turned off in `motor_disarm()` and in the break callback. Break events are now reported only while armed; while disarmed the outputs are already off and the nFAULT EXTI covers DRV faults. Stage 5 behaviour is unchanged for the software-break test, which is done while armed.
- **Hardening added (developer approved):**
  - **System break:** `pwm_init()` sets SYSCFG\_CFGR2 CLL, SPL and ECCL, connecting core lockup, SRAM parity and flash double-ECC errors to the TIM1 break (RM0440 Table 271; the bits lock until a system reset). SPF (write-1-to-clear) is masked out of the write. `pwm_init()` fails with `SYSCFG_CFGR2 system break` if the bits don't read back set. Expected boot line: `PWM sysbrk lockup+sram_parity+flash_ecc cfgr2=0x00B`. The arm path and `pwm_init()` now clear SBIF as well as BIF.
  - **NMI handler:** `NMI_Handler` (USER CODE block in `stm32g4xx_it.c`) calls `debug_nmi_report()`. It clears MOE, drives ENABLE low, prints `FAULT NMI flash_eccr=… syscfg_cfgr2=…` over SWO, then halts like the HardFault handler. NMI sources are SRAM parity, flash ECC and HSE CSS (RM0440 vector table); CSS isn't enabled. Like the HardFault handler, this is a deliberate exception to "only `motor_disarm()` clears MOE". The halt/blink code is now shared between the two handlers.
- **Bug fixed in `pwm_init()`:** pass/fail was decided by `*what` starting with 'O', and the mismatch message "OCxM (want PWM mode 1)" also starts with 'O'. A wrong channel mode would have reported OK. It now compares the pointer to the "OK" string.
- **Left as is (developer's decision):** TIM1 break filter BKF = 0, so any glitch on nFAULT trips the break. This fails safe; revisit only if nuisance trips appear from Stage 6.

### 2026-10-08: Stage 5 passed (developer's report)

- **Build note:** the first flash ran the old firmware (no `PWM sysbrk` line). `.vscode/launch.json` has no `preLaunchTask`, so F5 flashes the existing `build/Debug/Motor_Controller.elf` without building. **Build first, then F5.** With the rebuilt firmware the boot log was all OK.
- **Sending debugger commands:** pause, then type `set var g_app_cmd = 1` (or `set var g_fault_clear_request = 1`) in the Debug Console, then continue. The console passes input straight to GDB: a bare `g_app_cmd = 1` and the `-exec` prefix are both rejected. A pasted line with a trailing newline fails with "Problem parsing arguments", so type the command by hand or delete the newline. `print g_app_cmd` reads back 0 once the firmware has taken the command.
- **SWO dropped once during the session.** The circumstances weren't reported. Recovery without power cycling: stop the debug session fully and start it again, then reopen the SWO terminal.
- **Phase A:** INHA high 20 % of the period, INLA its complement.
- **Dead time:** 99 ns measured from the end of the INH falling edge to the end of the INL rising edge, and about 93 ns between the edge start points. Expected 100.0 ns (DTG = 17 × 5.88 ns). The difference is edge slope (GPIO speed low) and scope resolution. Pass.
- **Confirmed by the developer:** 20 kHz; INHB 50 % and INHC 80 %, so the phase mapping (A = CH3, B = CH2, C = CH1) is correct; all outputs went low on debugger pause, software break and disarm; the latch, clear and re-arm sequence worked.
- **Stage 5 passed.**

### 2026-10-08: Stage 6 firmware written (not yet built or tested)

- **Files:** `App/app_config.h` (`BRINGUP_STAGE 6`), `App/app.c` (PWM test mode shared by Stages 5 and 6, sweep, IDRIVE command), `App/drv8323.c/h` (run-time IDRIVE, shadow copy computed from it), `App/pwm.c/h` (`pwm_zero_command()`, NaN-safe duty clamp). No CubeMX change.
- **Arming (plan Stage 6):** `motor_can_arm()` already enforces VM\_OK, a configured DRV, nFAULT high, no latched fault and a zero command (Stage 6 checks under `#if BRINGUP_STAGE >= 6`). The DRV wakes and configures on VM\_OK as in Stage 4, so ENABLE is high whenever arming is possible.
- **Test mode:** target duties `g_duty_a/b/c` (0–0.93), set from the debugger. The applied duties slew toward the targets at 0.1 % per ms (full scale in 1 s), from the 50 % zero command at every arm. Defaults are A = 50 %, B = C = 0 % (procedure step 1, one phase switching; 0 % holds the low side on). A target that is negative or NaN becomes 0; above the cap it becomes 0.93. The Stage 5 test mode now uses the same code with its 20/50/80 % defaults.
- **Commands (`g_app_cmd`):** 1 arm, 2 disarm, 3 software break, 4 sweep, 5 apply IDRIVE.
  - **Sweep (step 4):** all phases 5, 10, 25, 50, 75, 90, 93 %, 4 s per step. Each step logs the expected switch-node average (duty × measured VM). It refuses unless armed, stops on any disarm, break or fault, and ends at 50 %. **Plan change:** the top of the sweep is 93 % (`PWM_DUTY_MAX`), not 95 %; the procedure text was corrected.
  - **IDRIVE (step 2):** `g_drv_idrive` = IDRIVEP << 4 | IDRIVEN (datasheet Tables 8-16/8-17), applied to both sides. Command 5 puts the DRV to sleep (disarming first), sets the code (accepted only while asleep), and the next tick wakes the DRV and configures, verifies, locks and lock-tests it with the new value. The readback shadow copy follows. Re-arm by command to test. Code 0x22 is the bring-up value (60 mA source / 120 mA sink).
  - The datasheet (8.3.1.4.2) says the gate must reach its VGS threshold within tDRIVE (4 µs here), otherwise the DRV reports a gate drive fault. Very low IDRIVE codes may therefore trip GDF. That's fail-safe (nFAULT, hardware break, latched fault).
- **nFAULT short test (step 5):** with the DRV awake, the falling edge triggers both the hardware break (outputs off in hardware) and the nFAULT EXTI. The first latched fault is DRV\_NFAULT with fault registers 0x000/0x000, because it was an external short and not a DRV fault. The status line's `brk` count rises by 1, which shows the break path fired.
- **Status line every second:** `PWM armed= A= B= C= vm= drv=CFG|AWAKE|ASLEEP nf= idrive= brk= fault=`.
- **CSA reference and gain re-checked (developer's question):** the DRV VREF and the STM32 VDDA/VREF+ are both the filtered 3.3 V rail (gate-driver sheet: VREF from VDDA with C33 100 nF). VREF 3.3 V is inside the 3.0–5.5 V range (datasheet 7.3). 0x06 = 0x2C3 decodes (Table 8-19) to: SPx input, VREF/2 bidirectional reference, LS_REF 0, gain 40 V/V, sense OCP enabled at 1 V, calibration off. The amplifier is supplied from VREF, so SOx can't exceed the ADC's VDDA. The shared rail keeps the zero-current level at mid-scale (about 2048 counts) even if 3.3 V drifts. The usable current range is ±7.0 A (VLINEAR), not the ideal ±8.25 A; Current sensing updated.
- **Schematic:** the gate-driver sheet shows TP6–TP11 on the INH/INL lines and LED6 (R28 360 Ω to 3.3 V) on the nFAULT net with R22. The power-stage sheet with J4–J9 isn't in `docs/`, so which gate each test point carries isn't checked here.

### 2026-10-08: Stage 6 first arm: VDS overcurrent on phase A high side

- **Log (developer):** DRV configured (`drv=CFG`), VM 11.83 V. `PWM armed from zero command; slewing to A=500 B=0 C=0`, then about 1 ms later `FAULT latched DRV_NFAULT ctx1=0x620`, `FAULT drv f1=0x620 f2=0x000 FAULT VDS_OCP VDS_HA nfault=LOW`, DRV asleep, `brk=1`, and later `PWM arm refused: fault latched`.
- **Decode (Table 8-12):** 0x620 = FAULT (bit 10), VDS\_OCP (bit 9), VDS\_HA (bit 5). The phase A high-side FET's VDS exceeded 0.06 V for longer than the 4 µs deglitch while it was commanded on.
- **Protection chain worked as designed:** DRV latched shutdown, hardware break (brk = 1), fault registers read before sleep, fault latched, arm refused.
- **Analysis:** every arm starts at the 50 % zero command on all phases, so the A, B and C high sides switched on together with no load, and only A tripped. That rules out firmware, timer and common gate-drive settings, and points to something phase A specific. Candidates: (1) a scope probe ground on MOTA or a phase A gate or test point, shorting the switch node to earth when the high side turns on; (2) a short on MOTA (solder bridge, damaged low-side FET); (3) the phase A high-side FET not turning on (GHA joint, FET); (4) a false VDS reading from an open SHA or VDRAIN sense connection.
- **Disarm command ruled out as the cause:** the developer had also sent `g_app_cmd = 2`. Disarm only clears MOE and drives ENABLE low. The 10 s before the arm show `drv=CFG fault=NONE` with readback passing, and no disarm line; the fault came right after the arm with the fault registers readable, so the DRV was awake.
- **Bug found and fixed (`drv8323.c`, `app.c`):** `motor_disarm()` drops ENABLE, which puts the DRV to sleep (registers reset, SPI disabled; datasheet 8.4.1.1). But `drv8323` still reported awake and configured, so it was never re-woken, the next readback raised a misleading DRV\_CONFIG\_MISMATCH, and for up to 100 ms an arm could pass the "configured" check with the DRV asleep (harmless, since a sleeping DRV holds the gates low, but wrong). Now `drv8323_is_awake()` and `drv8323_is_configured()` require ENABLE high, register access is refused otherwise, and `drv8323_sync()` (first thing in `drv_tick()`, every tick) finishes the sleep bookkeeping (SPI3 off, PD2 released, tSLEEP timer). The normal path then re-wakes and reconfigures the DRV. Expected log after `g_app_cmd = 2`: `PWM disarmed by command`, `DRV sleep (ENABLE low after disarm)`, `DRV wake #N ... cfg OK`.
- **Repeat after a fresh build (developer, same day):** the meter checks on MOTA/B/C were reported good, the DRV re-woke and configured cleanly (`DRV wake #2 ... cfg OK`, registers 0x000/0x622/0x722/0x110/0x2C3, fault registers 0x000/0x000). On the next arm, the same fault came about 1 ms after `PWM armed`: `f1=0x620 f2=0x000 FAULT VDS_OCP VDS_HA`. **Repeatable and phase A specific.** VDRAIN is shared by all three high-side VDS monitors, so it's ruled out. Remaining candidates: phase A high-side FET not turning on (GHA joint, gate resistor, FET gate); SHA sense pin open from MOTA (false VDS); phase A low side not turning off, or GHA/GLA swapped (real shoot-through). Next: the single-shot GHA/MOTA capture result, and unpowered continuity from DRV pins GHx, GLx, SHx to the FET gates and MOTx, A against B and C. No re-arm until one of these explains it.
- **Phase isolation added (developer's request), to confirm the fault is phase A only:** `g_phase_off` (bit 0 = A, 1 = B, 2 = C) is applied at each arm by `pwm_phase_output()`, while disarmed. A held-off phase has its INH and INL pins taken from TIM1 and driven low as GPIO outputs. In 6x mode INH = INL = 0 turns both gates off (Table 8-2), so the phase floats. They're driven, not just disabled in TIM1: with MOE = 1 a disabled channel is Hi-Z (RM0440 Table 282), which would leave INHA held only by the DRV's 100 kΩ pull-down beside switching traces. Clearing the bit and re-arming hands the pins back to TIM1. Test: `g_phase_off = 1`, `g_duty_b = 0.5`, arm. B and C should switch with no fault.
- **Phase isolation result (developer's log):** with `g_phase_off = 1` (A held off), B at 50 % and C at 50 % slewing to 0 %, the board armed and ran 30+ s with `fault=NONE brk=0`. The square wave was seen on MOTB. **The VDS_HA fault is confirmed phase A specific** (same DRV, configuration and code on B and C). (A first arm command sent at boot was correctly refused with `no VM_OK`, before the DRV had woken.)
- **MOTB 13.3 V pk-pk with VM = 11.8 V:** switching overshoot and undershoot from ringing (stray inductance, partly from the probe ground lead, against FET capacitance), not the switch-node level. To confirm: scope Top/Base (expect about 11.8 V / 0 V), and zoom on one edge with a short ground spring. Well inside the 40 V FET and 60 V DRV ratings. Relevant to IDRIVE tuning (step 2).
- **Phase A only (B and C held off, `g_phase_off = 6`):** the same fault again, `f1=0x620 VDS_OCP VDS_HA`. The developer confirmed the phase A high-side FET drain connects to VM and its source to MOTA, and found the FET pins not shorted. One capture at 100 µs/div (CH2 green, trigger; CH3 blue): one trace rose 0 → about 12.7 V and stayed, the other rose to about 1 V; no switching edges. The channel mapping isn't confirmed. Neither trace fell after the fault, which doesn't match the DRV switching its gates off, so it isn't interpreted yet. Remaining candidates: GHA not reaching the FET gate (DRV output, pin 6 joint, gate resistor), or the FET not conducting when driven (defective, wrong part, rotated). Next: channel mapping; pin 6 → gate continuity against phase B; FET marking and orientation; capture triggered on nFAULT falling at 2 µs/div with GHA, MOTA and nFAULT.
- **Root cause found (capture at 10 µs/div, phase A only; CH2 = phase A high-side gate to GND at 5 V/div, CH3 = MOTA at 10 V/div; fault log unchanged, `VDS_HA`):**
  - Idle: both phase A FETs off, MOTA floats to about 12 V (leakage, no load), gate at the same level.
  - First high-side pulse: gate about 22.5 V (10.5 V above MOTA); no fault, because MOTA was already at VM.
  - Low side on: MOTA pulled cleanly to 0 V for about 23 µs. **Phase A low side works.**
  - High side on again: gate about 22 V, held for about 4.3 µs (the 4 µs OCP\_DEG), more than 10 V above MOTA throughout. MOTA only steps about 4 V (gate-edge coupling through the FET capacitance), then crawls. At the trip the gate drops to MOTA's level, and **MOTA's slow rise continues unchanged**, so the channel was contributing no current.
  - **Conclusion:** the phase A high-side FET doesn't conduct with a full gate drive. The DRV, the gate path and the firmware are fine; earlier meter checks (no short, drain on VM, source on MOTA, body diode normal, gate connected) all agree. Likely a FET damaged inside, or a wrong or defective part.
  - **Probe note:** earlier captures showed MOTA at one tenth of its real swing because the CH3 probe was on ×10 with the channel set to 1:1.
- **Gate stress:** on each failed attempt the gate reached about 22 V with the source near 0 V, about 18–22 V VGS for about 4 µs (normally the source follows the gate). That may exceed the FET's VGS rating (check the IAUC80N04S6N036 datasheet, not in `docs/`). Phase A stays held off until the FET is replaced.
- **Drain confirmed (developer):** with VM on and the PWM disarmed, the phase A high-side FET's drain tab reads 12 V, so the drain connection is good. With the source on MOTA and the gate driven, **the FET itself is dead** (channel won't conduct).
- **Decision (developer): board swapped for a new PCB** instead of replacing the FET. The old board is retired with a dead phase A high-side FET. The Stage 0–5 results above were on the old board. The new board gets a short re-check before switching:
  1. Unpowered: visual inspection of the DRV and all six FETs; VM/3.3 V/5 V not shorted; MOTx to GND and to VM, and both body diodes on each phase.
  2. USB only: boot log all OK; TP1 = 3.3 V, TP3 = 5 V.
  3. 12 V: VM against the meter, DRV `cfg OK` with the expected registers, SOx ≈ 1.65 V, baseline supply current.
  4. Power stage one phase at a time with `g_phase_off` (A only = 6, B only = 5, C only = 3), then all three, then the rest of Stage 6.
  The cause of the old FET's failure is unknown (defective part, or a stress during the first arm such as a probe ground on a switch node). If the same failure appears on the new board, suspect a design or process cause.
- **New board, phase A only (`g_phase_off = 6`, A = 50 %):** armed without a fault; the developer reports it working (MOTA switching). Measurements still to record: MOTA levels, edge times, dead time, duty check at 25/75 %, supply current. Next: B only, C only, all three, then the rest of Stage 6.
- **New board, B only and all three phases:** the developer reports all three switch nodes switching with `g_phase_off = 0`. (A surprise where only A switched turned out to be `g_phase_off` still at 6 from the A-only test; it holds its value until changed or reset, and takes effect only at the next arm.) Still to record: switch-node duty against the meter, edge times and ringing per IDRIVE setting, dead-time gaps, sweep, nFAULT break test, supply current.
- **New board, dead time (developer):** about 60 ns at the MOTx rising edge, from the low-side gate fully low to the high-side gate starting to rise. That's a clear non-overlap; the window with both gates below threshold is wider, and the DRV's VGS handshake plus its 100 ns digital dead time back it up. The falling edge (high-side gate down to MOTx, then the low-side gate rising) isn't measured yet.
- **Decision (developer): IDRIVE stays at 0x22** (60 mA source / 120 mA sink); `DRV_CFG_IDRIVE` unchanged.
- **Stage 6 not formally passed.** Still open: falling-edge dead time, switch-node average against duty × VM (sweep), supply current with PWM on against off, and the **nFAULT short test (procedure step 5)**. The nFAULT test **must pass before the motor is connected** (bench rule 1, Stage 8). These tools stay in the Stage 7 build (`g_app_cmd` 4/5, `g_phase_off`), so they can be run there.
- **Earlier action (old board, superseded by the swap):** replace the phase A high-side FET (same batch as B and C, marking checked, drain tab fully soldered). Retest phase A alone (`g_phase_off = 6`), then all phases, then resume Stage 6.
- **Earlier next steps (superseded):** confirm C at 50 %; continuity from the DRV GHA pin to the phase A high-side gate and from SHA to MOTA, and the phase A high-side gate–MOTA resistance, each against B and C, to tell a joint or the DRV from the FET. Stage 6 steps 2, 4 and 5 can continue on B/C with A held off. Stage 6 can't pass until phase A is fixed and tested.
- **Next (no re-arm until done; old board):** probe grounds moved to GND only; SW1 off and VM below 1 V; meter checks MOTx–GND, MOTx–VM, and both body diodes on A against B and C; visual inspection of the phase A FETs and DRV pins GHA, SHA, GLA, VDRAIN. Fault left latched. If the checks are clean: one single-shot capture of GHA and MOTA at arm.

### 2026-10-08: Stage 7 firmware written (not yet built or tested)

- **Files:** `App/cursense.c/h` (new), `App/pwm.c/h` (`pwm_command_duty()` / `pwm_isr_update()`, Stage 7 arming rules), `App/debug.c/h` (capture buffer), `App/fault.c/h` (FAULT\_CTRL\_HEARTBEAT, FAULT\_CUR\_OFFSET), `App/app.c` (test mode for Stages 5–7, control monitor, Stage 7 status line), `App/app_config.h` (`BRINGUP_STAGE 7`), `CMakeLists.txt` (adds `cursense.c`).
- **CubeMX change required:** NVIC → Code generation → "ADC1 and ADC2 global interrupt" → untick "Generate IRQ handler", then regenerate. `cursense.c` provides `ADC1_2_IRQHandler` (register access only, no HAL in the control interrupt). Until then the link fails with a duplicate `ADC1_2_IRQHandler`. The existing ADC configuration (injected channel 12 on ADC1/2/3, trigger TIM1 TRGO2 rising, 2.5-cycle sampling, clock HCLK/4 synchronous, ADC1\_2\_IRQn priority 0) is used as is.
- **CubeMX change done (checked against the last commit):** `.ioc` ADC1\_2\_IRQn "Generate IRQ handler" true → false (still enabled, priority 0). The generated `ADC1_2_IRQHandler` and its prototype were removed from `stm32g4xx_it.c/h`; the USER CODE (debug.h include, NMI call) was kept. `adc.c` still enables ADC1\_2\_IRQn at priority 0; no other generated file changed.
- **Sampling instant:** OC4REF (PWM mode 1, CCR4 = 4200) rises when the down-counting counter passes 4200, 50 counts = 294 ns after the counter peak. At the 93 % cap the low sides are on for ±298 counts (±1.75 µs) around the peak, so the sample lands about 1.85 µs after a low side turns on (less dead time), against the 1.2 µs CSA settling (datasheet 7.5, tSET at 40 V/V), and about 1.4 µs before it turns off. To be confirmed on the scope (procedure steps 1 and 3).
- **Control interrupt (ADC1 JEOS, 20 kHz):**
  - PA3 is high for its whole duration. The rising edge is about 0.4 µs after the sample (conversion 15 ADC clocks = 353 ns, plus entry latency). PA3 is no longer used by the encoder read from Stage 7.
  - Waits up to about 1 µs for ADC2/ADC3 (same clock and trigger, so normally already done; counted as `late` otherwise), reads JDR1 (A = ADC2, B = ADC1, C = ADC3), and clears JEOC/JEOS only, leaving ADC1's regular EOC for the VM poll alone.
  - Converts to amps, positive into the motor: (count − offset) × 3.3/4096/0.2.
  - Keeps min/max/sum statistics and the offset accumulation, increments the heartbeat, writes SOA's raw count to the PA2 scope DAC, pushes into the capture buffer, and applies the PWM duty command while armed.
- **Duty path:** from Stage 7 the main loop only sets the command (`pwm_command_duty()`); the control interrupt writes the CCRs (plan: the main loop doesn't write CCR registers). The test arm zeroes the command before arming.
- **Offsets:** measured after every DRV configure (CSA just calibrated, disarmed): 200 samples (10 ms) skipped to settle, then 1,024 averaged per phase. Valid within 2048 ± 250 counts (DRV VOFF ±4 mV × 40 = ±199 counts, datasheet 7.5, plus margin). Out of range: FAULT\_CUR\_OFFSET, offsets stay invalid.
- **Heartbeat:** the control interrupt count must change within 3 ms of real time (HAL\_GetTick, so the main loop's catch-up ticks can't trip it; SysTick and TIM1 both stop under the debugger). Stale count: FAULT\_CTRL\_HEARTBEAT, and the fault response disarms.
- **Arming (Stage 7 rules added):** current offsets valid and heartbeat alive, on top of the Stage 6 rules.
- **Capture buffer:** 1,024 samples × 3 raw counts (51.2 ms at 20 kHz) in a ring. A trigger keeps 512 samples before and records 512 after. Triggered by `g_app_cmd = 6` and automatically when a fault latches; `g_app_cmd = 7` dumps it over SWO, one line per tick, then re-arms.
- **Status lines every second:** `CUR mA= pp= rms= off= valid= n=` (mean current per phase, peak-to-peak and RMS noise in counts, offsets, samples) and `CTRL isr_max= load= late=` (longest control interrupt, load against the 8,500-cycle period). Format as revised after the first run (below).
- **Test mode:** Stage 7 defaults are 50 % on all phases (procedure). `g_duty_a` up to 0.93 for step 3. The Stage 6 tools (sweep, IDRIVE, phase hold-off) remain.

### 2026-10-08: Stage 7 first run

- **Offsets:** 2023 / 2029 / 2042 counts (within 25 counts of 2048). Pass.
- **Sampling point (scope, developer):** PA3 lands in the middle of the low-side gate's high time. At 93 % duty, PA3 rises 2.2 µs after the low-side gate goes high, so the sample is about 1.8 µs after it (2.2 − 0.4 µs conversion and entry), against the 1.2 µs needed. Pass.
- **Control interrupt: fail.** `isr=5.3us` disarmed (log), about 6.6 µs armed (PA3 width; the extra is the duty write). That's 13 % of the 50 µs period, against the plan's 10 %. Cause: the Debug build is `-O0` (`cmake/gcc-arm-none-eabi.cmake`), the same reason the Stage 1 encoder read took 16 µs. **Fix:** the top-level `CMakeLists.txt` (user-owned) now compiles `App/cursense.c`, `App/pwm.c` and `App/debug.c` at `-O2` via `set_source_files_properties`; the rest stays `-O0`. To re-measure.
- **Noise (disarmed):** 18–23 counts peak-to-peak per phase over 20,000 samples (1 s). That statistic catches the rarest spike in a second (about ±3.5σ for Gaussian noise), so it overstates the typical noise. An RMS (standard deviation) per phase was added to the status line (`rms=`); the plan's "a few counts" is judged on that, with pp kept for spikes. The interrupt time and load moved to their own `CTRL isr_max= load= late=` line, with one decimal.
- **Possible noise source (not changed):** ADC1 alternates between VM\_SENSE (regular) and SOB (injected) with a 2.5-cycle sample time. Charge left on the sampling capacitor shares into the 2.2 nF filter capacitor (about 5 pF / 2.2 nF of the voltage difference, a few counts). If SOB is noisier than SOA/SOC, raise the injected sample time in CubeMX (6.5 or 12.5 cycles). In the first run SOB was not noisier (pp 19 against 23/19), so not indicated so far.

### 2026-10-08: Stage 7 passed (developer's report)

- After the `-O2` change the developer reports everything in Stage 7 working. The re-measured `isr_max`/load and the RMS noise values weren't sent, so they aren't recorded. Earlier results stand: offsets 2023/2029/2042, sample about 1.8 µs after the low side turns on at the 93 % cap.

### 2026-10-08: Stage 8 firmware written (not yet built or tested)

- **Prerequisite:** motor line-to-line resistance **4.4 Ω** (developer, meter). The plan expected 3.2 Ω (4.8 Ω per delta winding). 4.4 Ω is 8 % under 4.8 Ω, so the spec most likely gives line-to-line values. `MOTOR_R_PHASE_OHM = 2.2` (wye) in `motor_params.h`. L stays TODO\_MEASURED at 1.3 mH (2.6 mH read as line-to-line). Motor table, current-loop gain note, stall current, the Gain 40 V/V check and the Stage 8 expected values were updated for 2.2 Ω. Worth confirming: meter leads shorted and their resistance subtracted, and all three pairs (AB, BC, CA) within a few percent of each other.
- **CubeMX change required:** Computing → **CORDIC → Activated**, then regenerate. That enables its clock; `foc_init()` checks the clock and configures it at register level. Without it the boot log shows `FOC ... FAIL` and arming is refused.
- **Files:** `App/motor_params.h` (new), `App/foc_math.c/h` (new: CORDIC sin/cos, SVPWM), `App/ctrl.c/h` (new; module table updated), `App/cursense.c` (the control interrupt calls `ctrl_isr()`), `App/debug.c/h` (`debug_capture_get()`), `App/fault.c/h` (FAULT\_OVERCURRENT), `App/app.c` (Stage 8 vector test mode, overcurrent logging, inductance fit), `App/app_config.h` (`BRINGUP_STAGE 8`), `CMakeLists.txt` (adds `ctrl.c`, `foc_math.c`, both at `-O2`).
- **CORDIC (RM0440 17.3.6, Tables 106 and 118):** cosine function, RES1 = cos and RES2 = sin, modulus ARG2 = +1 from reset (NARGS = 0), 24 iterations in 6 cycles (max error 2⁻¹⁹), q1.31. Zero-overhead mode: write the angle (θ/π wrapped to [−1, 1)), read two results. Used only from the control interrupt.
- **SVPWM (plan: Space-vector PWM):** |V| limited to 0.497 × 12 V, inverse Clarke, midpoint injection, duty = 0.5 + v/12, clamped to [0, 0.93]. Vbus is fixed at 12.0 V.
- **Control interrupt, Stage 8 (`ctrl_isr`):**
  1. **Overcurrent trip:** any |I| over the trip level → `motor_disarm()` in the interrupt (outputs off, then ENABLE low), capture triggered, event flagged. The main loop raises FAULT\_OVERCURRENT and logs the phase and current. Trip level `g_oc_trip`, clamped to 0.1–1.5 A, default 1.0 A (plan: 1 A until the meter check, then 1.5 A); NaN gives the default.
  2. **Fixed vector:** V at θ, giving Vα = V cos θ and Vβ = V sin θ, then SVPWM, plus the common-mode shift `g_duty_shift` (0–0.43) on all three duties.
- **Stage 8 test mode:** `g_vec_v` (phase-voltage amplitude, default 0.5 V, capped at 1.2 V: 0.55 A at 2.2 Ω, under the 1 A trip), `g_vec_deg`, `g_duty_shift`, `g_oc_trip`, `g_vstep_v` (1.0 V). Every arm starts at 0 V and ramps at 1 V/s. Commands: 1 arm, 2 disarm, 3 software break, 6 capture, 7 dump, **8 inductance step**.
- **Inductance step (procedure step 7):** with the vector settled (0.5 V at 0°), command 8 sets the step value; the control interrupt applies it and triggers the capture in the same interrupt. The voltage reaches the motor at the next PWM update, half a sample before sample 512. When the capture is full, the main loop averages phase A over 100 samples before the step and the last 100 (20–25 ms after), finds the 63.2 % crossing by interpolation (τ), and logs `LFIT ... tau= L= (R from motor_params; V/I=)`. The fit assumes the vector is at 0°. The raw capture can still be dumped (command 7) to check the fit.
- **Status lines every second:** `CUR mA=A/B/C sum= pp= rms= off= valid= n=`, `CTRL isr_max= load= late=`, and `VEC armed= V= deg= shift= dmax= expect mA=A/B/C trip= fault=` (expected currents from Ohm's law on the commanded phase voltages and R = 2.2 Ω).
- **Before connecting the motor (bench rule 1):** the Stage 6 nFAULT short test must pass. It can be done in this build with the motor disconnected: arm with `g_vec_v = 0` (50 % on all phases), then short nFAULT. The falling-edge dead time can be measured in the same setup.
- **Wiring order:** any order of the three motor leads on J13 works for Stage 8. Swapping two leads reverses the electrical rotation relative to the encoder, which Stage 10 measures (DIR and offset). Don't change the order after Stage 10; label the leads.

### 2026-10-08: Stage 8 first run (motor connected, developer's log)

- **At rest:** currents within ±7 mA of zero, RMS 4.1/2.7/2.0 counts. The first status line after boot (−200 mA, pp about 2,050) is an artefact: its window includes samples taken while the DRV was asleep with its amplifiers off.
- **0°, 0.5 V:** Ia/Ib/Ic = **+220 / −105 / −97 mA** against +227 / −113 / −113 expected from 2.2 Ω (−3 %, −7 %, −14 %), sum +17 mA. Right channel and sign; within the 15 % criterion. Dead time accounts for the shortfall at such a low voltage. Meter reading, 120°/240°, trip test, max duty and the L step still to do.
- **Bug fixed (`pwm.c`): "VEC arm refused: MOE did not set" while the outputs were actually on** (the next status line read `armed=1` and the vector ramped normally). MOE is written on the asynchronous path, and RM0440 says a read straight after setting it can still show 0 ("a delay must be inserted (dummy instruction) before reading it correctly"). At `-O0` the code was slow enough; at `-O2` the read came too soon. Now `motor_can_arm()` polls MOE up to 16 times, and if it really didn't set, forces the outputs off before returning "refused", so "refused but armed" can't happen.
- **Interrupt time:** `isr_max=7.9us` (15.8 %) armed, 3.5 µs (7.1 %) disarmed, but that figure was the maximum since boot, which one rare event (for example the first, uncached run of the new vector code) can set. The `CTRL` line now reports mean and maximum per second plus the since-boot worst; the plan's 10 % is judged on the per-second maximum. To re-measure (log and PA3 width).
- **First motor movement, 0°, 0.5 V (after the MOE fix; armed at the first attempt):** Ia/Ib/Ic = 218 / −110 / −100 mA (expect 227 / −113 / −113), sum +5 to +13 mA, RMS noise unchanged (4.1/2.7/1.8 counts). Pass on channel, sign and magnitude.
- **Interrupt too slow armed: mean 7.7 µs, max 7.8 µs per second (15.7 %)**, against 2.3 µs disarmed. That's steady, not a one-off. The armed path only adds CORDIC sin/cos, SVPWM and three CCR writes (well under 1 µs of instructions at `-O2`). Cause: code fetched from flash at 4 wait states with prefetch off (`PREFETCH_ENABLE 0U` in `stm32g4xx_hal_conf.h`) and a small instruction cache, so every new block of interrupt code costs flash waits. **Fix:** the control-interrupt path runs from SRAM. `APP_RAMFUNC` (`app_config.h`) puts functions in the linker script's existing `.RamFunc` section (copied to SRAM by the startup code; no linker or CubeMX change): `ADC1_2_IRQHandler`, `ctrl_isr`, `foc_sincos`, `foc_svpwm`, `pwm_set_duty`, `pwm_is_armed`, `pwm_isr_update`, `debug_capture_push`. Rare paths (`motor_disarm`, capture trigger) stay in flash. To re-measure; the plan's limit is 10 % (5 µs).
- **Rotor motion not noticed by eye at the first arm** (developer). Expected: at arm the rotor moves only if it wasn't already near the 0° position, and 120° electrical is only 120/11 = 10.9° of shaft (496 counts). **Added (`app.c`):** the encoder position (`enc=`, multi-turn counts) on the `VEC` line, and a `MOVE` line about 0.8 s after an arm and 0.3 s after each `g_vec_deg` change, giving the shaft movement in counts and degrees. That's the objective version of procedure step 6, and the sign also gives a first look at the direction Stage 10 needs.
- **Steps 2–6 (developer's log, 0.5 V):** 0° → A/B/C = 219/−100/−97 mA; 120° → −104/235/−90; 240° → −105/−104/230 (expected ±227/−113). **The large positive current is on the right channel with the right sign at every angle**, magnitudes within about 10 % of V/R. Sum +20 to +30 mA while switching (about 0 at rest): a small switching-related offset, under 10 % of the phase current; to watch.
- **Rotor motion (encoder):** the rotor moves at every step and returns to the same place for the same angle (240°: 14050 then 14038 counts). Step sizes −551, −410 and +949 counts instead of a steady ±496, the last one the long way round. At 0.22 A the holding torque is close to the cogging and friction (and the shaft may have been touched), so the rotor settles off target. Not a blocker: Stage 10 measures direction with slow rotation. Optional clean repeat at 1.0 V without touching.
- **Interrupt time after the SRAM change:** mean = max = 6.8 µs (13.6 %), down from 7.7 µs. **Correction:** the 10 % criterion is Stage 7's, for the current-sensing interrupt alone (passed at 2.3 µs disarmed, 4.6 %). The full control interrupt's budget is "target under 50 %" (Key hardware facts, TIM1 and control timing), so 13.6 % is within budget.
- **Meter check not done** (developer: not easy right now). The ADC readings agree with Ohm's law using the meter-measured R, but that isn't the independent check the plan requires. Until it's done: **trip stays at 1.0 A** (prerequisite: "current readings confirmed against a multimeter before the trip is raised above 1 A"), and Stage 8 isn't passed. The trip test, max-duty test and L step go ahead.
- **Overcurrent trip test (step 7): passed** (developer). The trip lowered below the 0° phase A current fired, disarmed and latched FAULT\_OVERCURRENT; cleared and re-armed afterwards. The exact log line wasn't sent. The fault also froze a capture automatically, so the next L step was refused until it was dumped (`LSTEP refused: a finished capture is waiting`): the capture holds one recording at a time, by design.
- **L-step usability fix (`app.c`, `debug.c`):** the step kept being refused. Likely cause: commands 7 and 8 sent in one pause (one command variable, so 8 overwrote 7), or 8 sent while the dump was still printing. Now command 8 re-arms the capture itself (`debug_capture_rearm()`, discarding any finished recording; refused only while a dump is printing), waits 10 ms (200 fresh samples, so the 100-sample pre-step average has no old data), then applies the step. Each refusal reason now has its own message.
- **Inductance (step 7), first run:** `LFIT I 208 -> 442 mA, tau=239 us, L=525 uH (R=2200 mOhm; V/I=2262 mOhm)`. V/I agrees with the meter's 2.2 Ω within 3 %. L is below both spec readings (1.3 / 0.87 mH). τ spans about 5 samples, so a single fit carries about ±10–15 %. **Recorded** in `motor_params.h` (`MOTOR_L_PHASE_H = 0.525e-3`) and the Motor table; current-loop starting gains become Kp ≈ 3.3 V/A, Ki ≈ 13,800 V/(A·s). To confirm with two more steps (should agree within about 10 %) or an LCR meter (about 1.05 mH line-to-line).
- **Inductance repeats:** `tau=277 us, L=611 uH (V/I=2243 mOhm)` and `tau=219 us, L=483 uH (V/I=2266 mOhm)`. Three runs 525 / 611 / 483 µH, **mean 540 µH**, spread ±12 % (τ is about 5 samples). Recorded: `MOTOR_L_PHASE_H = 0.54e-3`, Motor table and gain note updated (Kp ≈ 3.4 V/A).
- **Max-duty check (step 7):** done (developer); the values weren't sent.
- **Meter check: the developer can't do it** (no DC ammeter in series available). Consequences per the plan: the trip stays at **1.0 A** (prerequisite before going above 1 A), and the Stage 8 pass item "Ia matches the multimeter within 5 %" is open. Supporting evidence so far: ADC currents agree with V/R using the meter-measured R (−3 % on Ia), and V/I from the L steps (2.24–2.27 Ω) agrees with the meter's 2.2 Ω. Both use the commanded voltage, so they don't independently check the ADC chain. Offered alternative: DC voltage across MOTA–MOTB with a voltmeter at 0° (Ia = V_AB / 1.5 R_phase), which needs no ammeter.
- **Stage 8 status:** done: R, L, channel/sign at 0/120/240°, sum ≈ 0 (+20–30 mA), rotor holds, trip test, max duty. Open: independent current check (meter or voltage method). Trip stays at 1.0 A until it's done.
- **Decision (developer): Stage 8 passed except the independent current check, which is deferred to before Stage 11** (added to the Stage prerequisites). Moving to Stage 9; Stage 9's open-loop spin (about 1 V, 0.45–0.6 A) fits inside the 1.0 A trip.
- **nFAULT break test (Stage 6 step 5): passed** (developer). Armed at 0 V vector, nFAULT shorted to GND: the hardware break shut the outputs off and the fault latched. Bench rule 1 satisfied. Still open from Stage 6: falling-edge dead time, sweep, supply current comparison.

### 2026-10-08: Stage 9 firmware written (not yet built or tested)

- **Files:** `App/ctrl.c/h` (forced-angle generator), `App/foc_math.h` (inverse Park; `foc_sqrtf` moved to the header), `App/pwm.c` (Stage 9 arming rule), `App/fault.c/h` (FAULT\_ENCODER), `App/app.c` (vector test mode now serves Stages 8–9, spin control and log, encoder monitor), `App/app_config.h` (`BRINGUP_STAGE 9`). No CubeMX change.
- **Forced angle (in the control interrupt):** the electrical speed slews toward the target at the set acceleration (Hz/s); the angle integrates it every 50 µs, and whole electrical cycles are counted. Vector: inverse Park with Vd = V, Vq = 0 at the forced angle, then SVPWM.
- **Test mode:** arm → V ramps to `g_vec_v` (default 1.0 V, procedure step 1) at 0° and holds 1.5 s to align, then the speed ramps to `g_spin_hz` (default 5 Hz electrical = 27 rpm) at `g_spin_accel` (default 2.5 Hz/s). Negative `g_spin_hz` reverses (step 2); 0 decelerates and holds. Limits: ±10 Hz (the back-EMF at 10 Hz is 0.89 V, close to the 1 V applied, so open loop would lose sync beyond it), 10 Hz/s, 1.2 V; trip stays 1.0 A (Stage 8 current check deferred).
- **Encoder against forced angle (step 3), logged every second:** `SPIN armed= V= f= tgt= rpm forced= meas= cnt/ecyc= pp= Iamp= fault=`. cnt/ecyc = encoder counts per electrical cycle over the last second (expect ±1489.5 = 16384/11; the sign is the encoder direction relative to A→B→C, which Stage 10 needs). pp = 16384/|cnt/ecyc| (expect 11.0). Iamp = current amplitude from the latest sample (Clarke: Iα = Ia, Iβ = (Ib − Ic)/√3; plan pass: about 0.6 A).
- **Encoder faults now disarm (plan: Encoder faults, Stage 9):** while armed, a new CRC error, SPI timeout or impossible jump, or an unhealthy status (field, loss of track), latches FAULT\_ENCODER; the fault response disarms. The encoder is still read in the main loop (1 kHz), so the response is within 1 ms. **Arming rule added:** encoder healthy.

### 2026-10-08: Stage 9 first spin (developer's log, +5 Hz, 1.0 V)

- **The motor spins open loop.** Over 30 s: cnt/ecyc −1482 to −1499 (mean about −1489 = −16384/11), **pole pairs 10.93–11.05: 11 confirmed**. The encoder advances exactly 1489 counts per forced electrical cycle, so the rotor is locked to the forced speed (27.3 rpm). Iamp 400–440 mA (CUR rms about 72 counts gives the same), matching 1 V / 2.2 Ω less a little back-EMF (the plan's "about 0.6 A" assumed 1.6 Ω). No faults, encoder CRC 0, interrupt 7.2 µs (14.6 %).
- **Direction: cnt/ecyc is negative.** The encoder count falls as the electrical angle advances A → B → C, so **DIR = −1** for Stage 10 (θe = wrap(DIR × 11 × θm − θoffset)), with the motor leads in their present order.
- **`meas` rpm read −18 against 27 forced:** the speed-estimate (PLL) snapshot taken every 1 s, which at 5 Hz always lands at the same point of the electrical cycle and catches the open-loop speed ripple. The position-based figure shows no slip. Fixed: `meas` is now the average over the second from the encoder position; the snapshot stays as `pll`. To revisit when the encoder read moves into the control interrupt (Stage 10 onward).
- Still to do: reverse direction (step 2); smoothness by eye.

### 2026-10-08: Stage 9 passed

- **Reverse, −5 Hz:** cnt/ecyc −1482 to −1499, pp 10.93–11.05, Iamp 395–452 mA, no faults. cnt/ecyc keeps its sign when the direction reverses (Δposition and Δcycles both flip); a sign that's the same both ways confirms **DIR = −1**. (The chat had predicted a sign flip, which was wrong.)
- **Reversal −5 → +8 Hz:** smooth deceleration through zero and acceleration the other way (developer: "slowing down very smoothly"), no faults.
- **+8 Hz (43.6 rpm):** cnt/ecyc −1480 to −1495, pp 10.95–11.06, Iamp 368–420 mA (slightly lower: more back-EMF). The `meas` figures in this run are still the PLL snapshot (build before the averaging fix).
- **Pass (plan):** turns smoothly ✔; 11 electrical cycles per mechanical turn ✔; encoder speed matches the forced speed (counts per cycle, no slip) ✔; current 0.37–0.45 A (the plan's "near 0.6 A" assumed 1.6 Ω; with the measured 2.2 Ω, 1 V gives about 0.45 A) ✔; both directions ✔.
- **Recorded:** `MOTOR_ENC_DIR = -1` in `motor_params.h` (to be confirmed by Stage 10), plus `MOTOR_ENC_OFFSET_RAD` as TODO\_MEASURED for Stage 10. Motor leads must stay in their present order on J13.
- **Reminder:** the deferred Stage 8 current check (Stage prerequisites) must be done before Stage 11.
