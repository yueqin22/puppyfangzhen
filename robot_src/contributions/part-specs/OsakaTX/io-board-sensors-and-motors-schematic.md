# OOMWOO I/O Board Schematic — Sensor & Motor Pinout Compilation

> **Source:** `makerspet/oomwoo-io-board` repository — KiCad reference schematic (committed Jul 12, 2026)
> and `docs/SPEC.md` (updated Jul 14, 2026).
> **Extraction date:** July 17, 2026
> **Method:** Parsed `.kicad_sch` files for component values, hierarchical net labels, and footprint assignments.
> **Scope:** All sensor and motor sub-sheets beyond the wheel motor sheet (which is covered separately in
> the companion file `io-board-wheel-connector-and-caster.md`).

## Summary of Findings

This document compiles part numbers, connector types, and net assignments for every motor
and sensor sub-circuit on the oomwoo I/O board reference schematic. The data is extracted
directly from KiCad schematic files — these are the oomwoo project's own design files, not
reverse-engineering of a commercial product.

**Key discoveries:**
- Motor driver IC confirmed as **TI DRV8870DDAR** (not TMI8870 or DRV8871)
- Side brush and main brush use **low-side N-MOSFET switches** (AO3400), not H-bridges
- Main fan uses **P-MOSFET high-side switch** (AO3401) with tach feedback
- Bumper sensors are **ITR9606 photomicroswitches** (optical interrupter, not mechanical switches)
- Dock IR sensors use **TSOP34138** IR receivers (38kHz, same family as TSOP38238)
- LiDAR connector is 4-pin (M+, M-, RX, 5V) with low-side MOSFET motor control
- Anti-fall (cliff) sensors are 4× analog IR (left-up, left-down, right-up, right-down)
- Side proximity sensors use discrete IR LEDs with 2N3904 NPN PWM drivers

---

## 1. Motor Driver — Drive Wheels

| Property | Value |
|----------|-------|
| IC | **DRV8870DDAR** (Texas Instruments) |
| Package | HSOP-8 with exposed pad |
| KiCad footprint | `HSOP-8_L5.0-W4.0-P1.27-LS6.2-BL-EP` |
| Quantity | 2 (U14 = right wheel, U15 = left wheel) |
| Motor voltage | BAT-VCC (14.4V nominal, 12–16.8V range) |
| Current sense | 0.2Ω shunt resistor (R83 right, R84 left) → ADC |
| Logic supply | VCC-3V3-P (3.3V) |

### DRV8870 Pin Assignments (from KiCad symbol)

| Pin | Name | Function |
|-----|------|----------|
| 1 | GND | Ground |
| 2 | IN2 | Input 2 (PWM/direction) |
| 3 | IN1 | Input 1 (PWM/direction) |
| 4 | VREF | Current reference voltage |
| 5 | VM | Motor supply (BAT-VCC) |
| 6 | OUT1 | Motor output 1 |
| 7 | ISEN | Current sense output |
| 8 | OUT2 | Motor output 2 |
| 9 | EP | Exposed pad (GND) |

### Drive Wheel Signal Nets

| Net | Direction | Function |
|-----|-----------|----------|
| `WHEEL-M-RIGHT-IN1` / `WHEEL-M-LEFT-IN1` | STM32 → DRV8870 | Motor direction/PWM input 1 |
| `WHEEL-M-RIGHT-IN2` / `WHEEL-M-LEFT-IN2` | STM32 → DRV8870 | Motor direction/PWM input 2 |
| `WHEEL-M-RIGHT-ENCODE-A` / `WHEEL-M-LEFT-ENCODE-A` | Wheel encoder → STM32 | Single-channel encoder pulse |
| `WHEEL-M-R-F-ADC` / `WHEEL-M-L-F-ADC` | Current sense → STM32 ADC | Motor current feedback |
| `WHEEL-M-EN-V` | STM32 → power control | Motor power enable (P-MOSFET gate) |
| `VCC-5V-WHEEL` | Power supply | +5V for wheel encoders |

> **Confirmation:** Only `ENCODE-A` exists — no `ENCODE-B`. The encoder is definitively
> single-channel on the oomwoo reference design, matching Scowt PR #13's physical inspection
> of the Roborock S5 wheel module (single blue encoder wire).

### Additional Wheel Circuit Components

| Ref | Value | Function |
|-----|-------|----------|
| Q12 | AO3401 | P-MOSFET — motor power enable switch |
| Q13 | 2N3904 | NPN transistor — power control logic |
| R83/R84 | 0.2Ω | Current sense shunt resistor |
| D19/D20 | 1N4007 | Flyback diode (motor back-EMF protection) |
| C85/C89 | 100µF | Bulk decoupling capacitor |
| C86/C90 | 0.1µF | High-frequency decoupling |
| C84/C88 | 0.22µF | Filter capacitor |

---

## 2. Side Brush Motors

| Property | Value |
|----------|-------|
| Connectors | J15 (right), J16 (left) |
| Connector value labels | "SIDE BRUSH RIGHT", "SIDE BRUSH LEFT" |
| Motor driver | **AO3400 N-MOSFET** (low-side switch, NOT H-bridge) |
| MOSFETs | Q16 (right), Q18 (left) |
| Flyback diodes | D8 (right), D9 (left) — **SS34** (Schottky, 3A, 40V) |
| Current sense | 10kΩ pull-up (R107 right, R112 left) → ADC |

### Side Brush Signal Nets

| Net | Function |
|-----|----------|
| `SIDE-BRUSH-V-R-CTRL` / `SIDE-BRUSH-V-L-CTRL` | STM32 PWM → MOSFET gate (speed control) |
| `SIDE-BRUSH-R-F-ADC` / `SIDE-BRUSH-L-F-ADC` | Current sense → STM32 ADC |

> **Design note:** Side brushes use unidirectional low-side MOSFET switches (single
> direction only), not H-bridges. This matches the SPEC.md entry: "Side brush — bridge or
> FET TBD." The schematic resolves this: it's a FET (low-side N-MOSFET).
>
> The SS34 Schottky diode (3A, 40V) is the flyback protection — rated higher than the
> side brush stall current (1.3A per SPEC.md).

---

## 3. Main Brush Motor

| Property | Value |
|----------|-------|
| Connector | J11 ("MAIN BRUSH") |
| Motor driver | **AO3400 N-MOSFET** (low-side switch, Q10) |
| Flyback diode | D7 — **SS34** (Schottky, 3A, 40V) |
| Flyback diode 2 | D18 — **1N4007** (secondary protection) |
| Current sense | 10kΩ pull-up (R82) → ADC |

### Main Brush Signal Nets

| Net | Function |
|-----|----------|
| `MAIN-BRUSH` | STM32 PWM → MOSFET gate (speed control) |
| `MAIN-BRUSH-F-ADC` | Current sense → STM32 ADC |

> **Design note:** Like the side brushes, the main brush uses a unidirectional low-side
> MOSFET switch. SPEC.md lists stall current as "22A??" — the SS34 (3A) seems undersized
> for 22A stall, suggesting either the stall current estimate is wrong or additional
> protection exists. The SPEC.md TODO marker is appropriate.

---

## 4. Suction Fan (Main Fan)

| Property | Value |
|----------|-------|
| Connector | J14 ("MAIN-FAN") |
| Connector type | **4-pin PH 2.0mm** SMD (`XUNPU WAFER-PH2.0-4PWB`) |
| Motor driver | **AO3401 P-MOSFET** (high-side switch, Q14) |
| Control transistor | Q15 — **2N3904** NPN (drives P-MOSFET gate) |
| Tach feedback | Yes (FG sense) |
| Current sense | R103 (22kΩ) |

### Main Fan Signal Nets

| Net | Function |
|-----|----------|
| `MAIN-FAN-S-CTRL` | STM32 PWM → P-MOSFET gate (via NPN) — speed control |
| `MAIN-FAN-V-CTRL` | STM32 → power enable |
| `MAIN-FAN-S-SENSE` | Fan tachometer (FG) → STM32 |

> **Design note:** The main fan uses a high-side P-MOSFET switch (AO3401) controlled via a
> 2N3904 NPN transistor, with tachometer feedback. SPEC.md states "PWM input to fan, FG
> feedback to STM32" — confirmed by the schematic. The 4-pin connector carries: PWM control,
> FG sense, power (BAT-VCC), and GND.

---

## 5. LiDAR Motor

| Property | Value |
|----------|-------|
| Connector | J17 ("LiDAR") |
| Connector pins | 4 (M+, M-, RX, 5VCC — from text labels) |
| Motor driver | **AO3400 N-MOSFET** (low-side switch, Q20) |
| Flyback diode | D10 — **SS34** (Schottky) |
| Current limit | R114 (2kΩ), R113 (1kΩ) |

### LiDAR Signal Nets

| Net | Function |
|-----|----------|
| `LiDAR-MOTOR-CTRL` | STM32 PWM → MOSFET gate (RPM control) |
| `LiDAR-RXD` | LiDAR serial data → STM32 UART RX |

> **Design note:** The LiDAR motor is controlled via a low-side N-MOSFET (speed via PWM),
> with the LiDAR's serial data going to the STM32 UART. The 4-pin connector carries:
> motor+ (always on via BAT-VCC), motor- (switched via MOSFET), serial RX, and +5V supply.
> SPEC.md confirms: "5V 0.35A max, Mabuchi-style RF-500TB-14350 or similar, low-side load
> switch N-FET."

---

## 6. Bumper & Docking IR Sensors

| Property | Value |
|----------|-------|
| Schematic sheet | "DOCKING-IR-BUMPER-SENSORs" |
| Bumper sensors | **2× ITR9606** (photomicroswitch / optical interrupter) |
| Bumper references | P-SW1, P-SW2 |
| Dock IR receivers | **2× TSOP34138** (38kHz IR receiver) |
| Dock IR references | IR1, IR2 |

### Component Details

| Ref | Value | Function |
|-----|-------|----------|
| P-SW1 | ITR9606 | Bumper switch 1 (optical interrupter) |
| P-SW2 | ITR9606 | Bumper switch 2 (optical interrupter) |
| IR1 | TSOP34138 | Dock IR sensor 1 (38kHz receiver) |
| IR2 | TSOP34138 | Dock IR sensor 2 (38kHz receiver) |
| R61/R65 | 1kΩ | ITR9606 LED current limiting |
| R62/R64 | 10kΩ | ITR9606 output pull-up |
| R63/R66 | 4.7kΩ | TSOP34138 pull-up |

### Signal Nets

| Net | Function |
|-----|----------|
| `BUMPER-SW1` | Bumper 1 → STM32 digital input |
| `BUMPER-SW2` | Bumper 2 → STM32 digital input |
| `DOC-IR-SENS1` | Dock IR sensor 1 → STM32 |
| `DOC-IR-SENS2` | Dock IR sensor 2 → STM32 |

> **Key finding — bumper is optical, not mechanical:** The ITR9606 is a **photomicroswitch**
> (slotted optical switch), not a mechanical microswitch. This is a non-contact bumper
> sensor that detects physical deflection via an optical interrupter. The SPEC.md lists
> "bumper (IR or micro-switches)" — the schematic confirms: **IR (optical interrupter)**.
>
> **TSOP34138 vs TSOP38238:** The dock IR sensors use TSOP34138, which is from the same
> TSOP family as the TSOP38238 wall sensor (PR #24). The TSOP34138 operates at 38kHz
> with a 2.0–5.5V supply, similar to the TSOP38238. The difference: TSOP34138 has shorter
> carrier burst minimum (8 cycles vs 10+) making it more responsive for docking beacon
> detection.

---

## 7. Anti-Fall (Cliff) IR Sensors

| Property | Value |
|----------|-------|
| Schematic sheet | "ANTI-FALL-IR-SENSORs" |
| Connectors | J4 (left-down), J5 (right-down) + implied J2/J3 (left-up, right-up) |
| Quantity | 4 sensors (left-up, left-down, right-up, right-down) |
| Signal type | **Analog** (ADC, not digital) |

### Signal Nets

| Net | Function |
|-----|----------|
| `ANTI-FALL-LEFT-UP-ADC` | Left-up cliff sensor → STM32 ADC |
| `ANTI-FALL-LEFT-DOWN-ADC` | Left-down cliff sensor → STM32 ADC |
| `ANTI-FALL-RIGHT-UP-ADC` | Right-up cliff sensor → STM32 ADC |
| `ANTI-FALL-RIGHT-DOWN-ADC` | Right-down cliff sensor → STM32 ADC |

### Components

| Ref | Value | Function |
|-----|-------|----------|
| R67/R68/R71/R72 | 100Ω | IR LED current limiting |
| R69/R70/R73/R74 | 10kΩ | Phototransistor output pull-up |

> **Design note:** Four cliff sensors with analog output — the STM32 reads the IR
> reflectance level via ADC, allowing threshold-based cliff detection. SPEC.md GPIO list
> confirms: "anti-fall left up/down, right up/down (analog in because IR sensors are
> analog)." The 100Ω LED resistor suggests ~30mA IR LED current at 3.3V (or ~50mA at 5V).

---

## 8. Side Proximity IR Sensors

| Property | Value |
|----------|-------|
| Schematic sheet | "SIDE-PROXIMITY-IR-SENSOR" |
| Connectors | J8 (left), J9 (right) |
| IR LED drivers | **2× 2N3904** NPN transistors (Q6, Q7) for PWM |
| Signal type | Analog (ADC) |

### Signal Nets

| Net | Function |
|-----|----------|
| `SIDE-PROXI-LEFT` | Left side proximity → STM32 ADC |
| `SIDE-PROXI-RIGHT` | Right side proximity → STM32 ADC |
| `TOUCHL` | Left touch/bumper → STM32 digital input |
| `TOUCHR` | Right touch/bumper → STM32 digital input |

### Components

| Ref | Value | Function |
|-----|-------|----------|
| Q6 | 2N3904 | Left IR LED PWM driver |
| Q7 | 2N3904 | Right IR LED PWM driver |
| R75/R79 | 1kΩ | IR LED current limiting |
| R76/R78 | 2kΩ | NPN base resistor |
| R77/R80 | 1kΩ | Output pull-up |

> **Design note:** Side proximity sensors use discrete IR LED + phototransistor pairs with
> NPN transistor PWM drivers. The `TOUCHL`/`TOUCHR` nets suggest a secondary touch-sensing
> function (possibly the same sensor at close range). SPEC.md confirms: "Side proximity IR
> sensor left/right (analog in)" and "Side proximity IR LED left/right PWM (digital out)."

---

## 9. Connector Summary Table

| Connector | Function | Type | Pins |
|-----------|----------|------|------|
| J4 | Anti-fall left-down | IR sensor | 2 |
| J5 | Anti-fall right-down | IR sensor | 2 |
| J8 | Side proximity left | IR sensor pair | 3–4 |
| J9 | Side proximity right | IR sensor pair | 3–4 |
| J11 | Main brush motor | DC motor | 2 |
| J12 | Drive wheel right | Wheel module | 5 (JST ZH 1.5mm) |
| J13 | Drive wheel left | Wheel module | 5 (JST ZH 1.5mm) |
| J14 | Main fan (suction) | BLDC fan | 4 (PH 2.0mm) |
| J15 | Side brush right | DC motor | 2 |
| J16 | Side brush left | DC motor | 2 |
| J17 | LiDAR | Motor + serial | 4 |

---

## 10. SPEC.md Motor Specifications (Cross-Reference)

The `oomwoo-io-board/docs/SPEC.md` provides these motor specs (with original TODO markers):

| Motor | Qty | Voltage | Stall Current | Driver | Schematic Confirms |
|-------|-----|---------|---------------|--------|-------------------|
| Drive wheel | 2 | 14.4V | 3.5A (TODO) | DRV8870 H-bridge | ✅ DRV8870DDAR |
| Suction fan | 1 | 14.4V | 10A (TODO) | P-FET high-side | ✅ AO3401 |
| LiDAR | 1 | 5V | 0.35A | N-FET low-side | ✅ AO3400 |
| Main brush | 1 | 14.4V | 22A?? (TODO) | N-FET low-side | ✅ AO3400 |
| Side brush | 2 | 14.4V | 1.3A (TODO) | N-FET low-side | ✅ AO3400 |

> **Note:** SPEC.md originally said drive wheel H-bridge was "DRV8231, DRV8871, or similar"
> — the KiCad schematic confirms it is specifically **DRV8870DDAR**.

---

## 11. Updated Remaining Gaps

| Gap | Status | Notes |
|-----|--------|-------|
| Encoder PPR | ✅ Derived (~228 raw PPR, 4464 ticks/m) | From VacuumTiger calibration |
| Gearbox ratio | ✅ Derived (~190:1) | From VacuumTiger velocity scale |
| Full J25/J26 pinout | ⚠️ Partially resolved | OOMWOO I/O board uses 5-pin ZH (this doc). Original Roborock J25/J26 still needs physical probing. |
| Caster wheel specs | ⚠️ Partially resolved | OEM part HA00021, ~46×52mm, snap-in mount (see separate file) |
| Bumper sensor model | ✅ **NEW** | ITR9606 photomicroswitch (optical interrupter) |
| Dock IR sensor model | ✅ **NEW** | TSOP34138 38kHz IR receiver |
| Side brush driver | ✅ **NEW** | AO3400 N-MOSFET low-side switch (unidirectional) |
| Main brush driver | ✅ **NEW** | AO3400 N-MOSFET low-side switch (unidirectional) |
| Main fan driver | ✅ **NEW** | AO3401 P-MOSFET high-side switch with tach feedback |
| LiDAR motor driver | ✅ **NEW** | AO3400 N-MOSFET low-side switch |
| Main fan connector | ✅ **NEW** | 4-pin PH 2.0mm (XUNPU WAFER-PH2.0-4PWB) |
| Cliff sensor type | ✅ **NEW** | 4× analog IR (ADC), 100Ω LED resistor |
| Side proximity type | ✅ **NEW** | Discrete IR pair with 2N3904 PWM driver |

---

## Sources

- KiCad schematic files: `https://github.com/makerspet/oomwoo-io-board/tree/main/kicad/`
  - `WHEEL-MOTORs .kicad_sch` — drive wheel motor circuit
  - `SIDE-BRUSH-MOTORs .kicad_sch` — side brush motor circuits
  - `MAIN-BRUSH-MOTOR .kicad_sch` — main brush motor circuit
  - `MAIN-FAN .kicad_sch` — suction fan circuit
  - `LiDAR .kicad_sch` — LiDAR motor and serial circuit
  - `DOCKING IR-BUMPER-SENSOR.kicad_sch` — bumper and dock IR sensors
  - `ANTI-FALL-IR-SENSORs.kicad_sch` — cliff sensors
  - `SIDE-PROXIMITY-IR-SENSOR .kicad_sch` — side proximity sensors
- SPEC.md: `https://github.com/makerspet/oomwoo-io-board/blob/main/docs/SPEC.md`
- GPIO budget: `https://github.com/makerspet/oomwoo-io-board/blob/main/docs/GPIO.md` (referenced in README)
