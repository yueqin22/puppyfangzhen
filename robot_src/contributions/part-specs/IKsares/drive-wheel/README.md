# Drive wheel module — motor, encoder, gearbox and wheel-drop sensor

Spec sheet for the aftermarket Roborock S5 Max–family drive wheel module, measured on a
physical unit. Covers the electrical side (motor + Hall encoder wiring, verified pinout)
and the mechanical side that odometry depends on (gear teeth counted, encoder edges per
revolution, mm per edge).

Everything marked **VERIFIED** was measured by me on the unit described under
*Provenance*. Everything else is flagged as estimated, inherited from generic family data,
or open.

## Provenance

| Field | Value |
|---|---|
| Part | Drive wheel module (left/right pair), aftermarket replacement |
| Advertised compatibility | Roborock S5 Max / S6 MaxV / S6 Pure / S7 Pro / E4 |
| Vendor | AliExpress — [listing 1005007359089056](https://es.aliexpress.com/item/1005007359089056.html) |
| Unit tested | One module, disassembled |
| Motor can markings | `CDM-MOTOR` / `GM-RS360-16248` / `DC 12.0V` / `20231209C1` |
| Measured by | [@IKsares](https://github.com/IKsares), July 2026 |
| Instruments | Digital multimeter, bench supply, calipers |

> ⚠️ **This is an aftermarket module, not an OEM Roborock teardown.** The motor fitted here
> is a CDM `GM-RS360-16248`. An OEM module, or another aftermarket batch, may ship a
> different motor or a different wire colour code. Check the can markings against yours
> before trusting the pinout below.

## 1. Motor identification

| Field | Value | Status |
|---|---|---|
| Manufacturer | CDM (Chinese OEM) | from can marking |
| Model | `GM-RS360-16248` | from can marking |
| Frame family | RS-360 class | inferred from format |
| Type | Brushed DC, permanent magnet | VERIFIED |
| Rated voltage | 12 V DC | from can marking |
| Feedback | Single-channel Hall sensor on a rear-cap PCB, reading a magnetic ring on the shaft | VERIFIED |
| Mechanical output | Metal pinion on the front shaft (11 teeth) | VERIFIED |
| Interconnect | 5-wire pigtail soldered to the rear PCB | VERIFIED |
| Datasheet | **None published** for this variant | — |

`20231209C1` fits a date/lot code pattern (2023-12-09, lot C1); not confirmed by the
manufacturer.

Generic RS-360 family figures circulated by distributors — ≈12 000 rpm no-load at 12 V,
3–10 W, ≈52 g, 6–24 V operating range, can ≈Ø27 mm × ≈50 mm long — are **order-of-magnitude
only**. They were not measured on this unit and should not be quoted as the part's
specification, nor used as CAD dimensions.

## 2. Wiring — VERIFIED

Five wires leave the rear PCB. Two are the motor winding, three are the Hall sensor.

| Wire colour | Function | Notes |
|---|---|---|
| red | motor terminal 1 | polarity only sets rotation direction |
| black | motor terminal 2 | interchangeable with red |
| orange | Hall sensor **VCC** | verified working at 3.3 V, ≈2.5 mA |
| brown | Hall sensor **GND** | sensor reference |
| blue | Hall sensor **OUT** | swings 0 V ↔ VCC, no external pull-up needed |

> ⚠️ **Colour-code warning.** The sensor wiring does **not** follow the common industrial
> convention (brown = +, blue = 0 V). On this unit **brown is GND and orange is VCC**.
> Any harness, schematic or assembly instruction must state this explicitly — wiring it
> "the usual way" reverses the sensor supply.

The PCB silkscreen reads `红 黑 棕 蓝 橙` (red, black, brown, blue, orange) next to the five
solder pads. It labels only the **wire colour** expected at each pad — it does not document
the signal function. The pad order on the board is therefore red, black, brown, blue,
orange; the function mapping is the one in the table above.

![Rear cap of the motor: Hall PCB, five solder pads and the magnetic ring on the shaft](motor-rear-hall-pcb.webp)

The motor winding is **galvanically isolated** from the sensor circuit (no continuity
between the red/black pair and any of the three sensor wires).

### How it was verified

1. **Power circuit.** Continuity confirmed between red/black and the two brush tabs on the
   can. The other three wires show no continuity to the tabs.
2. **Diode-mode mapping** of the three sensor wires (red probe on the first wire listed):

   | Pair | Reading |
   |---|---|
   | brown → blue | 0.43 V |
   | orange → brown | 0.957 V |
   | brown → orange | OL |
   | blue → brown | OL |
   | blue → orange | OL |
   | orange → blue | OL |

   The 0.957 V drop orange→brown corresponds to two junctions in series (series protection
   diode on the supply + internal path of the IC), identifying **orange as VCC**. Conduction
   brown→blue with no conduction at all *from* blue is consistent with **brown as ground**.
3. **Functional test.** 3.3 V applied to orange through a 220 Ω series resistor, brown to
   supply ground. Measured 2.76 V at orange → 0.54 V across the resistor → **≈2.5 mA draw**,
   consistent with a digital Hall IC. Spinning the shaft by hand, blue **toggles between
   0 V and 2.7 V** — essentially rail to rail.

Two conclusions follow from the functional test: the sensor is operational at **3.3 V**,
which rules out the classic 4.5–24 V Hall parts (A3144 class) and points to a modern
low-voltage IC; and the output reaches nearly VCC with no external resistor, so **no
external pull-up is required** (the stage is push-pull, or the board carries its own
pull-up — a resistor is visible on the silkscreen).

## 3. Gearbox — VERIFIED

Compound spur gear train, four reduction stages from the motor pinion to the wheel output.
Teeth counted on the opened gearbox.

| Stage | Driver (teeth) | Driven (teeth) | Ratio |
|---|---|---|---|
| Motor pinion → gear 1 | 11 | 36 | 3.273 |
| Gear 1 pinion → gear 2 | 13 | 42 | 3.231 |
| Gear 2 pinion → gear 3 | 12 | 34 | 2.833 |
| Gear 3 pinion → gear 4 (output) | 11 | 24 | 2.182 |

**Total reduction i = (36·42·34·24) / (11·13·12·11) = 1 233 792 / 18 876 ≈ 65.36 : 1**
(motor revolutions per wheel revolution).

## 4. Encoder resolution and odometry — MEASURED, with a caveat

Base measurement: **4 rising edges per motor revolution**, i.e. 4 pulse cycles per
revolution → the magnetic ring has **4 pole pairs (8 poles)**.

**Method, and its limits.** The motor was out of the gearbox and its shaft turned slowly by
hand while watching the blue line toggle on a multimeter — so this counts edges per *motor*
revolution, not per wheel revolution. **No oscilloscope and no frequency counter were used.**
A multimeter's refresh rate is slow, so this is only reliable if the shaft is turned slowly
enough that no transition is missed. If any were missed, **4 is a lower bound** and the real
count would be a multiple of it — 8 pole pairs would halve every distance below. The figure
is consistent with the independent cross-check in §7, but a scope capture would settle it.

Fixed data used below: gearbox reduction 65.36 : 1; wheel outer diameter **71.5 mm**
(measured with calipers); wheel circumference π × 71.5 = **224.6 mm**.

| Quantity | Rising edges only | Both edges (rising + falling) |
|---|---|---|
| Edges per motor revolution | 4 | 8 |
| Edges per wheel revolution | 4 × 65.36 ≈ **261.5** | 8 × 65.36 ≈ **522.9** |
| **Distance per edge** | 224.6 / 261.5 ≈ **0.859 mm** | 224.6 / 522.9 ≈ **0.430 mm** |
| Ticks per metre | ≈ **1164** | ≈ **2328** |
| Angular resolution at the wheel | ≈ 1.38° | ≈ 0.69° |

Firmware can count rising edges only (0.859 mm/edge) or both edges to double the resolution
(0.430 mm/edge) with no hardware change.

> **Design limitation:** single channel. This gives speed and distance, **not direction and
> not absolute position**. Direction has to be inferred from the polarity the controller
> applies to the motor. Closed-loop position control or feedback-based direction sensing
> would need a second sensor in quadrature.

## 5. Wheel-drop sensor — PARTIALLY characterized

A second sensor sits in the module's wire bundle: the wheel-drop (wheel-lift) detector that
[SPEC.md](https://github.com/makerspet/oomwoo-one-cad/blob/main/docs/SPEC.md) expects one of
per wheel.

| Field | Value | Status |
|---|---|---|
| Markings | `MG01-13` / `5P30-M55-W8W` | recorded |
| Manufacturer | Unknown — neither marking resolves to a public datasheet (OEM part) | — |
| Type | **Mechanical switch** (not an optical or Hall sensor) | confirmed on the part |
| Wiring | **Two wires, both brown** | confirmed |
| Polarity | None — a dry contact, so the two wires are electrically interchangeable | — |
| NO / NC at rest | Not yet determined | open |

> ⚠️ **Three brown wires, three different functions.** In this module brown is the Hall
> sensor GND (§2) *and* both wheel-drop switch wires. Colour cannot identify a conductor
> here — identify by connector position or by continuity, never by colour. This is a real
> assembly hazard: connecting a switch wire where the Hall ground belongs shorts nothing but
> leaves the sensor unreferenced and the encoder dead, with no visible clue why.

Note this also differs from [Scowt's](../../Scowt/DriveWheel.md) description, which has the
limit switch on **two grey wires**. Either the colour varies between OEM and aftermarket
batches, or between production runs — either way, harness documentation should not key off
wire colour for this switch.

### Still open

Measurable now, even with the switch out of the module (multimeter, no power):

- [ ] **NO or NC at rest** — press the actuator and watch continuity. One minute of work, and
      it is half of what the firmware needs
- [ ] Contact rating, if printed on the body
- [ ] Where each marking is printed (switch body vs harness/connector — `5P30-M55-W8W` may be
      a harness code rather than the switch model)

Needs the module assembled, or at least the switch offered up to its seat:

- [ ] **Which mechanical state means *wheel retracted*** (robot resting on the floor, wheel
      pushed up into the body) vs *wheel dropped* (robot lifted, spring extends the wheel).
      Firmware needs this to fail safe — a wheel-drop that reads inverted means the robot
      stops on the floor and drives happily while held in the air
- [ ] What actuates it — typically the suspension arm as the wheel travels up

## 6. Integration requirements

1. **Sensor supply: 3.3 V or 5 V**, to match the controller logic. Verified at 3.3 V here;
   5 V not tested on this unit. **Do not feed the sensor from 12 V** until the Hall IC is
   identified and its maximum supply confirmed.
2. **Common ground.** Tie sensor GND (brown) to the motor driver negative at a single point
   near the controller, so motor return current does not flow through the sensor reference.
3. **Decoupling.** 100 nF between orange and brown, as close to the motor as possible.
4. **Signal filtering.** A brushed motor at speed generates significant commutation noise.
   If spurious pulses appear under load, add an RC filter (1 kΩ + 10 nF) on the blue line
   before the MCU input.
5. **Harness.** Keep the sensor wires physically away from the power wires; twisting the
   red/black pair reduces emission.

## 7. Cross-checks against existing contributions

### Confirms [Scowt/DriveWheel.md](../../Scowt/DriveWheel.md)

Scowt's pinout is explicitly flagged as tentative — inferred from photos and a
StackExchange thread, with "some uncertainty around the encoder wires". The mapping guessed
there is **confirmed correct on this physical unit**:

| Scowt (tentative) | This unit (verified) | Result |
|---|---|---|
| Encoder 5 V, orange | orange = Hall VCC | ✅ confirmed (verified at 3.3 V) |
| Encoder Signal, blue | blue = Hall OUT | ✅ confirmed |
| Encoder Ground, brown | brown = Hall GND | ✅ confirmed |
| Motor power, black / red | red, black = winding | ✅ confirmed |
| Hall-effect encoder | Hall sensor + magnetic ring | ✅ confirmed |

One thing does not match: Scowt has the limit switch on **two grey wires**, while on this unit
both are brown (§5). Colour is not a reliable identifier for that switch across variants.

Still open from that document: connector model (JST family/pitch) and cable length. Note that
the encoder being single-channel — verified here — rules out the 6-pin `HALL_DIR` pinout that
[io-pcb](../../../io-pcb/README.md) currently references from the AlieksieievYurii schematic,
at least for this module.

### Closes two open items in [OsakaTX's spec sheets](../../OsakaTX/vacuumtiger-verified-specs.md)

OsakaTX's drive wheel figures are derived — from the VacuumTiger firmware's calibration
constants, a Nidec catalogue entry, and merged PRs — with the physical work explicitly listed
as pending: *"Exact gearbox ratio via tooth count — Open gearbox, count teeth"*, and the pole
count *"speculative without physical inspection"*. This document supplies both measurements.

| Quantity | OsakaTX (derived) | Measured here | How |
|---|---|---|---|
| Gearbox ratio | ~190 : 1 | **65.36 : 1** | teeth counted, 4 spur stages |
| Magnetic ring | ~32 poles (speculative) | **8 poles (4 pole pairs)** | edges counted by hand per motor revolution, multimeter — see §4 |
| Wheel diameter | 65 mm, from alvarosamudio's simulation URDF | **71.5 mm** | calipers, on the physical wheel |
| Motor | Nidec 20N704RC70, 14.4 V (catalogue, flagged "in development") | **CDM GM-RS360-16248, 12 V** | read off the can |

The motor difference may be genuine: this is an aftermarket module, and OsakaTX's own note
warns that "the actual motor used in production Roborock wheels may differ from the catalogue
entry".

### Reconciling with `ticks_per_meter = 4464`

The one hard number on that side is `ticks_per_meter = 4464.0`, empirically calibrated on a
real robot and consistent across several VacuumTiger source files. At first glance it looks
incompatible with the measurements here (1164 ticks/m counting rising edges). It isn't — the
gap is the GD32's decoding, which OsakaTX documents: the hardware timer performs **4× edge
counting** on the single pulse train.

Applying that 4× to the measured mechanics:

```
4 cycles/motor rev  ×  4 (GD32 decoding)      =  16 ticks/motor rev
16  ×  65.36 (measured ratio)                 =  1046 ticks/wheel rev
1046  /  0.2246 m (π × 71.5 mm)               =  4656 ticks/m
```

**4656 vs the calibrated 4464 — within 4%.** The same correction runs the other way: taking
4464 ticks/m and the real 71.5 mm wheel gives 1003 ticks/wheel rev → 251 raw cycles/wheel rev,
against the 261.5 measured here. Again ~4%.

So the calibrated constant and the physical measurements agree. What does not survive are the
two intermediate derivations:

- **~228 PPR and ~32 poles** — the pole count came from dividing by a 65 mm wheel diameter
  taken from the simulation URDF. With the measured 71.5 mm the arithmetic lands on the
  8-pole ring counted here, no speculation needed.
- **~190 : 1** — derived by assuming the configured `max_linear_speed = 0.3 m/s` corresponds
  to the motor at no-load speed. It need not: with the measured 65.36 : 1 and an RS-360-class
  no-load figure (order of magnitude ≈12 000 rpm at 12 V), the wheel would top out near
  0.69 m/s, making 0.3 m/s a deliberate software limit rather than a mechanical ceiling.

Residual ~4% could be the aftermarket module differing from the OEM one, the effective
rolling diameter under load, or the calibration itself.

> **What this cross-check cannot distinguish.** It only pins down the product — **16 counts
> per motor revolution**. Two hypotheses produce it and this arithmetic cannot tell them
> apart: 4 pole pairs with the GD32's 4× decoding (assumed above), or 8 pole pairs with plain
> both-edge counting. The hand count in §4 favours the first, but it is a lower bound, so the
> second is not excluded — and it would halve every distance-per-edge figure in §4.

**Suggested checks:** put a scope on the blue line and turn the shaft one revolution — that
resolves the pole count directly. Separately, roll a wheel a measured distance (e.g. 2 m) on
the bench and count edges, which settles ticks/m end to end, independently of every
derivation above.

### Note for the I/O board design

[OsakaTX/io-board-wheel-connector-and-caster.md](../../OsakaTX/io-board-wheel-connector-and-caster.md)
has the OOMWOO wheel connector supplying the encoder at **+5 V** (`VCC-5V-WHEEL`). This sensor
is confirmed working at 3.3 V drawing ≈2.5 mA; 5 V remains untested here, and the Hall IC's
absolute maximum is unknown.

Also worth carrying into firmware: VacuumTiger resolves the single-channel direction ambiguity
with the **IMU gyro**, not with the commanded motor polarity. That is a more robust answer than
the one in §4 and is worth reusing.

## 8. Coverage against the part-specs checklist

Against the "Drive wheel assembly" list in [part-specs/README.md](../../README.md):

| Requested | Status |
|---|---|
| Motor model | ✅ `CDM GM-RS360-16248` |
| Motor/assembly datasheet | ❌ none published for this variant |
| Encoder type + PPR | ⚠️ single-channel Hall ✅; 4 rising edges/motor rev counted by hand — lower bound, see §4 |
| Gearbox ratio | ✅ 65.36 : 1, teeth counted |
| Wheel diameter | ✅ 71.5 mm OD |
| Rated voltage | ✅ 12 V (can marking) |
| Max voltage | ❌ not established |
| Current (no-load & stall) | ❌ not measured |
| Torque | ❌ not measured |
| Max / rated wheel speed | ❌ not measured |
| Cable lengths | ❌ not recorded |
| Connector models (both ends) | ❌ not recorded |
| Full connector + motor pinouts | ⚠️ motor-side 5-wire pinout ✅ verified; module connector pinout not mapped |
| Wheel-drop sensor model + pinout | ⚠️ mechanical switch confirmed, 2 brown wires, no polarity; NO/NC and mechanical polarity open — §5 |
| Signal waveforms | ❌ no scope captures (multimeter only) |
| Assembly weight | ❌ not measured |

## 9. Open points

- [ ] Motor can dimensions (diameter, body length, shaft diameter) — also needed for CAD
- [ ] Hall IC part number — requires lifting the PCB, the marked face is hidden
- [ ] Winding resistance, no-load current @ 12 V, stall current
- [ ] Wheel-drop switch: NO/NC at rest, and which mechanical state means wheel retracted (§5)
- [ ] Module connector model, pin order and cable length
- [ ] **Confirm the pole count with a scope or frequency counter** — the hand count in §4 is
      a lower bound, and §7 cannot separate 4 pole pairs from 8
- [ ] Scope captures of the encoder output under load (noise, edge quality)
- [ ] Bench roll test (edges over a measured distance) to close the residual ~4% in §7
- [ ] Confirm whether an OEM Roborock module carries the same 4-pole-pair ring and 65.36 : 1
      train as this aftermarket one

---

Licensed under Apache 2.0, in line with the repository.
