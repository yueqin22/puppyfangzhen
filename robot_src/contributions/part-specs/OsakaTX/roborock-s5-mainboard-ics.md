# Roborock S5 Mainboard IC Identification

> **Source:** feliggs/RoborockS5Hardware (GitHub), published Oct 3, 2025
> **URL:** https://github.com/feliggs/RoborockS5Hardware
> **Method:** Physical disassembly with adhesive removal, IC label reading
> **License:** CC0-1.0
> **Last updated:** July 16, 2026

## Summary

A physical teardown of the Roborock S5 mainboard (board mark: Ruby_S Main-B V3) was
published by feliggs (Felix Hubenthal) in October 2025. The entire conformal adhesive
coating was removed, exposing IC markings. This is the first publicly documented
complete IC-level component list for the Roborock S5 mainboard.

> ⚠️ **Important note on MCU identity.** The Roborock S5 mainboard uses an
> **STM32F103VCT6** (ARM Cortex-M3, 72 MHz, 256 kB Flash) — NOT a GD32. The
> codetiger/VacuumRobot research documented a GD32F103VCT6 on the **3irobotix
> CRL-200S** (a different robot platform). The two robots share a similar
> architecture (A33 SoC + Cortex-M3 motor controller) but are different products.
> Our existing specs file (`vacuumtiger-verified-specs.md`) derives from the
> CRL-200S; the Roborock S5 BOM part uses a different mainboard. Both the GD32 and
> STM32F103 are pin-compatible Cortex-M3 MCUs from GigaDevice/ST, which may explain
> the confusion.

## Main Processor Architecture

| Role | IC | Architecture |
|------|----|-------------|
| Main SoC | Allwinner R16 (marking unreadable) | Quad-core ARM Cortex-A7, Mali400MP2 |
| Motor controller MCU | **STM32F103VCT6** | ARM Cortex-M3, 72 MHz, 256 kB Flash |
| WiFi | Realtek RTL8189ETV (RMC 8189ET) | 2.4 GHz |
| PMIC | X-Powers AXP223 | Power management |
| RAM/Flash | SKhynix H5TQ4G63CFR | DDR3L + NAND |

## Motor Driver ICs

| # | Label | Description | Datasheet |
|---|-------|-------------|-----------|
| 2 | 8TI8832 | **DRV8832** Low-Voltage Motor Driver IC | [TI DRV8832](https://www.ti.com/lit/ds/symlink/drv8832.pdf) |
| 4 | B9930 CKH0174 | Dual-N/Dual-P logic level MOSFET array (H-bridge) | [StackExchange](https://electronics.stackexchange.com/questions/550727/what-is-this-ic-b9930-ckh0174) |
| 26 | B9930 CKH0174 | Dual-N/Dual-P logic level MOSFET array (H-bridge) | same |
| 27 | 8870 TI 86A P3T064 | Likely **DRV8870**-class H-bridge (top mark "8870") | — |

> **Correction to existing specs.** Our `vacuumtiger-verified-specs.md` section 5
> identifies the motor driver as **TMI8870** (TOLL Microelectronic, pin-compatible
> with TI DRV8870) based on the CRL-200S platform. On the actual Roborock S5
> mainboard, IC #27 has the top mark "8870" with TI branding — consistent with a
> genuine **TI DRV8870**. IC #2 is a **TI DRV8832** (low-voltage motor driver,
> likely for the side brush or main brush). Two B9930 MOSFET arrays (#4, #26) form
> additional H-bridges for other motors.

## Battery Management

| # | Label | Description | Datasheet |
|---|-------|-------------|-----------|
| 11 | BQ24773 TI 8AI A403 | Battery charge controller / power monitor | [TI BQ24773](https://www.ti.com/lit/ds/symlink/bq24773.pdf) |

## Op-Amps and Comparators

| # | Label | Type | Datasheet |
|---|-------|------|-----------|
| 1 | 3PEAK 1544A BBIF | Quad OpAmp | [3PEAK TP1544](https://uploadcdn.oneyac.com/attachments/files/brand/pdf/3peakit-1544a.pdf) |
| 10 | SGM8622 XMS 1025C | OpAmp | [SGM8622](https://www.sg-micro.com/product/SGM8622) |
| 13 | SGM8748 YMSB 1821N | Dual high-speed power comparator | [SGM8748](https://www.sg-micro.com/product/SGM8748) |
| 20 | SGM8622E2 XMS1825C | OpAmp | same as #10 |
| 22 | SGM8748MSB 1821N | Power comparator | same as #13 |
| 23 | 3PEAK 1562A BBIKL | OpAmp | [3PEAK TP1562](https://www.3peak.com/low-voltage-op-amps/tp1562a) |
| 24-25 | C3021LLD 1828 | MOSFET | [C3021LL](https://alldatasheet.net/datasheet-pdf/view-main/254492/UTC/C3021LL.html) |

## Unidentified ICs

| # | Label | Notes |
|---|-------|-------|
| 3, 5 | EKk1251 (partially readable) | Label not fully readable |
| 6, 7 | D12 N04 CKF0577 | No public information |
| 8 | EK0659 | No public information |
| 9 | TI IC (type unreadable) | OpAmp (TI branded) |
| 12 | QVH TI 87A PSHT64 | No public information |
| 17 | (unreadable) | Label not readable |

## Relevance to oomwoo BOM

The Roborock S5 drive wheel assembly (BOM item) connects to this mainboard via:
- **J25** (left, 16-pin 1.0mm SHD): encoder + cliff + bumper + dustbox signals
- **J26** (right, 16-pin 1.0mm SHD): encoder + sweeper motor + cliff + bumper signals
- **J24/J27** (2-pin 2.0mm PH): left/right motor power only

The STM32F103VCT6 handles real-time motor PWM, encoder counting (timer inputs),
and safety sensor (cliff/bumper) GPIO. The A33 handles WiFi, navigation, and
high-level logic.

## Sources

- feliggs/RoborockS5Hardware: https://github.com/feliggs/RoborockS5Hardware (CC0-1.0)
- Board model: Ruby_S Main-B V3 (confirmed via spare parts listing)
- codetiger/VacuumRobot (CRL-200S, for architecture comparison): https://github.com/codetiger/VacuumRobot
