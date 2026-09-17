# MCU I/O Board Firmware (STM32G473 — Arduino + FreeRTOS)

The firmware that runs on the OOMWOO [I/O board](https://github.com/makerspet/oomwoo-io-board)
MCU. In OOMWOO's CPU/MCU split ([ARCHITECTURE.md §5.4](../../docs/ARCHITECTURE.md)),
the *CPU* (CM4/CM5-class module) runs ROS2/Nav2/SLAM, and the *MCU* on the I/O
board owns *motors, encoders, sensors, battery charging, and hard safety*. This
module is that MCU firmware. Its defining constraint: *safety must never depend on
Linux/ROS2* — and, per the architecture below, it must never depend on the friendly
Arduino layer either.

The full design lives in the firmware repo — this RFC is the request for
contribution and the acceptance bar.

> *Status — ready to start work.* The repo exists but has *no code yet*. Build it
> on a *Nucleo-G474* dev board now; move to the real board when it's fabbed. Say so
> in the [discussions](https://github.com/makerspet/oomwoo/discussions) so we can
> coordinate.

# Important References

- *Firmware repo + full spec* — [makerspet/oomwoo-io-firmware](https://github.com/makerspet/oomwoo-io-firmware).
  The repo README is the detailed architecture, peripheral map, and milestones; read it first.
- *Board spec (authoritative)* — [oomwoo-io-board SPEC.md](https://github.com/makerspet/oomwoo-io-board/blob/main/docs/SPEC.md)
  — the motors, sensors, charging, and GPIO budget the firmware must serve (work in
  progress; note its open TODOs, e.g. the GPIO 36/46 bumper-label question).
- *CPU ↔ MCU serial contract* — [io-board-interface RFC](../io-board-interface) —
  the custom serial framing, command/telemetry set, and health/watchdog handshake.
- *System architecture* — [ARCHITECTURE.md §5.4](../../docs/ARCHITECTURE.md) — the CPU/MCU split and safety rationale.
- *Simulated MCU link* — the [oomwoo-install](https://github.com/makerspet/oomwoo-install)
  serial stub lets the CPU side be developed/tested before the board exists.
- [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32) · [STM32FreeRTOS](https://github.com/stm32duino/STM32FreeRTOS)
- [Project discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=) · [Discord](https://discord.gg/3y2JKz5T25)

# The architecture, in one rule

Arduino-*or*-real-time-safety is a false choice; we get both by *layering*, so the
layer a contributor touches is not the layer that keeps the robot safe:

- *Layer 3 — Arduino (STM32duino) API:* the contributor-friendly surface where new
  behaviours, features, and peripheral bring-up happen.
- *Layer 2 — FreeRTOS tasks* (static allocation): comms, control, telemetry,
  charging, and safety supervisors; watchdog-fed, bounded reaction times.
- *Layer 1 — HAL/timer-ISR real-time core:* motor control, the hard-safety cutoffs,
  and the CPU watchdog. Maintainer-owned, safety-reviewed.

*The rule:* the safety and motor-control core is *structurally isolated* from the
Arduino layer — a bug or infinite loop in a contributor's sketch *cannot* defeat a
cliff-stop, an overcurrent cutoff, or the CPU watchdog, because those live in
interrupts and a hardware watchdog the upper layers can't starve.

# Request for Contribution — Instructions

Phased bring-up (details and the peripheral map are in the firmware repo README):

- *Blink + SWD + serial echo* on a G473 dev board.
- *CPU serial link* — implement the [io-board-interface](../io-board-interface)
  framing + health/watchdog handshake; loopback tests green.
- *One drive motor, closed loop* — H-bridge PWM + encoder capture + velocity PID in
  the real-time core (the pattern every other motor follows).
- *All actuators* — suction fan (BLDC + FG), main/side brush, LiDAR spin, water
  pump, mop motors/servos — each with current sense.
- *All sensors* — cliff / dock / side-proximity IR (ADC), bumpers, wheel-drop, IMU
  (SPI), current channels.
- *Safety layer* — ISR-level cliff/bumper/wheel-drop stop, per-motor overcurrent
  limiting, IWDG, CPU watchdog/reset; *measure and document* each cutoff's
  worst-case reaction time; include a hazard note.
- *Charging supervisor* — power-path charger control, 0.5C cap, input DPM, graceful
  "insufficient charger" handling.
- *Integration* — run end-to-end against the CPU (or the simulated MCU serial tool).
- Contribute in the firmware repo; announce progress in [Project Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=).
- iterate with review
- TBD, expect the RFC to evolve

*Safety-critical code requires maintainer safety review before merge* (over-current,
thermal, short, mechanical pinch — include a hazard note).

## Acceptance criteria

Objective, measurable. Examples:
- *Deterministic real-time core* — measured, documented worst-case reaction time for
  each safety cutoff.
- *Safety is layer-independent* — a deliberately hung Arduino-level task must *not*
  defeat a cliff-stop, overcurrent cutoff, or the CPU watchdog; demonstrate it.
- *Implements the [io-board-interface](../io-board-interface) serial contract* —
  loopback + integration tested.
- *Every actuator and sensor exercised* on the bench, documented, reproducible by
  someone else.
- *Charging behaves per spec* — 0.5C cap held; graceful degradation on a weak charger.
- *Safety-critical code passed maintainer safety review* with a hazard note.
- TBD, expect criteria to evolve.

The maintainer selects among compliant candidates using these criteria. Multiple
attempts are welcome and useful even if not selected — a non-selected design is
still a valid learning exercise and a fallback.
