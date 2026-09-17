# ESP32-P4 (experimental compute + safety track)

A home for **ESP32-P4** experiments on OOMWOO: can a ~$4 dual-core RISC-V MCU
with 32 MB in-package PSRAM run onboard 2D SLAM, and what safety architecture
would a P4-based robot need? This is an **alternative compute path**, deliberately
separate from the consumer ROS 2 profile — see
[compute-benchmark](../compute-benchmark) for the Pi 4 / CM4 onboard-ROS 2 work.

> **Status — exploratory.** A *bounded feasibility experiment*, not a committed
> direction. The goal is a measured yes/no, not a port. Develop against a
> recorded dataset and the official Espressif dev board; coordinate in
> [issue #18](https://github.com/makerspet/oomwoo/issues/18) and the
> [discussions](https://github.com/makerspet/oomwoo/discussions).

## Why P4 is interesting

- **Cheap and capable-ish.** ESP32-P4: dual 400 MHz RISC-V HP cores + a 40 MHz
  LP core, up to **32 MB in-package PSRAM**, ~**$4** including the PSRAM.
- **Storage + I/O.** UHS-I 4-lane SDIO (fast SD/eMMC — a large map could be paged
  into PSRAM by region), dedicated CSI camera and DSI display pins *outside* the
  GPIO range, and a hardware H.264 encoder (up to 1080p/30) that can consume the
  CSI stream with little CPU load.
- **Pairs with a radio coprocessor.** ESP32-C5/C6 for Wi-Fi/BLE and 802.15.4
  (Matter over Thread).

## Why it is *not* a drop-in for the consumer profile

- **No NPU.** Modern vacuums lean on an NPU for real-time camera-based obstacle
  avoidance / object recognition; the P4 has none. (It can run small NNs in
  software with AI-acceleration instructions — e.g. a pedestrian-detection demo —
  but that is not an NPU.) OOMWOO's current vision plan is **two OV5647 stereo
  NIR wide-FoV cameras** (stereo depth + object recognition) — exactly the
  real-time workload an NPU normally accelerates, so whether the P4 can carry it
  is an open question (below).
- **Not enough GPIO for the MCU role.** The base controller needs 50+ GPIO (see
  the tentative pin list in issue #18). A TCA9554-class I²C IO expander adds
  *slow* binary in/out only; PWM, encoders, and fast interfaces must stay on a
  real MCU.
- **PSRAM is not free RAM.** Espressif's docs note accesses larger than the cache
  fall back to PSRAM speed, and task stacks live in internal RAM by default — so
  both *capacity* and *effective real-time bandwidth* must be measured, not
  assumed.
- **Hackability.** OOMWOO declares ROS 2 support so SLAM/nav is hackable to
  ~1.3M ROS 2 users; a native ESP-IDF SLAM firmware is hackable to ESP32 firmware
  specialists only. A P4 SLAM firmware is an *option*, not the official path.

## The bounded experiment

Prove or disprove "P4 can run onboard 2D SLAM" with one apples-to-apples test —
**do not port ROS 2 / Nav2 first**:

1. **Record one dataset** — LiDAR (5 Hz) + odometry, from the sim or the
   [placeholder Proscenic M6 Pro](https://makerspet.com/blog/tutorial-connect-robot-vacuum-cleaner-to-ros-2-proscenic-m6-pro/).
2. **Replay it two ways** — Pi 4 / CM4 with `slam_toolbox`, and the P4 with a
   *minimal native ESP-IDF 2D SLAM* prototype.
3. **Measure** against a fixed budget:
   - **~200 ms per scan** (5 Hz, no scan dropping);
   - **closes the loop within 32 MB PSRAM** without pathological fragmentation;
   - effective PSRAM bandwidth under cache-miss fallback; internal-RAM stack
     headroom.
4. **Use the official [ESP32-P4X-Function-EV-Board]** so silicon + memory config
   are explicit. Note the **"X"**: the board uses the **ESP32-P4NRW32X** die,
   recommended for new designs; many 3rd-party boards ship the non-"X"
   (older-revision) part.

**Pass** → the P4 earns a deeper prototype. **Fail** → the same dataset gives a
useful lower bound and reinforces the micro-ROS / offboard *educational* profile
without speculation.

## Safety architecture (ADR — to be written)

A P4 is a high-level compute module; it must **not** be in the hard-stop chain.
The direction from the thread is a layered, fail-low design:

1. **External hardware gate / window watchdog** — the final motor-power cutoff.
   Motor enable defaults **OFF** and requires a valid *periodic* signal; the
   passive electrical state (external pull-down / latch) means **stop**, so an
   unpowered or reset system cannot authorize motion.
2. **STM32G473 base controller** (selected) — owns wheel/brush
   PWM, encoders, current sensing, bumper/cliff/wheel-drop, charging supervision,
   and deterministic stop. It services the hardware gate *only* while its full
   safety state is valid. These functions can't be replaced by a gate IC without
   recreating them as a large discrete circuit.
3. **ESP32-P4** — SLAM / navigation / high-level behavior. Its **LP core** may
   monitor an HP heartbeat in shared/LP SRAM and assert a dedicated fail-low
   **`P4_HEALTHY`** line (and reset the HP cores), improving diagnostics — but it
   is *optional* to the hard-stop chain and never issues motor commands.

Related, separate milestone — **OTA update of the safety MCU** (later, after the
hard-stop and base-control firmware are bench-validated): the P4 can drive
`BOOT0` / `nRESET` into the STM32 factory ROM bootloader (AN2606 / AN3155 USART,
or USB DFU), so end users need no ST-Link or STM32 SDK. It must be gated by a
signed, version-checked, immutable STM32 boot stage; motors + charging forced OFF
throughout; dual-bank staging with rollback; and the P4 must not be able to
bypass signature verification. `esp-serial-flasher` is useful host/target prior
art but speaks Espressif ROM protocols, so it needs a separate STM32 backend.

## Open questions

- **Can the P4 perform real-time obstacle avoidance?** OOMWOO plans **two OV5647
  stereo NIR wide-FoV cameras** — stereo depth + object recognition at frame
  rate is the workload that normally needs an NPU/SoC. Can the P4's dual RISC-V
  cores + software AI-acceleration path meet it, or does obstacle avoidance stay
  off the P4 (host-side, or a separate accelerator)? Two OV5647s also need two
  CSI streams (dual-camera interface or a mux) — worth confirming on the P4X.
- Does a minimal native ESP-IDF SLAM meet the 200 ms / 32 MB budget at all?
- Is the LP core enough isolation, or is a dedicated hardware-gate IC still
  required as the final cutoff? (Leaning: keep the external gate regardless.)
- Motor type: 2× BLDC with FOC on the P4 vs. DC. Safety implication — if P4-side
  FOC stops executing, a BLDC coasts to a stationary hold as its fields stop
  rotating, which is not true for DC motors.

## Request for contribution

Add your work under your GitHub username:

```text
contributions/esp32-p4/<your-github-username>/
```

Keep native-SLAM prototypes, board bring-up notes, measurements, and the safety
ADR here. Coordinate scope in
[issue #18](https://github.com/makerspet/oomwoo/issues/18) so tracks don't overlap
with [compute-benchmark](../compute-benchmark),
[recovery-safety](../recovery-safety), or the I/O board work
([io-board-interface](../io-board-interface), [mcu-io-firmware](../mcu-io-firmware)).

[ESP32-P4X-Function-EV-Board]: https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4x-function-ev-board/index.html
