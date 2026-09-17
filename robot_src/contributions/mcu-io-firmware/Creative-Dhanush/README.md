# MCU I/O Firmware — Creative-Dhanush

**The CPU↔MCU link runs end to end today, on a laptop, with no hardware.** A ROS 2 stack drives the
real MCU safety logic over the real binary wire format — and when the ROS 2 side is killed, the MCU
stops the motors by itself.

Two repos, per this module's contribution model:

| Repo | What | CI |
|---|---|---|
| **[oomwoo-io-firmware](https://github.com/Creative-Dhanush/oomwoo-io-firmware)** | MCU side: frame codec, streaming decoder, safety state machine, link layer, and an MCU simulator | [![firmware CI](https://github.com/Creative-Dhanush/oomwoo-io-firmware/actions/workflows/ci.yml/badge.svg)](https://github.com/Creative-Dhanush/oomwoo-io-firmware/actions/workflows/ci.yml) |
| **[oomwoo-mcu-bridge](https://github.com/Creative-Dhanush/oomwoo-mcu-bridge)** | CPU side: the `oomwoo_mcu_bridge` ROS 2 node [`ros2_mapping.md`](../../io-board-interface/xbattlax/docs/ros2_mapping.md) drafted and named | [![bridge CI](https://github.com/Creative-Dhanush/oomwoo-mcu-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/Creative-Dhanush/oomwoo-mcu-bridge/actions/workflows/ci.yml) |

> **Read this before the rest.** What is proven: the wire format is correct against an independent
> implementation, the parser survives damaged and fragmented input, and the safety policy behaves
> correctly under every specified condition — *on a laptop*. What is **not** proven: anything about
> the real chip. No measured reaction times, no interrupt priorities, no evidence that a hung
> Arduino-level task cannot defeat a cutoff. That is [milestone 6](../README.md) and it needs a
> Nucleo. **This is the foundation under the safety system, not the safety system.**

## See it work

Needs Linux (the simulator uses `openpty`) and ROS 2 Jazzy. Run both clones side by side:

```bash
# MCU side — the safety core as a host binary, on a pseudo-terminal
git clone https://github.com/Creative-Dhanush/oomwoo-io-firmware
pip install -U platformio
(cd oomwoo-io-firmware && pio run -e native_sim)

# CPU side — the ROS 2 bridge
git clone https://github.com/Creative-Dhanush/oomwoo-mcu-bridge
cd oomwoo-mcu-bridge && colcon build && source install/setup.bash

ros2 launch oomwoo_mcu_bridge demo.launch.py \
    sim_binary:="$PWD/../oomwoo-io-firmware/.pio/build/native_sim/program"
```

The launch file starts the simulator, creates the pty, and walks the bridge through
`configure` → `activate`, so the heartbeat is running and telemetry is flowing by the time it
settles. `ros2 lifecycle get /oomwoo_mcu_bridge` should say `active`.

Then drive it, break it, and watch who stops the robot:

| Do this | What happens |
|---|---|
| `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | wheels turn, `/joint_states` and `/odom` advance |
| type `cliff left on` into the simulator's stdin | `/oomwoo/io/cliff` → 1, wheels stop, **`/cmd_vel` cannot override it** |
| type `cliff left off` | still stopped — a cliff latch does not self-clear |
| `ros2 service call /oomwoo/io/clear_faults std_srvs/srv/Trigger` | fault releases, motion resumes |
| **`kill` the bridge** | heartbeat stops → **the MCU stops the motors on its own**, with nothing in ROS 2 asking it to |

That last row is the design, not a fallback.

## What is running

```
  Nav2 / recovery / jobs                       ow_sim_mcu  (today, a laptop)
          │                                    real I/O board (later, unchanged protocol)
          ▼                                              ▲
   oomwoo_mcu_bridge  ──── OW binary frames, CRC-16 ──────┘
          │                  (pty now, UART later)
          ▼
   /joint_states  /odom  /battery_state  /oomwoo/io/*  /diagnostics
```

The simulator is a **drop-in replacement** for the newline-delimited-JSON stub in
[oomwoo-install](https://github.com/makerspet/oomwoo-install) (`ubuntu/tools/oomwoo_sim_mcu_serial.py`)
— same `--link`, `--period`, `--battery-mv` — but it speaks the actual contract. And it is not a
*model* of the firmware: it **runs** the firmware. The same `ow_frame` / `ow_stream_decoder` /
`ow_safety_core` translation units that cross-compile for the STM32G473. There is one
implementation, so the simulator and the target cannot drift apart.

## What is implemented

| Module | Purpose |
|---|---|
| [`ow_frame.{h,cpp}`](https://github.com/Creative-Dhanush/oomwoo-io-firmware/blob/main/src/ow_frame.h) | CRC-16/CCITT-FALSE, little-endian helpers, encode/decode for all 12 defined message types — built byte-at-a-time, never `memcpy`'d through a struct, to sidestep the C++/`struct.pack` padding mismatch |
| [`ow_stream_decoder.{h,cpp}`](https://github.com/Creative-Dhanush/oomwoo-io-firmware/blob/main/src/ow_stream_decoder.h) | Fixed-buffer streaming decoder: buffers partial frames across reads, resyncs one byte at a time after corruption |
| [`ow_safety_core.{h,cpp}`](https://github.com/Creative-Dhanush/oomwoo-io-firmware/blob/main/src/ow_safety_core.h) + [design doc](https://github.com/Creative-Dhanush/oomwoo-io-firmware/blob/main/docs/design-safety-core.md) | The safety state machine — heartbeat timeout, setpoint expiry, bumper/cliff/wheel-drop/overcurrent/e-stop, latched faults |
| [`ow_link.{h,cpp}`](https://github.com/Creative-Dhanush/oomwoo-io-firmware/blob/main/src/ow_link.h) | Ties those three into a working MCU: bytes in → policy → framed bytes out, with outbound sequencing. Owns **no transport**, which is why the same code unit-tests with zero I/O, runs on a pty, and cross-builds for the target |
| [`ow_telemetry.{h,cpp}`](https://github.com/Creative-Dhanush/oomwoo-io-firmware/blob/main/src/ow_telemetry.h) | Periodic `FAST_TELEMETRY` at the 50–100 Hz `ros2_mapping.md` asks for |
| [`ow_sim_mcu`](https://github.com/Creative-Dhanush/oomwoo-io-firmware/blob/main/sim/ow_sim_mcu_main.cpp) | The MCU over a pseudo-terminal, with stdin fault injection |
| [`oomwoo_mcu_bridge`](https://github.com/Creative-Dhanush/oomwoo-mcu-bridge) | rclpy lifecycle node: 9 subscribed and 9 published interfaces, the four specified lifecycle states, and the arbitration clamps |

The bridge **imports [xbattlax's `oomwoo_mcu_frame.py`](../../io-board-interface/xbattlax/tools/oomwoo_mcu_frame.py)
unmodified** rather than reimplementing it, so both ends of the link share one definition of the wire
format instead of two independent readings of the contract.

## Evidence

Everything below runs in CI on every push, in ~2 minutes, on a machine that is not mine.

| Check | Result |
|---|---|
| Frame codec, streaming decoder, safety core, **link loopback** (`pio test -e native`, ASan+UBSan) | **54 tests** — 10 / 9 / 15 / 20 |
| Golden frame vectors regenerate byte-identical from the upstream reference | pass |
| Differential fuzz: streaming decoder vs the Python reference | **2000 cases, 0 mismatches** |
| Simulator over a pty, every byte decoded *and re-encoded* by the upstream reference | **6 cases**, byte-exact |
| Bridge framing unit tests | **10** |
| Bridge ↔ simulator end to end, asserting on ROS 2 topics only | **5** |
| `pio run -e nucleo_g474re` cross-build | pass |

[firmware run](https://github.com/Creative-Dhanush/oomwoo-io-firmware/actions/runs/31571515099) ·
[bridge run](https://github.com/Creative-Dhanush/oomwoo-mcu-bridge/actions/runs/31570892334)

Two of those deserve a note, because they are the ones that could have been faked:

- **The wire-format check uses somebody else's implementation as the oracle.** Every frame the
  simulator emits is decoded by the vendored upstream reference, re-encoded, and required to
  reproduce the original bytes exactly. "Speaks the real protocol" is checked, not asserted.
- **The loopback suite goes through the wire.** Unlike the safety-core tests, which hand
  `SafetyCore` an already-decoded frame, every test in `test/test_link` enters and leaves as bytes —
  so a fault in framing, CRC, sequencing, or payload packing fails a test.

From the bridge's CI log, verbatim:

```
/cmd_vel moved the wheels 0.000 -> 6.743 rad, odom x=0.236 m
cliff latched; /cmd_vel could not override it and the sensor clearing did not release it
/oomwoo/io/clear_faults released the latch and motion resumed
heartbeat stopped on deactivate and the MCU stopped the wheels by itself
5/5 passed
```

## The stance: MCU-owned safety stays MCU-owned, and stays dumb on purpose

- The safety core only **stops** actuators on a fault. It does not back up, scan, replan, or decide
  where the robot goes next. Navigation, mapping, and anything needing "complete" context stays on
  the CPU. (This is the split @kaiaai asked about in
  [#49](https://github.com/makerspet/oomwoo/discussions/49) — it is enforced here by the core having
  no way to command motion, only to withhold it.)
- **Latched faults do not self-clear.** Cliff, wheel-drop, and e-stop release only on an explicit
  `CLEAR_LATCHED_FAULT`. A robot that resumes driving the instant a cliff sensor flickers is the
  failure mode this rules out. Wheel-drop is the one deliberate exception — it clears on contact
  returning, matching the contract's literal wording.
- **Overcurrent stops only the affected motor** and reports which one in `SAFETY_EVENT.detail`,
  rather than a blanket stop.
- **Every entry point takes the current time as an argument** and never reads a clock, so a 150 ms
  timeout is tested at exactly 149 ms and 151 ms, deterministically, with no sleeping.
- **No heap, no exceptions, no Arduino headers** in the core. `operator new` is deleted at compile
  time. It builds as plain hosted C++17 and as target firmware from the same source.

## Bring-up progress

Against the [module's milestones](../README.md):

| # | Milestone | State |
|---|---|---|
| 1 | Blink + SWD + serial echo on a G473 dev board | not started — needs hardware |
| 2 | CPU serial link: framing + health/watchdog handshake, **loopback tested** | **done on host** |
| 3 | One drive motor, closed loop | not started |
| 4 | All actuators | not started |
| 5 | All sensors | not started |
| 6 | ISR-level safety, IWDG, **measured** worst-case reaction times | not started — the *policy* exists, the ISR and timing work does not |
| 7 | Charging supervisor | not started |
| 8 | Integration against the CPU or a simulated MCU serial tool | **the simulated-MCU half is done**, and the ROS 2 bridge drives it end to end |

Milestones 3–7 are open and unclaimed. Nothing here forecloses a different firmware framework
either — the wire contract is the interface, so a Zephyr MCU that speaks it works with this bridge
unchanged.

## What is not implemented, stated plainly

- **No STM32 HAL or board bring-up.** All of the above is host-testable logic.
- **No motor PWM, motor-power-enable GPIO, charging, or IWDG.** The safety core emits *intents*
  (stop, `SAFETY_EVENT`, `NACK`); a caller wires them to hardware. No motor load should be connected
  to anything here.
- **No measured worst-case reaction time.** Simulator timing is laptop timing under a preemptive
  scheduler and says nothing about the real part.
- **Encoder and wheel-base constants are placeholders,** labelled as such in the source, because
  [`SPEC.md`](https://github.com/makerspet/oomwoo-io-board/blob/main/docs/SPEC.md) does not fix
  gearbox ratio or encoder resolution yet. Odometry *distance* from the simulator is meaningless;
  direction, sign, and the fact that it stops when safety says stop are not.
- **`POWER_TELEMETRY` and `MCU_DIAGNOSTIC` have no payload layout** in the contract, which is what
  blocks `/battery_state`'s charge fields, `/oomwoo/io/mcu_status`, and the four `/oomwoo/dock_ir/*`
  topics. Those are left unpublished rather than filled with plausible values — a fabricated
  `battery.percentage` on a standard ROS 2 topic is worse than a missing one.

## Open questions, and three contract gaps found by building both ends

These are the useful output of having implemented the CPU side and the MCU side against the same
document. None is a blocker; all three want a decision from someone who owns the contract.

1. **`FAST_TELEMETRY.safety_latched_flags` is one byte, but `SafetyEvent` runs 1..10.**
   `CPU_HEARTBEAT_TIMEOUT` (9) and `ESTOP` (10) cannot appear in the periodic snapshot at all. Both
   ends currently work around it by tracking those two from their `SAFETY_EVENT` frames instead, so
   latch state arrives by two different routes. Widening the field would fix it properly.
2. **A CPU that attaches late misses `MCU_HELLO`,** and the message catalog has no CPU→MCU
   "identify" request to ask again with. On a pseudo-terminal the frame waits in the buffer; on a
   real UART it is simply gone. A `CPU_HELLO` or an identify request would close this.
3. **The reference codec's own `StreamDecoder` loses a frame when a read ends on a lone `O`** —
   it clears its buffer when it cannot find `OW`, discarding a trailing partial magic byte. Both
   implementations here deliberately diverge and keep it, with a test that fails if upstream ever
   fixes it, so the workaround gets deleted rather than outliving its reason.

On the **CPU-heartbeat timeout**: @makers-pet answered in
[#49](https://github.com/makerspet/oomwoo/discussions/49) with **~5 minutes, for CPU boot time**.
That is a different timer from the steady-state heartbeat timeout, which the contract still marks
draft at "100, 150, or 250 ms?" — so the steady-state value is a `Config` field defaulting to 150 ms,
not a hardcoded constant. Boot is handled structurally rather than with a long timeout: actuators
stay disabled after boot or reconnect until a fresh `HEARTBEAT` **and** a fresh `DRIVE_SETPOINT` both
arrive, so a slow-booting CPU cannot produce motion regardless of the number. One open sub-question:
the MCU currently *reports* a heartbeat timeout ~150 ms after boot, while Linux is still coming up.
Harmless, since motion is already gated — but if the intent is to stay quiet until the CPU has ever
spoken, that is a small change and worth deciding.

## References

- Contract this is built against: [io-board-interface](../../io-board-interface) — @xbattlax's
  [serial contract](../../io-board-interface/xbattlax/docs/cpu_mcu_serial_contract.md) and
  [ROS 2 mapping](../../io-board-interface/xbattlax/docs/ros2_mapping.md)
- Board: [oomwoo-io-board](https://github.com/makerspet/oomwoo-io-board) ·
  [SPEC.md](https://github.com/makerspet/oomwoo-io-board/blob/main/docs/SPEC.md)
- ROS 2 interfaces: [SOFTWARE_INTERFACES.md](../../../docs/SOFTWARE_INTERFACES.md) §"Hardware Bridge Draft"
- Status updates and discussion: [#49](https://github.com/makerspet/oomwoo/discussions/49)
