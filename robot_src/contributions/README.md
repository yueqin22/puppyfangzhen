# OOMWOO Contributions — RFC board

Each folder here is a **Request for Contribution (RFC)** — a self-contained module
(software, firmware, hardware, or procurement) you can pick up and build. Submit
your work under `contributions/<rfc>/<your-github-username>/`. New here? Read
[CONTRIBUTING](../docs/CONTRIBUTING.md), and see the
[RFC lifecycle](../docs/CONTRIBUTING.md#rfc-lifecycle) for how RFCs progress and
retire.

**Status legend** — *Active:* `exploratory` → `ready to start work` /
`design-first` → `in progress`. *Retired* (closed to new contributions, but
**kept in place** for provenance): `completed` · `superseded` · `descoped` ·
`merged`. Each RFC's own `> **Status —**` line is the source of truth; this table
is the at-a-glance view.

## Active

| RFC | What it is | Status |
|-----|------------|--------|
| [clean-and-map](clean-and-map) | First clean: coverage + SLAM + exploration | ready to start work |
| [nav-localize](nav-localize) | Localization & navigation on a known map | ready to start work |
| [floor-care](floor-care) | Wall/edge following, surfaces, mop lift | ready to start work |
| [cleaning-jobs](cleaning-jobs) | Cleaning modes, zones, job orchestration | ready to start work |
| [recovery-safety](recovery-safety) | Recovery behaviors & safety | ready to start work |
| [dock-cycle](dock-cycle) | Undock, dock, recharge & station services | ready to start work |
| [obstacle-avoidance](obstacle-avoidance) | Near-field camera + ToF avoidance | ready (experimental) |
| [stair-climbing](stair-climbing) | Multi-floor: stair climbing (drive-in exoskeleton) | exploratory |
| [control-app](control-app) | Control app & UX | ready (design track) |
| [live-robot-bringup](live-robot-bringup) | Live robot bring-up & validation | ready to start work |
| [health-monitor](health-monitor) | Stack health monitor & software watchdog | design-first, ready |
| [compute-benchmark](compute-benchmark) | Compute benchmark & memory reduction | in progress |
| [esp32-p4](esp32-p4) | ESP32-P4 experimental compute + safety track | exploratory |
| [mcu-io-firmware](mcu-io-firmware) | MCU I/O board firmware (STM32G473) | ready to start work |
| [io-board-interface](io-board-interface) | I/O board software interface | active |
| [urdf-gazebo-sim](urdf-gazebo-sim) | oomwoo URDF + Gazebo simulation | active |
| [mac-dev-env](mac-dev-env) | macOS (Apple Silicon) dev environment (pixi) | in progress (experimental) |
| [io-pcb](io-pcb) | I/O + motor-driver PCB (KiCad) | active |
| [dust-bin](dust-bin) | Dust bin (mechanical module) | active |
| [vacuum-fan](vacuum-fan) | Blower fan assembly (mechanical module) | active |
| [part-specs](part-specs) | Procure part specs & datasheets | active |
| [source-3d-models](source-3d-models) | Source 3D models (STEP) for BOM parts | active |

## Retired

Closed to new contributions, **kept in place** for provenance — the per-contributor
submissions and the learning stay put. _None yet._

When an RFC is retired it gets a banner at the top of its `README.md`, its
`Status` set to a retired state, a row here, and a link to where the work went.
See the [RFC lifecycle](../docs/CONTRIBUTING.md#rfc-lifecycle) for the steps.

| RFC | Retired as | Where it went / successor |
|-----|------------|---------------------------|
| _—_ | _—_ | _—_ |
