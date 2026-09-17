# RFC Backlog (planned / not-yet-active)

Planned modules that are *not yet active RFCs*. The RFCs ready to work on *now* live in the
[README status table](../README.md#requests-for-contributions), and each active RFC's full
spec is under [contributions/](../contributions). An item here graduates into an active RFC
(its own `contributions/<module>/` folder) once it is unblocked and ready.

*Phase legend:* `MVP` = targeted for the bare-bones build · `P2` = next phase ·
`P3+` = later. *Safety* = requires a maintainer safety review.

## On hold (have an RFC, paused pending sourced parts + a 3D reference design)

| Module | RFC | Notes |
|---|---|---|
| Dust bin 3D design | [dust-bin](../contributions/dust-bin) | Design / print / test the dust bin — waits on sourced parts + a 3D design |
| Blower fan assembly | [vacuum-fan](../contributions/vacuum-fan) | Fans already sourced (see BOM); the volute / gasket housing waits on the 3D design |

## Planned hardware (mechanical design)

Waits on sourced parts + a 3D reference-design sketch, then becomes an active
`contributions/` RFC. Part *specs* and *STEP models* are already active — see
[part-specs](../contributions/part-specs) and [source-3d-models](../contributions/source-3d-models).
The motor-driver / power PCB and battery charging are now the active
[io-pcb](../contributions/io-pcb) RFC.

| Module | ID | Phase | Notes |
|---|---|---|---|
| Chassis / base frame (reference) | `hw-chassis` | MVP | Integration backbone; defines the mechanical interface. Maintainer-owned. |
| Drive-wheel mounts (L/R) | `hw-wheel-mount` | MVP | Mount + suspension around the sourced wheel modules. |
| Compute mount (RPi 5) | `hw-compute-mount` | MVP | Mount + airflow for the Pi 5. |
| LiDAR mount | `hw-lidar-mount` | MVP | Centered turret; parametric for other models. |
| Main brush assembly | `hw-main-brush` | MVP | *Tapered rubber anti-tangle roller* + drive. |
| Bumper (mechanical + switches) | `hw-bumper` | MVP | Contact detection. |
| Cliff-sensor mounts | `hw-cliff` | MVP | IR drop detection at edges / stairs. |
| Top cover / shell | `hw-shell` | MVP | Cosmetic + protective; LiDAR clearance. |
| Wiring harness | `hw-harness` | MVP | Connector pinouts per [io-pcb](../contributions/io-pcb) + [part-specs](../contributions/part-specs). |
| Side brush | `hw-side-brush` | P2 | Edge cleaning. |
| Mop module (dual-spinning) | `hw-mop` | P2 | 3D-printed *dual-spinning* pads; skip the self-washing roller. |
| Charging dock (basic) | `hw-dock` | MVP | *Safety.* Contacts + alignment. |
| Charging dock (auto-empty) | `hw-dock` | P2 | |
| Charging dock (mop wash/dry) | `hw-dock` | P2 | |

## Planned software (later phase)

Foundational software (URDF, sim, SLAM, Nav2, coverage, teleop, sensors) is already covered by
the active RFCs. Remaining, later-phase work:

| Module | ID | Phase | Notes |
|---|---|---|---|
| Diagnostics / telemetry | `sw-diagnostics` | P2 | Health, logs. |
| Regression / CI tests | `sw-regression-tests` | P2 | Sim-based tests gating PRs. |
| Home Assistant integration | `sw-homeassistant` | P2 | MQTT / HA entity, map, control. |
| App runtime layer | `sw-app-runtime` | P3+ | ROS2-agnostic app sandbox. North star. |
| Web UI / dashboard | `sw-webui` | P3+ | Local control + map view. |

## Non-engineering contributions (also wanted)

| Track | Phase | Notes |
|---|---|---|
| 3D-print validation | MVP | Confirm parts print cleanly on common FDM printers. |
| Real-home testing | MVP/P2 | Build and report on real floors. |
| Docs / build guides | MVP | Turn working modules into step-by-step instructions. |
| Posts / videos / demos | ongoing | Content that grows the project (highly valued). |
| SOTA research | ongoing | Best-in-class prior art per module (part of each RFC). |
