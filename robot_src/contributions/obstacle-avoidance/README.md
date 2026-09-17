# Near-Field Obstacle Avoidance: camera + ToF (ROS2 package)

Stop the robot getting stuck on the things a 2D LiDAR *cannot see*. The turret
LiDAR is blind below its ~10 cm scan plane — cables, socks, shoes, pet bowls,
thresholds — which is the *top real-world complaint* about robot vacuums. This
package adds *near-field forward perception* from a *front camera* and/or a
*multizone Time-of-Flight (ToF) sensor*, detects obstacles in front of the robot,
and publishes them so navigation and recovery can react. Because the physical
robot isn't built yet, the *software* is developed in a *Gazebo simulation*; it is
later re-validated on hardware in the [live-robot-bringup RFC](../live-robot-bringup).

> *Status — ready to start work (forward-looking / experimental).* This is the
> v2 "never gets stuck" direction, not a v1 promise — the v1 fallback for low
> obstacles is the *bumper* ([recovery-safety](../recovery-safety)). Develop the
> software in the Gazebo sim ([urdf-gazebo-sim](../urdf-gazebo-sim)); say so in the
> [discussions](https://github.com/makerspet/oomwoo/discussions) so we can coordinate.

> *Two tracks in this RFC.* (1) A *software* track — the sensor-agnostic ROS2
> package that detects and publishes obstacles (sim-first). (2) A *hardware* track —
> building and benchmarking the candidate front-depth sensors to decide which one
> the robot ships. They meet at a shared obstacle interface: the software must not
> care *which* sensor produced the depth/obstacle data.

# Important References

- [recovery-safety RFC](../recovery-safety) — the v1 bumper-based reaction this augments; a detected obstacle should feed the recovery/avoidance ladder.
- [clean-and-map RFC](../clean-and-map) / [cleaning-jobs RFC](../cleaning-jobs) — coverage/nav that consumes the obstacle signal to slow, steer, or replan.
- [floor-care RFC](../floor-care) — reactive wall/edge *following*: the near-field neighbor to obstacle *avoidance* (follow a boundary vs. steer around it), same reactive-sensing lineage.
- [urdf-gazebo-sim RFC](../urdf-gazebo-sim) — robot URDF; this package needs a *front camera* and a *front ToF* modeled.
- [ROS2 software interfaces](../../docs/SOFTWARE_INTERFACES.md) — shared topic/action/service contract for simulation-first modules.
- [OOMWOO ROS2 development](https://github.com/makerspet/oomwoo-install) — build OOMWOO ROS2 Docker image(s) with your packages.
- [Project discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=) · [Discord](https://discord.gg/3y2JKz5T25)

# Front sensor — three candidate approaches (under evaluation)

The 2D LiDAR handles 360° mapping/localization. This module fills the *below-LiDAR,
forward* gap, which needs a **wide horizontal FoV (target ≥120°)** so obstacles
across the robot's path — not just dead ahead — are seen. There are three
candidates; **which one (or combination) ships is exactly what the hardware track
below is meant to decide.**

### A. Stereo depth — 2× OV5647 + IR pattern illuminator

- Two rolling-shutter cameras with **wide lenses (target 120–160° horizontal)**,
  computing dense depth by disparity; doubles as the semantic camera.
- **Textureless surfaces** (blank walls/floors) give stereo no features → pair it
  with an **840 nm random-dot pattern illuminator** (eye-safe *LED* + engineered
  diffuser or a printed random-dot mask; **not** a laser projector — avoids the
  Class-1/DOE-interlock burden). Short range is fine; that's all we need.
- **Sync is mandatory and not automatic**: two *independent* OV5647 on the two CSI
  ports free-run and won't genlock. Drive frame-sync over a **shared I²C control
  line, broadcasting a single write** to both sensors (+ FREX/snapshot mode). Naive
  "2× cameras" stereo is a known trap.
- Cost: uses **both CM4/CM5 CSI lanes** and real depth-matching compute (competes
  with SLAM/NPU on the Pi). Widest FoV, richest data.

### B. Multizone ToF — 2× VL53L7CX/CH, angled ±30°

- Each VL53L7CX/CH is an 8×8 multizone ToF with **~60° horizontal FoV** (the "90°"
  in the datasheet is *diagonal*). Place **two side-by-side, one aimed 30° left, one
  30° right → ~120° horizontal total.**
- **I²C, not CSI** → frees both camera lanes, near-zero compute (128 range zones,
  not image frames). **Class-1 eye-safe** (VCSEL sealed in the module). **Works in
  total darkness and on textureless/blank surfaces** — no illuminator, no pattern.
- Sourcing: bare **VL53L7CX/CH ~$5.50 at DigiKey**; order **unpopulated carrier
  PCBs**, populate and **reflow at home**. ⚠️ It's an **optical LGA** — keep flux
  off the aperture and follow ST's reflow profile (this part has a cover-glass;
  no-clean, no post-wash near the window).
- Trade-off: coarse (8×8 per sensor) — great for *"something is there, this far,
  this tall"*, poor at *shape/contour* and no semantics.

### C. Both combined

- ToF for reliable, dark/blank-proof **geometry**; stereo (or a single wide camera)
  for **wide semantics + dense shape**. Fuse them. Most capable, most cost/compute.

> *Sim modeling.* Gazebo has no VL53L7CX model — approximate a multizone ToF with a
> *low-resolution depth camera* or a small *ray grid* over the same FoV/zones, and
> the camera(s) with standard Gazebo camera sensors. Add sensors *inside your
> submission* and propose folding the stable ones back into
> [urdf-gazebo-sim](../urdf-gazebo-sim) later — please don't rewrite that RFC.

# Hardware track — build & benchmark the candidates

**Goal:** determine, on real obstacles, which of **A / B / C** best detects the
near-field, below-LiDAR obstacles — and whether one suffices *alone* or they must
be *combined*. This produces a data-backed sensor decision for the BOM.

### Build notes

- **B (2× ToF)** is the cheapest, lowest-risk build (I²C, no CSI, no illuminator,
  eye-safe, dark/blank-proof) — a natural *first* thing to characterize. Mount two
  carriers at ±30°, verify the seam/overlap near boresight, and publish combined
  zones.
- **A (stereo)** needs the sync harness (shared-I²C broadcast + FREX) and the
  **840 nm illuminator**; validate *passive* stereo first (textured obstacles) then
  measure how much the pattern illuminator helps on blank floors/walls.
- Whichever is characterized, publish the **same obstacle interface** (below) so the
  software track and Nav2 don't care which sensor won.

### Test protocol — obstacles (primary)

Model the obstacle set on **[how Vacuum Wars tests obstacle avoidance](https://vacuumwars.com/how-vacuum-wars-tests-robot-vacuums/)**
so results are comparable to commercial robots:

- ~**24 objects** across categories, scored **1 point per object avoided** (their
  scale), run at max sensitivity. Categories include **cords/cables, socks, shoes,
  simulated pet waste**, plus a **"torture" pass** mixing everything — deliberately
  including **objects unlikely to be in any trained image library** (this is where
  ToF's geometry-only detection can *beat* a camera that relies on recognition).
- Include their **rug cases**: a *solid-color rug with same-color items* and a
  *patterned rug with a contrasting item* — both stress depth/segmentation.
- Add OOMWOO-specific cases the reviews under-cover: a **cable lying flat**, a **~2 cm
  threshold**, and a **cliff/edge** in-frame.
- **Run every case in two lighting conditions — normal light and full darkness** —
  and on **≥2 floor types**. Darkness and blank surfaces are the decisive
  differentiators: ToF is lighting-independent; stereo needs the IR illuminator and
  texture.

### Test protocol — object recognition (secondary)

Lower priority than raw obstacle avoidance. Where a camera is present, measure
**classification accuracy** for the categories above (feeds semantic behaviors like
"never smear pet waste — stop and alert"). A miss here should still *avoid* the
object as an unknown obstacle; recognition only changes the *reaction*, not the
*detection*.

### Metrics to report (per approach)

- **Detection / avoidance rate** per obstacle type (true-positive %), and the
  aggregate Vacuum-Wars-style score (/24).
- **False-positive rate** on clear floor (does normal cleaning get disrupted?).
- **Min detection distance & reaction distance**; blind spots / FoV coverage across
  the ≥120° target (map the seam between the two ToFs / stereo edges).
- **Darkness** and **blank-surface** performance deltas vs. normal.
- **Depth accuracy** vs. ground truth; **latency / frame rate**.
- **Pi compute load** (CPU/NPU %) — a real cost for stereo.
- **Integration cost**: CSI lanes used, power, BOM $, build/reflow difficulty.

### Decision framework

Pick the approach that clears the acceptance criteria at the **lowest cost +
compute + integration burden**. Expected shape of the answer: **B (2× ToF) as the
robust, cheap, dark/blank-proof geometry baseline**; add **A (camera/stereo)** if
semantic recognition or shape/contour proves necessary → i.e. **C** for the premium
tier. Let the data decide; publish it so the choice is reviewable.

# ROS2 obstacle interface (shared by both tracks)

Whichever sensor is used, publish obstacles in `base_link`: **distance + bearing /
coordinates**, plus `object_type` (`unknown` in Phase 1; typed in Phase 2), and/or
a `PointCloud2` / `Range`-grid the nav stack can consume as a costmap layer. Feed
[recovery-safety](../recovery-safety) / navigation — *coordinate the interface
rather than driving `cmd_vel` directly* where a nav layer owns it.

# Request for Contribution — Instructions

- *add the sensors to the sim robot*
  - front camera (image topic) and front ToF (zone/range topic), co-located and
    calibrated to a shared front frame
  - post in [Project Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=) to let everyone know you're working on it, and post progress
- *Phase 1 — detect, don't classify*
  - from the ToF (primary) and camera, detect *any* obstacle in the forward
    near-field and publish it: *distance + bearing / coordinates in `base_link`*,
    with *object type = unknown*. Free-space vs blocked is enough to be useful.
  - the point is coverage of the *below-LiDAR* zone the 2D scan misses
- *Phase 2 — classify (later)*
  - run object detection on the camera, fuse with the ToF, and publish *typed*
    detections (cable, sock, pet-waste, threshold, furniture …) with confidence.
    This is the NPU-relevant workload; keep it optional/pluggable.
- *react*
  - feed detections to navigation / [recovery-safety](../recovery-safety): slow,
    steer around, or stop-and-alert (e.g. never smear pet waste). Coordinate the
    interface rather than driving `cmd_vel` directly from here where a nav layer owns it.
- *(hardware track)* build a candidate front sensor (A/B/C above), run the test
  protocol, and publish the metrics table so the sensor choice is data-backed.
  Coordinate in [Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=)
  before ordering parts so effort isn't duplicated.
- test it well
  - build worlds with *low obstacles the LiDAR can't see* (cable on the floor,
    sock, shoe, a 2 cm threshold) and verify they are detected and avoided
  - verify the robot still cleans normally when the near-field is clear (low false-positive rate)
- regression tests (headless, CI-friendly)
  - detection rate on a set of placed low obstacles (true-positive %)
  - false-positive rate on clear floor
  - "did not run over the flagged obstacle" success rate
- submit a PR (pull request) linking your ROS2 package(s) + sim sensor/world additions
  - instructions, documentation — install, run, configure, troubleshoot, test results
  - videos of low-obstacle detection and avoidance
  - announce your submission in [Project Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=)
- iterate with review
- TBD, expect the RFC to evolve

## Acceptance criteria

Objective, measurable. Examples:
- *Below-LiDAR obstacles* (cable, sock, low threshold) are *detected* and
  published with distance/bearing, and the robot *avoids running them over*
- *Low false-positive rate* on clear floor — normal cleaning is not disrupted
- Phase 1 works *without* classification; Phase 2 classification is optional/pluggable
- *Regression tests* pass (detection %, false-positive %, avoidance success), runnable headless in CI
- Works in at least one world seeded with LiDAR-invisible obstacles
- *(hardware track)* a candidate is characterized against the Vacuum-Wars-style
  protocol *in light and darkness*, with the metrics table filled in — enough to
  compare it against the other approaches
- Documented and reliably reproducible by someone else
- TBD, expect criteria to evolve

The maintainer selects among compliant candidates using these criteria. Multiple
attempts are welcome and useful even if not selected — modules are swappable, and
a non-selected design is still a valid learning exercise and a fallback.
