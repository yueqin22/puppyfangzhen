# Dock Cycle: undock, dock, recharge & station services (ROS2 package)

Everything around the charging/service *dock*. The robot must *undock* at the
start of a job, *return to the dock* (on completion or on low battery), *dock
precisely* to make charge contact, trigger *dock station services* (recharge,
and — where the dock supports them — auto-empty and mop-wash), and *find the dock
when it is lost* and relocalization has already failed. Because the physical robot
isn't built yet, this is a *Gazebo simulation*; it is later re-validated on
hardware in the [live-robot-bringup RFC](../live-robot-bringup).

> *Status — ready to start work.* No need to wait for OOMWOO hardware — develop it in the
> Gazebo sim ([urdf-gazebo-sim](../urdf-gazebo-sim)) or on the real
> [placeholder Proscenic M6 Pro](https://makerspet.com/blog/tutorial-connect-robot-vacuum-cleaner-to-ros-2-proscenic-m6-pro/).
> Say so in the [discussions](https://github.com/makerspet/oomwoo/discussions) so we can coordinate.

# Important References
- [nav-localize RFC](../nav-localize) — localization + Nav2; the find-the-dock fallback is invoked when its relocalization fails.
- [clean-and-map RFC](../clean-and-map) and [cleaning-jobs RFC](../cleaning-jobs) — trigger return-to-dock for recharge / auto-empty / mop-wash mid-job.
- [urdf-gazebo-sim RFC](../urdf-gazebo-sim) — robot URDF and Gazebo world(s) to model the dock in.
- [ROS2 software interfaces](../../docs/SOFTWARE_INTERFACES.md) — shared topic/action/service contract for simulation-first modules.
- Model a *generic basic charging dock* (charge contacts + a detectable marker). Exact dock geometry is TBD — the old teardown reference vacuum is no longer used.
- [OOMWOO ROS2 development](https://github.com/makerspet/oomwoo-install) — build OOMWOO ROS2 Docker image(s) with your packages.
- Nav2 docking (`opennav_docking`) is a good starting point for precise approach.
- [wennycooper/Auto-Docking-Design](https://github.com/wennycooper/Auto-Docking-Design) — reference IR homing design (dock IR emitter + baffled robot-side receivers).
- [Project discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=)
- [Discord server](https://discord.gg/3y2JKz5T25)

# Reference homing & dock-detection design (a good starting point)

Use `opennav_docking` for the docking *state machine*; this design is how you
*produce the dock pose* it needs, mirroring how consumer vacuums actually home.

- *In the dock:* an *IR homing beacon*. Start minimal — one *modulated* IR
  emitter (see [wennycooper/Auto-Docking-Design](https://github.com/wennycooper/Auto-Docking-Design)),
  enough for a centered final approach. A more robust dock (later) uses *coded
  left/right "buoy" beams + an omni "force-field" beam* (iRobot-style) so the
  robot can tell which side of the dock it is on from a distance.
- *On the robot, final approach:* *2× front IR receivers separated by a baffle* —
  the differential signal steers the robot to center on the beacon.
- *On the robot, search/acquisition:* *1× left + 1× right dock-detection IR
  receiver* (wider field of view) so the robot can detect the dock beacon and
  turn toward it *even when the dock location is unknown* (the find-the-dock case).

> *Shared hardware — coordinate with [floor-care](../floor-care).* These left/right
> side IR receivers do double duty: passive *dock detection* here, and active
> *wall following* in floor-care (same receiver, plus a side IR emitter, sensing
> the robot's own reflection). This matches the io-board GPIO (`Dock IR`,
> `Side proximity IR L/R`, `IR LED PWM`) and the merged `part-specs` (TSOP38238 +
> 940 nm). Because the beams are *modulated*, keep the dock beacon and the
> wall-illumination emitter *distinguishable* (different mode / time-multiplexed)
> so a wall reflection is never mistaken for the dock.

> *Sim note.* Gazebo has no IR sensor. Simulate the beacon receivers with a small
> *plugin / logical sensor* that computes line-of-sight + bearing (and a
> range-attenuated "strength") to the dock model, and any active proximity IR
> with a short-range narrow *ray* sensor. Add these inside your submission for now
> and propose folding the stable ones back into
> [urdf-gazebo-sim](../urdf-gazebo-sim) later — please don't rewrite that RFC.

# Request for Contribution - Instructions

- model the dock in Gazebo
  - add charge contacts and a *detectable marker* for the final approach (IR beacon, fiducial/AprilTag, or a reflective-intensity pattern the LiDAR can see)
  - model a *battery* (drain during cleaning, charge curve while docked) so low-battery behavior can be tested
  - model a *generic basic dock* (contacts + a detectable marker); exact geometry TBD
  - post in [Project Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=) to let everyone know you're working on it, and post your progress
- undocking
  - safely back / drive out of the dock to a known start pose, then hand off to cleaning
- return-to-dock
  - navigate to the dock vicinity, triggered by *job complete* OR *low battery / full bin / mop needs washing* mid-job (coordinate with [cleaning-jobs](../cleaning-jobs))
- precise docking
  - final approach using the dock marker; align to the charge contacts; *confirm charging started*
- dock station services
  - once docked, trigger and wait for *recharge*, and — where the dock supports them — *auto-empty* and *mop-wash*; report completion so the job can resume
- find-the-dock-when-lost
  - fallback used when [nav-localize](../nav-localize) relocalization has *failed*: run a search pattern to reacquire the dock marker, dock, and re-establish pose from the known dock location
- test it well
  - start from *various poses and battery levels*; verify undock → clean → return → dock → service → resume
  - *kidnap* the robot so relocalization fails and verify find-the-dock recovers it
- regression tests (headless, CI-friendly)
  - docking success rate and final alignment error
  - find-the-dock success rate from random lost poses
  - low-battery return-then-resume completes without losing the job
- submit a PR (pull request) to `contributions/dock-cycle/<your-github-username>/`
  - link to ROS2 package(s) and the dock model / world additions
  - instructions, documentation - how to install, run, configure, troubleshoot, test results
  - videos of docking, find-the-dock, and low-battery return-and-resume
  - announce your submission in [Project Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=)
- iterate with review
- TBD, expect the RFC to evolve

## Acceptance criteria

Objective, measurable. Examples:
- Robot *undocks* reliably to a known start pose
- Robot *returns and docks precisely*, making charge contact, from various poses (alignment within a stated tolerance)
- *Station services* (recharge; auto-empty / mop-wash where modeled) are triggered, completed, and reported
- *Low-battery mid-job* triggers return-to-dock and the job resumes after charging
- *Find-the-dock-when-lost* recovers a kidnapped robot after relocalization fails
- *Regression tests* pass (dock success, alignment error, find-the-dock success), runnable headless in CI
- Documented and reliably reproducible by someone else
- TBD, expect criteria to evolve

The maintainer selects among compliant candidates using these criteria. Multiple
attempts are welcome and useful even if not selected — modules are swappable, and
a non-selected design is still a valid learning exercise and a fallback.
