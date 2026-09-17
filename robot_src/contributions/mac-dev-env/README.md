# macOS dev environment (Apple Silicon)

A one-command, host-isolated developer setup for the OOMWOO simulation + ROS 2
stack **on macOS** — clone, run one install command, and drive a simulated robot.
This RFC owns the **Mac** piece of the developer story: everyone gets the same
toolbox from a locked list of versions, isolated from whatever is already on the
machine.

> **Status — in progress (experimental).** There is an initial working recipe:
> [@DingoOz](DingoOz)'s [pixi](https://pixi.sh) + [RoboStack](https://robostack.github.io)
> setup that brings up the Gazebo sim with teleop on **Apple Silicon** (and, as a
> bonus, Linux and Pi 5). It lives on a branch and wants Mac testing and
> hardening — see [DingoOz/](DingoOz) and the
> [discussions](https://github.com/makerspet/oomwoo/discussions).

# Why this exists — the Mac gap

The dev/runtime environments are already owned elsewhere:

- **Linux + Windows dev** → [oomwoo-install](https://github.com/makerspet/oomwoo-install)'s
  Docker image.
- **On-robot runtime (Pi)** → oomwoo-install's apt-based install.

What's missing is **macOS**: there is no easy Docker/apt path to a working ROS 2 +
Gazebo build on Apple Silicon, so Mac users are effectively shut out of
contributing to the sim. This RFC fills that gap. It complements oomwoo-install
rather than replacing it — Linux/Windows/Pi stay there; the Mac path lives here.

Two properties the Mac path should keep, beyond just "runs":

- *Host-isolated.* It must not touch or depend on a system Python/Homebrew/ROS; an
  existing install is left untouched.
- *Reproducible.* A committed lockfile pins the exact versions so a build is
  reproducible months later.

# Recommended approach — pixi + RoboStack

[pixi](https://pixi.sh) installs a project's entire toolbox (here: ROS 2, Gazebo,
and their dependencies) into a self-contained folder from a locked manifest,
without touching anything already installed. The ROS 2 binaries come from
[RoboStack](https://robostack.github.io) (ROS packaged on conda-forge), which is
the key enabler: it's what provides ROS 2 for **Apple Silicon** at all, where no
first-party ROS binaries exist. The same manifest also resolves on Linux and Pi,
so the Mac recipe happens to be cross-platform — a bonus, not the remit.

The RFC is open to other reproducible macOS approaches (e.g. Nix, a hand-rolled
conda env, a Lima/Docker VM), but pixi is the direction with a working
submission — make the case if you bring an alternative.

# Considerations / open questions

- *Gazebo rendering on Apple Silicon.* This is the sharp edge for a Mac sim
  (Metal-backed GL/rendering, headless vs. windowed). Documenting exactly what
  works — and any flags — is the most valuable part of a Mac submission.
- *ROS distro alignment.* The project's canonical branches standardize on **ROS 2
  Jazzy**; the initial recipe targets **ROS 2 Lyrical** (what RoboStack currently
  ships cleanly for Apple Silicon). Whether to pin to Jazzy for parity, or track a
  newer distro and accept the drift, is an open decision — call it out and justify
  it in a submission.
- *Keeping it from bit-rotting.* A locked env still builds against the `oomwoo-one`
  description/launch files. A small CI job on a macOS runner that runs
  `pixi install` + `pixi run build` (and a headless `pixi run sim` smoke test if a
  runner allows) would catch drift.
- *Scope.* This is a *developer/sim* environment for macOS. It does not target the
  on-robot runtime, and it does not replace the Docker image for Linux/Windows.

# Important References

- [DingoOz/](DingoOz) — the initial pixi + RoboStack recipe (this RFC's first
  submission), verified on Apple Silicon.
- [urdf-gazebo-sim](../urdf-gazebo-sim) — the robot description + Gazebo world this
  environment builds and runs.
- [oomwoo-install](https://github.com/makerspet/oomwoo-install) — owns the Linux +
  Windows dev image and the Pi runtime; this RFC fills its macOS gap.
- [live-robot-bringup](../live-robot-bringup) · [compute-benchmark](../compute-benchmark)
  — the on-hardware / Pi paths (the pixi recipe also runs on Pi 5, a shared target).
- [pixi](https://pixi.sh) · [RoboStack](https://robostack.github.io) — the tools
  the recommended approach builds on.
- [Project discussions](https://github.com/makerspet/oomwoo/discussions) ·
  [Discord](https://discord.gg/3y2JKz5T25)

# Request for Contribution — Instructions

Per the [contribution model](../../docs/CONTRIBUTING.md#how-contributions-are-structured),
the environment *code* (pixi manifest, lockfile, tasks) lives in **your** repo/branch;
submit a **link** plus in-tree notes:

- *bring it up on a Mac and report* — run the documented steps on Apple Silicon
  and report whether the sim + teleop come up cleanly, with macOS version, chip,
  ROS distro, and any fixes. **Mac first-run reports are the whole point** of this
  RFC.
- *nail the rendering* — get Gazebo rendering reliably on Apple Silicon and
  document the exact recipe / flags (the likeliest blocker).
- *harden it* — pin/refresh the lockfile, smooth rough edges, note Mac-specific
  gotchas.
- *align the distro* — make the Jazzy-vs-Lyrical call (above) and, if aligning to
  Jazzy, show it building against the canonical description.
- *keep it honest* — propose a macOS-runner CI job that runs `pixi install` +
  `pixi run build` so the env can't silently rot.

Submit a PR adding your notes under
`contributions/mac-dev-env/<your-github-username>/` — a short README with your
repo/branch link, the exact install + run + test steps, your macOS matrix (OS +
chip), and any videos. Announce it in
[Project Discussions](https://github.com/makerspet/oomwoo/discussions).

# Acceptance criteria

- The documented steps bring up the **Gazebo sim with a drivable robot + working
  teleop** from a clean **Apple Silicon Mac**, with rendering working.
- The setup is **reproducible** (a committed lockfile) and **host-isolated** (does
  not touch or require a system ROS/Python/Homebrew).
- A stated position on **ROS distro** (Jazzy parity vs. newer) and how it builds
  against the `oomwoo-one` description.
- Reproducible by someone else on macOS, with macOS version + chip recorded.
- TBD; expect criteria to evolve as more Macs are validated.

The maintainer selects among compliant candidates using these criteria. Multiple
attempts are welcome — a non-selected approach is still a useful fallback.
