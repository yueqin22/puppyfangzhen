# Contributing to OOMWOO

Thanks for your interest. OOMWOO is an open-source robot vacuum you build
yourself, and it's at a very early stage. That's the best time to get involved,
the foundations are still being laid and your input can shape the direction.

The project is built *module by module* so many people can work in parallel.
Browse the [module list in the README](../README.md#requests-for-contributions),
and see [ARCHITECTURE.md](ARCHITECTURE.md) for how the pieces fit together.

## Ways to help right now

You don't need to be a roboticist to contribute:

- *Ideas and feedback* — open a [Discussion](https://github.com/makerspet/oomwoo/discussions)
  about features, design choices, or what would make you build one.
- *Code* — firmware, ROS2 packages, Home Assistant integration.
- *Hardware* — 3D-printable chassis design, mechanical parts, PCB.
- *Documentation* — build guides, wiring diagrams, troubleshooting notes.
- *Testing* — once there's something to build, real-world build reports are gold.
- *Spread the word* — star the repo, share your build, post a demo.

## Getting started

1. *Pick a module.* Browse the
   [module list in the README](../README.md#requests-for-contributions) and choose a
   hardware or software module. Software and simulation modules can start
   immediately; hardware modules wait on the interface specs in
   [ARCHITECTURE.md](ARCHITECTURE.md). Read the module's `README.md` so you know
   the contract.
2. *Start a conversation first.* Claim or ask about the module in its
   [Issue](https://github.com/makerspet/oomwoo/issues) or
   [Discussion](https://github.com/makerspet/oomwoo/discussions) before writing
   code, so we align on the approach and avoid wasted effort.
3. *Build it in your own repo.* For code and simulation modules, develop your
   package in your **own public repository** — you own it, version it, and keep the
   credit. Build against the ROS2 interface contract in
   [SOFTWARE_INTERFACES.md](SOFTWARE_INTERFACES.md) so your work stays interoperable
   with other modules. (Docs and small reference material are handled differently —
   see below.)
4. *Submit a pointer PR.* Add a link to your repo in the module's entry with a
   one-line description. It's small, easy to review, and lets several
   implementations of the same module sit side by side. Keep the PR focused.
5. *Iterate in the open.* Modules are swappable — the best implementation surfaces
   over time, with the maintainer having the last call. A design that isn't
   selected is still a useful fallback.

## How contributions are structured

OOMWOO keeps the core small and lets the community grow around it:

- *Canonical / reference code stays first-party.* [oomwoo-one](https://github.com/makerspet/oomwoo-one)
  (robot description + sim), [oomwoo-install](https://github.com/makerspet/oomwoo-install)
  (dev environment), and the `kaiaai_*` packages are maintained by the project so the
  out-of-the-box build always works.
- *Module implementations (code) live in your repo.* You build a competing
  implementation of a module — a sim, a navigation stack, a behavior — in your own
  repository and submit a *link*. The project features accepted work from the
  module's page, credited to you. When a contribution is *featured*, we pin a
  specific commit or tag (and may fork it into the makerspet org) so the reference
  build stays reproducible even if the upstream repo moves.
- *Docs, specs and small reference material stay in-tree.* Part specifications,
  datasheets, STEP-model sourcing notes, PCB notes, and benchmarks are lightweight
  and best kept alongside the project — contribute those under
  `contributions/<module>/<your-username>/` as files in a PR.

Why links for code? You keep ownership, credit, and freedom to iterate; the project
stays lean and avoids absorbing third-party code and its licensing; and multiple
implementations of a module can coexist and be compared. The shared
[SOFTWARE_INTERFACES.md](SOFTWARE_INTERFACES.md) contract is what keeps
independently-built modules compatible.

## RFC lifecycle

Each `contributions/<rfc>/` folder is an RFC with a `> **Status —**` line; the
[RFC board](../contributions/README.md) is the at-a-glance index. RFCs move
through:

- **Active** — `exploratory` (a bounded experiment) → `ready to start work` /
  `design-first` → `in progress`. Open to new contributions.
- **Retired** — closed to new contributions, for one of four reasons:
  `completed` (a contribution was selected and shipped), `superseded` (replaced by
  a different approach — link the successor), `descoped` (parked / out of scope),
  or `merged` (folded into another RFC — link it).

**Retiring is a maintainer action, and RFCs are retired *in place* — never deleted
or moved.** The submissions under a retired RFC are provenance and fallbacks and
stay put; moving a folder would also break external links (GitHub doesn't redirect
moved paths). To retire an RFC:

1. Add a banner at the **top** of its `README.md` (before the intro) and set its
   `Status` line to the retired state, e.g.:
   `> 🏁 **Retired — completed.** Closed to new contributions. Selected work
   shipped in [<package>](<url>) (contributor: @user, PR #NN). Kept for provenance;
   follow-on work lives in [<successor-rfc>](../<successor-rfc>).`
2. Move its row to **Retired** on the [RFC board](../contributions/README.md), with
   the reason and a link to where the work went.
3. Credit the selected contribution; leave the non-selected submissions in place.

Retirement is reversible — reopen an RFC by flipping the banner/`Status` back if
the shipped approach needs rework.

## Working in-tree (docs & specs)

When your contribution lives in the tree (docs/specs, not a linked repo), two
rules keep PRs clean and mergeable — most review friction comes from breaking
them:

- *Pull `main` before you branch, and rebase before you open the PR.* Stale
  branches re-add files that already merged and re-touch files that moved, which
  turns a small change into a pile of conflicts. If your PR's diff shows files
  you didn't mean to change, your branch is behind `main`.
- *Only edit files inside your own `contributions/<module>/<your-username>/`.*
  Don't edit the module's top-level `README.md`, another contributor's folder,
  the root `README.md` table, the [RFC board](../contributions/README.md),
  `ARCHITECTURE.md`, or `SOFTWARE_INTERFACES.md` directly — those are
  maintainer-owned or belong to someone else. If your work
  *implies* a change to one of them (a new module row, a new interface topic, an
  architecture note), **describe it in the PR body** and let the maintainer apply
  it. This keeps ownership clear and avoids two PRs fighting over the same lines.

Also make sure your PR description matches your actual diff — if the summary says
"one new file, no shared-file changes" but the diff touches five files, that's
the stale-branch tell above.

## Hardware contributions

For CAD and mechanical work, please include source files (not just exported STLs)
where possible, so others can modify your design. Note the tool and version you
used. If your change affects the bill of materials, mention it in the PR. Each
hardware module must stay within the mechanical/electrical interfaces in
[ARCHITECTURE.md](ARCHITECTURE.md).

*Safety:* battery, charging, motor-driver, and mains-adjacent modules require a
maintainer safety review before merge. Include a hazard note in your submission.

## Code style

Conventions are still being established. For now: keep it simple, readable, and
consistent with the surrounding code. ROS2 packages should follow standard ROS2
layout and naming. We'll formalize linting and style as the codebase grows.

## AI usage

OOMWOO is built with substantial AI assistance (Anthropic's Claude, via Claude
Code), directed and reviewed by humans. AI helps draft and iterate RFCs and docs,
write and review ROS2 code, debug the simulation, and write tutorials — but the
maintainer makes the design and architecture calls, and nothing merges without
human review and testing in simulation or on hardware.

Using AI on your own contribution is welcome. In return: understand, test, and
stand behind what you submit (you own it, not the tool), and note AI co-authorship
in your commits — e.g. a `Co-Authored-By:` trailer — the way this project does.

## Licensing

By contributing, you agree that your contributions are licensed under the
project's [Apache License 2.0](../LICENSE). Hardware design files will be released
under an open hardware license (to be finalized); contributions of hardware
files are made on that same open basis.

## Community and conduct

Be respectful, helpful, and welcoming. We want OOMWOO to be an easy, friendly
place for makers of every skill level. Harassment or hostility isn't tolerated.

Questions? Open a [Discussion](https://github.com/makerspet/oomwoo/discussions?discussions_q=)
or join us on [Discord](https://discord.gg/3y2JKz5T25).
