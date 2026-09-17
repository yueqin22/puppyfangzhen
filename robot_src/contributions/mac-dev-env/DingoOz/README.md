# pixi ROS 2 (Lyrical) dev + sim setup — by @DingoOz

> Discord message verbatim copy

A reproducible dev setup for OOMWOO: a simulated robot you can drive around in
Gazebo, running on **ROS 2 Lyrical**, installed with [pixi](https://pixi.sh).

pixi installs the entire toolbox for the project — ROS 2, Gazebo, and everything
they need — into a self-contained folder from a locked list of versions. Everyone
gets the exact same setup, and it does **not** touch or depend on anything already
on your system: **no Docker, no apt, no "works on my machine"**. If you already
have ROS installed, pixi doesn't use it or touch it — it isolates itself and stays
out of the way. The ROS 2 binaries come from [RoboStack](https://robostack.github.io)
(ROS on conda-forge), which is what lets the same manifest resolve on Apple Silicon
as well as Linux.

> **Status — early, on a branch.** Source lives on
> [`feature/pixi-ros2-lyrical`](https://github.com/DingoOz/oomwoo/tree/feature/pixi-ros2-lyrical)
> of [github.com/DingoOz/oomwoo](https://github.com/DingoOz/oomwoo). Contributed
> via Discord on 2026-08-09. Per the project's
> [contribution model](../../../docs/CONTRIBUTING.md#how-contributions-are-structured),
> the code stays in the contributor's repo; this page is the in-tree pointer + notes.

## Platforms

Works on **Apple Silicon Macs** (this RFC's focus), and also **Linux** and **Raspberry Pi 5**.

## Setup

```bash
# 1. Install pixi (once)
curl -fsSL https://pixi.sh/install.sh | bash

# 2. Get the code and build
git clone -b feature/pixi-ros2-lyrical https://github.com/DingoOz/oomwoo
cd oomwoo
pixi install     # downloads ROS 2 + Gazebo into the project
pixi run build
```

## Run

```bash
pixi run sim       # opens Gazebo with the robot in a room
pixi run teleop    # in a second terminal, drive with i/j/k/l
```

That's it. The robot has a 2D LiDAR, so you can watch it sense the walls as you
drive.

## Notes

- **ROS distro:** this targets **ROS 2 Lyrical** (what RoboStack currently ships
  cleanly across these platforms). The project's canonical branches standardize on
  **ROS 2 Jazzy**, so this env is ahead of the project baseline — a deliberate open
  point for the RFC (see [../README.md](../README.md#considerations--open-questions)).
- **Isolation:** pixi keeps everything under the project folder; a pre-existing ROS
  install is neither used nor modified.

## What would help

Still early and on a branch. If you give it a go, report whether it comes up
cleanly on your machine — **especially on a Mac (Apple Silicon)** — with your
macOS version + chip (or platform), and anything you had to change. Post results in
[Project Discussions](https://github.com/makerspet/oomwoo/discussions) or
[Discord](https://discord.gg/3y2JKz5T25).

## Source

- Branch: <https://github.com/DingoOz/oomwoo/tree/feature/pixi-ros2-lyrical>
- Contributor: [@DingoOz](https://github.com/DingoOz)
