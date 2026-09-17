# Health Monitor by xbattlax

The maintained ROS 2 package, tests, simulator, and detailed contract docs now
live in
[xbattlax/health-monitor](https://github.com/xbattlax/health-monitor).

Stable release:
[v0.1.0](https://github.com/xbattlax/health-monitor/releases/tag/v0.1.0).

This code was first accepted in
[oomwoo#36](https://github.com/makerspet/oomwoo/pull/36) and moved to a
self-hosted package at the maintainer's request so it can be integrated,
tested, released, and vendored independently.

The standalone repository includes:

- the `oomwoo_health_monitor` ROS 2 Jazzy package
- dependency-free core and adapter tests
- a deterministic health/fault simulator
- stack-watchdog and recovery-safety integration documents
- CI for the core and a real ROS 2 Jazzy package build

Consumers should pin `v0.1.0` or an exact commit rather than tracking `main`.
The public OOMWOO topic contract remains in
[`docs/SOFTWARE_INTERFACES.md`](../../../docs/SOFTWARE_INTERFACES.md).
