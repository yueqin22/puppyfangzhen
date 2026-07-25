# Puppy C++ Workspace

This directory is a standalone ROS 2 C++ workspace. Build it from `cpp_src` so
the legacy Python packages under `../src` are not mixed with the C++ package
names.

```powershell
cd E:\puppyfangzhen\cpp_src
colcon build --packages-up-to puppy_bringup_cpp
```

`puppy_bringup_cpp` builds the CoppeliaSim ZMQ bridge only when
`coppeliaSimZMQ.hpp` is discoverable. Set `COPPELIASIM_ROOT` to your
CoppeliaSim install if you need that bridge.
